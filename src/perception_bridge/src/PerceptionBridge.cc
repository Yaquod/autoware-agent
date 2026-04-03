/*
 * Copyright 2026 wafdy
 *
 * Licensed under the Apache License, Version 2.0 (the "License");
 * you may not use this file except in compliance with the License.
 * You may obtain a copy of the License at
 *
 * http://www.apache.org/licenses/LICENSE-2.0
 *
 * Unless required by applicable law or agreed to in writing, software
 * distributed under the License is distributed on an "AS IS" BASIS,
 * WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
 * See the License for the specific language governing permissions and
 * limitations under the License.
 */

#include "PerceptionBridge.h"

#include <boost/asio/bind_executor.hpp>

#include <pcl/filters/voxel_grid.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl_conversions/pcl_conversions.h>

class PerceptionBridge::PerceptionServiceImpl final
  : public vehicle_frame::PerceptionService::Service {
 public:
  explicit PerceptionServiceImpl(PerceptionBridge& parent) : parent_(parent){};

  grpc::Status Subscribe(grpc::ServerContext* context,
                         const vehicle_frame::SubscribeRequest* request,
                         grpc::ServerWriter<vehicle_frame::PerceptionFrame>* writer) override {
    auto session = std::make_shared<ClientSession>();
    session->writer = writer;

    {
      std::lock_guard<std::mutex> lk(parent_.clients_mutex_);
      parent_.grpc_clients_.push_back(session);
    }

    while (!context->IsCancelled() && session->alive) {
      vehicle_frame::PerceptionFrame frame;
      {
        std::unique_lock<std::mutex> lk(session->mu);
        session->cv.wait_for(lk, std::chrono::milliseconds(100),
                             [&session] { return !session->pending.empty() || !session->alive; });
        if (session->pending.empty())
          continue;
        frame = std::move(session->pending.front());
        session->pending.pop();
      }
      if (!writer->Write(frame))
        break;
    }

    {
      std::lock_guard<std::mutex> lk(parent_.clients_mutex_);
      auto& vec = parent_.grpc_clients_;
      vec.erase(std::remove(vec.begin(), vec.end(), session), vec.end());
    }
    return grpc::Status::OK;
  }

  grpc::Status GetLatestFrame(grpc::ServerContext* context,
                              const vehicle_frame::SubscribeRequest* request,
                              vehicle_frame::PerceptionFrame* response) override {
    std::promise<vehicle_frame::PerceptionFrame> promise;
    auto future = promise.get_future();

    boost::asio::post(parent_.strand_, [&parent = parent_, p = std::move(promise)]() mutable {
      p.set_value(parent.buildFrame());
    });

    *response = future.get();
    return grpc::Status::OK;
  }

 private:
  PerceptionBridge& parent_;
};

PerceptionBridge::PerceptionBridge(rclcpp::Node::SharedPtr node, grpc::ServerBuilder& builder)
  : frame_seq_(0)
  , io_context_()
  , strand_(io_context_)
  , work_guard_(boost::asio::make_work_guard(io_context_))
  , publisher_timer_(io_context_)
  , node_(std::move(node)) {
  RCLCPP_INFO(node_->get_logger(), "[PerceptionBridge] Booting..");
  io_thread_ = std::thread([this]() { io_context_.run(); });

  auto sensor_qos = rclcpp::SensorDataQoS();
  auto reliable_qos = rclcpp::QoS(1).reliable().durability_volatile();
  auto transient_local_qos = rclcpp::QoS(1).transient_local();
  auto perception_qos = rclcpp::QoS(1).best_effort().durability_volatile();

  surrounding_object_sub_ =
    node_->create_subscription<autoware_perception_msgs::msg::PredictedObjects>(
      "/perception/object_recognition/objects", sensor_qos,
      [this](const autoware_perception_msgs::msg::PredictedObjects::SharedPtr msg) {
        onSurroundingObject(msg);
      });

  point_cloud_sub_ = node_->create_subscription<sensor_msgs::msg::PointCloud2>(
    "/perception/obstacle_segmentation/pointcloud", sensor_qos,
    [this](const sensor_msgs::msg::PointCloud2::SharedPtr msg) { onPointCloud(msg); }

  );

  occupancy_grid_sub_ = node_->create_subscription<nav_msgs::msg::OccupancyGrid>(
    "/perception/occupancy_grid_map/map", sensor_qos,
    [this](const nav_msgs::msg::OccupancyGrid::SharedPtr msg) { onOccupancyGrid(msg); });

  traffic_signal_sub_ =
    node_->create_subscription<autoware_perception_msgs::msg::TrafficLightGroupArray>(
      "/perception/traffic_light_recognition/traffic_signals", sensor_qos,
      [this](const autoware_perception_msgs::msg::TrafficLightGroupArray::SharedPtr msg) {
        onTrafficSignal(msg);
      });

  grpc_service_ = std::make_unique<PerceptionServiceImpl>(*this);
  builder.RegisterService(grpc_service_.get());
  boost::asio::post(strand_, [this]() { scheduleNextTick(); });
  RCLCPP_INFO(node_->get_logger(), "[PerceptionBrdige] Ready...");
}

PerceptionBridge::~PerceptionBridge() {
  shutdown();
}

void PerceptionBridge::shutdown() {
  publisher_timer_.cancel();
  work_guard_.reset();

  if (io_thread_.joinable()) {
    io_thread_.join();
  }
}

void PerceptionBridge::scheduleNextTick() {
  publisher_timer_.expires_after(std::chrono::microseconds(16667));  // 60hz
  publisher_timer_.async_wait(
    boost::asio::bind_executor(strand_, [this](const boost::system::error_code& ec) {
      if (ec == boost::asio::error::operation_aborted)
        return;
      ontick();
    }));
}

void PerceptionBridge::ontick() {
  auto t0 = std::chrono::steady_clock::now();
  broadcastFrame(buildFrame());
  auto dt = std::chrono::steady_clock::now() - t0;
  if (dt > std::chrono::milliseconds(5)) {
    RCLCPP_WARN(node_->get_logger(), "[PlanningBridge] ontick delayed %ldms",
                std::chrono::duration_cast<std::chrono::milliseconds>(dt).count());
  }
  scheduleNextTick();
}

vehicle_frame::PerceptionFrame PerceptionBridge::buildFrame() {
  vehicle_frame::PerceptionFrame frame;
  frame.set_stamp_ns(node_->now().nanoseconds());
  frame.set_seq(frame_seq_++);

  *frame.mutable_surrounding_objects() = {state_.surrounding_objects.begin(),
                                          state_.surrounding_objects.end()};

  *frame.mutable_points_cloud() = {state_.points_cloud.begin(), state_.points_cloud.end()};

  *frame.mutable_occupancy_grid() = state_.occupancy_grid;

  *frame.mutable_traffic_lights() = {state_.traffic_lights.begin(), state_.traffic_lights.end()};

  return frame;
}

void PerceptionBridge::broadcastFrame(const vehicle_frame::PerceptionFrame& frame) {
  std::lock_guard<std::mutex> lk(clients_mutex_);
  for (auto& session : grpc_clients_) {
    {
      std::lock_guard<std::mutex> slk(session->mu);
      if (session->pending.size() > 5)
        session->pending.pop();
      session->pending.push(frame);
    }
    session->cv.notify_one();
  }
}

// ROS Callbacks called on ROS executor thread, each one does only one thing post the message to
// strand
// TODO: complete all on-##name functions

void PerceptionBridge::onSurroundingObject(
  const autoware_perception_msgs::msg::PredictedObjects::SharedPtr msg) {
  boost::asio::post(strand_,
                    [this, msg = std::move(msg)]() mutable { onSurroundingObjectImpl(msg); });
}

void PerceptionBridge::onPointCloud(const sensor_msgs::msg::PointCloud2::SharedPtr msg) {
  boost::asio::post(strand_, [this, msg = std::move(msg)]() mutable { onPointCloudImpl(msg); });
}

void PerceptionBridge::onOccupancyGrid(const nav_msgs::msg::OccupancyGrid::SharedPtr msg) {
  boost::asio::post(strand_, [this, msg = std::move(msg)]() mutable { onOccupancyGridImpl(msg); });
}

void PerceptionBridge::onTrafficSignal(
  const autoware_perception_msgs::msg::TrafficLightGroupArray::SharedPtr msg) {
  boost::asio::post(strand_, [this, msg = std::move(msg)]() mutable { onTrafficSignalImpl(msg); });
}

// TODO: complete all on-##nameImpl functions

void PerceptionBridge::onSurroundingObjectImpl(
  const autoware_perception_msgs::msg::PredictedObjects::SharedPtr msg) {
  for (const auto& object : msg->objects) {
    if (object.classification.empty())
      continue;
    if (object.existence_probability < 0.5f)
      continue;
    vehicle_frame::SurroundingObject so;
    so.set_id(static_cast<uint32_t>(state_.surrounding_objects.size()));
    so.set_object_class(toObjectClass(object.classification[0].label));
    so.mutable_bounding_box()->set_length(object.shape.dimensions.x);
    so.mutable_bounding_box()->set_height(object.shape.dimensions.z);
    so.mutable_bounding_box()->set_width(object.shape.dimensions.y);
    so.mutable_object_velocity()->set_vx_object(
      object.kinematics.initial_twist_with_covariance.twist.linear.x);
    so.mutable_object_velocity()->set_vy_object(
      object.kinematics.initial_twist_with_covariance.twist.linear.y);
    so.mutable_object_pose()->set_x(object.kinematics.initial_pose_with_covariance.pose.position.x);
    so.mutable_object_pose()->set_y(object.kinematics.initial_pose_with_covariance.pose.position.y);
    so.mutable_object_pose()->set_z(object.kinematics.initial_pose_with_covariance.pose.position.z);
    const auto& h = object.kinematics.initial_pose_with_covariance.pose.orientation;
    float temp = std::atan2(2.0f * (h.w * h.z + h.x * h.y), 1.0f - 2.0f * (h.y * h.y + h.z * h.z));
    so.set_heading(temp);
    state_.surrounding_objects.push_back(std::move(so));
  }
}

void PerceptionBridge::onPointCloudImpl(const sensor_msgs::msg::PointCloud2::SharedPtr msg) {
  pcl::PointCloud<pcl::PointXYZ>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZ>);

  pcl::fromROSMsg(*msg, *cloud);

  pcl::PointCloud<pcl::PointXYZ>::Ptr downsampled_cloud(new pcl::PointCloud<pcl::PointXYZ>);
  pcl::VoxelGrid<pcl::PointXYZ> vg;
  vg.setInputCloud(cloud);
  vg.setLeafSize(0.2f, 0.2f, 0.2f);
  vg.filter(*downsampled_cloud);

  state_.points_cloud.clear();
  for (const auto& point : downsampled_cloud->points) {
    if (!std::isfinite(point.x) || !std::isfinite(point.y) || !std::isfinite(point.z))
      continue;
    vehicle_frame::PointCloud pc;
    pc.mutable_point()->set_x(point.x);
    pc.mutable_point()->set_y(point.y);
    pc.mutable_point()->set_z(point.z);
    state_.points_cloud.push_back(std::move(pc));
  }
}

void PerceptionBridge::onOccupancyGridImpl(const nav_msgs::msg::OccupancyGrid::SharedPtr msg) {
  vehicle_frame::OccupancyGrid og;
  og.set_resolution(msg->info.resolution);
  og.set_width(msg->info.width);
  og.set_height(msg->info.height);
  og.set_origin_x(static_cast<float>(msg->info.origin.position.x));
  og.set_origin_y(static_cast<float>(msg->info.origin.position.y));

  for (const int8_t cell : msg->data) {
    if (cell == -1) {
      og.add_grid_data(vehicle_frame::CELL_UNKNOWN);
    } else if (cell < 50) {
      og.add_grid_data(vehicle_frame::CELL_FREE);
    } else {
      og.add_grid_data(vehicle_frame::CELL_OCCUPIED);
    }
  }

  state_.occupancy_grid = std::move(og);
}

void PerceptionBridge::onTrafficSignalImpl(
  const autoware_perception_msgs::msg::TrafficLightGroupArray::SharedPtr msg) {
  state_.traffic_lights.clear();
  for (const auto& group : msg->traffic_light_groups) {
    if (group.elements.empty())
      continue;

    const auto& maxx =
      *std::max_element(group.elements.begin(), group.elements.end(),
                        [](const auto& a, const auto& b) { return a.confidence < b.confidence; });
    vehicle_frame::TrafficLight tl;
    tl.set_traffic_id(group.traffic_light_group_id);
    tl.set_confidence(maxx.confidence);
    tl.set_traffic_light_color(toTrafficLightColor(maxx.color));
    state_.traffic_lights.push_back(std::move(tl));
  }
}

vehicle_frame::ObjectClass PerceptionBridge::toObjectClass(uint8_t v) {
  using OC = autoware_perception_msgs::msg::ObjectClassification;
  switch (v) {
    case OC::CAR:
      return vehicle_frame::CAR;
    case OC::TRUCK:
      return vehicle_frame::TRUCK;
    case OC::BUS:
      return vehicle_frame::BUS;
    case OC::TRAILER:
      return vehicle_frame::TRAILER;
    case OC::MOTORCYCLE:
      return vehicle_frame::MOTORCYCLE;
    case OC::BICYCLE:
      return vehicle_frame::BICYCLE;
    case OC::PEDESTRIAN:
      return vehicle_frame::PEDESTRIAN;
    case OC::ANIMAL:
      return vehicle_frame::ANIMAL;
    default:
      return vehicle_frame::UNKNOWN;
  }
}

vehicle_frame::TrafficLightColor PerceptionBridge::toTrafficLightColor(uint8_t v) {
  using TLE = autoware_perception_msgs::msg::TrafficLightElement;
  switch (v) {
    case TLE::RED:
      return vehicle_frame::TRAFFIC_LIGHT_RED;
    case TLE::AMBER:
      return vehicle_frame::TRAFFIC_LIGHT_AMBER;
    case TLE::GREEN:
      return vehicle_frame::TRAFFIC_LIGHT_GREEN;
    default:
      return vehicle_frame::TRAFFIC_LIGHT_UNKNOWN;
  }
}