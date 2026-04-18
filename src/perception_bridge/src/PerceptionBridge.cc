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

#include <algorithm>
#include <memory>
#include <utility>

#include <pcl/filters/voxel_grid.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl_conversions/pcl_conversions.h>

PerceptionBridge::PerceptionBridge(rclcpp::Node::SharedPtr node,
                                   const std::shared_ptr<zenoh::Session>& zsession)
  : ZenohPublisher(zsession, "autoware/perception/")
  , strand_(io_context_)
  , work_guard_(boost::asio::make_work_guard(io_context_))
  , publisher_timer_(io_context_)
  , node_(std::move(node)) {
  RCLCPP_INFO(node_->get_logger(), "[PerceptionBridge] Booting..");
  io_thread_ = std::thread([this]() { io_context_.run(); });

  auto sensor_qos = rclcpp::SensorDataQoS();
  surrounding_object_sub_ =
    node_->create_subscription<autoware_perception_msgs::msg::PredictedObjects>(
      "/perception/object_recognition/objects", sensor_qos,
      [this](autoware_perception_msgs::msg::PredictedObjects::SharedPtr msg) {
        onSurroundingObject(std::move(msg));
      });

  point_cloud_sub_ = node_->create_subscription<sensor_msgs::msg::PointCloud2>(
    "/perception/obstacle_segmentation/pointcloud", sensor_qos,
    [this](sensor_msgs::msg::PointCloud2::SharedPtr msg) { onPointCloud(std::move(msg)); }

  );

  occupancy_grid_sub_ = node_->create_subscription<nav_msgs::msg::OccupancyGrid>(
    "/perception/occupancy_grid_map/map", sensor_qos,
    [this](nav_msgs::msg::OccupancyGrid::SharedPtr msg) { onOccupancyGrid(std::move(msg)); });

  traffic_signal_sub_ =
    node_->create_subscription<autoware_perception_msgs::msg::TrafficLightGroupArray>(
      "/perception/traffic_light_recognition/traffic_signals", sensor_qos,
      [this](autoware_perception_msgs::msg::TrafficLightGroupArray::SharedPtr msg) {
        onTrafficSignal(std::move(msg));
      });

  boost::asio::post(strand_, [this]() { scheduleNextTick(); });
  RCLCPP_INFO(node_->get_logger(), "[PerceptionBridge] Ready...");
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
      if (ec == boost::asio::error::operation_aborted) {
        return;
      }
      onTick();
    }));
}

void PerceptionBridge::onTick() {
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
  publish(frame.SerializeAsString());
}

void PerceptionBridge::onSurroundingObject(
  autoware_perception_msgs::msg::PredictedObjects::SharedPtr msg) {
  boost::asio::post(strand_,
                    [this, msg = std::move(msg)]() mutable { onSurroundingObjectImpl(msg); });
}

void PerceptionBridge::onPointCloud(sensor_msgs::msg::PointCloud2::SharedPtr msg) {
  boost::asio::post(strand_, [this, msg = std::move(msg)]() mutable { onPointCloudImpl(msg); });
}

void PerceptionBridge::onOccupancyGrid(nav_msgs::msg::OccupancyGrid::SharedPtr msg) {
  boost::asio::post(strand_, [this, msg = std::move(msg)]() mutable { onOccupancyGridImpl(msg); });
}

void PerceptionBridge::onTrafficSignal(
  autoware_perception_msgs::msg::TrafficLightGroupArray::SharedPtr msg) {
  boost::asio::post(strand_, [this, msg = std::move(msg)]() mutable { onTrafficSignalImpl(msg); });
}

// TODO: complete all on-##nameImpl functions

void PerceptionBridge::onSurroundingObjectImpl(
  const autoware_perception_msgs::msg::PredictedObjects::SharedPtr& msg) {
  state_.surrounding_objects.clear();
  for (const auto& object : msg->objects) {
    if (object.classification.empty()) {
      continue;
    }
    if (object.existence_probability < 0.5F) {
      continue;
    }
    vehicle_frame::SurroundingObject so;
    so.set_id(static_cast<uint32_t>(state_.surrounding_objects.size()));
    so.set_object_class(toObjectClass(object.classification.at(0).label));
    so.mutable_bounding_box()->set_length(static_cast<float>(object.shape.dimensions.x));
    so.mutable_bounding_box()->set_height(static_cast<float>(object.shape.dimensions.z));
    so.mutable_bounding_box()->set_width(static_cast<float>(object.shape.dimensions.y));
    so.mutable_object_velocity()->set_vx_object(
      static_cast<float>(object.kinematics.initial_twist_with_covariance.twist.linear.x));
    so.mutable_object_velocity()->set_vy_object(
      static_cast<float>(object.kinematics.initial_twist_with_covariance.twist.linear.y));
    so.mutable_object_pose()->set_x(
      static_cast<float>(object.kinematics.initial_pose_with_covariance.pose.position.x));
    so.mutable_object_pose()->set_y(
      static_cast<float>(object.kinematics.initial_pose_with_covariance.pose.position.y));
    so.mutable_object_pose()->set_z(
      static_cast<float>(object.kinematics.initial_pose_with_covariance.pose.position.z));
    const auto& h = object.kinematics.initial_pose_with_covariance.pose.orientation;
    auto const TEMP = static_cast<float>(
      std::atan2(2.0F * (h.w * h.z + h.x * h.y), 1.0F - (2.0F * (h.y * h.y + h.z * h.z))));
    so.set_heading(TEMP);
    state_.surrounding_objects.push_back(std::move(so));
  }
}

void PerceptionBridge::onPointCloudImpl(const sensor_msgs::msg::PointCloud2::SharedPtr& msg) {
  pcl::PointCloud<pcl::PointXYZ>::Ptr const CLOUD =
    std::make_shared<pcl::PointCloud<pcl::PointXYZ>>();

  pcl::fromROSMsg(*msg, *CLOUD);

  pcl::PointCloud<pcl::PointXYZ>::Ptr const DOWNSAMPLED_CLOUD =
    std::make_shared<pcl::PointCloud<pcl::PointXYZ>>();
  pcl::VoxelGrid<pcl::PointXYZ> vg;
  vg.setInputCloud(CLOUD);
  vg.setLeafSize(0.2F, 0.2F, 0.2F);
  vg.filter(*DOWNSAMPLED_CLOUD);

  state_.points_cloud.clear();
  for (const auto& point : DOWNSAMPLED_CLOUD->points) {
    if (!std::isfinite(point.x) || !std::isfinite(point.y) || !std::isfinite(point.z)) {
      continue;
    }
    vehicle_frame::PointCloud pc;
    pc.mutable_point()->set_x(point.x);
    pc.mutable_point()->set_y(point.y);
    pc.mutable_point()->set_z(point.z);
    state_.points_cloud.push_back(std::move(pc));
  }
}

void PerceptionBridge::onOccupancyGridImpl(const nav_msgs::msg::OccupancyGrid::SharedPtr& msg) {
  vehicle_frame::OccupancyGrid og;
  og.set_resolution(msg->info.resolution);
  og.set_width(msg->info.width);
  og.set_height(msg->info.height);
  og.set_origin_x(static_cast<float>(msg->info.origin.position.x));
  og.set_origin_y(static_cast<float>(msg->info.origin.position.y));

  for (const int8_t CELL : msg->data) {
    if (CELL == -1) {
      og.add_grid_data(vehicle_frame::CELL_UNKNOWN);
    } else if (CELL < 50) {
      og.add_grid_data(vehicle_frame::CELL_FREE);
    } else {
      og.add_grid_data(vehicle_frame::CELL_OCCUPIED);
    }
  }

  state_.occupancy_grid = std::move(og);
}

void PerceptionBridge::onTrafficSignalImpl(
  const autoware_perception_msgs::msg::TrafficLightGroupArray::SharedPtr& msg) {
  state_.traffic_lights.clear();
  for (const auto& group : msg->traffic_light_groups) {
    if (group.elements.empty()) {
      continue;
    }

    const auto& maxx = *std::ranges::max_element(
      group.elements, [](const auto& a, const auto& b) { return a.confidence < b.confidence; });
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