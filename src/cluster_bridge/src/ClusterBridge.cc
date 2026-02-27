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

#include "ClusterBridge.h"

#include <boost/asio/bind_executor.hpp>

class ClusterBridge::ClusterServiceImpl final : public vehicle_frame::ClusterService::Service {
 public:
  explicit ClusterServiceImpl(ClusterBridge& parent) : parent_(parent) {};

  grpc::Status Subscribe(grpc::ServerContext* context,
                         const vehicle_frame::SubscribeRequest* request,
                         grpc::ServerWriter<vehicle_frame::VehicleFrame>* writer) override {
    RCLCPP_INFO(parent_.node_->get_logger(), "[ClusterBridge] IVI/Cluster client connected: '%s'",
                request->client_id().c_str());
    {
      std::lock_guard<std::mutex> lk(parent_.clients_mutex_);
      parent_.grpc_clients_.emplace_back(writer);
    }

    while (!context->IsCancelled()) {
      std::this_thread::sleep_for(std::chrono::milliseconds(50));
    }

    {
      std::lock_guard<std::mutex> lk(parent_.clients_mutex_);
      auto& vec = parent_.grpc_clients_;
      vec.erase(std::remove(vec.begin(), vec.end(), writer), vec.end());
    }

    RCLCPP_INFO(parent_.node_->get_logger(),
                "[ClusterBridge] IVI/Cluster client disconnected : '%s'",
                request->client_id().c_str());
    return grpc::Status::OK;
  }
  grpc::Status GetLatestFrame(grpc::ServerContext* context,
                              const vehicle_frame::SubscribeRequest* request,
                              vehicle_frame::VehicleFrame* response) override {
    std::promise<vehicle_frame::VehicleFrame> promise;
    auto future = promise.get_future();

    boost::asio::post(parent_.strand_, [&parent = parent_, p = std::move(promise)]() mutable {
      p.set_value(parent.buildFrame());
    });

    *response = future.get();
    return grpc::Status::OK;
  }

 private:
  ClusterBridge& parent_;
};

ClusterBridge::ClusterBridge(rclcpp::Node::SharedPtr node, const std::string& grpc_address)
  : node_(std::move(node))
  , grpc_address_(grpc_address)
  , work_guard_(boost::asio::make_work_guard(io_context_))
  , publisher_timer_(io_context_)
  , strand_(io_context_) {
  RCLCPP_INFO(node_->get_logger(), "[ClusterBridge] Booting..");
  io_thread_ = std::thread([this]() { io_context_.run(); });

  auto sensor_qos = rclcpp::SensorDataQoS();
  auto reliable_qos = rclcpp::QoS(1).reliable().durability_volatile();
  auto transient_local_qos = rclcpp::QoS(1).transient_local();

  velocity_sub_ = node_->create_subscription<autoware_vehicle_msgs::msg::VelocityReport>(
    "/vehicle/status/velocity_status", sensor_qos,
    [this](const autoware_vehicle_msgs::msg::VelocityReport::SharedPtr msg) { onVelocity(msg); });

  // TODO: complete other subscribers

  grpc_service_ = std::make_unique<ClusterServiceImpl>(*this);
  boost::asio::post(strand_, [this]() { scheduleNextTick(); });
  RCLCPP_INFO(node_->get_logger(), "[ClusterBrdige] Ready...");
}

ClusterBridge::~ClusterBridge() {
  shutdown();
}

void ClusterBridge::shutdown() {
  if (grpc_server_) {
    grpc_server_->Shutdown();
  }
  work_guard_.reset();
  publisher_timer_.cancel();
  if (io_thread_.joinable()) {
    io_thread_.join();
  }
}

void ClusterBridge::runGrpcServer() {
  grpc::ServerBuilder builder;
  builder.AddListeningPort(grpc_address_, grpc::InsecureServerCredentials());
  builder.RegisterService(grpc_service_.get());
  grpc_server_ = builder.BuildAndStart();
  RCLCPP_INFO(node_->get_logger(), "[ClusterBridge] gRPC server on %s", grpc_address_.c_str());
  grpc_server_->Wait();
}

void ClusterBridge::scheduleNextTick() {
  publisher_timer_.expires_after(std::chrono::milliseconds(16667));  // 60hz
  publisher_timer_.async_wait(
    boost::asio::bind_executor(strand_, [this](const boost::system::error_code& ec) {
      if (ec == boost::asio::error::operation_aborted)
        return;
      ontick();
    }));
}

void ClusterBridge::ontick() {
  broadcastFrame(buildFrame());
  scheduleNextTick();
}

vehicle_frame::VehicleFrame ClusterBridge::buildFrame() {
  vehicle_frame::VehicleFrame frame;
  frame.set_stamp_ns(node_->now().nanoseconds());
  frame.set_seq(frame_seq_++);

  frame.mutable_velocity()->set_speed_mps(state_.speed_mps);
  frame.mutable_velocity()->set_speed_kmh(state_.speed_kmh);
  frame.set_gear(state_.gear);

  // TODO: complete all frame from state struct which is being update on ROS callbacks

  return frame;
}

void ClusterBridge::broadcastFrame(const vehicle_frame::VehicleFrame& frame) {
  std::lock_guard<std::mutex> lk(clients_mutex_);
  for (auto& client : grpc_clients_) {
    client->Write(frame);
  }
}

// ROS Callbacks called on ROS executor thread, each one does only one thing post the message to
// strand
// TODO: complete all on-##name functions

void ClusterBridge::onVelocity(const autoware_vehicle_msgs::msg::VelocityReport::SharedPtr msg) {
  boost::asio::post(strand_, [this, msg = std::move(msg)]() mutable { onVelocityImpl(msg); });
}

void ClusterBridge::onGear(const autoware_vehicle_msgs::msg::GearReport::SharedPtr msg) {
  boost::asio::post(strand_, [this, msg = std::move(msg)]() mutable { onGearImpl(msg); });
}

// TODO: complete all on-##nameImpl functions

void ClusterBridge::onVelocityImpl(
  const autoware_vehicle_msgs::msg::VelocityReport::SharedPtr msg) {
  state_.speed_mps = msg->longitudinal_velocity;
  state_.speed_kmh = msg->longitudinal_velocity * 3.6f;
}

void ClusterBridge::onGearImpl(const autoware_vehicle_msgs::msg::GearReport::SharedPtr msg) {
  state_.gear = toGear(msg->report);
}

vehicle_frame::GearState ClusterBridge::toGear(uint8_t v) {
  using GR = autoware_vehicle_msgs::msg::GearReport;
  switch (v) {
    case GR::PARK:
      return vehicle_frame::GEAR_PARK;

    case GR::REVERSE:
      return vehicle_frame::GEAR_REVERSE;

    case GR::NEUTRAL:
      return vehicle_frame::GEAR_NEUTRAL;

    case GR::DRIVE:
      return vehicle_frame::GEAR_DRIVE;

    default:
      return vehicle_frame::GEAR_UNKNOWN;
  }
}

vehicle_frame::TurnSignal ClusterBridge::toTurn(uint8_t v) {
  using TIR = autoware_vehicle_msgs::msg::TurnIndicatorsReport;
  switch (v) {
    case TIR::ENABLE_LEFT:
      return vehicle_frame::TURN_LEFT;

    case TIR::ENABLE_RIGHT:
      return vehicle_frame::TURN_RIGHT;

    default:
      return vehicle_frame::TURN_NONE;
  }
}

vehicle_frame::ControlMode ClusterBridge::toCtrlMode(uint8_t v) {
  using CMR = autoware_vehicle_msgs::msg::ControlModeReport;
  return (v == CMR::AUTONOMOUS) ? vehicle_frame::MODE_AUTO : vehicle_frame::MODE_MANUAL;
}

vehicle_frame::MotionState ClusterBridge::toMotionState(uint8_t v) {
  using MS = autoware_adapi_v1_msgs::msg::MotionState;
  switch (v) {
    case MS::STOPPED:
      return vehicle_frame::MOTION_STOPPED;
    case MS::MOVING:
      return vehicle_frame::MOTION_MOVING;
    case MS::STARTING:
      return vehicle_frame::MOTION_DECELERATING;
    default:
      return vehicle_frame::MOTION_UNKNOWN;
  }
}