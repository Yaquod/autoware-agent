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

#include "TripBridge.h"

#include <boost/asio/bind_executor.hpp>

class TripBridge::TripServiceImpl final : public vehicle_frame::TripService::Service {
 public:
  explicit TripServiceImpl(TripBridge& parent) : parent_(parent) {};

  grpc::Status Subscribe(grpc::ServerContext* context,
                         const vehicle_frame::SubscribeRequest* request,
                         grpc::ServerWriter<vehicle_frame::TripFrame>* writer) override {
    auto session = std::make_shared<ClientSession>();
    session->writer = writer;

    {
      std::lock_guard<std::mutex> lk(parent_.clients_mutex_);
      parent_.grpc_clients_.push_back(session);
    }

    while (!context->IsCancelled() && session->alive) {
      vehicle_frame::TripFrame frame;
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
                              vehicle_frame::TripFrame* response) override {
    std::promise<vehicle_frame::TripFrame> promise;
    auto future = promise.get_future();

    boost::asio::post(parent_.strand_, [&parent = parent_, p = std::move(promise)]() mutable {
      p.set_value(parent.buildFrame());
    });

    *response = future.get();
    return grpc::Status::OK;
  }

 private:
  TripBridge& parent_;
};

TripBridge::TripBridge(std::shared_ptr<AutowareAgent::AutowareController> controller,
                       rclcpp::Node::SharedPtr node, grpc::ServerBuilder& builder)
  : controller_(controller)
  , frame_seq_(0)
  , io_context_()
  , strand_(io_context_)
  , work_guard_(boost::asio::make_work_guard(io_context_))
  , publisher_timer_(io_context_)
  , node_(std::move(node)) {
  RCLCPP_INFO(node_->get_logger(), "[TripBridge] Booting..");
  last_heartbeat_time_ = node_->now();
  io_thread_ = std::thread([this]() { io_context_.run(); });

  auto sensor_qos = rclcpp::SensorDataQoS();
  auto reliable_qos = rclcpp::QoS(1).reliable().durability_volatile();
  auto transient_local_qos = rclcpp::QoS(1).transient_local();
  auto perception_qos = rclcpp::QoS(1).best_effort().durability_volatile();

  localization_state_sub_ =
    node_->create_subscription<autoware_adapi_v1_msgs::msg::LocalizationInitializationState>(
      "/api/localization/initialization_state", sensor_qos,
      [this](const autoware_adapi_v1_msgs::msg::LocalizationInitializationState::SharedPtr msg) {
        onLocalizationState(msg);
      });

  operation_mode_sub_ = node_->create_subscription<autoware_adapi_v1_msgs::msg::OperationModeState>(
    "/api/operation_mode/state", sensor_qos,
    [this](const autoware_adapi_v1_msgs::msg::OperationModeState::SharedPtr msg) {
      onOperationMode(msg);
    });

  mrm_state_sub_ = node_->create_subscription<autoware_adapi_v1_msgs::msg::MrmState>(
    "/api/fail_safe/mrm_state", sensor_qos,
    [this](const autoware_adapi_v1_msgs::msg::MrmState::SharedPtr msg) { onMrmState(msg); });
  heartbeat_sub_ = node_->create_subscription<autoware_adapi_v1_msgs::msg::Heartbeat>(
    "/api/system/heartbeat", sensor_qos,
    [this](const autoware_adapi_v1_msgs::msg::Heartbeat::SharedPtr msg) { onHeartbeat(msg); });

  diag_state_sub_ = node_->create_subscription<tier4_system_msgs::msg::DiagGraphStatus>(
    "/diagnostics_graph/status", sensor_qos,
    [this](const tier4_system_msgs::msg::DiagGraphStatus::SharedPtr msg) { onDiagState(msg); });

  grpc_service_ = std::make_unique<TripServiceImpl>(*this);
  builder.RegisterService(grpc_service_.get());
  boost::asio::post(strand_, [this]() { scheduleNextTick(); });
  RCLCPP_INFO(node_->get_logger(), "[TripBrdige] Ready...");
}

TripBridge::~TripBridge() {
  shutdown();
}

void TripBridge::shutdown() {
  publisher_timer_.cancel();
  work_guard_.reset();

  if (io_thread_.joinable()) {
    io_thread_.join();
  }
}

void TripBridge::scheduleNextTick() {
  publisher_timer_.expires_after(std::chrono::microseconds(16667));  // 60hz
  publisher_timer_.async_wait(
    boost::asio::bind_executor(strand_, [this](const boost::system::error_code& ec) {
      if (ec == boost::asio::error::operation_aborted)
        return;
      ontick();
    }));
}

void TripBridge::ontick() {
  auto dur = (node_->now() - last_heartbeat_time_).seconds();
  state_.system_alive = (dur < 1.0);

  // manage data comes from autoware controller
  auto status = controller_->getTripStatusSync();
  state_.trip_state = toTripState(status.state);
  state_.start_lanelet_id = status.start_lanelet_id;
  state_.start_x = status.start_x;
  state_.start_y = status.start_y;
  state_.start_z = status.start_z;
  state_.goal_lanelet_id = status.goal_lanelet_id;
  state_.goal_x = status.goal_x;
  state_.goal_y = status.goal_y;
  state_.goal_z = status.goal_z;
  state_.goal_distance_m = status.goal_distance_m;

  auto t0 = std::chrono::steady_clock::now();
  broadcastFrame(buildFrame());
  auto dt = std::chrono::steady_clock::now() - t0;
  if (dt > std::chrono::milliseconds(5)) {
    RCLCPP_WARN(node_->get_logger(), "[TripBridge] ontick delayed %ldms",
                std::chrono::duration_cast<std::chrono::milliseconds>(dt).count());
  }
  scheduleNextTick();
}

vehicle_frame::TripFrame TripBridge::buildFrame() {
  vehicle_frame::TripFrame frame;
  frame.set_stamp_ns(node_->now().nanoseconds());
  frame.set_seq(frame_seq_++);

  frame.set_localization_state(state_.localization_state);
  frame.set_operation_mode(state_.operation_mode);
  frame.mutable_mrm_state()->CopyFrom(state_.mrm_state);
  frame.set_system_alive(state_.system_alive);

  *frame.mutable_component_health() = {state_.components.begin(), state_.components.end()};

  frame.set_trip_state(state_.trip_state);
  frame.set_start_lanelet_id(state_.start_lanelet_id);
  frame.set_start_x(state_.start_x);
  frame.set_start_y(state_.start_y);
  frame.set_start_z(state_.start_z);
  frame.set_goal_lanelet_id(state_.goal_lanelet_id);
  frame.set_goal_x(state_.goal_x);
  frame.set_goal_y(state_.goal_y);
  frame.set_goal_z(state_.goal_z);
  frame.set_goal_distance_m(state_.goal_distance_m);

  return frame;
}

void TripBridge::broadcastFrame(const vehicle_frame::TripFrame& frame) {
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

void TripBridge::onLocalizationState(
  const autoware_adapi_v1_msgs::msg::LocalizationInitializationState::SharedPtr msg) {
  boost::asio::post(strand_,
                    [this, msg = std::move(msg)]() mutable { onLocalizationStateImpl(msg); });
}
void TripBridge::onOperationMode(
  const autoware_adapi_v1_msgs::msg::OperationModeState::SharedPtr msg) {
  boost::asio::post(strand_, [this, msg = std::move(msg)]() mutable { onOperationModeImpl(msg); });
}
void TripBridge::onMrmState(const autoware_adapi_v1_msgs::msg::MrmState::SharedPtr msg) {
  boost::asio::post(strand_, [this, msg = std::move(msg)]() mutable { onMrmStateImpl(msg); });
}

void TripBridge::onHeartbeat(const autoware_adapi_v1_msgs::msg::Heartbeat::SharedPtr msg) {
  boost::asio::post(strand_, [this, msg = std::move(msg)]() mutable { onHeartbeatImpl(msg); });
}
void TripBridge::onDiagState(const tier4_system_msgs::msg::DiagGraphStatus::SharedPtr msg) {
  boost::asio::post(strand_, [this, msg = std::move(msg)]() mutable { onDiagStateImpl(msg); });
}

// TODO: complete all on-##nameImpl functions

void TripBridge::onLocalizationStateImpl(
  const autoware_adapi_v1_msgs::msg::LocalizationInitializationState::SharedPtr msg) {
  state_.localization_state = toLocalizationState(msg->state);
}

void TripBridge::onOperationModeImpl(
  const autoware_adapi_v1_msgs::msg::OperationModeState::SharedPtr msg) {
  state_.operation_mode = toOperationMode(msg->mode);
}

void TripBridge::onMrmStateImpl(const autoware_adapi_v1_msgs::msg::MrmState::SharedPtr msg) {
  using MRM = autoware_adapi_v1_msgs::msg::MrmState;
  state_.mrm_state.set_is_active(msg->state != MRM::NORMAL && msg->state != MRM::UNKNOWN);
  state_.mrm_state.set_behavior(toMrmBehavior(msg->behavior));

  switch (msg->state) {
    case MRM::NORMAL:
      state_.mrm_state.set_description("Normal");
      break;
    case MRM::MRM_OPERATING:
      state_.mrm_state.set_description("MRM Operating");
      break;
    case MRM::MRM_SUCCEEDED:
      state_.mrm_state.set_description("MRM Succeeded");
      break;
    case MRM::MRM_FAILED:
      state_.mrm_state.set_description("MRM Failed");
      break;
    default:
      state_.mrm_state.set_description("Unknown");
      break;
  }
}

void TripBridge::onHeartbeatImpl(const autoware_adapi_v1_msgs::msg::Heartbeat::SharedPtr msg) {
  last_heartbeat_time_ = node_->now();
  state_.system_alive = true;
}

void TripBridge::onDiagStateImpl(const tier4_system_msgs::msg::DiagGraphStatus::SharedPtr msg) {
  state_.components.clear();
  for (const auto& diag : msg->diags) {
    if (diag.hardware_id.empty())
      continue;
    vehicle_frame::ComponentHealth ch;
    ch.set_name(diag.hardware_id);
    ch.set_level(toDiagLevel(diag.level));
    ch.set_message(diag.message);
    state_.components.push_back(std::move(ch));
  }
}

vehicle_frame::LocalizationState TripBridge::toLocalizationState(uint16_t v) {
  using LS = autoware_adapi_v1_msgs::msg::LocalizationInitializationState;
  switch (v) {
    case LS::UNINITIALIZED:
      return vehicle_frame::LocalizationState::LOCALIZATION_STATE_UNINITIALIZED;
    case LS::INITIALIZING:
      return vehicle_frame::LocalizationState::LOCALIZATION_STATE_INITIALIZING;
    case LS::INITIALIZED:
      return vehicle_frame::LocalizationState::LOCALIZATION_STATE_INITIALIZED;
    default:
      return vehicle_frame::LocalizationState::LOCALIZATION_STATE_UNKNOWN;
  }
}

vehicle_frame::OperationMode TripBridge::toOperationMode(uint8_t v) {
  using OM = autoware_adapi_v1_msgs::msg::OperationModeState;
  switch (v) {
    case OM::STOP:
      return vehicle_frame::OperationMode::OPERATION_MODE_STOP;
    case OM::AUTONOMOUS:
      return vehicle_frame::OperationMode::OPERATION_MODE_AUTONOMOUS;
    case OM::LOCAL:
      return vehicle_frame::OperationMode::OPERATION_MODE_LOCAL;
    case OM::REMOTE:
      return vehicle_frame::OperationMode::OPERATION_MODE_REMOTE;
    default:
      return vehicle_frame::OperationMode::OPERATION_MODE_UNKNOWN;
  }
}

vehicle_frame::DiagLevel TripBridge::toDiagLevel(uint8_t v) {
  switch (v) {
    case 0:
      return vehicle_frame::DIAG_OK;
    case 1:
      return vehicle_frame::DIAG_WARN;
    case 2:
      return vehicle_frame::DIAG_ERROR;
    default:
      return vehicle_frame::DIAG_STALE;
  }
}

vehicle_frame::MrmBehavior TripBridge::toMrmBehavior(uint8_t v) {
  using MRM = autoware_adapi_v1_msgs::msg::MrmState;
  switch (v) {
    case MRM::EMERGENCY_STOP:
      return vehicle_frame::MRM_EMERGENCY_STOP;
    case MRM::PULL_OVER:
      return vehicle_frame::MRM_PULL_OVER;
    default:
      return vehicle_frame::MRM_NONE;
  }
}

vehicle_frame::TripState TripBridge::toTripState(TripState v) {
  switch (v) {
    case TripState::IDLE:
      return vehicle_frame::TRIP_IDLE;
    case TripState::PUBLISHING_INITIAL_POSE:
      return vehicle_frame::TRIP_PUBLISHING_INITIAL_POSE;
    case TripState::WAITING_LOCALISATION:
      return vehicle_frame::TRIP_WAITING_LOCALISATION;
    case TripState::PUBLISHING_GOAL:
      return vehicle_frame::TRIP_PUBLISHING_GOAL;
    case TripState::WAITING_ROUTE:
      return vehicle_frame::TRIP_WAITING_ROUTE;
    case TripState::ENGAGING:
      return vehicle_frame::TRIP_ENGAGING;
    case TripState::RUNNING:
      return vehicle_frame::TRIP_RUNNING;
    case TripState::COMPLETED:
      return vehicle_frame::TRIP_COMPLETED;
    case TripState::FAILED:
      return vehicle_frame::TRIP_FAILED;
    default:
      return vehicle_frame::TRIP_IDLE;
  }
}
