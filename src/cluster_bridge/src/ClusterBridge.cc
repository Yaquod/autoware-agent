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
  explicit ClusterServiceImpl(ClusterBridge& parent) : parent_(parent){};

  grpc::Status Subscribe(grpc::ServerContext* context,
                         const vehicle_frame::SubscribeRequest* request,
                         grpc::ServerWriter<vehicle_frame::VehicleFrame>* writer) override {
    auto session = std::make_shared<ClientSession>();
    session->writer = writer;

    {
      std::lock_guard<std::mutex> lk(parent_.clients_mutex_);
      parent_.grpc_clients_.push_back(session);
    }

    while (!context->IsCancelled() && session->alive) {
      vehicle_frame::VehicleFrame frame;
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
  auto perception_qos = rclcpp::QoS(1).best_effort().durability_volatile();

  velocity_sub_ = node_->create_subscription<autoware_vehicle_msgs::msg::VelocityReport>(
    "/vehicle/status/velocity_status", sensor_qos,
    [this](const autoware_vehicle_msgs::msg::VelocityReport::SharedPtr msg) { onVelocity(msg); });

  // TODO: complete other subscribers
  gear_sub_ = node_->create_subscription<autoware_vehicle_msgs::msg::GearReport>(
    "/vehicle/status/gear_status", sensor_qos,
    [this](const autoware_vehicle_msgs::msg::GearReport::SharedPtr msg) { onGear(msg); });

  steering_sub_ = node_->create_subscription<autoware_vehicle_msgs::msg::SteeringReport>(
    "/vehicle/status/steering_status", sensor_qos,
    [this](const autoware_vehicle_msgs::msg::SteeringReport::SharedPtr msg) { onSteering(msg); });

  hazard_lights_sub_ = node_->create_subscription<autoware_vehicle_msgs::msg::HazardLightsReport>(
    "/vehicle/status/hazard_lights_status", sensor_qos,
    [this](const autoware_vehicle_msgs::msg::HazardLightsReport::SharedPtr msg) {
      onHazardLights(msg);
    });

  turn_indicators_sub_ =
    node_->create_subscription<autoware_vehicle_msgs::msg::TurnIndicatorsReport>(
      "/vehicle/status/turn_indicators_status", sensor_qos,
      [this](const autoware_vehicle_msgs::msg::TurnIndicatorsReport::SharedPtr msg) {
        onTurnIndicators(msg);
      });

  control_mode_sub_ = node_->create_subscription<autoware_vehicle_msgs::msg::ControlModeReport>(
    "/vehicle/status/control_mode", sensor_qos,
    [this](const autoware_vehicle_msgs::msg::ControlModeReport::SharedPtr msg) {
      onControlMode(msg);
    });

  battery_status_sub_ = node_->create_subscription<tier4_vehicle_msgs::msg::BatteryStatus>(
    "/vehicle/status/battery_charge", sensor_qos,
    [this](const tier4_vehicle_msgs::msg::BatteryStatus::SharedPtr msg) { onBatteryStatus(msg); });

  Kinematics_status_sub_ =
    node_->create_subscription<autoware_adapi_v1_msgs::msg::VehicleKinematics>(
      "/api/vehicle/kinematics", sensor_qos,
      [this](const autoware_adapi_v1_msgs::msg::VehicleKinematics::SharedPtr msg) {
        onKinematicsStatus(msg);
      });

  motion_state_sub_ = node_->create_subscription<autoware_adapi_v1_msgs::msg::MotionState>(
    "/api/motion/state", sensor_qos,
    [this](const autoware_adapi_v1_msgs::msg::MotionState::SharedPtr msg) { onMotionState(msg); });

  target_velocity_sub_ =
    node_->create_subscription<autoware_internal_debug_msgs::msg::Float32Stamped>(
      "/planning/scenario_planning/velocity_smoother/closest_velocity", sensor_qos,
      [this](const autoware_internal_debug_msgs::msg::Float32Stamped::SharedPtr msg) {
        onTargetVelocity(msg);
      });

  velocity_limit_sub_ =
    node_->create_subscription<autoware_internal_planning_msgs::msg::VelocityLimit>(
      "/planning/scenario_planning/current_max_velocity", sensor_qos,
      [this](const autoware_internal_planning_msgs::msg::VelocityLimit::SharedPtr msg) {
        onVelocityLimit(msg);
      });

  eta_sub_ = node_->create_subscription<autoware_internal_msgs::msg::MissionRemainingDistanceTime>(
    "/planning/mission_remaining_distance_time", sensor_qos,
    [this](const autoware_internal_msgs::msg::MissionRemainingDistanceTime::SharedPtr msg) {
      onEta(msg);
    });

  traffic_light_group_array_sub_ =
    node_->create_subscription<autoware_perception_msgs::msg::TrafficLightGroupArray>(
      "/perception/traffic_light_recognition/traffic_signals", perception_qos,
      [this](const autoware_perception_msgs::msg::TrafficLightGroupArray::SharedPtr msg) {
        onTrafficSignals(msg);
      });

  predicted_object_sub_ =
    node_->create_subscription<autoware_perception_msgs::msg::PredictedObjects>(
      "/perception/object_recognition/objects", perception_qos,
      [this](const autoware_perception_msgs::msg::PredictedObjects::SharedPtr msg) {
        onObjects(msg);
      });

  mrm_state_sub_ = node_->create_subscription<autoware_adapi_v1_msgs::msg::MrmState>(
    "/api/fail_safe/mrm_state", sensor_qos,
    [this](const autoware_adapi_v1_msgs::msg::MrmState::SharedPtr msg) { onMrmState(msg); });

  grpc_service_ = std::make_unique<ClusterServiceImpl>(*this);
  boost::asio::post(strand_, [this]() { scheduleNextTick(); });
  RCLCPP_INFO(node_->get_logger(), "[ClusterBrdige] Ready...");
}

ClusterBridge::~ClusterBridge() {
  shutdown();
}

void ClusterBridge::prepareGrpcServer() {
  builder_.AddListeningPort(grpc_address_, grpc::InsecureServerCredentials());
  builder_.RegisterService(grpc_service_.get());
}

grpc::ServerBuilder& ClusterBridge::getBuilder() {
  return builder_;
}


void ClusterBridge::shutdown() {


  publisher_timer_.cancel();
  work_guard_.reset();

  if (io_thread_.joinable()) {
    io_thread_.join();
  }

  if (grpc_server_) {
    grpc_server_->Shutdown();
  }


}


void ClusterBridge::runGrpcServer() {
  grpc_server_ = builder_.BuildAndStart();
  RCLCPP_INFO(node_->get_logger(), "[ClusterBridge] gRPC server on %s", grpc_address_.c_str());
  grpc_server_->Wait();
}




void ClusterBridge::scheduleNextTick() {
  publisher_timer_.expires_after(std::chrono::microseconds(16667));  // 60hz
  publisher_timer_.async_wait(
    boost::asio::bind_executor(strand_, [this](const boost::system::error_code& ec) {
      if (ec == boost::asio::error::operation_aborted)
        return;
      ontick();
    }));
}

void ClusterBridge::ontick() {
  auto t0 = std::chrono::steady_clock::now();
  broadcastFrame(buildFrame());
  auto dt = std::chrono::steady_clock::now() - t0;
  if (dt > std::chrono::milliseconds(5)) {
    RCLCPP_WARN(node_->get_logger(), "[ClusterBridge] ontick delayed %ldms",
                std::chrono::duration_cast<std::chrono::milliseconds>(dt).count());
  }
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
  frame.set_steering_angle_deg(state_.steering_angle_deg);
  frame.set_hazard_on(state_.hazard_on);
  frame.set_turn_signal(state_.turn_signal);
  frame.set_control_mode(state_.control_mode);
  frame.set_battery_pct(state_.battery_pct);
  frame.mutable_kinematics()->set_accel_mps2(state_.accel_mps2);
  frame.mutable_kinematics()->set_yaw_rate(state_.yaw_rate);
  frame.set_motion_state(state_.motion_state);
  frame.set_target_speed_mps(state_.target_speed_mps);
  frame.set_speed_limit_mps(state_.speed_limit_mps);
  frame.mutable_eta()->set_remaining_distance_m(state_.remaining_distance_m);
  frame.mutable_eta()->set_remaining_time_s(state_.remaining_time_s);
  frame.mutable_adas()->set_traffic_light_red(state_.traffic_light_red);
  frame.mutable_adas()->set_traffic_light_green(state_.traffic_light_green);
  frame.mutable_adas()->set_traffic_light_yellow(state_.traffic_light_yellow);
  frame.mutable_adas()->set_pedestrian_count(state_.pedestrian_count);
  frame.mutable_adas()->set_vehicle_count(state_.vehicle_count);
  frame.clear_surround_objects();
  for (const auto& so : state_.surround_objects) {
    frame.add_surround_objects()->CopyFrom(so);
  }

  frame.mutable_mrm()->set_is_active(state_.is_active);
  frame.mutable_mrm()->set_description(state_.description);
  frame.mutable_mrm()->set_behavior(state_.mrm_behavior);

  return frame;
}

void ClusterBridge::broadcastFrame(const vehicle_frame::VehicleFrame& frame) {
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

void ClusterBridge::onVelocity(const autoware_vehicle_msgs::msg::VelocityReport::SharedPtr msg) {
  boost::asio::post(strand_, [this, msg = msg]() mutable { onVelocityImpl(msg); });
}

void ClusterBridge::onGear(const autoware_vehicle_msgs::msg::GearReport::SharedPtr msg) {
  boost::asio::post(strand_, [this, msg = msg]() mutable { onGearImpl(msg); });
}

void ClusterBridge::onSteering(autoware_vehicle_msgs::msg::SteeringReport::SharedPtr msg) {
  boost::asio::post(strand_, [this, msg = std::move(msg)]() mutable { onSteeringImpl(msg); });
}

void ClusterBridge::onHazardLights(
  const autoware_vehicle_msgs::msg::HazardLightsReport::SharedPtr msg) {
  boost::asio::post(strand_, [this, msg = std::move(msg)]() mutable { onHazardLightsImpl(msg); });
}

void ClusterBridge::onTurnIndicators(
  const autoware_vehicle_msgs::msg::TurnIndicatorsReport::SharedPtr msg) {
  boost::asio::post(strand_, [this, msg = std::move(msg)]() mutable { onTurnIndicatorsImpl(msg); });
}

void ClusterBridge::onControlMode(
  const autoware_vehicle_msgs::msg::ControlModeReport::SharedPtr msg) {
  boost::asio::post(strand_, [this, msg = std::move(msg)]() mutable { onControlModeImpl(msg); });
}

void ClusterBridge::onBatteryStatus(const tier4_vehicle_msgs::msg::BatteryStatus::SharedPtr msg) {
  boost::asio::post(strand_, [this, msg = std::move(msg)]() mutable { onBatteryStatusImpl(msg); });
}

void ClusterBridge::onKinematicsStatus(
  const autoware_adapi_v1_msgs::msg::VehicleKinematics::SharedPtr msg) {
  boost::asio::post(strand_,
                    [this, msg = std::move(msg)]() mutable { onKinematicsStatusImpl(msg); });
}

void ClusterBridge::onMotionState(const autoware_adapi_v1_msgs::msg::MotionState::SharedPtr msg) {
  boost::asio::post(strand_, [this, msg = std::move(msg)]() mutable { onMotionStateImpl(msg); });
}

void ClusterBridge::onTargetVelocity(
  const autoware_internal_debug_msgs::msg::Float32Stamped::SharedPtr msg) {
  boost::asio::post(strand_, [this, msg = std::move(msg)]() mutable { onTargetVelocityImpl(msg); });
}

void ClusterBridge::onVelocityLimit(
  const autoware_internal_planning_msgs::msg::VelocityLimit::SharedPtr msg) {
  boost::asio::post(strand_, [this, msg = std::move(msg)]() mutable { onVelocityLimitImpl(msg); });
}

void ClusterBridge::onEta(
  const autoware_internal_msgs::msg::MissionRemainingDistanceTime::SharedPtr msg) {
  boost::asio::post(strand_, [this, msg = std::move(msg)]() mutable { onEtaImpl(msg); });
}

void ClusterBridge::onTrafficSignals(
  const autoware_perception_msgs::msg::TrafficLightGroupArray::SharedPtr msg) {
  boost::asio::post(strand_, [this, msg = std::move(msg)]() mutable { onTrafficSignalsImpl(msg); });
}

void ClusterBridge::onObjects(
  const autoware_perception_msgs::msg::PredictedObjects::SharedPtr msg) {
  boost::asio::post(strand_, [this, msg = std::move(msg)]() mutable {
    onSurroundObjectsImpl(msg);
    onObjectsImpl(msg);
  });
}

// void ClusterBridge::onSurroundObjects(const
// autoware_perception_msgs::msg::PredictedObjects::SharedPtr msg) {
//   boost::asio::post(strand_, [this, msg = std::move(msg)]() mutable {onSurroundObjectsImpl(msg);
//   });
//
// }

void ClusterBridge::onMrmState(const autoware_adapi_v1_msgs::msg::MrmState::SharedPtr msg) {
  boost::asio::post(strand_, [this, msg = std::move(msg)]() mutable { onMrmStateImpl(msg); });
}
// TODO: complete all on-##nameImpl functions

void ClusterBridge::onVelocityImpl(
  const autoware_vehicle_msgs::msg::VelocityReport::SharedPtr msg) {
  state_.speed_mps = msg->longitudinal_velocity;
  state_.speed_kmh = msg->longitudinal_velocity * 3.6f;
}

static constexpr float DEG_PER_RAD = 180.0f / 3.14159265358979323846f;

void ClusterBridge::onGearImpl(const autoware_vehicle_msgs::msg::GearReport::SharedPtr msg) {
  state_.gear = toGear(msg->report);
}

void ClusterBridge::onSteeringImpl(autoware_vehicle_msgs::msg::SteeringReport::SharedPtr msg) {
  state_.steering_angle_deg = msg->steering_tire_angle * DEG_PER_RAD;
}

void ClusterBridge::onHazardLightsImpl(
  const autoware_vehicle_msgs::msg::HazardLightsReport::SharedPtr msg) {
  state_.hazard_on = toHazard(msg->report);
}

void ClusterBridge::onTurnIndicatorsImpl(
  const autoware_vehicle_msgs::msg::TurnIndicatorsReport::SharedPtr msg) {
  state_.turn_signal = toTurn(msg->report);
}

void ClusterBridge::onControlModeImpl(
  const autoware_vehicle_msgs::msg::ControlModeReport::SharedPtr msg) {
  state_.control_mode = toCtrlMode(msg->mode);
}

void ClusterBridge::onBatteryStatusImpl(
  const tier4_vehicle_msgs::msg::BatteryStatus::SharedPtr msg) {
  state_.battery_pct = msg->energy_level * 100.0f;
}

void ClusterBridge::onKinematicsStatusImpl(
  const autoware_adapi_v1_msgs::msg::VehicleKinematics::SharedPtr msg) {
  state_.accel_mps2 = msg->accel.accel.accel.linear.x;
  state_.yaw_rate = msg->twist.twist.twist.angular.z;
}

void ClusterBridge::onMotionStateImpl(
  const autoware_adapi_v1_msgs::msg::MotionState::SharedPtr msg) {
  state_.motion_state = toMotionState(msg->state);
}

void ClusterBridge::onTargetVelocityImpl(
  const autoware_internal_debug_msgs::msg::Float32Stamped::SharedPtr msg) {
  state_.target_speed_mps = msg->data;
}

void ClusterBridge::onVelocityLimitImpl(
  const autoware_internal_planning_msgs::msg::VelocityLimit::SharedPtr msg) {
  state_.speed_limit_mps = msg->max_velocity;
}

void ClusterBridge::onEtaImpl(
  const autoware_internal_msgs::msg::MissionRemainingDistanceTime::SharedPtr msg) {
  state_.remaining_distance_m = msg->remaining_distance;
  state_.remaining_time_s = msg->remaining_time;
}

void ClusterBridge::onTrafficSignalsImpl(
  const autoware_perception_msgs::msg::TrafficLightGroupArray::SharedPtr msg) {
  using TLE = autoware_perception_msgs::msg::TrafficLightElement;
  bool red = false, green = false, yellow = false;
  for (const auto& group : msg->traffic_light_groups) {
    for (const auto& ele : group.elements) {
      if (ele.color == TLE::RED)
        red = true;
      if (ele.color == TLE::GREEN)
        green = true;
      if (ele.color == TLE::AMBER)
        yellow = true;
    }
  }

  state_.traffic_light_red = red;
  state_.traffic_light_green = green;
  state_.traffic_light_yellow = yellow;
}

void ClusterBridge::onObjectsImpl(
  const autoware_perception_msgs::msg::PredictedObjects::SharedPtr msg) {
  using OC = autoware_perception_msgs::msg::ObjectClassification;
  uint32_t obstacle_count = 0;
  uint32_t pedestrian_count = 0;
  uint32_t vehicle_count = 0;

  for (const auto& object : msg->objects) {
    if (object.classification.empty())
      continue;
    const auto label = object.classification[0].label;

    if (label == OC::CAR || label == OC::TRUCK || label == OC::BUS || label == OC::TRAILER ||
        label == OC::MOTORCYCLE)
      vehicle_count++;

    else if (label == OC::PEDESTRIAN)
      pedestrian_count++;

    obstacle_count++;
  }

  state_.obstacle_count = obstacle_count;
  state_.pedestrian_count = pedestrian_count;
  state_.vehicle_count = vehicle_count;
}

void ClusterBridge::onSurroundObjectsImpl(
  const autoware_perception_msgs::msg::PredictedObjects::SharedPtr msg) {
  using OC = autoware_perception_msgs::msg::ObjectClassification;
  std::vector<vehicle_frame::SurroundObject> surrounds;
  for (const auto& object : msg->objects) {
    if (object.classification.empty())
      continue;
    if (object.existence_probability < 0.5f)
      continue;

    const auto label = object.classification[0].label;

    vehicle_frame::SurroundObject so;
    so.set_x(static_cast<float>(object.kinematics.initial_pose_with_covariance.pose.position.x));
    so.set_y(static_cast<float>(object.kinematics.initial_pose_with_covariance.pose.position.y));
    so.set_vx(static_cast<float>(object.kinematics.initial_twist_with_covariance.twist.linear.x));
    so.set_vy(static_cast<float>(object.kinematics.initial_twist_with_covariance.twist.linear.y));

    if (label == OC::PEDESTRIAN) {
      so.set_type("PEDESTRIAN");
    } else if (label == OC::CAR || label == OC::TRUCK || label == OC::BUS || label == OC::TRAILER ||
               label == OC::MOTORCYCLE) {
      so.set_type("VEHICLE");
    } else {
      so.set_type("UNKNOWN");
    }
    surrounds.push_back(std::move(so));
  }
  state_.surround_objects = std::move(surrounds);
}

void ClusterBridge::onMrmStateImpl(const autoware_adapi_v1_msgs::msg::MrmState::SharedPtr msg) {
  using MRM = autoware_adapi_v1_msgs::msg::MrmState;
  state_.is_active = (msg->state != MRM::NORMAL && msg->state != MRM::UNKNOWN);
  state_.mrm_behavior = toMrmState(msg->behavior);

  switch (msg->state) {
    case MRM::NORMAL:
      state_.description = "Normal";
      break;
    case MRM::MRM_OPERATING:
      state_.description = "MRM Operating";
      break;
    case MRM::MRM_SUCCEEDED:
      state_.description = "MRM Succeeded";
      break;
    case MRM::MRM_FAILED:
      state_.description = "MRM Failed";
      break;
    default:
      state_.description = "Unknown";
      break;
  }
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

bool ClusterBridge::toHazard(uint8_t v) {
  switch (v) {
    case 1:
      return false;
    case 2:
      return true;
    default:
      return false;
  }
}

vehicle_frame::MrmBehavior ClusterBridge::toMrmState(uint8_t v) {
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
