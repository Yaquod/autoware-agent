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

#include "PlanningBridge.h"

#include <boost/asio/bind_executor.hpp>

class PlanningBridge::PlanningServiceImpl final : public vehicle_frame::PlanningService::Service {
 public:
  explicit PlanningServiceImpl(PlanningBridge& parent) : parent_(parent) {};

  grpc::Status Subscribe(grpc::ServerContext* context,
                         const vehicle_frame::SubscribeRequest* request,
                         grpc::ServerWriter<vehicle_frame::PlanningFrame>* writer) override {
    auto session = std::make_shared<ClientSession>();
    session->writer = writer;

    {
      std::lock_guard<std::mutex> lk(parent_.clients_mutex_);
      parent_.grpc_clients_.push_back(session);
    }

    while (!context->IsCancelled() && session->alive) {
      vehicle_frame::PlanningFrame frame;
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
                              vehicle_frame::PlanningFrame* response) override {
    std::promise<vehicle_frame::PlanningFrame> promise;
    auto future = promise.get_future();

    boost::asio::post(parent_.strand_, [&parent = parent_, p = std::move(promise)]() mutable {
      p.set_value(parent.buildFrame());
    });

    *response = future.get();
    return grpc::Status::OK;
  }

 private:
  PlanningBridge& parent_;
};

PlanningBridge::PlanningBridge(rclcpp::Node::SharedPtr node, grpc::ServerBuilder& builder)
  : frame_seq_(0)
  , io_context_()
  , strand_(io_context_)
  , work_guard_(boost::asio::make_work_guard(io_context_))
  , publisher_timer_(io_context_)
  , node_(std::move(node)) {
  RCLCPP_INFO(node_->get_logger(), "[PlanningBridge] Booting..");
  io_thread_ = std::thread([this]() { io_context_.run(); });

  auto sensor_qos = rclcpp::SensorDataQoS();
  auto reliable_qos = rclcpp::QoS(1).reliable().durability_volatile();
  auto transient_local_qos = rclcpp::QoS(1).transient_local();
  auto perception_qos = rclcpp::QoS(1).best_effort().durability_volatile();

  trajectory_point_sub_ = node_->create_subscription<autoware_planning_msgs::msg::Trajectory>(
    "/planning/scenario_planning/trajectory", sensor_qos,
    [this](const autoware_planning_msgs::msg::Trajectory::SharedPtr msg) {
      onTrajectoryPoint(msg);
    });
  full_route_sub_ = node_->create_subscription<autoware_planning_msgs::msg::LaneletRoute>(
    "/planning/mission_planning/route", transient_local_qos,
    [this](const autoware_planning_msgs::msg::LaneletRoute::SharedPtr msg) { onFullRoute(msg); });

  trajectory_lane_sub_ = node_->create_subscription<autoware_planning_msgs::msg::Trajectory>(
    "/planning/scenario_planning/lane_driving/trajectory", sensor_qos,
    [this](const autoware_planning_msgs::msg::Trajectory::SharedPtr msg) {
      onTrajectoryLane(msg);
    });
  velocity_factor_sub_ =
    node_->create_subscription<autoware_adapi_v1_msgs::msg::VelocityFactorArray>(
      "/api/planning/velocity_factors", sensor_qos,
      [this](const autoware_adapi_v1_msgs::msg::VelocityFactorArray::SharedPtr msg) {
        onVelocityFactor(msg);
      });

  steering_factor_sub_ =
    node_->create_subscription<autoware_adapi_v1_msgs::msg::SteeringFactorArray>(
      "/api/planning/steering_factors", sensor_qos,
      [this](const autoware_adapi_v1_msgs::msg::SteeringFactorArray::SharedPtr msg) {
        onSteeringFactor(msg);
      });

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

  route_state_sub_ = node_->create_subscription<autoware_adapi_v1_msgs::msg::RouteState>(
    "/api/routing/state", transient_local_qos,
    [this](const autoware_adapi_v1_msgs::msg::RouteState::SharedPtr msg) { onRouteState(msg); });

  scenario_state_sub_ = node_->create_subscription<autoware_internal_planning_msgs::msg::Scenario>(
    "/planning/scenario_planning/scenario", reliable_qos,
    [this](const autoware_internal_planning_msgs::msg::Scenario::SharedPtr msg) {
      onScenarioState(msg);
    });

  grpc_service_ = std::make_unique<PlanningServiceImpl>(*this);
  builder.RegisterService(grpc_service_.get());
  boost::asio::post(strand_, [this]() { scheduleNextTick(); });
  RCLCPP_INFO(node_->get_logger(), "[PlanningBrdige] Ready...");
}

PlanningBridge::~PlanningBridge() {
  shutdown();
}

void PlanningBridge::shutdown() {
  publisher_timer_.cancel();
  work_guard_.reset();

  if (io_thread_.joinable()) {
    io_thread_.join();
  }
}

void PlanningBridge::scheduleNextTick() {
  publisher_timer_.expires_after(std::chrono::microseconds(16667));  // 60hz
  publisher_timer_.async_wait(
    boost::asio::bind_executor(strand_, [this](const boost::system::error_code& ec) {
      if (ec == boost::asio::error::operation_aborted)
        return;
      ontick();
    }));
}

void PlanningBridge::ontick() {
  auto t0 = std::chrono::steady_clock::now();
  broadcastFrame(buildFrame());
  auto dt = std::chrono::steady_clock::now() - t0;
  if (dt > std::chrono::milliseconds(5)) {
    RCLCPP_WARN(node_->get_logger(), "[PlanningBridge] ontick delayed %ldms",
                std::chrono::duration_cast<std::chrono::milliseconds>(dt).count());
  }
  scheduleNextTick();
}

vehicle_frame::PlanningFrame PlanningBridge::buildFrame() {
  vehicle_frame::PlanningFrame frame;
  frame.set_stamp_ns(node_->now().nanoseconds());
  frame.set_seq(frame_seq_++);

  *frame.mutable_trajectory_points() = {state_.trajectory_points.begin(),
                                        state_.trajectory_points.end()};

  *frame.mutable_lane_trajectory() = {state_.lane_trajectory.begin(), state_.lane_trajectory.end()};

  *frame.mutable_full_route() = state_.full_route;

  *frame.mutable_velocity_factors() = {state_.velocity_factors.begin(),
                                       state_.velocity_factors.end()};

  *frame.mutable_steering_factors() = {state_.steering_factors.begin(),
                                       state_.steering_factors.end()};

  frame.set_target_speed_mps(state_.target_speed_mps);
  frame.set_max_speed_mps(state_.max_speed_mps);

  frame.mutable_eta()->CopyFrom(state_.eta);

  *frame.mutable_routing_state() = state_.routing_state;
  *frame.mutable_scenario_state() = state_.scenario_state;

  return frame;
}

void PlanningBridge::broadcastFrame(const vehicle_frame::PlanningFrame& frame) {
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

void PlanningBridge::onTrajectoryPoint(
  const autoware_planning_msgs::msg::Trajectory::SharedPtr msg) {
  boost::asio::post(strand_,
                    [this, msg = std::move(msg)]() mutable { onTrajectoryPointImpl(msg); });
}

void PlanningBridge::onFullRoute(const autoware_planning_msgs::msg::LaneletRoute::SharedPtr msg) {
  boost::asio::post(strand_, [this, msg = std::move(msg)]() mutable { onFullRouteImpl(msg); });
}

void PlanningBridge::onTrajectoryLane(
  const autoware_planning_msgs::msg::Trajectory::SharedPtr msg) {
  boost::asio::post(strand_, [this, msg = std::move(msg)]() mutable { onTrajectoryLaneImpl(msg); });
}

void PlanningBridge::onVelocityFactor(
  const autoware_adapi_v1_msgs::msg::VelocityFactorArray::SharedPtr msg) {
  boost::asio::post(strand_, [this, msg = std::move(msg)]() mutable { onVelocityFactorImpl(msg); });
}

void PlanningBridge::onSteeringFactor(
  const autoware_adapi_v1_msgs::msg::SteeringFactorArray::SharedPtr msg) {
  boost::asio::post(strand_, [this, msg = std::move(msg)]() mutable { onSteeringFactorImpl(msg); });
}

void PlanningBridge::onTargetVelocity(
  const autoware_internal_debug_msgs::msg::Float32Stamped::SharedPtr msg) {
  boost::asio::post(strand_, [this, msg = std::move(msg)]() mutable { onTargetVelocityImpl(msg); });
}

void PlanningBridge::onVelocityLimit(
  const autoware_internal_planning_msgs::msg::VelocityLimit::SharedPtr msg) {
  boost::asio::post(strand_, [this, msg = std::move(msg)]() mutable { onVelocityLimitImpl(msg); });
}

void PlanningBridge::onEta(
  const autoware_internal_msgs::msg::MissionRemainingDistanceTime::SharedPtr msg) {
  boost::asio::post(strand_, [this, msg = std::move(msg)]() mutable { onEtaImpl(msg); });
}

void PlanningBridge::onRouteState(const autoware_adapi_v1_msgs::msg::RouteState::SharedPtr msg) {
  boost::asio::post(strand_, [this, msg = std::move(msg)]() mutable { onRouteStateImpl(msg); });
}

void PlanningBridge::onScenarioState(
  const autoware_internal_planning_msgs::msg::Scenario::SharedPtr msg) {
  boost::asio::post(strand_, [this, msg = std::move(msg)]() mutable { onScenarioStateImpl(msg); });
}

// TODO: complete all on-##nameImpl functions

void PlanningBridge::onTrajectoryPointImpl(
  const autoware_planning_msgs::msg::Trajectory::SharedPtr msg) {
  state_.trajectory_points.clear();
  for (const auto& point : msg->points) {
    vehicle_frame::TrajectoryPoint tp;
    tp.set_x(point.pose.position.x);
    tp.set_y(point.pose.position.y);
    tp.set_z(point.pose.position.z);
    tp.set_longitudinal_velocity_mps(point.longitudinal_velocity_mps);
    state_.trajectory_points.push_back(std::move(tp));
  }
}

void PlanningBridge::onFullRouteImpl(
  const autoware_planning_msgs::msg::LaneletRoute::SharedPtr msg) {
  vehicle_frame::FullRoute fr;
  fr.mutable_start_pose()->set_x(msg->start_pose.position.x);
  fr.mutable_start_pose()->set_y(msg->start_pose.position.y);
  fr.mutable_start_pose()->set_z(msg->start_pose.position.z);
  fr.mutable_goal_pose()->set_x(msg->goal_pose.position.x);
  fr.mutable_goal_pose()->set_y(msg->goal_pose.position.y);
  fr.mutable_goal_pose()->set_z(msg->goal_pose.position.z);
  for (const auto& seg : msg->segments) {
    auto* add_seg = fr.add_segments();
    add_seg->mutable_preferred()->set_id(seg.preferred_primitive.id);
    add_seg->mutable_preferred()->set_primitive_type(seg.preferred_primitive.primitive_type);

    for (const auto& prim : seg.primitives) {
      auto* add_prim = add_seg->add_primitives();
      add_prim->set_id(prim.id);
      add_prim->set_primitive_type(prim.primitive_type);
    }
  }
  state_.full_route = std::move(fr);
}

void PlanningBridge::onTrajectoryLaneImpl(
  const autoware_planning_msgs::msg::Trajectory::SharedPtr msg) {
  state_.lane_trajectory.clear();
  for (const auto& point : msg->points) {
    vehicle_frame::TrajectoryPoint tp;
    tp.set_x(point.pose.position.x);
    tp.set_y(point.pose.position.y);
    tp.set_z(point.pose.position.z);
    tp.set_longitudinal_velocity_mps(point.longitudinal_velocity_mps);
    state_.lane_trajectory.push_back(std::move(tp));
  }
}

void PlanningBridge::onVelocityFactorImpl(
  const autoware_adapi_v1_msgs::msg::VelocityFactorArray::SharedPtr msg) {
  state_.velocity_factors.clear();
  for (const auto& factor : msg->factors) {
    vehicle_frame::VelocityFactor vf;
    vf.set_reason(factor.behavior);
    vf.mutable_pose()->set_x(factor.pose.position.x);
    vf.mutable_pose()->set_y(factor.pose.position.y);
    vf.mutable_pose()->set_z(factor.pose.position.z);
    vf.set_status(toVelocityFactorStatus(factor.status));
    state_.velocity_factors.push_back(std::move(vf));
  }
}

void PlanningBridge::onSteeringFactorImpl(
  const autoware_adapi_v1_msgs::msg::SteeringFactorArray::SharedPtr msg) {
  state_.steering_factors.clear();
  for (auto& factor : msg->factors) {
    vehicle_frame::SteeringFactor sf;
    sf.set_reason(factor.behavior);
    sf.mutable_pose()->set_x(factor.pose[0].position.x);
    sf.mutable_pose()->set_y(factor.pose[0].position.y);
    sf.mutable_pose()->set_z(factor.pose[0].position.z);
    sf.set_status(toSteeringStatus(factor.status));
    sf.set_direction(toSteeringDirection(factor.direction));
    state_.steering_factors.push_back(std::move(sf));
  }
}

void PlanningBridge::onTargetVelocityImpl(

  const autoware_internal_debug_msgs::msg::Float32Stamped::SharedPtr msg) {
  state_.target_speed_mps = msg->data;
}

void PlanningBridge::onVelocityLimitImpl(
  const autoware_internal_planning_msgs::msg::VelocityLimit::SharedPtr msg) {
  state_.max_speed_mps = msg->max_velocity;
}

void PlanningBridge::onEtaImpl(
  const autoware_internal_msgs::msg::MissionRemainingDistanceTime::SharedPtr msg) {
  state_.eta.set_remaining_distance_m(msg->remaining_distance);
  state_.eta.set_remaining_time_s(msg->remaining_time);
}

void PlanningBridge::onRouteStateImpl(
  const autoware_adapi_v1_msgs::msg::RouteState::SharedPtr msg) {
  state_.routing_state.set_state(toRouteState(msg->state));
}

void PlanningBridge::onScenarioStateImpl(
  const autoware_internal_planning_msgs::msg::Scenario::SharedPtr msg) {
  state_.scenario_state.set_current_scenario(msg->current_scenario);
  state_.scenario_state.clear_active_scenarios();
  for (const auto& scenario : msg->activating_scenarios) {
    state_.scenario_state.add_active_scenarios(scenario);
  }
}

vehicle_frame::VelocityFactorStatus PlanningBridge::toVelocityFactorStatus(uint8_t v) {
  using VF = autoware_adapi_v1_msgs::msg::VelocityFactor;
  switch (v) {
    case VF::APPROACHING:
      return vehicle_frame::VELOCITY_APPROACHING;
    case VF::STOPPED:
      return vehicle_frame::VELOCITY_STOPPED;
    default:
      return vehicle_frame::VELOCITY_UNKNOWN;
  }
}

vehicle_frame::SteeringDirection PlanningBridge::toSteeringDirection(uint8_t v) {
  using SD = autoware_adapi_v1_msgs::msg::SteeringFactor;
  switch (v) {
    case SD::LEFT:
      return vehicle_frame::STEERING_LEFT;
    case SD::RIGHT:
      return vehicle_frame::STEERING_RIGHT;
    case SD::STRAIGHT:
      return vehicle_frame::STEERING_STRAIGHT;
    default:
      return vehicle_frame::STEERING_UNKNOWN;
  }
}
vehicle_frame::SteeringStatus PlanningBridge::toSteeringStatus(uint8_t v) {
  using SS = autoware_adapi_v1_msgs::msg::SteeringFactor;
  switch (v) {
    case SS::APPROACHING:
      return vehicle_frame::STEERING_STATUS_APPROACHING;
    case SS::TURNING:
      return vehicle_frame::STEERING_STATUS_TURNING;
    default:
      return vehicle_frame::STEERING_STATUS_UNKNOWN;
  }
}

vehicle_frame::RouteState PlanningBridge::toRouteState(uint8_t v) {
  using RS = autoware_adapi_v1_msgs::msg::RouteState;
  switch (v) {
    case RS::UNSET:
      return vehicle_frame::ROUTE_UNSET;
    case RS::SET:
      return vehicle_frame::ROUTE_SET;
    case RS::ARRIVED:
      return vehicle_frame::ROUTE_ARRIVED;
    case RS::CHANGING:
      return vehicle_frame::ROUTE_CHANGING;
    default:
      return vehicle_frame::ROUTE_UNKNOWN;
  }
}
