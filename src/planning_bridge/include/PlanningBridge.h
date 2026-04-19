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

#ifndef VEHICLEAUTOWAREAGENT_PLANNINGBRIDGE_H
#define VEHICLEAUTOWAREAGENT_PLANNINGBRIDGE_H

#include "FrameStates.h"
#include "vehicle_frame.pb.h"
#include "zenoh_publisher.h"

#include <autoware_adapi_v1_msgs/msg/route_state.hpp>
#include <autoware_adapi_v1_msgs/msg/steering_factor_array.hpp>
#include <autoware_adapi_v1_msgs/msg/velocity_factor_array.hpp>
#include <autoware_planning_msgs/msg/lanelet_route.hpp>
#include <autoware_planning_msgs/msg/trajectory.hpp>
#include <rclcpp/rclcpp.hpp>

#include <boost/asio/executor_work_guard.hpp>
#include <boost/asio/io_context.hpp>
#include <boost/asio/steady_timer.hpp>
#include <boost/asio/strand.hpp>

#include <queue>

#include <autoware_internal_debug_msgs/msg/float32_stamped.hpp>
#include <autoware_internal_msgs/msg/mission_remaining_distance_time.hpp>
#include <autoware_internal_planning_msgs/msg/scenario.hpp>
#include <autoware_internal_planning_msgs/msg/velocity_limit.hpp>

class PlanningBridge : public AutowareAgent::ZenohPublisher {
 public:
  explicit PlanningBridge(rclcpp::Node::SharedPtr node,
                          const std::shared_ptr<zenoh::Session>& zsession);

  ~PlanningBridge();

  void shutdown();

 private:
  PlanningFrameState state_;
  uint64_t frame_seq_{0};

  boost::asio::io_context io_context_;
  boost::asio::io_context::strand strand_;
  boost::asio::executor_work_guard<boost::asio::io_context::executor_type> work_guard_;
  std::thread io_thread_;
  // asio timer 60Hz
  boost::asio::steady_timer publisher_timer_;

  void scheduleNextTick();
  void onTick();

  // called on strand for grpc clients
  vehicle_frame::PlanningFrame buildFrame();

  // called on strand for grpc clients
  void broadcastFrame(const vehicle_frame::PlanningFrame& frame);

  // ros
  rclcpp::Node::SharedPtr node_;
  rclcpp::Subscription<autoware_planning_msgs::msg::Trajectory>::SharedPtr trajectory_point_sub_;
  rclcpp::Subscription<autoware_planning_msgs::msg::LaneletRoute>::SharedPtr full_route_sub_;
  rclcpp::Subscription<autoware_planning_msgs::msg::Trajectory>::SharedPtr trajectory_lane_sub_;
  rclcpp::Subscription<autoware_adapi_v1_msgs::msg::VelocityFactorArray>::SharedPtr
    velocity_factor_sub_;
  rclcpp::Subscription<autoware_adapi_v1_msgs::msg::SteeringFactorArray>::SharedPtr
    steering_factor_sub_;
  rclcpp::Subscription<autoware_internal_debug_msgs::msg::Float32Stamped>::SharedPtr
    target_velocity_sub_;
  rclcpp::Subscription<autoware_internal_planning_msgs::msg::VelocityLimit>::SharedPtr
    velocity_limit_sub_;
  rclcpp::Subscription<autoware_internal_msgs::msg::MissionRemainingDistanceTime>::SharedPtr
    eta_sub_;
  rclcpp::Subscription<autoware_adapi_v1_msgs::msg::RouteState>::SharedPtr route_state_sub_;
  rclcpp::Subscription<autoware_internal_planning_msgs::msg::Scenario>::SharedPtr
    scenario_state_sub_;

  // ROS callbacks that would be posted on strand
  void onTrajectoryPoint(const autoware_planning_msgs::msg::Trajectory::SharedPtr msg);
  void onFullRoute(const autoware_planning_msgs::msg::LaneletRoute::SharedPtr msg);
  void onTrajectoryLane(const autoware_planning_msgs::msg::Trajectory::SharedPtr msg);
  void onVelocityFactor(const autoware_adapi_v1_msgs::msg::VelocityFactorArray::SharedPtr msg);
  void onSteeringFactor(const autoware_adapi_v1_msgs::msg::SteeringFactorArray::SharedPtr msg);
  void onTargetVelocity(const autoware_internal_debug_msgs::msg::Float32Stamped::SharedPtr msg);
  void onVelocityLimit(const autoware_internal_planning_msgs::msg::VelocityLimit::SharedPtr msg);
  void onEta(const autoware_internal_msgs::msg::MissionRemainingDistanceTime::SharedPtr msg);
  void onRouteState(const autoware_adapi_v1_msgs::msg::RouteState::SharedPtr msg);
  void onScenarioState(const autoware_internal_planning_msgs::msg::Scenario::SharedPtr msg);

  // TODO: complete functions

  // strand implementation
  void onTrajectoryPointImpl(const autoware_planning_msgs::msg::Trajectory::SharedPtr msg);
  void onFullRouteImpl(const autoware_planning_msgs::msg::LaneletRoute::SharedPtr msg);
  void onTrajectoryLaneImpl(const autoware_planning_msgs::msg::Trajectory::SharedPtr msg);
  void onVelocityFactorImpl(const autoware_adapi_v1_msgs::msg::VelocityFactorArray::SharedPtr msg);
  void onSteeringFactorImpl(const autoware_adapi_v1_msgs::msg::SteeringFactorArray::SharedPtr msg);

  void onTargetVelocityImpl(const autoware_internal_debug_msgs::msg::Float32Stamped::SharedPtr msg);
  void onVelocityLimitImpl(
    const autoware_internal_planning_msgs::msg::VelocityLimit::SharedPtr msg);
  void onEtaImpl(const autoware_internal_msgs::msg::MissionRemainingDistanceTime::SharedPtr msg);
  void onRouteStateImpl(const autoware_adapi_v1_msgs::msg::RouteState::SharedPtr msg);
  void onScenarioStateImpl(const autoware_internal_planning_msgs::msg::Scenario::SharedPtr msg);

  // TODO: complete functions
  static vehicle_frame::VelocityFactorStatus toVelocityFactorStatus(uint8_t v);
  static vehicle_frame::SteeringDirection toSteeringDirection(uint8_t v);
  static vehicle_frame::SteeringStatus toSteeringStatus(uint8_t v);
  static vehicle_frame::RouteState toRouteState(uint8_t v);
};

#endif  // VEHICLEAUTOWAREAGENT_CLUSTERBRIDGE_H
