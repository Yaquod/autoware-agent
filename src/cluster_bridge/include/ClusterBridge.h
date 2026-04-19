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

#ifndef VEHICLEAUTOWAREAGENT_CLUSTERBRIDGE_H
#define VEHICLEAUTOWAREAGENT_CLUSTERBRIDGE_H

#include "FrameStates.h"
#include "vehicle_frame.pb.h"
#include "zenoh_publisher.h"

#include <autoware_adapi_v1_msgs/msg/motion_state.hpp>
#include <autoware_adapi_v1_msgs/msg/mrm_state.hpp>
#include <autoware_adapi_v1_msgs/msg/operation_mode_state.hpp>
#include <autoware_adapi_v1_msgs/msg/vehicle_kinematics.hpp>
#include <autoware_perception_msgs/msg/predicted_objects.hpp>
#include <autoware_perception_msgs/msg/traffic_light_group_array.hpp>
#include <autoware_planning_msgs/msg/trajectory.hpp>
#include <nav_msgs/msg/odometry.hpp>
#include <rclcpp/rclcpp.hpp>

#include <boost/asio/executor_work_guard.hpp>
#include <boost/asio/io_context.hpp>
#include <boost/asio/steady_timer.hpp>
#include <boost/asio/strand.hpp>

#include <autoware_internal_debug_msgs/msg/float32_stamped.hpp>
#include <autoware_internal_msgs/msg/mission_remaining_distance_time.hpp>
#include <autoware_internal_planning_msgs/msg/velocity_limit.hpp>
#include <autoware_vehicle_msgs/msg/control_mode_report.hpp>
#include <autoware_vehicle_msgs/msg/gear_report.hpp>
#include <autoware_vehicle_msgs/msg/hazard_lights_report.hpp>
#include <autoware_vehicle_msgs/msg/steering_report.hpp>
#include <autoware_vehicle_msgs/msg/turn_indicators_report.hpp>
#include <autoware_vehicle_msgs/msg/velocity_report.hpp>
#include <tier4_vehicle_msgs/msg/battery_status.hpp>

class ClusterBridge : public AutowareAgent::ZenohPublisher {
 public:
  explicit ClusterBridge(rclcpp::Node::SharedPtr node,
                         const std::shared_ptr<zenoh::Session>& zsession);

  ~ClusterBridge();
  void shutdown();

 private:
  std::atomic<bool> shutdown_called_{false};
  boost::asio::io_context io_context_;
  boost::asio::io_context::strand strand_;
  boost::asio::executor_work_guard<boost::asio::io_context::executor_type> work_guard_;
  std::thread io_thread_;
  // asio timer 60Hz
  boost::asio::steady_timer publisher_timer_;
  FrameState state_;
  uint64_t frame_seq_{0};

  void scheduleNextTick();
  void onTick();

  // called on strand for grpc clients
  vehicle_frame::VehicleFrame buildFrame();

  // called on strand for grpc clients
  void broadcastFrame(const vehicle_frame::VehicleFrame& frame);

  // ros
  rclcpp::Node::SharedPtr node_;
  rclcpp::Subscription<autoware_vehicle_msgs::msg::VelocityReport>::SharedPtr velocity_sub_;
  rclcpp::Subscription<autoware_vehicle_msgs::msg::GearReport>::SharedPtr gear_sub_;
  rclcpp::Subscription<autoware_adapi_v1_msgs::msg::MotionState>::SharedPtr motion_state_sub_;
  rclcpp::Subscription<autoware_adapi_v1_msgs::msg::OperationModeState>::SharedPtr
    operation_mode_state_;
  rclcpp::Subscription<autoware_vehicle_msgs::msg::SteeringReport>::SharedPtr steering_sub_;
  rclcpp::Subscription<autoware_vehicle_msgs::msg::TurnIndicatorsReport>::SharedPtr
    turn_indicators_sub_;
  rclcpp::Subscription<autoware_vehicle_msgs::msg::HazardLightsReport>::SharedPtr
    hazard_lights_sub_;
  rclcpp::Subscription<autoware_vehicle_msgs::msg::ControlModeReport>::SharedPtr control_mode_sub_;
  rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr odometry_sub_;
  rclcpp::Subscription<autoware_planning_msgs::msg::Trajectory>::SharedPtr trajectory_sub_;
  rclcpp::Subscription<tier4_vehicle_msgs::msg::BatteryStatus>::SharedPtr battery_status_sub_;
  rclcpp::Subscription<autoware_perception_msgs::msg::PredictedObjects>::SharedPtr
    predicted_object_sub_;
  rclcpp::Subscription<autoware_perception_msgs::msg::TrafficLightGroupArray>::SharedPtr
    traffic_light_group_array_sub_;
  rclcpp::Subscription<autoware_adapi_v1_msgs::msg::MrmState>::SharedPtr mrm_state_sub_;
  rclcpp::Subscription<autoware_internal_debug_msgs::msg::Float32Stamped>::SharedPtr
    target_velocity_sub_;
  rclcpp::Subscription<autoware_internal_planning_msgs::msg::VelocityLimit>::SharedPtr
    velocity_limit_sub_;
  rclcpp::Subscription<autoware_adapi_v1_msgs::msg::VehicleKinematics>::SharedPtr
    Kinematics_status_sub_;
  rclcpp::Subscription<autoware_internal_msgs::msg::MissionRemainingDistanceTime>::SharedPtr
    eta_sub_;

  // ROS callbacks that would be posted on strand
  void onVelocity(autoware_vehicle_msgs::msg::VelocityReport::SharedPtr msg);
  void onGear(autoware_vehicle_msgs::msg::GearReport::SharedPtr msg);
  void onMotionState(autoware_adapi_v1_msgs::msg::MotionState::SharedPtr msg);
  void onOperationModeState(autoware_adapi_v1_msgs::msg::OperationModeState::SharedPtr msg);
  void onSteering(autoware_vehicle_msgs::msg::SteeringReport::SharedPtr msg);
  void onTurnIndicators(autoware_vehicle_msgs::msg::TurnIndicatorsReport::SharedPtr msg);
  void onHazardLights(autoware_vehicle_msgs::msg::HazardLightsReport::SharedPtr msg);
  void onControlMode(autoware_vehicle_msgs::msg::ControlModeReport::SharedPtr msg);
  void onBatteryStatus(tier4_vehicle_msgs::msg::BatteryStatus::SharedPtr msg);
  void onKinematicsStatus(autoware_adapi_v1_msgs::msg::VehicleKinematics::SharedPtr msg);
  void onTargetVelocity(autoware_internal_debug_msgs::msg::Float32Stamped::SharedPtr msg);
  void onVelocityLimit(autoware_internal_planning_msgs::msg::VelocityLimit::SharedPtr msg);
  void onEta(autoware_internal_msgs::msg::MissionRemainingDistanceTime::SharedPtr msg);
  void onTrafficSignals(autoware_perception_msgs::msg::TrafficLightGroupArray::SharedPtr msg);
  void onObjects(autoware_perception_msgs::msg::PredictedObjects::SharedPtr msg);
  void onMrmState(autoware_adapi_v1_msgs::msg::MrmState::SharedPtr msg);

  // strand implementation
  void onVelocityImpl(autoware_vehicle_msgs::msg::VelocityReport::SharedPtr msg);
  void onGearImpl(autoware_vehicle_msgs::msg::GearReport::SharedPtr msg);
  void onSteeringImpl(autoware_vehicle_msgs::msg::SteeringReport::SharedPtr msg);
  void onHazardLightsImpl(autoware_vehicle_msgs::msg::HazardLightsReport::SharedPtr msg);
  void onTurnIndicatorsImpl(autoware_vehicle_msgs::msg::TurnIndicatorsReport::SharedPtr msg);
  void onControlModeImpl(autoware_vehicle_msgs::msg::ControlModeReport::SharedPtr msg);
  void onBatteryStatusImpl(tier4_vehicle_msgs::msg::BatteryStatus::SharedPtr msg);
  void onKinematicsStatusImpl(autoware_adapi_v1_msgs::msg::VehicleKinematics::SharedPtr msg);
  void onMotionStateImpl(autoware_adapi_v1_msgs::msg::MotionState::SharedPtr msg);
  void onTargetVelocityImpl(autoware_internal_debug_msgs::msg::Float32Stamped::SharedPtr msg);
  void onVelocityLimitImpl(autoware_internal_planning_msgs::msg::VelocityLimit::SharedPtr msg);
  void onEtaImpl(autoware_internal_msgs::msg::MissionRemainingDistanceTime::SharedPtr msg);
  void onTrafficSignalsImpl(autoware_perception_msgs::msg::TrafficLightGroupArray::SharedPtr msg);
  void onObjectsImpl(autoware_perception_msgs::msg::PredictedObjects::SharedPtr msg);
  void onSurroundObjectsImpl(autoware_perception_msgs::msg::PredictedObjects::SharedPtr msg);
  void onMrmStateImpl(autoware_adapi_v1_msgs::msg::MrmState::SharedPtr msg);

  static vehicle_frame::GearState toGear(uint8_t v);
  static vehicle_frame::TurnSignal toTurn(uint8_t v);
  static vehicle_frame::ControlMode toCtrlMode(uint8_t v);
  static vehicle_frame::MotionState toMotionState(uint8_t v);
  static vehicle_frame::MrmBehavior toMrmState(uint8_t v);
  static bool toHazard(uint8_t v);
};

#endif  // VEHICLEAUTOWAREAGENT_CLUSTERBRIDGE_H
