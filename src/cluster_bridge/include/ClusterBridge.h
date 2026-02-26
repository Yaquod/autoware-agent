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

#include <rclcpp/rclcpp.hpp>
#include <autoware_vehicle_msgs/msg/velocity_report.hpp>
#include <autoware_vehicle_msgs/msg/gear_report.hpp>
#include <autoware_vehicle_msgs/msg/steering_report.hpp>
#include <autoware_vehicle_msgs/msg/turn_indicators_report.hpp>
#include <autoware_vehicle_msgs/msg/hazard_lights_report.hpp>
#include <autoware_vehicle_msgs/msg/control_mode_report.hpp>
#include <autoware_adapi_v1_msgs/msg/operation_mode_state.hpp>
#include <autoware_adapi_v1_msgs/msg/motion_state.hpp>
#include <autoware_adapi_v1_msgs/msg/operation_mode_state.hpp>
#include <autoware_planning_msgs/msg/trajectory.hpp>

// #include <autoware_perception_msgs/msg/predicted_object.hpp>
// #include <autoware_auto_perception_msgs/msg/traffic_signal_array.hpp>
#include <nav_msgs/msg/odometry.hpp>

#include <boost/asio/executor_work_guard.hpp>
#include <boost/asio/io_context.hpp>
#include <boost/asio/steady_timer.hpp>
#include <boost/asio/strand.hpp>
// #include <tier4_vehicle_msgs/msg/battery_status.hpp>

#include "vehicle_frame.grpc.pb.h"
#include "vehicle_frame.pb.h"


class ClusterBridge {
public:
  explicit ClusterBridge(rclcpp::Node::SharedPtr node, const std::string& grpc_address = "0.0.0.0:50051");

  ~ClusterBridge();

  void runGrpcServer();

  void shutdown();

private:
  boost::asio::io_context io_context_;
  boost::asio::io_context::strand strand_;
  boost::asio::executor_work_guard<boost::asio::io_context::executor_type> work_guard_;
  std::thread io_thread_;

  // asio timer 60Hz
  boost::asio::steady_timer publisher_timer_;
  void scheduleNextTick();
  void ontick();

  vehicle_frame::VehicleFrame buildFramestates();

  // ros
  rclcpp::Node::SharedPtr node_;
  rclcpp::Subscription<autoware_vehicle_msgs::msg::VelocityReport>::SharedPtr velocity_sub_;
  rclcpp::Subscription<autoware_vehicle_msgs::msg::GearReport>::SharedPtr gear_sub_;

  // ongearGear.onVel
};

#endif  // VEHICLEAUTOWAREAGENT_CLUSTERBRIDGE_H
