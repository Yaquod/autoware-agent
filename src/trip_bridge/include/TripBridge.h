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

#ifndef VEHICLEAUTOWAREAGENT_TRIPBRIDGE_H
#define VEHICLEAUTOWAREAGENT_TRIPBRIDGE_H

#include "AutowareController.h"
#include "FrameStates.h"
#include "vehicle_frame.pb.h"
#include "zenoh.hxx"
#include "zenoh_publisher.h"

#include <autoware_adapi_v1_msgs/msg/heartbeat.hpp>
#include <autoware_adapi_v1_msgs/msg/localization_initialization_state.hpp>
#include <autoware_adapi_v1_msgs/msg/mrm_state.hpp>
#include <autoware_adapi_v1_msgs/msg/operation_mode_state.hpp>
#include <autoware_planning_msgs/msg/lanelet_route.hpp>
#include <rclcpp/rclcpp.hpp>

#include <boost/asio/executor_work_guard.hpp>
#include <boost/asio/io_context.hpp>
#include <boost/asio/steady_timer.hpp>
#include <boost/asio/strand.hpp>

#include <queue>

#include <grpcpp/grpcpp.h>
#include <tier4_system_msgs/msg/diag_graph_status.hpp>

class TripBridge : public AutowareAgent::ZenohPublisher {
 public:
  explicit TripBridge(std::shared_ptr<AutowareAgent::AutowareController> controller,
                      rclcpp::Node::SharedPtr node,
                      const std::shared_ptr<zenoh::Session>& zsession);

  ~TripBridge();

  void shutdown();

 private:
  TripFrameState state_;
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
  vehicle_frame::TripFrame buildFrame();

  // called on strand for grpc clients
  void broadcastFrame(const vehicle_frame::TripFrame& frame);

  // added to see if system alive
  rclcpp::Time last_heartbeat_time_;
  // added to be able to access tripcontroller::status
  std::shared_ptr<AutowareAgent::AutowareController> controller_;

  // ros
  rclcpp::Node::SharedPtr node_;

  rclcpp::Subscription<autoware_adapi_v1_msgs::msg::LocalizationInitializationState>::SharedPtr
    localization_state_sub_;
  rclcpp::Subscription<autoware_adapi_v1_msgs::msg::OperationModeState>::SharedPtr
    operation_mode_sub_;
  rclcpp::Subscription<autoware_adapi_v1_msgs::msg::MrmState>::SharedPtr mrm_state_sub_;
  rclcpp::Subscription<autoware_adapi_v1_msgs::msg::Heartbeat>::SharedPtr heartbeat_sub_;
  rclcpp::Subscription<tier4_system_msgs::msg::DiagGraphStatus>::SharedPtr diag_state_sub_;

  // ROS callbacks that would be posted on strand
  void onLocalizationState(
    const autoware_adapi_v1_msgs::msg::LocalizationInitializationState::SharedPtr msg);
  void onOperationMode(const autoware_adapi_v1_msgs::msg::OperationModeState::SharedPtr msg);
  void onMrmState(const autoware_adapi_v1_msgs::msg::MrmState::SharedPtr msg);
  void onHeartbeat(const autoware_adapi_v1_msgs::msg::Heartbeat::SharedPtr msg);
  void onDiagState(const tier4_system_msgs::msg::DiagGraphStatus::SharedPtr msg);

  // TODO: complete functions

  // strand implementation
  void onLocalizationStateImpl(
    const autoware_adapi_v1_msgs::msg::LocalizationInitializationState::SharedPtr msg);
  void onOperationModeImpl(const autoware_adapi_v1_msgs::msg::OperationModeState::SharedPtr msg);
  void onMrmStateImpl(const autoware_adapi_v1_msgs::msg::MrmState::SharedPtr msg);
  void onHeartbeatImpl(const autoware_adapi_v1_msgs::msg::Heartbeat::SharedPtr msg);
  void onDiagStateImpl(const tier4_system_msgs::msg::DiagGraphStatus::SharedPtr msg);

  // TODO: complete functions
  static vehicle_frame::LocalizationState toLocalizationState(uint16_t v);
  static vehicle_frame::OperationMode toOperationMode(uint8_t v);
  static vehicle_frame::DiagLevel toDiagLevel(uint8_t v);
  static vehicle_frame::TripState toTripState(TripState v);
  static vehicle_frame::MrmBehavior toMrmBehavior(uint8_t v);
};

#endif  // VEHICLEAUTOWAREAGENT_CLUSTERBRIDGE_H
