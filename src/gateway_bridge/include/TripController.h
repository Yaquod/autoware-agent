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

#ifndef VEHICLEAUTOWAREAGENT_TRIPCONTROLLER_H
#define VEHICLEAUTOWAREAGENT_TRIPCONTROLLER_H

#include "RouteConfig.h"
#include "TripStates.h"
#include "TripStatus.h"
#include "TripTimings.h"

#include <autoware_adapi_v1_msgs/msg/localization_initialization_state.hpp>
#include <autoware_adapi_v1_msgs/msg/operation_mode_state.hpp>
#include <autoware_adapi_v1_msgs/msg/route_state.hpp>
#include <autoware_adapi_v1_msgs/srv/change_operation_mode.hpp>
#include <autoware_planning_msgs/msg/lanelet_route.hpp>
#include <geometry_msgs/msg/pose_stamped.hpp>
#include <geometry_msgs/msg/pose_with_covariance_stamped.hpp>
#include <rclcpp/rclcpp.hpp>

#include <boost/asio.hpp>

#include <chrono>
#include <functional>
#include <memory>

#include <tier4_external_api_msgs/srv/engage.hpp>

namespace AutowareAgent {

using StateChangeCb = std::function<void(TripState prev, TripState next)>;

class TripController {
 public:
  TripController(rclcpp::Node::SharedPtr node, const RouteConfig& route_config,
                 std::shared_ptr<boost::asio::io_context::strand> strand,
                 TripTimings timings = TripTimings{});

  bool startTrip(GPSCoordinate goal_gps);
  void cancel();
  void tick();

  TripStatus status() const;
  void setStateChangeCallback(StateChangeCb cb);

 private:
  void doPublishInitialPose();
  void doPublishGoal();
  void doPollRoute();
  void doEngage();

  // FIXED: Changed parameter type to match subscription
  void onRouteReceived(const autoware_planning_msgs::msg::LaneletRoute& msg);

  void transitionTo(TripState next);
  long elapsed_ms(std::chrono::steady_clock::time_point since);

  rclcpp::Node::SharedPtr node_;
  const RouteConfig& route_config_;
  std::shared_ptr<boost::asio::io_context::strand> strand_;
  TripTimings timings_;

  rclcpp::Publisher<geometry_msgs::msg::PoseWithCovarianceStamped>::SharedPtr initial_pose_pub_;
  rclcpp::Publisher<geometry_msgs::msg::PoseStamped>::SharedPtr goal_pub_;
  rclcpp::Client<autoware_adapi_v1_msgs::srv::ChangeOperationMode>::SharedPtr mode_client_;
  rclcpp::Client<tier4_external_api_msgs::srv::Engage>::SharedPtr engage_client_;
  rclcpp::Subscription<autoware_planning_msgs::msg::LaneletRoute>::SharedPtr route_sub_;
  rclcpp::Subscription<autoware_adapi_v1_msgs::msg::LocalizationInitializationState>::SharedPtr
    loc_state_sub_;
  rclcpp::Subscription<autoware_adapi_v1_msgs::msg::RouteState>::SharedPtr route_state_sub_;
  rclcpp::Subscription<autoware_adapi_v1_msgs::msg::OperationModeState>::SharedPtr mode_state_sub_;

  // Thread-safe state storage
  autoware_adapi_v1_msgs::msg::LocalizationInitializationState::SharedPtr current_loc_state_;
  autoware_adapi_v1_msgs::msg::RouteState::SharedPtr current_route_state_;
  autoware_adapi_v1_msgs::msg::OperationModeState::SharedPtr current_mode_state_;

  std::chrono::steady_clock::time_point last_publish_time_;

  TripStatus status_;
  StateChangeCb on_state_change_;
  std::chrono::steady_clock::time_point state_entered_at_;
  bool route_received_ = false;
};

}  // namespace AutowareAgent

#endif  // VEHICLEAUTOWAREAGENT_TRIPCONTROLLER_H