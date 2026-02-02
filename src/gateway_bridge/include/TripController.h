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

#include <chrono>
#include <functional>
#include <geometry_msgs/msg/pose_stamped.hpp>
#include <geometry_msgs/msg/pose_with_covariance_stamped.hpp>
#include <memory>
#include <rclcpp/rclcpp.hpp>
#include <string>
#include <tier4_external_api_msgs/srv/engage.hpp>

#include "RouteConfig.h"
#include "TripStates.h"
#include "TripStatus.h"
#include "TripTimings.h"

namespace AutowareAgent {
/**
 * Owns the "send a trip" lifecycle as a state machine Lives inside
 * AutowareController, driven by that node's periodic tick() call.
 */
class TripController {
 public:
  TripController(rclcpp::Node::SharedPtr node, const RouteConfig& route_config,
                 TripTimings timings = {});

  ~TripController() = default;

  /**
   * @brief Kick off a new trip to the given GPS coordinate.
   * @param goal_gps Location of the goal needed to go to.
   * @return Returns false only if the trip is in progress.
   */
  bool startTrip(GPSCoordinate goal_gps);

  /**
   * @brief Advance the state machine by one tick.
   */
  void tick();

  /**
   * @brief Cancel the current trip.
   */
  void cancel();

  /**
   * @brief A snapshot from a trip without blocking or throwing exceptions.
   * @return Returns a Read-only snapshot from a trip.
   */
  TripStatus status() const;

  /**
   * @brief Callback fired on every state transition.  Pass nullptr to clear.
   */
  using StateChangeCb = std::function<void(TripState priv, TripState next)>;

  void setStateChangeCallback(StateChangeCb cb);

 private:
  rclcpp::Node::SharedPtr node_;
  rclcpp::Publisher<geometry_msgs::msg::PoseWithCovarianceStamped>::SharedPtr
      initial_pose_pub_;
  rclcpp::Publisher<geometry_msgs::msg::PoseStamped>::SharedPtr goal_pub_;
  rclcpp::Client<tier4_external_api_msgs::srv::Engage>::SharedPtr
      engage_client_;
  rclcpp::Subscription<geometry_msgs::msg::PoseStamped>::SharedPtr
      route_sub_;  // TODO: The only thing we read is the route_received_ flag;
  //  the actual message type can be swapped later ,
  //  for the real autoware_planning_msgs/LaneletRoute without changing logic.

  const RouteConfig& route_config_;
  TripStatus status_;
  TripTimings timings_;
  StateChangeCb on_state_change_;
  bool route_received_{false};
  std::chrono::steady_clock::time_point state_entered_at_;

  void transitionTo(TripState next);

  void doPublishInitialPose();

  void doPublishGoal();

  void doPollRoute();

  void doEngage();

  void onRouteReceived(const geometry_msgs::msg::PoseStamped& msg);

  static long elapsed_ms(std::chrono::steady_clock::time_point since);
};
}  // namespace AutowareAgent
#endif  // VEHICLEAUTOWAREAGENT_TRIPCONTROLLER_H
