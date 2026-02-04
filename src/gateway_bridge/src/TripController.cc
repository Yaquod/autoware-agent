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

#include "../include/TripController.h"

#include <spdlog/spdlog.h>

namespace AutowareAgent {
TripController::TripController(rclcpp::Node::SharedPtr node,
                               const RouteConfig& route_config,
                               TripTimings timings)
    : node_(std::move(node)), route_config_(route_config), timings_(timings) {
  initial_pose_pub_ =
      node_->create_publisher<geometry_msgs::msg::PoseWithCovarianceStamped>(
          "/initialpose", 1);

  goal_pub_ = node_->create_publisher<geometry_msgs::msg::PoseStamped>(
      "/planning/mission_planning/goal", 1);

  engage_client_ = node_->create_client<tier4_external_api_msgs::srv::Engage>(
      "/api/autoware/set/engage");

  route_sub_ = node_->create_subscription<geometry_msgs::msg::PoseStamped>(
      "/planning/mission_planning/route", 1,
      [this](const geometry_msgs::msg::PoseStamped& msg) {
        onRouteReceived(msg);
      });  // TODO: swap PoseStamped for autoware_planning_msgs/LaneletRoute
           // when
           //       that message type is available in your workspace.  The only
           //       thing the state machine reads is route_received_.

  RCLCPP_INFO(node_->get_logger(),
              "[AutowareAgent] TripController initialized");
  spdlog::info("[AutowareAgent] TripController initialized");
}

bool TripController::startTrip(GPSCoordinate goal_gps) {
  if (status_.state != TripState::IDLE) {
    spdlog::warn("[AutowareAgent] Trip already in progress");
    RCLCPP_WARN(node_->get_logger(),
                "[AutowareAgent] Trip already in progress (state=%d)",
                status_.state);
    return false;
  }

  // A fresh start
  route_received_ = false;
  status_.goal_gps = goal_gps;
  status_.trip_started_at = std::chrono::steady_clock::now();
  const FixedStartPosition* start = route_config_.getDefaultStart();
  if (!start) {
    spdlog::error("[AutowareAgent] No default start found");
    transitionTo(TripState::FAILED);
    return false;
  }

  status_.start_lanelet_id = std::to_string(start->lane_id);
  status_.start_x = start->local.x;
  status_.start_y = start->local.y;
  status_.start_z = start->local.z;
  status_.start_qw = start->orientation.w;
  status_.start_qz = start->orientation.z;

  // FindNearestLane does GPS -> local internally.
  const LaneInfo* goal_lane = route_config_.FindNearestLane(goal_gps);
  if (!goal_lane) {
    spdlog::error("[AutowareAgent] Find nearst lane failed");
    transitionTo(TripState::FAILED);
    return false;
  }

  status_.goal_lanelet_id = std::to_string(goal_lane->lane_id);
  status_.goal_x = goal_lane->local.x;
  status_.goal_y = goal_lane->local.y;
  status_.goal_z = goal_lane->local.z;
  status_.goal_qw = goal_lane->orientation.w;
  status_.goal_qz = goal_lane->orientation.z;

  LocalCoordinate raw_local = route_config_.gpsToLocalCoordinate(goal_gps);
  double dx = goal_lane->local.x - raw_local.x;
  double dy = goal_lane->local.y - raw_local.y;
  status_.goal_distance_m = std::sqrt(dx * dx + dy * dy);

  RCLCPP_INFO(
      node_->get_logger(),
      "[AutowareAgent] Starting trip\n"
      "  start: lane %s  (%.2f, %.2f, %.2f)  qz=%.6f qw=%.6f\n"
      "  goal:  lane %s  (%.2f, %.2f, %.2f)  qz=%.6f qw=%.6f  [snap %.2f m]",
      status_.start_lanelet_id.c_str(), status_.start_x, status_.start_y,
      status_.start_z, status_.start_qz, status_.start_qw,
      status_.goal_lanelet_id.c_str(), status_.goal_x, status_.goal_y,
      status_.goal_z, status_.goal_qz, status_.goal_qw,
      status_.goal_distance_m);

  spdlog::info(
      "[AutowareAgent] Starting trip\n"
      "  start: lane {} ({:.2f}, {:.2f}, {:.2f}) qz={:.6f} qw={:.6f}\n"
      "  goal:  lane {} ({:.2f}, {:.2f}, {:.2f}) qz={:.6f} qw={:.6f} [snap "
      "{:.2f} m]",
      status_.start_lanelet_id, status_.start_x, status_.start_y,
      status_.start_z, status_.start_qz, status_.start_qw,
      status_.goal_lanelet_id, status_.goal_x, status_.goal_y, status_.goal_z,
      status_.goal_qz, status_.goal_qw, status_.goal_distance_m);

  transitionTo(TripState::PUBLISHING_INITIAL_POSE);
  return true;
}

void TripController::cancel() {
  if (status_.state != TripState::IDLE) {
    RCLCPP_INFO(node_->get_logger(), "[AutwareAgent] Trip Canceled");
    spdlog::info("[AutowareAgent] Trip Canceled");
  }
  status_ = TripStatus{};
  route_received_ = false;
  transitionTo(TripState::IDLE);
}

void TripController::tick() {
  switch (status_.state) {
    case TripState::IDLE:
    case TripState::FAILED:
    case TripState::COMPLETED:
      break;

    case TripState::PUBLISHING_INITIAL_POSE:
      doPublishInitialPose();
      transitionTo(TripState::WAITING_LOCALISATION);
      break;

    case TripState::WAITING_LOCALISATION:
      if (elapsed_ms(state_entered_at_) >= timings_.initial_pose_delay_ms)
        transitionTo(TripState::PUBLISHING_GOAL);
      break;

    case TripState::PUBLISHING_GOAL:
      doPublishGoal();
      transitionTo(TripState::WAITING_ROUTE);
      break;

    case TripState::WAITING_ROUTE:
      doPollRoute();
      break;

    case TripState::ENGAGING:
      if (elapsed_ms(state_entered_at_) >= timings_.engage_delay_ms) {
        doEngage();
        transitionTo(TripState::RUNNING);
      }
      break;

    case TripState::RUNNING:
      // TODO: Future-> add watch /tf or odometry for arrival detection.
      break;
  }
}

TripStatus TripController::status() const { return status_; }

void TripController::setStateChangeCallback(StateChangeCb cb) {
  on_state_change_ = std::move(cb);
}

void TripController::doEngage() {
  if (!engage_client_->service_is_ready()) {
    RCLCPP_WARN(node_->get_logger(),
                "[AutowareAgent] Engage service is not ready");
    spdlog::warn("[AutowareAgent] Engage service is not ready");
    return;
  }

  auto request =
      std::make_shared<tier4_external_api_msgs::srv::Engage::Request>();
  request->engage = true;
  engage_client_->async_send_request(request);
  RCLCPP_INFO(node_->get_logger(), "[AutowareAgent] Engage request sent");
  spdlog::info("[AutowareAgent] Engage request sent");
}

void TripController::doPollRoute() {
  if (route_received_) {
    RCLCPP_INFO(node_->get_logger(),
                "[AutowareAgent] Route received moving to engaging.");
    spdlog::info("[AutowareAgent] Route received moving to engaging");
    transitionTo(TripState::ENGAGING);
  }

  if (elapsed_ms(state_entered_at_) >= timings_.route_timeout_ms) {
    std::string error = "[AutowareAgent] Timeout waiting for route lanes" +
                        status_.start_lanelet_id + " and " +
                        status_.goal_lanelet_id + " may not be connected";
    RCLCPP_ERROR(node_->get_logger(), error.c_str());
    spdlog::error(error);
    transitionTo(TripState::FAILED);
  }
}

void TripController::doPublishGoal() {
  geometry_msgs::msg::PoseStamped msg;
  msg.header.frame_id = "map";
  msg.header.stamp = node_->now();

  msg.pose.position.x = status_.goal_x;
  msg.pose.position.y = status_.goal_y;
  msg.pose.position.z = status_.goal_z;
  msg.pose.orientation.x = 0.0;
  msg.pose.orientation.y = 0.0;
  msg.pose.orientation.z = status_.goal_qz;
  msg.pose.orientation.w = status_.goal_qw;

  goal_pub_->publish(msg);
  RCLCPP_INFO(node_->get_logger(),
              "[AutowareAgent] Published goal (%.2f, %.2f)", status_.goal_x,
              status_.goal_y);
  spdlog::info("[AutowareAgent] Published goal (%.2f, %.2f)", status_.goal_x,
               status_.goal_y);
}

void TripController::doPublishInitialPose() {
  geometry_msgs::msg::PoseWithCovarianceStamped msg;
  msg.header.frame_id = "map";
  msg.header.stamp = node_->now();

  msg.pose.pose.position.x = status_.start_x;
  msg.pose.pose.position.y = status_.start_y;
  msg.pose.pose.position.z = status_.start_z;
  msg.pose.pose.orientation.x = 0.0;
  msg.pose.pose.orientation.y = 0.0;
  msg.pose.pose.orientation.z = status_.start_qz;
  msg.pose.pose.orientation.w = status_.start_qw;

  // Covariance: 0.25 m² on x & y, ~0.0685 rad² on yaw (index 35).
  msg.pose.covariance.fill(0.0);
  msg.pose.covariance[0] = 0.25;
  msg.pose.covariance[7] = 0.25;
  msg.pose.covariance[35] = 0.06853891909122467;

  initial_pose_pub_->publish(msg);
  RCLCPP_INFO(node_->get_logger(),
              "[AutowareAgent] Published initial pose (%.2f,%0.2f)",
              status_.start_x, status_.start_y);
  spdlog::info("[AutowareAgent] Published initial pose (%.2f,%0.2f)",
               status_.start_x, status_.start_y);
}

void TripController::onRouteReceived(
    const geometry_msgs::msg::PoseStamped& msg) {
  route_received_ = true;
}

void TripController::transitionTo(TripState next) {
  TripState prev = status_.state;
  if (prev == next) return;
  status_.state = next;
  status_.last_state_change = std::chrono::steady_clock::now();
  state_entered_at_ = status_.last_state_change;

  if (on_state_change_) on_state_change_(prev, next);
}

long TripController::elapsed_ms(std::chrono::steady_clock::time_point since) {
  return std::chrono::duration_cast<std::chrono::milliseconds>(
             std::chrono::steady_clock::now() - since)
      .count();
}

}  // namespace AutowareAgent