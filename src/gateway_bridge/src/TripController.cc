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

#include <boost/asio/post.hpp>

namespace AutowareAgent {
TripController::TripController(
    rclcpp::Node::SharedPtr node, const RouteConfig& route_config,
    std::shared_ptr<boost::asio::io_context::strand> strand,
    TripTimings timings)
    : node_(std::move(node)),
      route_config_(route_config),
      strand_(strand),
      timings_(timings) {
  auto pose_qos = rclcpp::QoS(1).reliable().durability_volatile();
  initial_pose_pub_ =
      node_->create_publisher<geometry_msgs::msg::PoseWithCovarianceStamped>(
          "/initialpose", pose_qos);

  goal_pub_ = node_->create_publisher<geometry_msgs::msg::PoseStamped>(
      "/planning/mission_planning/goal", 1);

  auto state_qos = rclcpp::QoS(1).reliable().transient_local();

  loc_state_sub_ = node_->create_subscription<
      autoware_adapi_v1_msgs::msg::LocalizationInitializationState>(
      "/api/localization/initialization_state", state_qos,
      [this](const autoware_adapi_v1_msgs::msg::
                 LocalizationInitializationState::SharedPtr msg) {
        strand_->post([this, msg]() { current_loc_state_ = msg; });
      });

  route_state_sub_ =
      node_->create_subscription<autoware_adapi_v1_msgs::msg::RouteState>(
          "/api/routing/state", state_qos,
          [this](const autoware_adapi_v1_msgs::msg::RouteState::SharedPtr msg) {
            strand_->post([this, msg]() { current_route_state_ = msg; });
          });

  mode_state_sub_ = node_->create_subscription<
      autoware_adapi_v1_msgs::msg::OperationModeState>(
      "/api/operation_mode/state", state_qos,
      [this](const autoware_adapi_v1_msgs::msg::OperationModeState::SharedPtr
                 msg) {
        strand_->post([this, msg]() { current_mode_state_ = msg; });
      });

  engage_client_ = node_->create_client<tier4_external_api_msgs::srv::Engage>(
      "/api/autoware/set/engage");

  mode_client_ =
      node_->create_client<autoware_adapi_v1_msgs::srv::ChangeOperationMode>(
          "/api/operation_mode/change_to_autonomous");
  auto qos = rclcpp::QoS(rclcpp::KeepLast(1))
                 .reliability(RMW_QOS_POLICY_RELIABILITY_RELIABLE)
                 .durability(RMW_QOS_POLICY_DURABILITY_TRANSIENT_LOCAL);

  route_sub_ =
      node_->create_subscription<autoware_planning_msgs::msg::LaneletRoute>(
          "/planning/mission_planning/route", qos,
          [this](
              const autoware_planning_msgs::msg::LaneletRoute::SharedPtr msg) {
            strand_->post([this, msg]() { onRouteReceived(*msg); });
          });

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
    spdlog::error("[AutowareAgent] Find nearest lane failed");
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
    RCLCPP_INFO(node_->get_logger(), "[AutowareAgent] Trip Canceled");
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
      if (!current_loc_state_ ||
          current_loc_state_->state !=
              autoware_adapi_v1_msgs::msg::LocalizationInitializationState::
                  INITIALIZED) {
        if (elapsed_ms(last_publish_time_) >= 1000) {
          doPublishInitialPose();
          last_publish_time_ = std::chrono::steady_clock::now();
        }
      } else {
        RCLCPP_INFO(node_->get_logger(),
                    "Localization INITIALIZED. Moving to Goal.");
        transitionTo(TripState::PUBLISHING_GOAL);
      }
      break;

    case TripState::PUBLISHING_GOAL:
      doPublishGoal();
      transitionTo(TripState::WAITING_ROUTE);
      break;

    case TripState::WAITING_ROUTE:
      if (!current_route_state_ ||
          current_route_state_->state !=
              autoware_adapi_v1_msgs::msg::RouteState::SET) {
        if (elapsed_ms(last_publish_time_) >= 2000) {
          doPublishGoal();
          last_publish_time_ = std::chrono::steady_clock::now();
        }
      } else {
        RCLCPP_INFO(node_->get_logger(), "Route is SET. Ready to Engage.");
        transitionTo(TripState::ENGAGING);
      }
      break;

    case TripState::ENGAGING:
      if (elapsed_ms(last_publish_time_) >= 2000) {
        doEngage();
        last_publish_time_ = std::chrono::steady_clock::now();
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
  if (!mode_client_->service_is_ready() ||
      !engage_client_->service_is_ready()) {
    RCLCPP_WARN(node_->get_logger(), "Engagement services not ready yet");
    return;
  }

  auto mode_request = std::make_shared<
      autoware_adapi_v1_msgs::srv::ChangeOperationMode::Request>();

  mode_client_->async_send_request(
      mode_request,
      [this](rclcpp::Client<autoware_adapi_v1_msgs::srv::ChangeOperationMode>::
                 SharedFuture mode_future) {
        auto mode_response = mode_future.get();
        if (!mode_response->status.success) {
          RCLCPP_ERROR(node_->get_logger(), "Failed to change mode: %s",
                       mode_response->status.message.c_str());
          return;
        }

        RCLCPP_INFO(node_->get_logger(),
                    "Mode changed to Autonomous. Sending Engage signal...");

        auto engage_req =
            std::make_shared<tier4_external_api_msgs::srv::Engage::Request>();
        engage_req->engage = true;

        engage_client_->async_send_request(
            engage_req,
            [this](rclcpp::Client<tier4_external_api_msgs::srv::Engage>::
                       SharedFuture engage_future) {
              auto engage_res = engage_future.get();
              if (engage_res->status.code == 1) {
                RCLCPP_INFO(node_->get_logger(),
                            "Car Engaged! Motion started.");

                strand_->post([this]() { transitionTo(TripState::RUNNING); });
              } else {
                RCLCPP_ERROR(node_->get_logger(), "Engage failed: %s",
                             engage_res->status.message.c_str());
              }
            });
      });
}

void TripController::doPollRoute() {
  if (route_received_) {
    RCLCPP_INFO(node_->get_logger(),
                "[AutowareAgent] Route received, moving to engaging.");
    spdlog::info("[AutowareAgent] Route received, moving to engaging");
    transitionTo(TripState::ENGAGING);
    return;
  }

  if (elapsed_ms(state_entered_at_) >= timings_.route_timeout_ms) {
    std::string error = "[AutowareAgent] Timeout waiting for route. Lanes " +
                        status_.start_lanelet_id + " and " +
                        status_.goal_lanelet_id + " may not be connected";
    RCLCPP_ERROR(node_->get_logger(), "%s", error.c_str());
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
  spdlog::info("[AutowareAgent] Published goal ({:.2f}, {:.2f})",
               status_.goal_x, status_.goal_y);
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
              "[AutowareAgent] Published initial pose (%.2f, %.2f)",
              status_.start_x, status_.start_y);
  spdlog::info("[AutowareAgent] Published initial pose ({:.2f}, {:.2f})",
               status_.start_x, status_.start_y);
}

void TripController::onRouteReceived(
    const autoware_planning_msgs::msg::LaneletRoute& msg) {
  RCLCPP_INFO(
      node_->get_logger(),
      "[AutowareAgent] Route callback executed! Start lane: %ld, segments: %zu",
      msg.segments.empty() ? -1 : msg.segments[0].preferred_primitive.id,
      msg.segments.size());
  spdlog::info("[AutowareAgent] Route callback executed! Segments: {}",
               msg.segments.size());
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