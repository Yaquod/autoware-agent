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

#include <boost/asio/post.hpp>

#include <utility>

#include <spdlog/spdlog.h>

namespace autoware_agent {
TripController::TripController(rclcpp::Node::SharedPtr node, const RouteConfig& route_config,
                               std::shared_ptr<boost::asio::io_context::strand> strand,
                               TripTimings timings)
  : node_(std::move(node))
  , route_config_(route_config)
  , strand_(std::move(std::move(strand)))
  , timings_(timings) {
  auto pose_qos = rclcpp::QoS(1).reliable().durability_volatile();
  initial_pose_pub_ = node_->create_publisher<geometry_msgs::msg::PoseWithCovarianceStamped>(
    "/initialpose", pose_qos);

  goal_pub_ =
    node_->create_publisher<geometry_msgs::msg::PoseStamped>("/planning/mission_planning/goal", 1);

  auto state_qos = rclcpp::QoS(1).reliable().transient_local();

  loc_state_sub_ =
    node_->create_subscription<autoware_adapi_v1_msgs::msg::LocalizationInitializationState>(
      "/api/localization/initialization_state", state_qos,
      [this](const autoware_adapi_v1_msgs::msg::LocalizationInitializationState::SharedPtr msg) {
        boost::asio::post(*strand_,[this, msg]() { current_loc_state_ = msg; });
      });

  route_state_sub_ = node_->create_subscription<autoware_adapi_v1_msgs::msg::RouteState>(
    "/api/routing/state", state_qos,
    [this](const autoware_adapi_v1_msgs::msg::RouteState::SharedPtr MSG) {
      boost::asio::post(*strand_,[this, MSG]() { current_route_state_ = MSG; });
    });

  mode_state_sub_ = node_->create_subscription<autoware_adapi_v1_msgs::msg::OperationModeState>(
    "/api/operation_mode/state", state_qos,
    [this](const autoware_adapi_v1_msgs::msg::OperationModeState::SharedPtr msg) {
      boost::asio::post(*strand_,[this, msg]() { current_mode_state_ = msg; });
    });

  engage_client_ =
    node_->create_client<tier4_external_api_msgs::srv::Engage>("/api/autoware/set/engage");

  mode_client_ = node_->create_client<autoware_adapi_v1_msgs::srv::ChangeOperationMode>(
    "/api/operation_mode/change_to_autonomous");
  auto qos = rclcpp::QoS(rclcpp::KeepLast(1))
               .reliability(RMW_QOS_POLICY_RELIABILITY_RELIABLE)
               .durability(RMW_QOS_POLICY_DURABILITY_TRANSIENT_LOCAL);

  route_sub_ = node_->create_subscription<autoware_planning_msgs::msg::LaneletRoute>(
    "/planning/mission_planning/route", qos,
    [this](const autoware_planning_msgs::msg::LaneletRoute::SharedPtr msg) {
      boost::asio::post(*strand_,[this, msg]() { onRouteReceived(*msg); });
    });

  tf_buffer_   = std::make_shared<tf2_ros::Buffer>(node_->get_clock());
  tf_listener_ = std::make_shared<tf2_ros::TransformListener>(*tf_buffer_);

  RCLCPP_INFO(node_->get_logger(), "[AutowareAgent] TripController initialized");
  spdlog::info("[AutowareAgent] TripController initialized");
}

bool TripController::startTrip() {
  if (!pickup_route_.has_value() || !trip_route_.has_value()) {
    spdlog::error("[AutowareAgent] Cannot start trip: missing pickup or trip route");
    RCLCPP_ERROR(node_->get_logger(),
                 "[AutowareAgent] Cannot start trip: missing pickup or trip route");
    return false;
  }

  if (status_.state != TripState::ROUTES_READY && status_.state != TripState::IDLE) {
    spdlog::warn("[AutowareAgent] Trip already in progress");
    RCLCPP_WARN(node_->get_logger(), "[AutowareAgent] Trip already in progress (state=%d)",
                static_cast<int>(status_.state));
    return false;
  }

  // Localise at the pickup point
  const HeldRoute& pickup = *pickup_route_;
  const LaneInfo* start_lane = route_config_.findNearestLane(pickup.goal_gps);
  if (!start_lane) {
    RCLCPP_ERROR(node_->get_logger(), "[AutowareAgent] Find nearest lane for pickup failed");
    transitionTo(TripState::FAILED);
    return false;
  }

  status_.start_lanelet_id = std::to_string(start_lane->lane_id);
  status_.start_x = start_lane->local.x;
  status_.start_y = start_lane->local.y;
  status_.start_z = start_lane->local.z;
  status_.start_qw = start_lane->orientation.w;
  status_.start_qz = start_lane->orientation.z;

  RCLCPP_INFO(node_->get_logger(),"[AutowareAgent] Trip started driving to pickup at lane %s (%.2f, %.2f)",
              status_.start_lanelet_id.c_str(), status_.start_x, status_.start_y);

  transitionTo(TripState::PUBLISHING_INITIAL_POSE);
  return true;
}

bool TripController::move() {
  if (status_.state != TripState::WAITING_FOR_MOVE) {
    RCLCPP_ERROR(node_->get_logger(),"Vehicle move is called but state is %d - ignoring",static_cast<int>(status_.state));
    return false;
  }

  if (!trip_route_.has_value()) {
    RCLCPP_ERROR(node_->get_logger(),"[AutowareAgent] Trip route is not in bank");
    transitionTo(TripState::FAILED);
    return false;
  }

  const HeldRoute& trip = *trip_route_;
  const LaneInfo* goal_lane = route_config_.findNearestLane(trip.goal_gps);
  if (!goal_lane) {
    RCLCPP_ERROR(node_->get_logger(), "[AutowareAgent] cannot resolve goal lane");
    transitionTo(TripState::FAILED);
    return false;
  }

  status_.goal_x  = goal_lane->local.x;
  status_.goal_y  = goal_lane->local.y;
  status_.goal_z  = goal_lane->local.z;
  status_.goal_qw = goal_lane->orientation.w;
  status_.goal_qz = goal_lane->orientation.z;
  status_.goal_lanelet_id  = std::to_string(goal_lane->lane_id);
  status_.goal_gps         = trip.goal_gps;
  status_.goal_distance_m  = trip.distance_m;

  RCLCPP_INFO(node_->get_logger(),
              "[AutowareAgent] Move to goal at lane %s (%.2f, %.2f), distance %.1fm",
              status_.goal_lanelet_id.c_str(), status_.goal_x, status_.goal_y, status_.goal_distance_m);
  doPublishGoal(status_.goal_x, status_.goal_y, status_.goal_z,
                status_.goal_qz, status_.goal_qw);
  transitionTo(TripState::ENGAGING);
  return true;
}

void TripController::cancel() {
  if (status_.state != TripState::IDLE) {
    RCLCPP_INFO(node_->get_logger(), "[AutowareAgent] Trip Canceled");
    spdlog::info("[AutowareAgent] Trip Canceled");
  }
  querying_     = false;
  query_cb_     = nullptr;
  pickup_route_.reset();
  trip_route_.reset();
  status_       = TripStatus{};
  route_received_ = false;
  transitionTo(TripState::IDLE);
}

void TripController::tick() {
  switch (status_.state) {
    case TripState::IDLE:
    case TripState::ROUTES_READY:
    case TripState::WAITING_FOR_MOVE:
    case TripState::DRIVING_TO_PICKUP:
    case TripState::RUNNING:
    case TripState::COMPLETED:
    case TripState::CANCELLED:
    case TripState::FAILED:
      break;


    case TripState::QUERY_PUBLISHING_INITIAL_POSE:
      doPublishInitialPose(status_.start_x, status_.start_y, status_.start_z,
                           status_.start_qz, status_.start_qw);
      transitionTo(TripState::QUERY_WAITING_LOCALISATION);
      break;

    case TripState::QUERY_WAITING_LOCALISATION:
      if (!current_loc_state_ ||
          current_loc_state_->state !=
            autoware_adapi_v1_msgs::msg::LocalizationInitializationState::INITIALIZED) {
        if (elapsed_ms(last_publish_time_) >= 1000) {
          doPublishInitialPose(status_.start_x, status_.start_y, status_.start_z,
                               status_.start_qz, status_.start_qw);
          last_publish_time_ = std::chrono::steady_clock::now();
        }
      } else {
        RCLCPP_INFO(node_->get_logger(), "Localization INITIALIZED. Publishing goal...");
        transitionTo(TripState::QUERY_PUBLISHING_GOAL);
      }
      break;

    case TripState::QUERY_PUBLISHING_GOAL:
      doPublishGoal(status_.goal_x, status_.goal_y, status_.goal_z,
                    status_.goal_qz, status_.goal_qw);
      transitionTo(TripState::QUERY_WAITING_ROUTE);
      break;

    case TripState::QUERY_WAITING_ROUTE:
      if (!current_route_state_ ||
          current_route_state_->state != autoware_adapi_v1_msgs::msg::RouteState::SET) {
        if (elapsed_ms(last_publish_time_) >= 2000) {
          doPublishGoal(status_.goal_x, status_.goal_y, status_.goal_z,
                        status_.goal_qz, status_.goal_qw);
          last_publish_time_ = std::chrono::steady_clock::now();
        }
        if (elapsed_ms(state_entered_at_) >= timings_.route_timeout_ms) {
          failQuery("timeout waiting for route on " +
                    (current_leg_ == QueryLeg::PICKUP ? std::string("pickup") : "trip") +
                    " leg (lanes " + status_.start_lanelet_id + " → " +
                    status_.goal_lanelet_id + " may not be connected)");
        }
          } else {
            RCLCPP_INFO(node_->get_logger(),
                        "[AutowareAgent] Route SET for %s leg — waiting for ETA topic",
                        current_leg_ == QueryLeg::PICKUP ? "pickup" : "trip");
            eta_received_ = false;
            transitionTo(TripState::QUERY_READING_ETA);
          }
      break;

    case TripState::QUERY_READING_ETA:
      if (eta_received_) {
        finishQueryLeg();
      } else if (elapsed_ms(state_entered_at_) >= timings_.eta_settle_ms) {
        failQuery("timeout waiting for ETA on " +
                  (current_leg_ == QueryLeg::PICKUP ? std::string("pickup") : "trip") +
                  " leg");
        eta_distance_m_ = 0.0;
        eta_seconds_    = 0.0;
        finishQueryLeg();
      }
      break;
    case TripState::PUBLISHING_INITIAL_POSE:
      doPublishInitialPose(status_.start_x, status_.start_y, status_.start_z,
                           status_.start_qz, status_.start_qw);
      transitionTo(TripState::WAITING_LOCALISATION);
      break;

    case TripState::WAITING_LOCALISATION:
      if (!current_loc_state_ ||
          current_loc_state_->state !=
            autoware_adapi_v1_msgs::msg::LocalizationInitializationState::INITIALIZED) {
        if (elapsed_ms(last_publish_time_) >= 1000) {
          doPublishInitialPose(status_.start_x, status_.start_y, status_.start_z,
                               status_.start_qz, status_.start_qw);
          last_publish_time_ = std::chrono::steady_clock::now();
        }
            } else {
              RCLCPP_INFO(node_->get_logger(),
                          "[AutowareAgent] Localised at pickup — engaging");
              // Publish the pickup-leg goal so Autoware has the route active
              doPublishGoal(status_.start_x, status_.start_y, status_.start_z,
                            status_.start_qz, status_.start_qw);
              transitionTo(TripState::ENGAGING);
            }
      break;

    case TripState::ENGAGING:
      if (elapsed_ms(last_publish_time_) >= 2000) {
        doEngage();
        last_publish_time_ = std::chrono::steady_clock::now();
      }
      break;
  }
}

TripStatus TripController::status() const {
  return status_;
}

void TripController::setStateChangeCallback(StateChangeCb cb) {
  on_state_change_ = std::move(cb);
}
void TripController::doEngage() {
  if (!mode_client_->service_is_ready() || !engage_client_->service_is_ready()) {
    RCLCPP_WARN(node_->get_logger(), "Engagement services not ready yet");
    return;
  }

  auto mode_request = std::make_shared<autoware_adapi_v1_msgs::srv::ChangeOperationMode::Request>();

  mode_client_->async_send_request(
    mode_request,
    [this](
      rclcpp::Client<autoware_adapi_v1_msgs::srv::ChangeOperationMode>::SharedFuture mode_future) {
      const auto& mode_response = mode_future.get();
      if (!mode_response->status.success) {
        RCLCPP_ERROR(node_->get_logger(), "Failed to change mode: %s",
                     mode_response->status.message.c_str());
        return;
      }

      RCLCPP_INFO(node_->get_logger(), "Mode changed to Autonomous. Sending Engage signal...");

      auto engage_req = std::make_shared<tier4_external_api_msgs::srv::Engage::Request>();
      engage_req->engage = true;

      engage_client_->async_send_request(
        engage_req,
        [this](rclcpp::Client<tier4_external_api_msgs::srv::Engage>::SharedFuture engage_future) {
          const auto& engage_res = engage_future.get();
          if (engage_res->status.code == 1) {
            RCLCPP_INFO(node_->get_logger(), "Car Engaged! Motion started.");

            boost::asio::post(*strand_,[this]() {
              // check whether we are in pickup or trip
              if (status_.state == TripState::ENGAGING) {
                if (trip_route_.has_value()) {
                  transitionTo(TripState::DRIVING_TO_PICKUP);
                } else {
                  transitionTo(TripState::RUNNING);
                }
              }
            });
          } else {
            RCLCPP_ERROR(node_->get_logger(), "Engage failed: %s",
                         engage_res->status.message.c_str());
          }
        });
    });
}

void TripController::doPublishGoal(double x, double y, double z,
                                   double qz, double qw) {
  geometry_msgs::msg::PoseStamped msg;
  msg.header.frame_id = "map";
  msg.header.stamp = node_->now();

  msg.pose.position.x = x;
  msg.pose.position.y = y;
  msg.pose.position.z = z;
  msg.pose.orientation.x = 0.0;
  msg.pose.orientation.y = 0.0;
  msg.pose.orientation.z = qz;
  msg.pose.orientation.w = qw;

  goal_pub_->publish(msg);
  RCLCPP_INFO(node_->get_logger(), "[AutowareAgent] Published goal (%.2f, %.2f)", x,
              y);
  spdlog::info("[AutowareAgent] Published goal ({:.2f}, {:.2f})", x, y);
}

void TripController::doPublishInitialPose(double x, double y, double z,
                                          double qz, double qw) {
  geometry_msgs::msg::PoseWithCovarianceStamped msg;
  msg.header.frame_id = "map";
  msg.header.stamp = node_->now();

  msg.pose.pose.position.x = x;
  msg.pose.pose.position.y = y;
  msg.pose.pose.position.z = z;
  msg.pose.pose.orientation.x = 0.0;
  msg.pose.pose.orientation.y = 0.0;
  msg.pose.pose.orientation.z = qz;
  msg.pose.pose.orientation.w = qw;

  // Covariance: 0.25 m² on x & y, ~0.0685 rad² on yaw (index 35).
  msg.pose.covariance.fill(0.0);
  msg.pose.covariance[0] = 0.25;
  msg.pose.covariance[7] = 0.25;
  msg.pose.covariance[35] = 0.06853891909122467;

  initial_pose_pub_->publish(msg);
  RCLCPP_INFO(node_->get_logger(), "[AutowareAgent] Published initial pose (%.2f, %.2f)",
              x, y);
  spdlog::info("[AutowareAgent] Published initial pose ({:.2f}, {:.2f})", x,
               y);
}

void TripController::onRouteReceived(
    const autoware_planning_msgs::msg::LaneletRoute& msg)
{
  last_route_     = std::make_shared<autoware_planning_msgs::msg::LaneletRoute>(msg);
  route_received_ = true;

  RCLCPP_INFO(
    node_->get_logger(),
    "[AutowareAgent] Route received — %zu segment(s), "
    "start primitive id: %ld",
    msg.segments.size(),
    msg.segments.empty() ? -1L
                         : static_cast<long>(msg.segments[0].preferred_primitive.id));

  if (status_.state == TripState::QUERY_WAITING_ROUTE ||
      status_.state == TripState::QUERY_READING_ETA) {
    RCLCPP_INFO(node_->get_logger(),
                "[AutowareAgent] Route message arrived during query "
                "%zu segment(s) cached for route bank",
                msg.segments.size());
      }
}
void TripController::transitionTo(TripState next) {
  TripState const prev = status_.state;
  if (prev == next) {
    return;
  }
  status_.state = next;
  status_.last_state_change = std::chrono::steady_clock::now();
  state_entered_at_ = status_.last_state_change;
  last_publish_time_  = std::chrono::steady_clock::time_point{};
  if (on_state_change_) {
    on_state_change_(prev, next);
  }
}

long TripController::elapsed_ms(std::chrono::steady_clock::time_point since) {
  return std::chrono::duration_cast<std::chrono::milliseconds>(std::chrono::steady_clock::now() -
                                                               since)
    .count();
}

bool TripController::queryEta(GPSCoordinate start_gps, GPSCoordinate goal_gps,
                              EtaQueryCallback cb) {
  if (status_.state != TripState::IDLE) {
    RCLCPP_ERROR(node_->get_logger(),
                 "[AutowareAgent] Cannot query ETA: trip in progress (state=%d)",
                 static_cast<int>(status_.state));
    if (cb) {
      EtaQueryResult result;
      result.success = false;
      result.error_message = "Trip in progress (state=" + std::to_string(static_cast<int>(status_.state)) + ")";
      cb(result);
    }
    return false;
  }

  auto vehicle_pose = getCurrentVehiclePose();
  if (!vehicle_pose) {
    RCLCPP_ERROR(node_->get_logger(),"[AutowareAgent] Cannot query ETA: failed to get current vehicle pose from TF");
    if (cb) {
      EtaQueryResult result;
      result.success = false;
      result.error_message = "No vehicle pose from /tf";
      cb(result);
    }
    return false;
  }

  querying_ = true;
  current_leg_ = QueryLeg::PICKUP;
  query_start_gps_ = start_gps;
  query_goal_gps_ = goal_gps;
  query_cb_ = std::move(cb);
  query_result_    = {};
  pickup_route_.reset();
  trip_route_.reset();

  spdlog::info("[AutowareAgent] Query ETA pickup leg: vehicle({:.6f},{:.6f}) → start({:.6f},{:.6f})",
               vehicle_pose->latitude, vehicle_pose->longitude,
               start_gps.latitude, start_gps.longitude);
  spdlog::info("[AutowareAgent] Query ETA trip leg: start({:.6f},{:.6f}) → goal({:.6f},{:.6f})",
               start_gps.latitude, start_gps.longitude,
               goal_gps.latitude, goal_gps.longitude);
  startQueryLeg(start_gps, goal_gps);
  return true;
}

void TripController::injectEta(double distance_m, double time_seconds) {
  if (status_.state != TripState::QUERY_READING_ETA) {
    return;
  }

  eta_distance_m_ = distance_m;
  eta_seconds_ = time_seconds;
  eta_received_ = true;
  RCLCPP_INFO(node_->get_logger(),
              "[AutowareAgent] ETA injected — %.1f m / %.1f s",
              distance_m, time_seconds);
}

void TripController::failQuery(const std::string& reason) {
  RCLCPP_ERROR(node_->get_logger(), "[AutowareAgent] Query failed: %s", reason.c_str());
  spdlog::error("[AutowareAgent] Query failed: {}", reason);

  if (query_cb_) {
    query_result_.success       = false;
    query_result_.error_message = reason;
    query_cb_(query_result_);
    query_cb_ = nullptr;
  }
  querying_     = false;
  pickup_route_.reset();
  trip_route_.reset();
  status_ = TripStatus{};
  transitionTo(TripState::IDLE);
}

void TripController::finishQueryLeg() {
  if (current_leg_ == QueryLeg::PICKUP) {
    // Store pickup leg result
    query_result_.pickup_leg = LegEta{
      .success       = true,
      .distance_m    = eta_distance_m_,
      .eta_seconds   = eta_seconds_,
      .start_lane_id = status_.start_lanelet_id,
      .goal_lane_id  = status_.goal_lanelet_id
    };

    // Store pickup route in bank
    pickup_route_ = HeldRoute{
      .route      = last_route_ ? *last_route_ : autoware_planning_msgs::msg::LaneletRoute{},
      .eta_seconds = eta_seconds_,
      .distance_m  = eta_distance_m_,
      .goal_gps    = query_start_gps_   // pickup destination = trip start point
    };

    RCLCPP_INFO(node_->get_logger(),
                "[AutowareAgent] Pickup leg done — ETA %.1f s / %.1f m — starting trip leg",
                eta_seconds_, eta_distance_m_);

    current_leg_ = QueryLeg::TRIP;
    current_route_state_.reset();
    route_received_ = false;
    startQueryLeg(query_start_gps_, query_goal_gps_);

  } else {
    // Trip leg done
    query_result_.trip_leg = LegEta{
      .success       = true,
      .distance_m    = eta_distance_m_,
      .eta_seconds   = eta_seconds_,
      .start_lane_id = status_.start_lanelet_id,
      .goal_lane_id  = status_.goal_lanelet_id
    };

    // Store trip route in bank
    trip_route_ = HeldRoute{
      .route       = last_route_ ? *last_route_ : autoware_planning_msgs::msg::LaneletRoute{},
      .eta_seconds = eta_seconds_,
      .distance_m  = eta_distance_m_,
      .goal_gps    = query_goal_gps_
    };

    RCLCPP_INFO(node_->get_logger(),
                "[AutowareAgent] Trip leg done — ETA %.1f s / %.1f m",
                eta_seconds_, eta_distance_m_);
    spdlog::info("[AutowareAgent] Trip leg done — {:.1f} s / {:.1f} m", eta_seconds_, eta_distance_m_);

    query_result_.success = true;
    querying_ = false;

    if (query_cb_) {
      query_cb_(query_result_);
      query_cb_ = nullptr;
    }

    // routes are in the bank, no motion yet
    status_ = TripStatus{};
    transitionTo(TripState::ROUTES_READY);
  }
}

void TripController::startQueryLeg(GPSCoordinate from_gps, GPSCoordinate to_gps) {
  // Resolve from-lane for initial pose
  const LaneInfo* from_lane = route_config_.findNearestLane(from_gps);
  if (!from_lane) {
    failQuery("cannot resolve lane for " +
              (current_leg_ == QueryLeg::PICKUP ? std::string("pickup origin") : "trip start"));
    return;
  }

  // Resolve to-lane for goal
  const LaneInfo* to_lane = route_config_.findNearestLane(to_gps);
  if (!to_lane) {
    failQuery("cannot resolve lane for " +
              (current_leg_ == QueryLeg::PICKUP ? std::string("pickup destination") : "trip goal"));
    return;
  }

  status_.start_x  = from_lane->local.x;
  status_.start_y  = from_lane->local.y;
  status_.start_z  = from_lane->local.z;
  status_.start_qw = from_lane->orientation.w;
  status_.start_qz = from_lane->orientation.z;
  status_.start_lanelet_id = std::to_string(from_lane->lane_id);

  status_.goal_x  = to_lane->local.x;
  status_.goal_y  = to_lane->local.y;
  status_.goal_z  = to_lane->local.z;
  status_.goal_qw = to_lane->orientation.w;
  status_.goal_qz = to_lane->orientation.z;
  status_.goal_lanelet_id = std::to_string(to_lane->lane_id);
  status_.goal_gps        = to_gps;

  LocalCoordinate raw = route_config_.gpsToLocalCoordinate(to_gps);
  double dx = to_lane->local.x - raw.x;
  double dy = to_lane->local.y - raw.y;
  status_.goal_distance_m = std::sqrt(dx * dx + dy * dy);

  RCLCPP_INFO(node_->get_logger(),
              "[AutowareAgent] Query %s leg: lane %s → lane %s",
              current_leg_ == QueryLeg::PICKUP ? "pickup" : "trip",
              status_.start_lanelet_id.c_str(),
              status_.goal_lanelet_id.c_str());

  transitionTo(TripState::QUERY_PUBLISHING_INITIAL_POSE);
}

std::optional<GPSCoordinate> TripController::getCurrentVehiclePose() const {
  try {
    geometry_msgs::msg::TransformStamped const TF =
      tf_buffer_->lookupTransform("map", "base_link", tf2::TimePointZero);

    LocalCoordinate const LOCAL{
      .x = TF.transform.translation.x,
      .y = TF.transform.translation.y,
      .z = TF.transform.translation.z
    };
    return route_config_.localCoordinateToGps(LOCAL);
  } catch (const tf2::TransformException& ex) {
    RCLCPP_WARN(node_->get_logger(),
                "[AutowareAgent] TF lookup failed: %s", ex.what());
    return std::nullopt;
  }
}
}  // namespace AutowareAgent