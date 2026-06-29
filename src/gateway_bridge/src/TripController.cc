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

#include "TripController.h"

#include <boost/asio/post.hpp>

#include <cmath>
#include <utility>

#include <spdlog/spdlog.h>

namespace autoware_agent {

TripController::TripController(rclcpp::Node::SharedPtr node, const LaneletMap& route_config,
                               std::shared_ptr<boost::asio::io_context::strand> strand,
                               TripTimings timings)
  : node_(std::move(node))
  , lanelet_map_(route_config)
  , strand_(std::move(strand))
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
      [this](autoware_adapi_v1_msgs::msg::LocalizationInitializationState::SharedPtr msg) {
        boost::asio::post(*strand_, [this, msg]() { current_loc_state_ = msg; });
      });

  eta_sub_ = node_->create_subscription<autoware_internal_msgs::msg::MissionRemainingDistanceTime>(
    "/planning/mission_remaining_distance_time", rclcpp::QoS(1),
    [this](autoware_internal_msgs::msg::MissionRemainingDistanceTime::SharedPtr msg) {
      boost::asio::post(*strand_,
                        [this, msg]() { injectEta(msg->remaining_distance, msg->remaining_time); });
    });

  route_state_sub_ = node_->create_subscription<autoware_adapi_v1_msgs::msg::RouteState>(
    "/api/routing/state", state_qos,
    [this](autoware_adapi_v1_msgs::msg::RouteState::SharedPtr msg) {
      boost::asio::post(*strand_, [this, msg]() { current_route_state_ = msg; });
    });

  mode_state_sub_ = node_->create_subscription<autoware_adapi_v1_msgs::msg::OperationModeState>(
    "/api/operation_mode/state", state_qos,
    [this](autoware_adapi_v1_msgs::msg::OperationModeState::SharedPtr msg) {
      boost::asio::post(*strand_, [this, msg]() { current_mode_state_ = msg; });
    });

  auto route_qos = rclcpp::QoS(rclcpp::KeepLast(1))
                     .reliability(RMW_QOS_POLICY_RELIABILITY_RELIABLE)
                     .durability(RMW_QOS_POLICY_DURABILITY_TRANSIENT_LOCAL);

  route_sub_ = node_->create_subscription<autoware_planning_msgs::msg::LaneletRoute>(
    "/planning/mission_planning/route", route_qos,
    [this](autoware_planning_msgs::msg::LaneletRoute::SharedPtr msg) {
      boost::asio::post(*strand_, [this, msg]() { onRouteReceived(*msg); });
    });

  engage_client_ =
    node_->create_client<tier4_external_api_msgs::srv::Engage>("/api/autoware/set/engage");

  mode_client_ = node_->create_client<autoware_adapi_v1_msgs::srv::ChangeOperationMode>(
    "/api/operation_mode/change_to_autonomous");

  tf_buffer_ = std::make_shared<tf2_ros::Buffer>(node_->get_clock());
  tf_listener_ = std::make_shared<tf2_ros::TransformListener>(*tf_buffer_);

  RCLCPP_INFO(node_->get_logger(), "[AutowareAgent] TripController initialized");
  spdlog::info("[AutowareAgent] TripController initialized");
}

void TripController::initializeInMap() {
  const FixedStartPosition* fixed_start = lanelet_map_.getDefaultStart();
  if (!fixed_start) {
    RCLCPP_ERROR(node_->get_logger(),
                 "[AutowareAgent] No fixed start position in route config — "
                 "cannot initialize vehicle in map");
    return;
  }

  RCLCPP_INFO(node_->get_logger(),
              "[AutowareAgent] Initializing vehicle in map at fixed start '%s' "
              "lane= (%.2f, %.2f)",
              fixed_start->name.c_str(), fixed_start->local.x, fixed_start->local.y);
  init_x_ = fixed_start->local.x;
  init_y_ = fixed_start->local.y;
  init_z_ = fixed_start->local.z;
  init_qz_ = fixed_start->orientation.z;
  init_qw_ = fixed_start->orientation.w;

  doPublishInitialPose(init_x_, init_y_, init_z_, init_qz_, init_qw_);
  transitionTo(TripState::INITIALIZING_IN_MAP);
}

bool TripController::queryEta(GPSCoordinate start_gps, GPSCoordinate goal_gps,
                              EtaQueryCallback cb) {
  std::cout << "Received queryEta request: start(" << start_gps.latitude << ", "
            << start_gps.longitude << ") → goal(" << goal_gps.latitude << ", " << goal_gps.longitude
            << ")\n";

  if (status_.state_ != TripState::IDLE) {
    RCLCPP_ERROR(node_->get_logger(), "[AutowareAgent] queryEta rejected — not IDLE (state=%d)",
                 static_cast<int>(status_.state_));
    if (cb) {
      EtaQueryResult r;
      r.success_ = false;
      r.error_message_ =
        "not idle (state=" + std::to_string(static_cast<int>(status_.state_)) + ")";
      cb(r);
    }
    return false;
  }

  // Vehicle must be localized before we can read /tf
  auto vehicle_pose = getCurrentVehiclePose();
  if (!vehicle_pose) {
    RCLCPP_ERROR(node_->get_logger(),
                 "[AutowareAgent] queryEta rejected — vehicle not yet localized in map. "
                 "Ensure initializeInMap() has completed before calling queryEta().");
    if (cb) {
      EtaQueryResult r;
      r.success_ = false;
      r.error_message_ = "vehicle not localized — call after IDLE (post map init)";
      cb(r);
    }
    return false;
  }

  querying_ = true;
  current_leg_ = QueryLeg::PICKUP;
  query_start_gps_ = start_gps;  // pickup point   (= trip leg start)
  query_goal_gps_ = goal_gps;    // final destination
  query_cb_ = std::move(cb);
  query_result_ = {};
  pickup_route_.reset();
  trip_route_.reset();

  spdlog::info(
    "[AutowareAgent] queryEta pickup leg: "
    "vehicle({:.6f},{:.6f}) → pickup({:.6f},{:.6f})",
    vehicle_pose->latitude, vehicle_pose->longitude, start_gps.latitude, start_gps.longitude);
  spdlog::info(
    "[AutowareAgent] queryEta trip leg: "
    "pickup({:.6f},{:.6f}) → goal({:.6f},{:.6f})",
    start_gps.latitude, start_gps.longitude, goal_gps.latitude, goal_gps.longitude);

  startQueryLeg(*vehicle_pose, start_gps);
  return true;
}

bool TripController::goToPickup() {
  if (status_.state_ != TripState::WAITING_FOR_PICKUP_START) {
    RCLCPP_ERROR(node_->get_logger(),
                 "[AutowareAgent] goToPickup() called in wrong state=%d — ignoring",
                 static_cast<int>(status_.state_));
    return false;
  }

  RCLCPP_INFO(node_->get_logger(),
              "[AutowareAgent] goToPickup — driving to pickup lane %s (%.2f, %.2f)",
              status_.start_lanelet_id_.c_str(), status_.start_x_, status_.start_y_);

  current_route_state_.reset();
  route_received_ = false;
  transitionTo(TripState::PUBLISHING_INITIAL_POSE);
  return true;
}

bool TripController::handleMoveCommand() {
  if (status_.state_ == TripState::WAITING_FOR_PICKUP_START) {
    return goToPickup();
  } else if (status_.state_ == TripState::WAITING_FOR_MOVE) {
    return move();
  }
  RCLCPP_WARN(node_->get_logger(), "[AutowareAgent] move command received in unexpected state=%d",
              static_cast<int>(status_.state_));
  return false;
}

bool TripController::startTrip() {
  // added

  engaging_for_pickup_ = true;

  RCLCPP_ERROR(
    node_->get_logger(), "[DEBUG startTrip] current_route_state_ before reset=%s",
    current_route_state_ ? std::to_string(current_route_state_->state).c_str() : "nullptr");

  current_route_state_.reset();

  RCLCPP_ERROR(
    node_->get_logger(), "[DEBUG startTrip] current_route_state_ after reset=%s",
    current_route_state_ ? std::to_string(current_route_state_->state).c_str() : "nullptr");

  if (!pickup_route_.has_value() || !trip_route_.has_value()) {
    RCLCPP_ERROR(node_->get_logger(),
                 "[AutowareAgent] startTrip — no routes in bank, call queryEta first");
    return false;
  }
  if (status_.state_ != TripState::ROUTES_READY && status_.state_ != TripState::IDLE) {
    RCLCPP_WARN(node_->get_logger(), "[AutowareAgent] startTrip rejected — state=%d",
                static_cast<int>(status_.state_));
    return false;
  }

  // const LaneInfo* start_lane = lanelet_map_.findNearestLane(pickup_route_->goal_gps_);
  const LaneInfo* start_lane = lanelet_map_.getLaneById(pickup_route_->goal_lane_id_);
  if (!start_lane) {
    RCLCPP_ERROR(node_->get_logger(), "[AutowareAgent] startTrip — cannot resolve pickup lane");
    transitionTo(TripState::FAILED);
    return false;
  }

  // assign start with pickup location
  status_.start_lanelet_id_ = std::to_string(start_lane->lane_id);
  status_.start_x_ = start_lane->local.x;
  status_.start_y_ = start_lane->local.y;
  status_.start_z_ = start_lane->local.z;
  status_.start_qw_ = start_lane->orientation.w;
  status_.start_qz_ = start_lane->orientation.z;
  status_.start_gps_ = pickup_route_->goal_gps_;

  // added
  RCLCPP_ERROR(node_->get_logger(),
               "[DEBUG startTrip] status_.start: lane=%s x=%.3f y=%.3f gps=(%.6f,%.6f)",
               status_.start_lanelet_id_.c_str(), status_.start_x_, status_.start_y_,
               status_.start_gps_.latitude, status_.start_gps_.longitude);

  // RCLCPP_INFO(node_->get_logger(),
  //             "[AutowareAgent] startTrip — driving to pickup lane %s (%.2f, %.2f)",
  //             status_.start_lanelet_id_.c_str(), status_.start_x_, status_.start_y_);
  // spdlog::info("[AutowareAgent] startTrip — pickup lane {} ({:.2f}, {:.2f})",
  //              status_.start_lanelet_id_, status_.start_x_, status_.start_y_);

  // transitionTo(TripState::PUBLISHING_INITIAL_POSE);
  // return true;

  RCLCPP_INFO(node_->get_logger(),
              "[AutowareAgent] startTrip — accepted, waiting for goToPickup() "
              "pickup lane %s (%.2f, %.2f)",
              status_.start_lanelet_id_.c_str(), status_.start_x_, status_.start_y_);

  transitionTo(TripState::WAITING_FOR_PICKUP_START);
  return true;
}

bool TripController::move() {
  if (status_.state_ != TripState::WAITING_FOR_MOVE) {
    RCLCPP_ERROR(node_->get_logger(), "[AutowareAgent] move() called in wrong state=%d — ignoring",
                 static_cast<int>(status_.state_));
    return false;
  }
  if (!trip_route_.has_value()) {
    RCLCPP_ERROR(node_->get_logger(), "[AutowareAgent] move() — trip route not in bank");
    transitionTo(TripState::FAILED);
    return false;
  }

  // const LaneInfo* goal_lane = lanelet_map_.findNearestLane(trip_route_->goal_gps_);
  // if (!goal_lane) {
  //   RCLCPP_ERROR(node_->get_logger(), "[AutowareAgent] move() — cannot resolve goal lane");
  //   transitionTo(TripState::FAILED);
  //   return false;
  // }

  lanelet::Id current_lane_id = std::stoll(status_.start_lanelet_id_);

  const LaneInfo* goal_lane = lanelet_map_.findNearestConnectedLane(
    trip_route_->goal_gps_, current_lane_id, /*must_be_reachable_from_ref=*/true);

  if (!goal_lane) {
    RCLCPP_ERROR(
      node_->get_logger(),
      "[AutowareAgent] move() — cannot resolve a goal lane reachable from current lane %ld",
      current_lane_id);
    transitionTo(TripState::FAILED);
    return false;
  }

  status_.goal_lanelet_id_ = std::to_string(goal_lane->lane_id);
  status_.goal_x_ = goal_lane->local.x;
  status_.goal_y_ = goal_lane->local.y;
  status_.goal_z_ = goal_lane->local.z;
  status_.goal_qw_ = goal_lane->orientation.w;
  status_.goal_qz_ = goal_lane->orientation.z;
  status_.goal_gps_ = trip_route_->goal_gps_;
  status_.goal_distance_m_ = trip_route_->distance_m_;

  RCLCPP_INFO(
    node_->get_logger(), "[AutowareAgent] move() — trip leg to lane %s (%.2f, %.2f) dist=%.1fm",
    status_.goal_lanelet_id_.c_str(), status_.goal_x_, status_.goal_y_, status_.goal_distance_m_);
  spdlog::info("[AutowareAgent] move() — lane {} ({:.2f}, {:.2f})", status_.goal_lanelet_id_,
               status_.goal_x_, status_.goal_y_);

  current_route_state_.reset();  // ADD THIS — track the new route's state fresh

  // added
  auto vehicle_pose = getCurrentVehiclePose();
  if (vehicle_pose) {
    RCLCPP_ERROR(node_->get_logger(), "[DEBUG move] vehicle current GPS = (%.6f, %.6f)",
                 vehicle_pose->latitude, vehicle_pose->longitude);
  } else {
    RCLCPP_ERROR(node_->get_logger(), "[DEBUG move] vehicle pose UNAVAILABLE (TF failed)");
  }

  // Publish the held trip-leg goal and engage
  doPublishGoal(status_.goal_x_, status_.goal_y_, status_.goal_z_, status_.goal_qz_,
                status_.goal_qw_);

  engaging_for_pickup_ = false;
  transitionTo(TripState::ENGAGING);
  return true;
}

void TripController::cancel() {
  if (status_.state_ != TripState::IDLE) {
    RCLCPP_INFO(node_->get_logger(), "[AutowareAgent] Trip cancelled");
    spdlog::info("[AutowareAgent] Trip cancelled");
  }
  querying_ = false;
  query_cb_ = nullptr;
  pickup_route_.reset();
  trip_route_.reset();
  status_ = TripStatus{};
  route_received_ = false;
  transitionTo(TripState::IDLE);
}

void TripController::tick() {
  switch (status_.state_) {
    case TripState::IDLE:
    case TripState::ROUTES_READY:
    case TripState::WAITING_FOR_MOVE:
    case TripState::COMPLETED:
    case TripState::CANCELLED:
    case TripState::FAILED:
      break;

    case TripState::INITIALIZING_IN_MAP:
      if (!current_loc_state_ ||
          current_loc_state_->state !=
            autoware_adapi_v1_msgs::msg::LocalizationInitializationState::INITIALIZED) {
        // Republish fixed start pose every second until Autoware localizes
        if (elapsed_ms(last_publish_time_) >= 1000) {
          doPublishInitialPose(init_x_, init_y_, init_z_, init_qz_, init_qw_);
          last_publish_time_ = std::chrono::steady_clock::now();
        }
      } else {
        RCLCPP_INFO(node_->get_logger(),
                    "[AutowareAgent] Vehicle localized in map — ready for queries");
        transitionTo(TripState::IDLE);
      }
      break;

    case TripState::QUERY_PUBLISHING_INITIAL_POSE:
      doPublishInitialPose(status_.start_x_, status_.start_y_, status_.start_z_, status_.start_qz_,
                           status_.start_qw_);
      transitionTo(TripState::QUERY_WAITING_LOCALISATION);
      break;

    case TripState::QUERY_WAITING_LOCALISATION:
      if (!current_loc_state_ ||
          current_loc_state_->state !=
            autoware_adapi_v1_msgs::msg::LocalizationInitializationState::INITIALIZED) {
        if (elapsed_ms(last_publish_time_) >= 1000) {
          doPublishInitialPose(status_.start_x_, status_.start_y_, status_.start_z_,
                               status_.start_qz_, status_.start_qw_);
          last_publish_time_ = std::chrono::steady_clock::now();
        }
        if (elapsed_ms(state_entered_at_) >= timings_.loc_timeout_ms) {
          failQuery("timeout waiting for localisation on " +
                    (current_leg_ == QueryLeg::PICKUP ? std::string("pickup") : "trip") + " leg");
        }
      } else {
        RCLCPP_INFO(node_->get_logger(), "[AutowareAgent] Query leg localised — publishing goal");
        transitionTo(TripState::QUERY_PUBLISHING_GOAL);
      }
      break;

    case TripState::QUERY_PUBLISHING_GOAL:
      doPublishGoal(status_.goal_x_, status_.goal_y_, status_.goal_z_, status_.goal_qz_,
                    status_.goal_qw_);
      route_received_ = false;
      current_route_state_.reset();

      // added
      spdlog::info("[TripController] goal pose: x={:.3f} y={:.3f} qz={:.4f} qw={:.4f}",
                   status_.goal_x_, status_.goal_y_, status_.goal_qz_, status_.goal_qw_);

      transitionTo(TripState::QUERY_WAITING_ROUTE);
      break;

    case TripState::QUERY_WAITING_ROUTE:
      if (!current_route_state_ ||
          current_route_state_->state != autoware_adapi_v1_msgs::msg::RouteState::SET) {
        if (elapsed_ms(last_publish_time_) >= 2000) {
          doPublishGoal(status_.goal_x_, status_.goal_y_, status_.goal_z_, status_.goal_qz_,
                        status_.goal_qw_);
          last_publish_time_ = std::chrono::steady_clock::now();

          // added
          spdlog::info("[TripController] route_state={}",
                       current_route_state_ ? current_route_state_->state : -1);
        }
        if (elapsed_ms(state_entered_at_) >= timings_.route_timeout_ms) {
          failQuery("timeout waiting for route on " +
                    (current_leg_ == QueryLeg::PICKUP ? std::string("pickup") : "trip") +
                    " leg (lanes " + status_.start_lanelet_id_ + " → " + status_.goal_lanelet_id_ +
                    " may not be connected)");
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
        RCLCPP_WARN(node_->get_logger(),
                    "[AutowareAgent] ETA topic did not arrive within %ld ms — using 0",
                    timings_.eta_settle_ms);
        eta_distance_m_ = 0.0;
        eta_seconds_ = 0.0;
        finishQueryLeg();
      }
      break;

    case TripState::PUBLISHING_INITIAL_POSE:
      // this intial pose is pickup not the vecile current pose wrong
      //  doPublishInitialPose(status_.start_x_, status_.start_y_, status_.start_z_,
      //  status_.start_qz_,
      //                       status_.start_qw_);
      doPublishInitialPose(init_x_, init_y_, init_z_, init_qz_, init_qw_);
      transitionTo(TripState::WAITING_LOCALISATION);
      break;

    case TripState::WAITING_LOCALISATION:
      if (!current_loc_state_ ||
          current_loc_state_->state !=
            autoware_adapi_v1_msgs::msg::LocalizationInitializationState::INITIALIZED) {
        if (elapsed_ms(last_publish_time_) >= 1000) {
          // this intial pose is pickup not the vecile current pose wrong
          // doPublishInitialPose(status_.start_x_, status_.start_y_, status_.start_z_,
          //                      status_.start_qz_, status_.start_qw_);
          doPublishInitialPose(init_x_, init_y_, init_z_, init_qz_, init_qw_);
          last_publish_time_ = std::chrono::steady_clock::now();
        }
      } else {
        RCLCPP_INFO(node_->get_logger(),
                    "[AutowareAgent] Localised at pickup — publishing goal and engaging");
        doPublishGoal(status_.start_x_, status_.start_y_, status_.start_z_, status_.start_qz_,
                      status_.start_qw_);
        transitionTo(TripState::ENGAGING);
      }
      break;

    case TripState::ENGAGING:
      if (elapsed_ms(last_publish_time_) >= 2000) {
        doEngage();
        last_publish_time_ = std::chrono::steady_clock::now();
      }
      break;

    case TripState::DRIVING_TO_PICKUP:
      if (current_route_state_ &&
          current_route_state_->state == autoware_adapi_v1_msgs::msg::RouteState::ARRIVED) {
        RCLCPP_INFO(node_->get_logger(), "[AutowareAgent] Arrived at pickup location (lane %s)",
                    status_.start_lanelet_id_.c_str());
        spdlog::info("[AutowareAgent] Arrived at pickup — lane {}", status_.start_lanelet_id_);

        if (on_pickup_arrived_) {
          on_pickup_arrived_();
        }

        // Reset so the trip leg's ARRIVED does not fire immediately
        current_route_state_.reset();
        transitionTo(TripState::WAITING_FOR_MOVE);
      }
      break;

    case TripState::RUNNING:
      if (current_route_state_ &&
          current_route_state_->state == autoware_adapi_v1_msgs::msg::RouteState::ARRIVED) {
        RCLCPP_INFO(node_->get_logger(), "[AutowareAgent] Arrived at destination (lane %s)",
                    status_.goal_lanelet_id_.c_str());
        spdlog::info("[AutowareAgent] Arrived at destination — lane {}", status_.goal_lanelet_id_);

        if (on_dropoff_arrived_) {
          on_dropoff_arrived_();
        }

        current_route_state_.reset();
        transitionTo(TripState::COMPLETED);
      }
      break;
  }
}

void TripController::startQueryLeg(GPSCoordinate from_gps, GPSCoordinate to_gps) {
  // const LaneInfo* from_lane = lanelet_map_.findNearestLane(from_gps);
  const LaneInfo* from_lane = nullptr;
  if (current_leg_ == QueryLeg::PICKUP) {
    from_lane = lanelet_map_.findNearestLane(from_gps);
  } else {
    from_lane = lanelet_map_.getLaneById(std::stoll(status_.goal_lanelet_id_));
  }

  if (!from_lane) {
    failQuery("cannot resolve lane for " +
              std::string(current_leg_ == QueryLeg::PICKUP ? "pickup origin" : "trip start"));
    return;
  }

  const LaneInfo* to_lane = lanelet_map_.findNearestConnectedLane(
    to_gps, static_cast<lanelet::Id>(from_lane->lane_id), true);

  if (!to_lane) {
    to_lane = lanelet_map_.findNearestLane(to_gps);
    failQuery("destination lane " + std::to_string(to_lane ? to_lane->lane_id : -1) +
              " not reachable from start lane " + std::to_string(from_lane->lane_id));
    return;
  }

  RCLCPP_ERROR(node_->get_logger(),
               "[DEBUG startQueryLeg] from_lane=%p lane_id=%ld   to_lane=%p lane_id=%ld",
               (void*)from_lane, from_lane->lane_id, (void*)to_lane, to_lane->lane_id);

  RCLCPP_ERROR(node_->get_logger(),
               "[DEBUG startQueryLeg] from_gps=(%.6f, %.6f) to_gps=(%.6f, %.6f)", from_gps.latitude,
               from_gps.longitude, to_gps.latitude, to_gps.longitude);

  status_.start_x_ = from_lane->local.x;
  status_.start_y_ = from_lane->local.y;
  status_.start_z_ = from_lane->local.z;
  status_.start_qw_ = from_lane->orientation.w;
  status_.start_qz_ = from_lane->orientation.z;
  status_.start_lanelet_id_ = std::to_string(from_lane->lane_id);

  status_.goal_x_ = to_lane->local.x;
  status_.goal_y_ = to_lane->local.y;
  status_.goal_z_ = to_lane->local.z;
  status_.goal_qw_ = to_lane->orientation.w;
  status_.goal_qz_ = to_lane->orientation.z;
  status_.goal_lanelet_id_ = std::to_string(to_lane->lane_id);
  status_.goal_gps_ = to_gps;

  // if (from_lane->lane_id == to_lane->lane_id) {

  //     LocalCoordinate goal_local = lanelet_map_.projectOntoLaneCenterline(to_gps,
  //     to_lane->lane_id); status_.goal_x_ = goal_local.x; status_.goal_y_ = goal_local.y;
  //     status_.goal_z_ = goal_local.z;
  //     status_.goal_qw_ = to_lane->orientation.w;
  //     status_.goal_qz_ = to_lane->orientation.z;
  //   } else {
  //     status_.goal_x_ = to_lane->local.x;
  //     status_.goal_y_ = to_lane->local.y;
  //     status_.goal_z_ = to_lane->local.z;
  //     status_.goal_qw_ = to_lane->orientation.w;
  //     status_.goal_qz_ = to_lane->orientation.z;
  //   }
  //    status_.goal_lanelet_id_ = std::to_string(to_lane->lane_id);
  //   status_.goal_gps_ = to_gps;

  RCLCPP_INFO(node_->get_logger(),
              "[LANE MATCH] From GPS(%.8f, %.8f) -> Lane %ld @ Local(%.2f, %.2f)",
              from_gps.latitude, from_gps.longitude, from_lane->lane_id, from_lane->local.x,
              from_lane->local.y);

  RCLCPP_INFO(node_->get_logger(),
              "[LANE MATCH] To GPS(%.8f, %.8f) -> Lane %ld @ Local(%.2f, %.2f)", to_gps.latitude,
              to_gps.longitude, to_lane->lane_id, to_lane->local.x, to_lane->local.y);

  LocalCoordinate raw = lanelet_map_.gpsToLocal(to_gps);
  double dx = to_lane->local.x - raw.x;
  double dy = to_lane->local.y - raw.y;
  status_.goal_distance_m_ = std::sqrt(dx * dx + dy * dy);

  RCLCPP_INFO(node_->get_logger(), "[AutowareAgent] Query %s leg: lane %s → lane %s",
              current_leg_ == QueryLeg::PICKUP ? "pickup" : "trip",
              status_.start_lanelet_id_.c_str(), status_.goal_lanelet_id_.c_str());

  transitionTo(TripState::QUERY_PUBLISHING_INITIAL_POSE);
}

void TripController::finishQueryLeg() {
  if (current_leg_ == QueryLeg::PICKUP) {
    query_result_.pickup_leg_ = LegEta{.success_ = true,
                                       .distance_m_ = eta_distance_m_,
                                       .eta_seconds_ = eta_seconds_,
                                       .start_lane_id_ = status_.start_lanelet_id_,
                                       .goal_lane_id_ = status_.goal_lanelet_id_};
    pickup_route_ =
      HeldRoute{.route_ = last_route_ ? *last_route_ : autoware_planning_msgs::msg::LaneletRoute{},
                .eta_seconds_ = eta_seconds_,
                .distance_m_ = eta_distance_m_,
                .goal_gps_ = query_start_gps_,  // pickup destination = trip start
                .goal_lane_id_ = std::stoll(status_.goal_lanelet_id_)};

    RCLCPP_INFO(node_->get_logger(),
                "[AutowareAgent] Pickup leg done — ETA %.1f s / %.1f m — starting trip leg",
                eta_seconds_, eta_distance_m_);
    spdlog::info("[AutowareAgent] Pickup leg done — {:.1f} s / {:.1f} m", eta_seconds_,
                 eta_distance_m_);

    current_leg_ = QueryLeg::TRIP;
    current_route_state_.reset();
    route_received_ = false;
    startQueryLeg(query_start_gps_, query_goal_gps_);

  } else {
    query_result_.trip_leg_ = LegEta{.success_ = true,
                                     .distance_m_ = eta_distance_m_,
                                     .eta_seconds_ = eta_seconds_,
                                     .start_lane_id_ = status_.start_lanelet_id_,
                                     .goal_lane_id_ = status_.goal_lanelet_id_};
    trip_route_ =
      HeldRoute{.route_ = last_route_ ? *last_route_ : autoware_planning_msgs::msg::LaneletRoute{},
                .eta_seconds_ = eta_seconds_,
                .distance_m_ = eta_distance_m_,
                .goal_gps_ = query_goal_gps_};

    RCLCPP_INFO(node_->get_logger(), "[AutowareAgent] Trip leg done — ETA %.1f s / %.1f m",
                eta_seconds_, eta_distance_m_);
    spdlog::info("[AutowareAgent] Trip leg done — {:.1f} s / {:.1f} m", eta_seconds_,
                 eta_distance_m_);

    query_result_.success_ = true;
    querying_ = false;

    if (query_cb_) {
      query_cb_(query_result_);
      query_cb_ = nullptr;
    }

    status_ = TripStatus{};
    transitionTo(TripState::ROUTES_READY);
  }
}

void TripController::failQuery(const std::string& reason) {
  RCLCPP_ERROR(node_->get_logger(), "[AutowareAgent] Query failed: %s", reason.c_str());
  spdlog::error("[AutowareAgent] Query failed: {}", reason);

  if (query_cb_) {
    query_result_.success_ = false;
    query_result_.error_message_ = reason;
    query_cb_(query_result_);
    query_cb_ = nullptr;
  }
  querying_ = false;
  pickup_route_.reset();
  trip_route_.reset();
  status_ = TripStatus{};
  route_received_ = false;
  transitionTo(TripState::IDLE);
}

void TripController::doPublishInitialPose(double x, double y, double z, double qz, double qw) {
  // added
  if (current_route_state_) {
    RCLCPP_ERROR(node_->get_logger(),
                 "[DEBUG doPublishInitialPose] route_state=%d state=%d x=%.2f y=%.2f",
                 current_route_state_->state, static_cast<int>(status_.state_), x, y);
  }

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

  msg.pose.covariance.fill(0.0);
  msg.pose.covariance[0] = 0.25;
  msg.pose.covariance[7] = 0.25;
  msg.pose.covariance[35] = 0.06853891909122467;

  initial_pose_pub_->publish(msg);
  RCLCPP_INFO(node_->get_logger(), "[AutowareAgent] Published initial pose (%.2f, %.2f)", x, y);
  spdlog::info("[AutowareAgent] Published initial pose ({:.2f}, {:.2f})", x, y);
}

void TripController::doPublishGoal(double x, double y, double z, double qz, double qw) {
  // added
  RCLCPP_ERROR(node_->get_logger(), "[DEBUG doPublishGoal] state=%d route_state=%d x=%.2f y=%.2f",
               static_cast<int>(status_.state_),
               current_route_state_ ? current_route_state_->state : -1, x, y);

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
  RCLCPP_INFO(node_->get_logger(), "[AutowareAgent] Published goal (%.2f, %.2f)", x, y);
  spdlog::info("[AutowareAgent] Published goal ({:.2f}, {:.2f})", x, y);
}

void TripController::doEngage() {
  if (!mode_client_->service_is_ready() || !engage_client_->service_is_ready()) {
    RCLCPP_WARN(node_->get_logger(), "[AutowareAgent] Engagement services not ready yet");
    return;
  }

  // ADD THIS: check if autonomous mode is available first
  if (current_mode_state_ && !current_mode_state_->is_autonomous_mode_available) {
    RCLCPP_WARN(node_->get_logger(),
                "[AutowareAgent] Autonomous mode not available yet — waiting for diagnostics");
    return;  // tick() will call doEngage() again next cycle
  }

  auto mode_req = std::make_shared<autoware_adapi_v1_msgs::srv::ChangeOperationMode::Request>();

  mode_client_->async_send_request(
    mode_req,
    [this](rclcpp::Client<autoware_adapi_v1_msgs::srv::ChangeOperationMode>::SharedFuture f) {
      const auto& resp = f.get();
      if (!resp->status.success) {
        RCLCPP_ERROR(node_->get_logger(), "[AutowareAgent] Mode change failed: %s",
                     resp->status.message.c_str());
        return;
      }
      RCLCPP_INFO(node_->get_logger(), "[AutowareAgent] Mode → Autonomous. Sending Engage...");

      auto engage_req = std::make_shared<tier4_external_api_msgs::srv::Engage::Request>();
      engage_req->engage = true;

      engage_client_->async_send_request(
        engage_req, [this](rclcpp::Client<tier4_external_api_msgs::srv::Engage>::SharedFuture ef) {
          const auto& er = ef.get();
          if (er->status.code == 1) {
            RCLCPP_INFO(node_->get_logger(), "[AutowareAgent] Engaged!");
            boost::asio::post(*strand_, [this]() {
              if (status_.state_ == TripState::ENGAGING) {
                // trip_route_ still in bank → we are on the pickup leg
                // trip route here is always true this will not distinguish pickup or trip
                if (engaging_for_pickup_) {
                  transitionTo(TripState::DRIVING_TO_PICKUP);
                } else {
                  transitionTo(TripState::RUNNING);
                }
              }
            });
          } else {
            RCLCPP_ERROR(node_->get_logger(), "[AutowareAgent] Engage failed: %s",
                         er->status.message.c_str());
          }
        });
    });
}

void TripController::onRouteReceived(const autoware_planning_msgs::msg::LaneletRoute& msg) {
  last_route_ = std::make_shared<autoware_planning_msgs::msg::LaneletRoute>(msg);
  route_received_ = true;

  RCLCPP_INFO(
    node_->get_logger(), "[AutowareAgent] Route received — %zu segment(s), start primitive id: %ld",
    msg.segments.size(),
    msg.segments.empty() ? -1L : static_cast<long>(msg.segments[0].preferred_primitive.id));
  spdlog::info("[AutowareAgent] Route received — {} segment(s)", msg.segments.size());
}

void TripController::injectEta(double distance_m, double time_seconds) {
  if (status_.state_ != TripState::QUERY_READING_ETA)
    return;
  eta_distance_m_ = distance_m;
  eta_seconds_ = time_seconds;
  eta_received_ = true;
  RCLCPP_INFO(node_->get_logger(), "[AutowareAgent] ETA injected — %.1f m / %.1f s", distance_m,
              time_seconds);
}

void TripController::setStateChangeCallback(StateChangeCb cb) {
  on_state_change_ = std::move(cb);
}
void TripController::setArrivalCallbacks(ArrivalCb pickup_cb, ArrivalCb dropoff_cb) {
  on_pickup_arrived_ = std::move(pickup_cb);
  on_dropoff_arrived_ = std::move(dropoff_cb);
}

TripStatus TripController::status() const {
  return status_;
}

void TripController::setCurrentGps(const GPSCoordinate& gps) {
  status_.current_gps_ = gps;
}

std::optional<GPSCoordinate> TripController::getCurrentVehiclePose() const {
  try {
    auto tf = tf_buffer_->lookupTransform("map", "base_link", tf2::TimePointZero);

    //  RCLCPP_ERROR(node_->get_logger(),
    //   "[DEBUG TF] base_link in map frame: x=%.3f y=%.3f z=%.3f",
    //   tf.transform.translation.x,
    //   tf.transform.translation.y,
    //   tf.transform.translation.z);

    LocalCoordinate const local{.x = tf.transform.translation.x,
                                .y = tf.transform.translation.y,
                                .z = tf.transform.translation.z};

    auto gps = lanelet_map_.localToGps(local);

    //                                RCLCPP_ERROR(node_->get_logger(),
    // "[DEBUG TF→GPS] lat=%.6f lon=%.6f",
    // gps.latitude, gps.longitude);
    if (gps.latitude < 35.0 || gps.latitude > 36.0 || gps.longitude < 139.0 ||
        gps.longitude > 140.0) {
      RCLCPP_WARN(node_->get_logger(),
                  "[AutowareAgent] TF→GPS out of Nishishinjuku range (%.6f, %.6f) — ignoring",
                  gps.latitude, gps.longitude);
      return std::nullopt;  // caller will use fixed start instead
    }

    return lanelet_map_.localToGps(local);
  } catch (const tf2::TransformException& ex) {
    RCLCPP_WARN(node_->get_logger(), "[AutowareAgent] TF lookup failed: %s", ex.what());
    return std::nullopt;
  }
}

long TripController::elapsed_ms(std::chrono::steady_clock::time_point since) {
  return std::chrono::duration_cast<std::chrono::milliseconds>(std::chrono::steady_clock::now() -
                                                               since)
    .count();
}

void TripController::transitionTo(TripState next) {
  TripState const prev = status_.state_;
  if (prev == next)
    return;
  status_.state_ = next;
  status_.last_state_change_ = std::chrono::steady_clock::now();
  state_entered_at_ = status_.last_state_change_;
  last_publish_time_ = std::chrono::steady_clock::time_point{};
  if (on_state_change_) {
    on_state_change_(prev, next);
  }
}

}  // namespace autoware_agent