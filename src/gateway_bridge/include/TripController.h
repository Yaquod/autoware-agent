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

#include "LaneletMap.h"
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
#include "autoware_adapi_v1_msgs/srv/set_route_points.hpp"
#include <rclcpp/rclcpp.hpp>

#include <boost/asio.hpp>

#include <chrono>
#include <functional>
#include <memory>

#include <tf2_ros/buffer.h>
#include <tf2_ros/transform_listener.h>
#include <tier4_external_api_msgs/srv/engage.hpp>

namespace autoware_agent {

class TripController {
 public:
  TripController(rclcpp::Node::SharedPtr node, const LaneletMap& route_config,
                 std::shared_ptr<boost::asio::io_context::strand> strand,
                 TripTimings timings = TripTimings{});

  using EtaQueryCallback = std::function<void(EtaQueryResult)>;
  using StateChangeCb = std::function<void(TripState, TripState)>;
  using ArrivalCb = std::function<void()>;

  /**
   * @brief Pops the pickup route from the bank, localises the vehicle at start_gps,
   * and engages.  When the vehicle arrives.  Requires queryEta() to have been called first to
   * populate the route bank.
   * @return Returns success or failure.
   */
  bool startTrip();

  void cancel();

  void tick();

  void initializeInMap();

  [[nodiscard]] TripStatus status() const;

  void setStateChangeCallback(StateChangeCb cb);

  void setArrivalCallbacks(ArrivalCb pickup_cb, ArrivalCb dropoff_cb);

  /**
   * @brief Drives the localisation, goal-publishing pipeline up to WAITING_ROUTE, waits for
   * Autoware to confirm the route is SET, then reads segment data to compute an ETA.
   * @param start_gps GPS coordinate for localising the vehicle at the start of the trip.
   * @param goal_gps Goal set for destination need to go.
   * @param cb A callback for the full Query data.
   * @return Returns success or failure.
   */
  bool queryEta(GPSCoordinate start_gps, GPSCoordinate goal_gps, EtaQueryCallback cb);

  /**
   * @brief Called when the TripMove RPC arrives.  Pops the trip-leg route from
   * the bank and engages to drive to the final goal.
   * @return Returns success or failure.
   */
  bool move();

  /**
   * @brief ETA injection called by ClusterBridge Impl.
   * @param distance_m Distance to be moved.
   * @param time_seconds Time that car needs to reach goal.
   */
  void injectEta(double distance_m, double time_seconds);

  [[nodiscard]] std::optional<GPSCoordinate> getCurrentVehiclePose() const;

  void setCurrentGps(const GPSCoordinate& gps);

 private:
  rclcpp::Node::SharedPtr node_;
  const LaneletMap& lanelet_map_;
  std::shared_ptr<boost::asio::io_context::strand> strand_;
  TripTimings timings_;
  std::thread io_thread_;
  double assumed_speed_ms_{8.33};  // ~30 km/h

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
  autoware_planning_msgs::msg::LaneletRoute::SharedPtr last_route_;

  std::shared_ptr<tf2_ros::Buffer> tf_buffer_;
  std::shared_ptr<tf2_ros::TransformListener> tf_listener_;

  std::chrono::steady_clock::time_point last_publish_time_;

  TripStatus status_;
  StateChangeCb on_state_change_;
  ArrivalCb on_pickup_arrived_;
  ArrivalCb on_dropoff_arrived_;
  std::chrono::steady_clock::time_point state_entered_at_;
  bool route_received_ = false;

  enum class QueryLeg { PICKUP, TRIP };

  double init_x_{0.0};
  double init_y_{0.0};
  double init_z_{0.0};
  double init_qz_{0.0};
  double init_qw_{1.0};

  bool querying_ = false;
  QueryLeg current_leg_ = QueryLeg::PICKUP;
  GPSCoordinate query_start_gps_{.latitude = 0.0, .longitude = 0.0};
  GPSCoordinate query_goal_gps_{.latitude = 0.0, .longitude = 0.0};
  EtaQueryCallback query_cb_;
  EtaQueryResult query_result_;

  bool eta_received_ = false;
  double eta_distance_m_ = 0.0;
  double eta_seconds_ = 0.0;

  std::optional<HeldRoute> pickup_route_;
  std::optional<HeldRoute> trip_route_;

  void doPublishInitialPose(double x, double y, double z, double qz, double qw);
  void doPublishGoal(double x, double y, double z, double qz, double qw);
  void doEngage();

  void onRouteReceived(const autoware_planning_msgs::msg::LaneletRoute& msg);

  void transitionTo(TripState next);
  void startQueryLeg(GPSCoordinate from_gps, GPSCoordinate to_gps);
  void finishQueryLeg();
  void failQuery(const std::string& reason);

  static long elapsed_ms(std::chrono::steady_clock::time_point since);
};

}  // namespace autoware_agent

#endif  // VEHICLEAUTOWAREAGENT_TRIPCONTROLLER_H