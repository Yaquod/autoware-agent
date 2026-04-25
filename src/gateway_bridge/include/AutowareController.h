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

#ifndef VEHICLEAUTOWAREAGENT_AUTOWARECONTROLLER_H
#define VEHICLEAUTOWAREAGENT_AUTOWARECONTROLLER_H

#include "TripController.h"
#include "TripStatus.h"

#include <rclcpp/rclcpp.hpp>

#include <boost/asio/executor_work_guard.hpp>
#include <boost/asio/io_context.hpp>
#include <boost/asio/strand.hpp>

namespace autoware_agent {

using RouteQueryCallback = TripController::EtaQueryCallback;
using RouteQueryResult = EtaQueryResult;

/**
 * @brief Responsibilities:
 * Load the YAML route config once at boot.
 * Expose startTrip / cancelTrip / getTripStatus (called by gRPC or CLI).
 * Run a periodic timer that drives TripController::tick().
 * No domain logic lives here.  RouteConfig owns the map data;
 * TripController owns the state machine.
 */
class AutowareController : public rclcpp::Node {
 public:
  explicit AutowareController(const std::string& map_path, double tick_hz = 10.0);

  ~AutowareController() override;

  enum class ArrivalKind { PICKUP, DROPOFF };
  using ArrivalCallback = std::function<void(ArrivalKind)>;

  void initialize();

  void queryEta(GPSCoordinate start_gps, GPSCoordinate goal_gps, RouteQueryCallback callback);

  void startTrip(const std::function<void(bool)>& callback);

  void move(const std::function<void(bool)>& callback);

  void cancelTrip();

  void getTripStatus(std::function<void(TripStatus)> callback) const;

  TripStatus getTripStatusSync() const;

  void setTripStateCallback(std::function<void(TripState, TripState)> cb);

  void setArrivalCallback(ArrivalCallback cb);

 private:
  std::unique_ptr<RouteConfig> route_config_;
  std::unique_ptr<TripController> trip_ctrl_;
  rclcpp::TimerBase::SharedPtr tick_timer_;
  double tick_hz_;
  std::shared_ptr<boost::asio::io_context> io_context_;
  std::shared_ptr<boost::asio::io_context::strand> strand_;
  std::unique_ptr<boost::asio::executor_work_guard<boost::asio::io_context::executor_type>>
    work_guard_;
  std::thread io_thread_;
  std::function<void(TripState, TripState)> trip_state_callback_;
  ArrivalCallback arrival_callback_;

  void onTickTimer();

  void onTripStateChanged(TripState prev, TripState next);

  void startTripImpl(std::function<void(bool)> callback);

  void moveImpl(std::function<void(bool)> callback);

  void getTripStatusImpl(std::function<void(TripStatus)> callback) const;

  void cancelTripImpl();
};
}  // namespace autoware_agent
#endif  // VEHICLEAUTOWAREAGENT_AUTOWARECONTROLLER_H
