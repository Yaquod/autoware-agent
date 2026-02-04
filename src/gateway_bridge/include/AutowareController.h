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

#include <rclcpp/rclcpp.hpp>

#include "TripController.h"
#include "TripStatus.h"

namespace AutowareAgent {

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
  AutowareController(const std::string& map_path, double tick_hz = 10.0);

  ~AutowareController() = default;

  bool startTrip(double latitude, double longitude);

  void cancelTrip();

  TripStatus getTripStatus() const;

 private:
  std::unique_ptr<RouteConfig> route_config_;
  std::unique_ptr<TripController> trip_ctrl_;
  rclcpp::TimerBase::SharedPtr tick_timer_;

  void onTickTimer();

  void onTripStateChanged(TripState prev, TripState next);
};
}  // namespace AutowareAgent
#endif  // VEHICLEAUTOWAREAGENT_AUTOWARECONTROLLER_H
