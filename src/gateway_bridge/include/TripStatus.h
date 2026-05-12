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

#ifndef VEHICLEAUTOWAREAGENT_TRIPSTATUS_H
#define VEHICLEAUTOWAREAGENT_TRIPSTATUS_H

#include "TripStates.h"
#include "map_routes/LaneletMap.h"

#include <autoware_planning_msgs/msg/lanelet_route.hpp>

#include <chrono>
namespace autoware_agent {

struct TripStatus {
  TripState state_{TripState::IDLE};

  std::string start_lanelet_id_;
  double start_x_{0.0}, start_y_{0.0}, start_z_{0.0};
  double start_qw_{0.0}, start_qz_{0.0};

  GPSCoordinate start_gps_{.latitude = 0.0, .longitude = 0.0};

  GPSCoordinate goal_gps_{.latitude = 0.0, .longitude = 0.0};
  std::string goal_lanelet_id_;
  double goal_x_{0.0}, goal_y_{0.0}, goal_z_{0.0};
  double goal_qw_{0.0}, goal_qz_{0.0};
  double goal_distance_m_{0.0};

  GPSCoordinate current_gps_{.latitude = 0.0, .longitude = 0.0};

  std::chrono::steady_clock::time_point trip_started_at_;
  std::chrono::steady_clock::time_point last_state_change_;
};

struct LegEta {
  bool success_{false};
  double distance_m_{0.0};
  double eta_seconds_{0.0};
  std::string start_lane_id_;
  std::string goal_lane_id_;
  std::string error_message_;
};

struct EtaQueryResult {
  bool success_{false};
  LegEta pickup_leg_;
  LegEta trip_leg_;
  std::string error_message_;
};

struct HeldRoute {
  autoware_planning_msgs::msg::LaneletRoute route_;
  double eta_seconds_ = 0.0;
  double distance_m_ = 0.0;
  GPSCoordinate goal_gps_{.latitude = 0.0, .longitude = 0.0};
};
}  // namespace autoware_agent
#endif  // VEHICLEAUTOWAREAGENT_TRIPSTATUS_H