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
#include "map_routes/RouteConfig.h"

struct TripStatus {
  TripState state{TripState::IDLE};
  std::string start_lanelet_id;
  double start_x{0.0}, start_y{0.0}, start_z{0.0};
  double start_qw{0.0}, start_qz{0.0};
  AutowareAgent::GPSCoordinate goal_gps{0.0, 0.0};
  std::string goal_lanelet_id;
  double goal_x{0.0}, goal_y{0.0}, goal_z{0.0};
  double goal_qw{0.0}, goal_qz{0.0};
  double goal_distance_m{0.0};
  std::chrono::steady_clock::time_point trip_started_at;
  std::chrono::steady_clock::time_point last_state_change;

};

#endif  // VEHICLEAUTOWAREAGENT_TRIPSTATUS_H