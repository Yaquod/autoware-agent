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


#ifndef VEHICLE_AUTOWARE_AGENT_AUTOWARECONTROLLERADAPTER_H
#define VEHICLE_AUTOWARE_AGENT_AUTOWARECONTROLLERADAPTER_H

#include "AutowareController.h"
#include "TripStatus.h"
#include "Providers.h"

#include <memory>
#include <string>

namespace vehicle_gateway {

class AutowareControllerTripAdapter : public ITripManager {
public:
  explicit AutowareControllerTripAdapter(
      std::shared_ptr<autoware_agent::AutowareController> controller)
      : controller_(std::move(controller)) {}

  TripInitData GetTripInitData() override {
    auto s = controller_->getTripStatusSync();
    return TripInitData{
      .request_id = 0,
      .start_long = s.start_x,
      .start_lat  = s.start_y,
      .end_long   = s.goal_gps.longitude,
      .end_lat    = s.goal_gps.latitude
    };
  }

  TripMoveData GetTripMoveData() override {
    auto s = controller_->getTripStatusSync();
    return TripMoveData{
      .trip_id   = 0,
      .longitude = 0.0,
      .latitude  = 0.0
    };
  }

  ArriveData GetArriveData() override {
    auto s = controller_->getTripStatusSync();
    return ArriveData{
      .trip_id   = 0,
      .longitude = s.goal_gps.longitude,
      .latitude  = s.goal_gps.latitude
    };
  }

  std::string GetCurrentStatus() override {
    auto s = controller_->getTripStatusSync();
    switch (s.state) {
      case ::TripState::IDLE:      return "idle";
      case ::TripState::PUBLISHING_INITIAL_POSE:
      case ::TripState::WAITING_LOCALISATION:
      case ::TripState::QUERY_PUBLISHING_GOAL:
      case ::TripState::QUERY_WAITING_ROUTE: return "preparing";
      case ::TripState::ENGAGING:  return "accepted";
      case ::TripState::RUNNING:   return "in_progress";
      case ::TripState::COMPLETED: return "completed";
      case ::TripState::CANCELLED: return "cancelled";
      case ::TripState::FAILED:    return "error";
      default:                     return "unknown";
    }
  }

  int64_t GetActiveTripId() override {
    return 0;
  }

private:
  std::shared_ptr<autoware_agent::AutowareController> controller_;
};
}  // namespace vehicle_gateway

#endif  // VEHICLE_AUTOWARE_AGENT_AUTOWARECONTROLLERADAPTER_H
