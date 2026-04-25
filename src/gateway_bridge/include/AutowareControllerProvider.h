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
#include "Providers.h"
#include "TripStatus.h"

#include <memory>
#include <string>

namespace vehicle_gateway {

class AutowareControllerTripAdapter : public ITripManager {
 public:
  explicit AutowareControllerTripAdapter(
    std::shared_ptr<autoware_agent::AutowareController> controller)
    : controller_(std::move(controller))
    , active_trip_id_(0) {}

  TripInitData GetTripInitData() override {
    auto s = controller_->getTripStatusSync();
    return TripInitData{.request_id = active_trip_id_.load(),
                        .start_long = s.start_gps_.longitude,
                        .start_lat = s.start_gps_.latitude,
                        .end_long = s.goal_gps_.longitude,
                        .end_lat = s.goal_gps_.latitude};
  }

  TripMoveData GetTripMoveData() override {
    auto s = controller_->getTripStatusSync();
    return TripMoveData{.trip_id = active_trip_id_.load(),
                        .longitude = s.current_gps_.longitude,
                        .latitude = s.current_gps_.latitude};
  }

  ArriveData GetArriveData() override {
    auto s = controller_->getTripStatusSync();
    return ArriveData{.trip_id = active_trip_id_.load(),
                      .longitude = s.current_gps_.longitude,
                      .latitude = s.current_gps_.latitude};
  }

  std::string GetCurrentStatus() override {
    auto s = controller_->getTripStatusSync();
    switch (s.state_) {
      case ::TripState::IDLE:
        return "idle";

        // Query pipeline vehicle is not moving, just planning
      case ::TripState::QUERY_PUBLISHING_INITIAL_POSE:
      case ::TripState::QUERY_WAITING_LOCALISATION:
      case ::TripState::QUERY_PUBLISHING_GOAL:
      case ::TripState::QUERY_WAITING_ROUTE:
      case ::TripState::QUERY_READING_ETA:
        return "planning";

        // Routes computed, waiting for server to send TripInit RPC
      case ::TripState::ROUTES_READY:
        return "routes_ready";

        // Localising at pickup point
      case ::TripState::PUBLISHING_INITIAL_POSE:
      case ::TripState::WAITING_LOCALISATION:
        return "preparing";

        // Engaging
      case ::TripState::ENGAGING:
        return "accepted";

        // Driving to pickup location
      case ::TripState::DRIVING_TO_PICKUP:
        return "driving_to_pickup";

        // At pickup, waiting for server TripMove RPC
      case ::TripState::WAITING_FOR_MOVE:
        return "waiting_for_move";

        // Trip leg active
      case ::TripState::RUNNING:
        return "in_progress";

      case ::TripState::COMPLETED:
        return "completed";

      case ::TripState::CANCELLED:
        return "cancelled";

      case ::TripState::FAILED:
        return "error";

      default:
        return "unknown";
    }
  }

  int64_t GetActiveTripId() override {
    return active_trip_id_.load();
  }

  void SetActiveTripId(int64_t id) override {
    active_trip_id_.store(id);
  }

 private:
  std::shared_ptr<autoware_agent::AutowareController> controller_;
  std::atomic<int64_t> active_trip_id_;
};
}  // namespace vehicle_gateway

#endif  // VEHICLE_AUTOWARE_AGENT_AUTOWARECONTROLLERADAPTER_H
