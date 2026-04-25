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

#ifndef VEHICLE_AUTOWARE_AGENT_PROVIDERS_H
#define VEHICLE_AUTOWARE_AGENT_PROVIDERS_H
#include <string>

struct EtaData {
  int64_t request_id{0};
  double time_seconds{0.0};  // remaining seconds from MissionRemainingDistanceTime
  double fare{0.0};          // computed externally; 0.0 if not yet known
};

struct LocationData {
  double longitude{0.0};
  double latitud{0.0};
};

struct TripInitData {
  int64_t request_id{0};
  double start_long{0.0};
  double start_lat{0.0};
  double end_long{0.0};
  double end_lat{0.0};
};

struct TripMoveData {
  int64_t trip_id{0};
  double longitude{0.0};
  double latitude{0.0};
};

struct ArriveData {
  int64_t trip_id{0};
  double longitude{0.0};
  double latitude{0.0};
};

class IEtaProvider {
 public:
  virtual ~IEtaProvider() = default;

  virtual EtaData GetEta() = 0;
};

class ILocationProvider {
 public:
  virtual ~ILocationProvider() = default;

  virtual LocationData GetCurrentLocation() = 0;
};

class ITripManager {
 public:
  virtual ~ITripManager() = default;

  /**
   * Reads start/end coordinates and request_id from the current
   * TripStatus (populated when AutowareController::startTrip() is called).
   */
  virtual TripInitData GetTripInitData() = 0;

  /**
   * Returns the current vehicle position *during* an active trip.
   * Typically delegates to ILocationProvider internally, or reads from
   * the same odometry state that ClusterBridge maintains.
   */
  virtual TripMoveData GetTripMoveData() = 0;

  /**
   * Returns the position snapshot at the moment the trip ended.
   * Should be the destination coordinates stored in TripStatus.
   */
  virtual ArriveData GetArriveData() = 0;

  /**
   * Maps the current TripState enum value to a server status string.
   * E.g. TripState::DRIVING → "in_progress",
   *      TripState::ARRIVED → "completed".
   */
  virtual std::string GetCurrentStatus() = 0;

  /** Server-side trip ID assigned at TripInit response time. */
  virtual int64_t GetActiveTripId() = 0;
  virtual void SetActiveTripId(int64_t id) = 0;
};

#endif  // VEHICLE_AUTOWARE_AGENT_PROVIDERS_H
