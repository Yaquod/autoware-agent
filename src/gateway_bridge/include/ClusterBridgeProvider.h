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

#ifndef VEHICLE_AUTOWARE_AGENT_CLUSTERBRIDGEPROVIDER_H
#define VEHICLE_AUTOWARE_AGENT_CLUSTERBRIDGEPROVIDER_H

#include "FrameStates.h"
#include "Providers.h"

#include <mutex>

namespace vehicle_gateway {

class ClusterEtaAdapter : public IEtaProvider {
 public:
  ClusterEtaAdapter(const FrameState& state, std::mutex& state_mtx, int64_t& request_id)
    : state_(state)
    , mtx_(state_mtx)
    , request_id_(request_id) {}

  EtaData GetEta() override {
    std::lock_guard<std::mutex> lock(mtx_);
    return EtaData{
      .request_id = request_id_,
      .time_seconds = static_cast<double>(state_.remaining_time_s),
      .fare = 0.0  // Cloud should takeover pricing

    };
  }

 private:
  const FrameState& state_;
  std::mutex& mtx_;
  int64_t& request_id_;
};

class ClusterLocationAdapter : public ILocationProvider {
 public:
  ClusterLocationAdapter(const FrameState& state, std::mutex& state_mtx)
    : state_(state)
    , mtx_(state_mtx) {}

  LocationData GetCurrentLocation() override {
    std::lock_guard<std::mutex> lock(mtx_);
    return LocationData{.longitude = state_.longitude, .latitud = state_.latitude};
  }

 private:
  const FrameState& state_;
  std::mutex& mtx_;
};
}  // namespace vehicle_gateway
#endif  // VEHICLE_AUTOWARE_AGENT_CLUSTERBRIDGEPROVIDER_H
