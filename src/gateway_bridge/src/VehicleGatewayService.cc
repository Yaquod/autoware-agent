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

#include "VehicleGatewayService.h"

#include <boost/asio/steady_timer.hpp>

#include <spdlog/spdlog.h>

namespace vehicle_gateway {

VehicleGatewayService::VehicleGatewayService(const std::string& target_vm,
                                             std::shared_ptr<IEtaProvider> eta_provider,
                                             std::shared_ptr<ILocationProvider> location_provider,
                                             std::shared_ptr<ITripManager> trip_manager,
                                             const std::string& vin_number,
                                             boost::asio::io_context& retry_io)
  : client_(target_vm)
  , eta_provider_(std::move(eta_provider))
  , location_provider_(std::move(location_provider))
  , trip_manager_(std::move(trip_manager))
  , vin_number_(vin_number)
  , retry_io_(retry_io) {}

VehicleGatewayService::~VehicleGatewayService() {
  Shutdown();
}

void VehicleGatewayService::Shutdown() {
  client_.ShutDown();
}

void VehicleGatewayService::ScheduleRetry(std::chrono::milliseconds delay,
                                          std::function<void()> fn) {
  auto timer = std::make_shared<boost::asio::steady_timer>(retry_io_, delay);
  timer->async_wait([timer, fn = std::move(fn)](const boost::system::error_code& ec) {
    (void)timer;
    if (!ec)
      fn();
  });
}

void VehicleGatewayService::ReportTripInit() {
  StartTrip();
}

void VehicleGatewayService::StartTrip() {
  struct State {
    TripInitRequest request;
    uint32_t attempts_left;
    std::chrono::milliseconds delay;
  };

  auto state = std::make_shared<State>();
  TripInitData data = trip_manager_->GetTripInitData();

  state->request.set_vin_number(vin_number_);
  state->request.set_request_id(data.request_id);
  state->request.set_start_lat(data.start_lat);
  state->request.set_start_long(data.start_long);
  state->request.set_end_lat(data.end_lat);
  state->request.set_end_long(data.end_long);
  state->attempts_left = 3;
  state->delay = std::chrono::milliseconds(500);

  spdlog::info(
    "[GatewayService] TripInit — request_id={} start=({:.6f},{:.6f}) "
    "end=({:.6f},{:.6f})",
    data.request_id, data.start_lat, data.start_long, data.end_lat, data.end_long);

  auto attempt = std::make_shared<std::function<void()>>();
  *attempt = [this, state, attempt]() {
    client_.TripInit(state->request, [this, state, attempt](bool ok,
                                                            const TripInitResponse& response) {
      if (ok) {
        // Server acks the call — trip_id comes later via MQTT TripMove.
        spdlog::info("[GatewayService] TripInit ack received (success={}): {}", response.success(),
                     response.message());
      } else {
        if (state->attempts_left > 0) {
          state->attempts_left--;
          state->delay *= 2;
          spdlog::warn("[GatewayService] TripInit failed, retrying in {}ms", state->delay.count());
          ScheduleRetry(state->delay, *attempt);
        } else {
          spdlog::error("[GatewayService] TripInit failed after all retries");
        }
      }
    });
  };
  (*attempt)();
}

void VehicleGatewayService::ReportArrive() {
  struct State {
    ArriveRequest request;
    uint32_t attempts_left;
    std::chrono::milliseconds delay;
  };

  auto state = std::make_shared<State>();
  ArriveData data = trip_manager_->GetArriveData();

  state->request.set_vin_number(vin_number_);
  state->request.set_trip_id(data.trip_id);
  state->request.set_lat(data.latitude);
  state->request.set_long_(data.longitude);
  state->attempts_left = 3;
  state->delay = std::chrono::milliseconds(500);

  spdlog::info("[GatewayService] Arrive — trip_id={} pos=({:.6f},{:.6f})", data.trip_id,
               data.latitude, data.longitude);

  auto attempt = std::make_shared<std::function<void()>>();
  *attempt = [this, state, attempt]() {
    client_.SendArrive(state->request,
                       [this, state, attempt](bool ok, const ArriveResponse& /*resp*/) {
                         if (ok) {
                           spdlog::info("[GatewayService] Arrive reported OK");
                         } else {
                           if (state->attempts_left > 0) {
                             state->attempts_left--;
                             state->delay *= 2;
                             ScheduleRetry(state->delay, *attempt);
                           } else {
                             spdlog::error("[GatewayService] Arrive failed after all retries");
                           }
                         }
                       });
  };
  (*attempt)();
}

void VehicleGatewayService::MoveTrip() {
  struct State {
    TripMoveRequest request;
    uint32_t attempts_left;
    std::chrono::milliseconds delay;
  };

  auto state = std::make_shared<State>();
  TripMoveData data = trip_manager_->GetTripMoveData();

  state->request.set_vin_number(vin_number_);
  state->request.set_trip_id(data.trip_id);
  state->request.set_latitude(data.latitude);
  state->request.set_longitude(data.longitude);
  state->attempts_left = 3;
  state->delay = std::chrono::milliseconds(500);

  spdlog::info("[GatewayService] TripMove — trip_id={} pos=({:.6f},{:.6f})", data.trip_id,
               data.latitude, data.longitude);

  auto attempt = std::make_shared<std::function<void()>>();
  *attempt = [this, state, attempt]() {
    client_.TripMove(state->request,
                     [this, state, attempt](bool ok, const TripMoveResponse& /*resp*/) {
                       if (ok) {
                         spdlog::info("[GatewayService] TripMove OK");
                       } else {
                         if (state->attempts_left > 0) {
                           state->attempts_left--;
                           state->delay *= 2;
                           ScheduleRetry(state->delay, *attempt);
                         } else {
                           spdlog::error("[GatewayService] TripMove failed after all retries");
                         }
                       }
                     });
  };
  (*attempt)();
}

void VehicleGatewayService::ReportAccepted() {
  ReportStatus("accepted");
}
void VehicleGatewayService::ReportDriving() {
  auto s = trip_manager_->GetCurrentStatus();
  ReportStatus(s.empty() ? "in_progress" : s);
}
void VehicleGatewayService::ReportCompleted() {
  ReportStatus("completed");
}
void VehicleGatewayService::ReportStatus() {
  ReportStatus(trip_manager_->GetCurrentStatus());
}

void VehicleGatewayService::ReportStatus(const std::string& status) {
  struct State {
    StatusRequest request;
    uint32_t attempts_left;
    std::chrono::milliseconds delay;
  };

  auto state = std::make_shared<State>();
  state->request.set_vin_number(vin_number_);
  state->request.set_trip_id(trip_manager_->GetActiveTripId());
  state->request.set_status(status);
  state->attempts_left = 3;
  state->delay = std::chrono::milliseconds(500);

  auto attempt = std::make_shared<std::function<void()>>();
  *attempt = [this, state, attempt]() {
    client_.SendStatus(state->request,
                       [this, state, attempt](bool ok, const StatusResponse& /*resp*/) {
                         if (!ok) {
                           if (state->attempts_left > 0) {
                             state->attempts_left--;
                             state->delay *= 2;
                             ScheduleRetry(state->delay, *attempt);
                           } else {
                             spdlog::error("[GatewayService] Status '{}' failed after all retries",
                                           state->request.status());
                           }
                         }
                       });
  };
  (*attempt)();
}

void VehicleGatewayService::ReportEta() {
  struct State {
    EtaRequest request;
    uint32_t attempts_left;
    std::chrono::milliseconds delay;
  };

  auto state = std::make_shared<State>();
  EtaData data = eta_provider_->GetEta();

  state->request.set_vin_number(vin_number_);
  state->request.set_request_id(data.request_id);
  state->request.set_time(data.time_seconds);
  state->request.set_fare(data.fare);
  state->attempts_left = 3;
  state->delay = std::chrono::milliseconds(500);

  auto attempt = std::make_shared<std::function<void()>>();
  *attempt = [this, state, attempt]() {
    client_.SendEta(state->request, [this, state, attempt](bool ok, const EtaResponse& /*resp*/) {
      if (!ok) {
        if (state->attempts_left > 0) {
          state->attempts_left--;
          state->delay *= 2;
          ScheduleRetry(state->delay, *attempt);
        } else {
          spdlog::error("[GatewayService] ETA failed after all retries");
        }
      }
    });
  };
  (*attempt)();
}

void VehicleGatewayService::ReportLocation() {
  struct State {
    UpdateVehicleLocationRequest request;
    uint32_t attempts_left;
    std::chrono::milliseconds delay;
  };

  auto state = std::make_shared<State>();
  LocationData data = location_provider_->GetCurrentLocation();

  state->request.set_vinnumber(vin_number_);
  state->request.set_latitude(data.latitud);
  state->request.set_longitude(data.longitude);
  state->attempts_left = 3;
  state->delay = std::chrono::milliseconds(500);

  auto attempt = std::make_shared<std::function<void()>>();
  *attempt = [this, state, attempt]() {
    client_.UpdateVehicleLocation(
      state->request,
      [this, state, attempt](bool ok, const UpdateVehicleLocationResponse& /*resp*/) {
        if (!ok) {
          if (state->attempts_left > 0) {
            state->attempts_left--;
            state->delay *= 2;
            ScheduleRetry(state->delay, *attempt);
          } else {
            spdlog::error("[GatewayService] Location failed after all retries");
          }
        }
      });
  };
  (*attempt)();
}

}  // namespace vehicle_gateway