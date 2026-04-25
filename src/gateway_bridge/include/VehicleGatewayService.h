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

#ifndef VEHICLE_AUTOWARE_AGENT_VEHICLEGATEWAYSERVICE_H
#define VEHICLE_AUTOWARE_AGENT_VEHICLEGATEWAYSERVICE_H

#include "Providers.h"
#include "VehicleGatewayClient.h"

#include <boost/asio/io_context.hpp>

#include <memory>
#include <string>

namespace vehicle_gateway {

class VehicleGatewayService {
 public:
  VehicleGatewayService(const std::string& target_vm, std::shared_ptr<IEtaProvider> eta_provider,
                        std::shared_ptr<ILocationProvider> location_provider,
                        std::shared_ptr<ITripManager> trip_manager, const std::string& vin_number,
                        boost::asio::io_context& retry_io);

  ~VehicleGatewayService();

  void ReportTripInit();

  void ReportArrive();

  // Status reports
  void ReportAccepted();
  void ReportDriving();
  void ReportCompleted();
  void ReportEta();
  void ReportLocation();
  void ReportStatus();
  void ReportStatus(const std::string& status);

  void MoveTrip();
  void Shutdown();

 private:
  VehicleGatewayClient client_;
  std::shared_ptr<IEtaProvider> eta_provider_;
  std::shared_ptr<ILocationProvider> location_provider_;
  std::shared_ptr<ITripManager> trip_manager_;
  std::string vin_number_;
  boost::asio::io_context& retry_io_;

  void StartTrip();
  void ScheduleRetry(std::chrono::milliseconds delay, std::function<void()> fn);
};
}  // namespace vehicle_gateway
#endif  // VEHICLE_AUTOWARE_AGENT_VEHICLEGATEWAYSERVICE_H
