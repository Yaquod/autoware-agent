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


#ifndef VEHICLE_AUTOWARE_AGENT_VEHICLEGATEWAY_H
#define VEHICLE_AUTOWARE_AGENT_VEHICLEGATEWAY_H

#pragma once

#include "vehicle_gateway.grpc.pb.h"
#include "vehicle_gateway.pb.h"

#include <functional>
#include <thread>
#include <stdexcept>
#include <string>

#include <grpcpp/grpcpp.h>

namespace vehicle_gateway {
using EtaCallback = std::function<void(bool ok, const vehicle_gateway::EtaResponse&)>;
using StatusCallback = std::function<void(bool ok, const vehicle_gateway::StatusResponse&)>;
using ArriveCallback = std::function<void(bool ok, const vehicle_gateway::ArriveResponse&)>;
using UpdateLocationCallback = std::function<void(bool ok, const vehicle_gateway::UpdateVehicleLocationResponse&)>;
using TripInitCallback = std::function<void(bool ok, const vehicle_gateway::TripInitResponse&)>;
using TripMoveCallback = std::function<void(bool ok, const vehicle_gateway::TripMoveResponse&)>;

class VehicleGatewayClient {
public:
  explicit VehicleGatewayClient(const std::string& target_vm);

  ~VehicleGatewayClient();

  VehicleGatewayClient(const VehicleGatewayClient&) = delete;
  VehicleGatewayClient& operator=(const VehicleGatewayClient&) = delete;

  void SendEta(const vehicle_gateway::EtaRequest& request, EtaCallback callback);

  void SendStatus(const vehicle_gateway::StatusRequest& request, StatusCallback callback);

  void SendArrive(const vehicle_gateway::ArriveRequest& request, ArriveCallback callback);

  void UpdateVehicleLocation(const vehicle_gateway::UpdateVehicleLocationRequest& request, UpdateLocationCallback callback);

  void TripInit(const vehicle_gateway::TripInitRequest& request, TripInitCallback callback);

  void TripMove(const vehicle_gateway::TripMoveRequest& request, TripMoveCallback callback);

  void ShutDown();

private:
  struct AsyncCallBase {
    virtual void OnComplete(bool ok) = 0;
    virtual ~AsyncCallBase() = default;
  };

  template<typename Response, typename Callback>
  struct AsyncCall : public AsyncCallBase {
    grpc::ClientContext context;
    Response response;
    Callback callback;
    grpc::Status status;
    std::unique_ptr<grpc::ClientAsyncResponseReader<Response>> reader;

    void OnComplete(bool ok) override {
      bool success = ok && status.ok();
      // invoke user callback
      callback(success, response);
      // delete self by caller
    }
  };

  std::unique_ptr<vehicle_gateway::VehicleGateway::Stub> stub_;
  grpc::CompletionQueue cq_;
  std::thread cq_thread_;

  void RunCompletionQueue();
};
}
#endif  // VEHICLE_AUTOWARE_AGENT_VEHICLEGATEWAY_H
