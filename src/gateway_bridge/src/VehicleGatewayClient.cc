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

#include "VehicleGatewayClient.h"

namespace vehicle_gateway {

VehicleGatewayClient::VehicleGatewayClient(const std::string& target_vm)
  : stub_(vehicle_gateway::VehicleGateway::NewStub(grpc::CreateChannel(target_vm, grpc::InsecureChannelCredentials())))
{
  // start completion queue to be ready for the first call
  cq_thread_ = std::thread(&VehicleGatewayClient::RunCompletionQueue, this);
}

VehicleGatewayClient::~VehicleGatewayClient() {
  ShutDown();
}

void VehicleGatewayClient::ShutDown() {
    cq_.Shutdown();
    if (cq_thread_.joinable()) {
      cq_thread_.join();
    }
}

void VehicleGatewayClient::RunCompletionQueue() {
  void* tag;
  bool ok;

  // Next() blocks here until a tag is ready, then returns true.
  // Returns false only after cq_.Shutdown() and the queue is drained.
  while (cq_.Next(&tag, &ok)) {
    // Tag is AsyncCallBase*; call OnComplete and delete the call object.
    auto* call = static_cast<AsyncCallBase*>(tag);
    if (call) {
      call->OnComplete(ok);
      delete call;
    }
  }
}

void VehicleGatewayClient::SendEta(const vehicle_gateway::EtaRequest& request,
                                   EtaCallback callback) {
  using Call = AsyncCall<EtaResponse,EtaCallback>;
  auto* call = new Call;
  call->callback = std::move(callback);
  call->reader = stub_->AsyncSendEta(&call->context, request, &cq_);
  call->reader->Finish(&call->response, &call->status, static_cast<void*>(call));

}

void VehicleGatewayClient::SendArrive(const vehicle_gateway::ArriveRequest& request,
                                      ArriveCallback callback) {
  using Call = AsyncCall<ArriveResponse,ArriveCallback>;
  auto* call = new Call;
  call->callback = std::move(callback);
  call->reader = stub_->AsyncSendArrive(&call->context, request, &cq_);
  call->reader->Finish(&call->response, &call->status, static_cast<void*>(call));
}

void VehicleGatewayClient::SendStatus(const vehicle_gateway::StatusRequest& request,
                                      StatusCallback callback) {
  using Call = AsyncCall<StatusResponse,StatusCallback>;
  auto* call = new Call;
  call->callback = std::move(callback);
  call->reader = stub_->AsyncSendStatus(&call->context, request, &cq_);
  call->reader->Finish(&call->response, &call->status, static_cast<void*>(call));
}

void VehicleGatewayClient::TripInit(const vehicle_gateway::TripInitRequest& request,
                                    TripInitCallback callback) {
  using Call = AsyncCall<TripInitResponse,TripInitCallback>;
  auto* call = new Call;
  call->callback = std::move(callback);
  call->reader = stub_->AsyncTripInit(&call->context, request, &cq_);
  call->reader->Finish(&call->response, &call->status, static_cast<void*>(call));
}

void VehicleGatewayClient::TripMove(const vehicle_gateway::TripMoveRequest& request,
                                    TripMoveCallback callback) {
  using Call = AsyncCall<TripMoveResponse,TripMoveCallback>;
  auto* call = new Call;
  call->callback = std::move(callback);
  call->reader = stub_->AsyncTripMove(&call->context, request, &cq_);
  call->reader->Finish(&call->response, &call->status, static_cast<void*>(call));
}

void VehicleGatewayClient::UpdateVehicleLocation(
  const vehicle_gateway::UpdateVehicleLocationRequest& request, UpdateLocationCallback callback) {
  using Call = AsyncCall<UpdateVehicleLocationResponse,UpdateLocationCallback>;
  auto* call = new Call;
  call->callback = std::move(callback);
  call->reader = stub_->AsyncUpdateVehicleLocation(&call->context, request, &cq_);
  call->reader->Finish(&call->response, &call->status, static_cast<void*>(call));
}

} // namespace vehicle_gateway