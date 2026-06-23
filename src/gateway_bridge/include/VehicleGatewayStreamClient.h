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
#ifndef VEHICLE_AUTOWARE_AGENT_VEHICLEGATEWAYSTREAMCLIENT_H
#define VEHICLE_AUTOWARE_AGENT_VEHICLEGATEWAYSTREAMCLIENT_H

#pragma once

#include "Providers.h"
#include "vehicle_gateway.grpc.pb.h"

#include <atomic>
#include <chrono>
#include <condition_variable>
#include <functional>
#include <mutex>
#include <queue>
#include <thread>

#include <grpcpp/grpcpp.h>
#include <spdlog/spdlog.h>

namespace vehicle_gateway {

struct StreamCommandHandlers {
  std::function<void(const TripInitRequest&)> on_trip_init;
  std::function<void(const TripMoveRequest&)> on_trip_move;
};

class VehicleGatewayStreamClient {
 public:
  VehicleGatewayStreamClient(const std::string& gateway_addr, ITripManager* trip_manager,
                             IEtaProvider* eta_provider, ILocationProvider* location_provider,
                             std::string vin)
    : gateway_addr_(gateway_addr)
    , trip_manager_(trip_manager)
    , eta_provider_(eta_provider)
    , location_provider_(location_provider)
    , vin_(std::move(vin))
    , shutdown_(false) {
    reconnect_thread_ = std::thread([this] { run_with_reconnect(); });
  }

  ~VehicleGatewayStreamClient() {
    shutdown();
  }

  void set_handlers(StreamCommandHandlers h) {
    handlers_ = std::move(h);
  }

  void shutdown() {
    if (shutdown_.exchange(true))
      return;
    cv_.notify_all();
    if (reconnect_thread_.joinable())
      reconnect_thread_.join();
  }

  void ReportTripInitAck() {
    VehicleEvent ev;
    auto* r = ev.mutable_trip_init_ack();
    r->set_success(true);
    r->set_message("accepted");
    enqueue(ev);
  }

  void ReportEta() {
     spdlog::info("[Stream] ReportEta called");  // ADD
    EtaData d = eta_provider_->GetEta();
    VehicleEvent ev;
    auto* r = ev.mutable_eta();
    r->set_vin_number(vin_);
    r->set_request_id(d.request_id);
    r->set_time(d.time_seconds);
    r->set_fare(d.fare);
    enqueue(ev);
    spdlog::info("[Stream] Eta event enqueued");  // ADD

  }



  void ReportEta(double distance_m, double time_seconds) {
     spdlog::info("[Stream] ReportEta called: distance={} time={}", distance_m, time_seconds);  // ADD
  VehicleEvent ev;
  auto* r = ev.mutable_eta();
  r->set_vin_number(vin_);
  r->set_request_id(eta_provider_->GetEta().request_id);  
  r->set_time(time_seconds);
  r->set_fare(0.0);  
  // r->set_distance(distance_m);
  enqueue(ev);
    spdlog::info("[Stream] Eta event enqueued");  // ADD

}


  void ReportStatus(const std::string& status) {
    VehicleEvent ev;
    auto* r = ev.mutable_status();
    r->set_vin_number(vin_);
    r->set_trip_id(trip_manager_->GetActiveTripId());
    r->set_status(status);
    enqueue(ev);
  }

  void ReportArrive() {
    ArriveData d = trip_manager_->GetArriveData();
    VehicleEvent ev;
    auto* r = ev.mutable_arrive();
    r->set_vin_number(vin_);
    r->set_trip_id(d.trip_id);
    r->set_lat(d.latitude);
    r->set_long_(d.longitude);
    enqueue(ev);
  }

  void ReportLocation() {
    LocationData d = location_provider_->GetCurrentLocation();
    VehicleEvent ev;
    auto* r = ev.mutable_location();
    r->set_vinnumber(vin_);
    r->set_latitude(d.latitud);
    r->set_longitude(d.longitude);
    enqueue(ev);
  }

 private:
  // runs forever until shutdown()
  void run_with_reconnect() {
    while (!shutdown_) {
      spdlog::info("[StreamClient] Connecting to gateway {}", gateway_addr_);
      try {
        run_stream();
      } catch (const std::exception& e) {
        spdlog::error("[StreamClient] Stream exception: {}", e.what());
      }
      if (!shutdown_) {
        spdlog::warn("[StreamClient] Reconnecting in 2s...");
        std::this_thread::sleep_for(std::chrono::seconds(2));
      }
    }
  }

  void run_stream() {
    auto channel = grpc::CreateChannel(gateway_addr_, grpc::InsecureChannelCredentials());
    auto stub = VehicleGateway::NewStub(channel);

    grpc::ClientContext ctx;
    std::atomic<bool> stream_dead{false};

    auto stream = stub->VehicleCommandStream(&ctx);
    spdlog::info("[StreamClient] Stream open — attempting first read");

    std::atomic<bool> read_received{false};
    std::thread watchdog([&ctx, &read_received, &stream_dead, this] {
      auto deadline = std::chrono::steady_clock::now() + std::chrono::seconds(8);
      while (!shutdown_ && std::chrono::steady_clock::now() < deadline) {
        std::this_thread::sleep_for(std::chrono::milliseconds(200));
        if (read_received.load())
          return;
      }
      if (!read_received.load()) {
        spdlog::warn("[StreamClient] No response from gateway in 8s — cancelling");
        stream_dead.store(true);
        cv_.notify_all();
        ctx.TryCancel();
      }
    });

    std::thread writer([this, &stream, &stream_dead, &ctx] {
      while (!shutdown_ && !stream_dead.load()) {
        VehicleEvent ev;
        {
          std::unique_lock<std::mutex> lk(mu_);
          cv_.wait_for(lk, std::chrono::milliseconds(100), [this, &stream_dead] {
            return !write_queue_.empty() || shutdown_ || stream_dead.load();
          });
          if (shutdown_ || stream_dead.load())
            break;
          if (write_queue_.empty())
            continue;
          ev = write_queue_.front();
          write_queue_.pop();
        }
        if (!stream->Write(ev)) {
          spdlog::warn("[StreamClient] Write failed — stream broken");
          stream_dead.store(true);
          cv_.notify_all();
          ctx.TryCancel();
          break;
        }
      }
      stream->WritesDone();
    });

    GatewayCommand cmd;
    while (stream->Read(&cmd)) {
      read_received.store(true);
      if (cmd.has_trip_init()) {
        spdlog::info("[StreamClient] Received TripInit reqId={}", cmd.trip_init().request_id());
        trip_manager_->SetActiveTripId(cmd.trip_init().request_id());
        if (handlers_.on_trip_init)
          handlers_.on_trip_init(cmd.trip_init());
      } else if (cmd.has_trip_move()) {
        spdlog::info("[StreamClient] Received TripMove tripId={}", cmd.trip_move().trip_id());
        trip_manager_->SetActiveTripId(cmd.trip_move().trip_id());
        if (handlers_.on_trip_move)
          handlers_.on_trip_move(cmd.trip_move());
      }
    }

    stream_dead.store(true);
    cv_.notify_all();

    watchdog.join();
    writer.join();

    auto status = stream->Finish();
    spdlog::warn("[StreamClient] Stream ended: code={} msg={}",
                 static_cast<int>(status.error_code()), status.error_message());
  }

  void enqueue(const VehicleEvent& ev) {
    {
      std::lock_guard<std::mutex> lk(mu_);
      write_queue_.push(ev);
    }
    cv_.notify_one();
  }

  std::string gateway_addr_;
  ITripManager* trip_manager_;
  IEtaProvider* eta_provider_;
  ILocationProvider* location_provider_;
  std::string vin_;
  StreamCommandHandlers handlers_;

  std::atomic<bool> shutdown_;
  std::thread reconnect_thread_;
  std::mutex mu_;
  std::condition_variable cv_;
  std::queue<VehicleEvent> write_queue_;
};

}  // namespace vehicle_gateway

#endif  // VEHICLE_AUTOWARE_AGENT_VEHICLEGATEWAYSTREAMCLIENT_H
