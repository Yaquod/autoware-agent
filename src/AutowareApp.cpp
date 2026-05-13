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
#include "AutowareApp.h"

#include "AutowareControllerProvider.h"
#include "ClusterBridgeProvider.h"
#include "Config.h"
#include "cluster_bridge/include/ClusterBridge.h"
#include "perception_bridge/include/PerceptionBridge.h"
#include "planning_bridge/include/PlanningBridge.h"
#include "trip_bridge/include/TripBridge.h"

#include <spdlog/spdlog.h>
#include <zenoh.hxx>

using namespace std::chrono_literals;

namespace autoware_agent {

AppHandles startAutowareApp(const std::string& yaml_path, const std::string& server_addr_in,
                            const std::shared_ptr<zenoh::Session>& zsession_in,
                            double controller_route_search_radius) {
  AppHandles h;

  // Prepare a zenoh session if none provided.
  if (zsession_in) {
    h.zsession_ = zsession_in;
  } else {
    zenoh::init_log_from_env_or("debug");
    auto zconfig = zenoh::Config::create_default();
    zconfig.insert_json5("scouting/multicast/enabled", "false");
    zconfig.insert_json5("transport/shared_memory/enabled", "false");
    zconfig.insert_json5("listen/endpoints", R"(["udp/0.0.0.0:7447"])");
    h.zsession_ = std::make_shared<zenoh::Session>(zenoh::Session::open(std::move(zconfig)));
  }

  std::string const SERVER_ADDR = "192.168.64.7:50051";

  spdlog::info("[AutowareApp] Yaml configs loaded : {}", yaml_path);

  h.controller_ =
    std::make_shared<autoware_agent::AutowareController>(yaml_path, controller_route_search_radius);
  h.controller_->initialize();

  // Small delay to let controller come up
  std::this_thread::sleep_for(100ms);

  auto node = std::static_pointer_cast<rclcpp::Node>(h.controller_);

  h.cluster_bridge_ = std::make_shared<ClusterBridge>(node, h.zsession_);
  h.planning_bridge_ = std::make_shared<PlanningBridge>(node, h.zsession_);
  h.perception_bridge_ = std::make_shared<PerceptionBridge>(node, h.zsession_);
  h.trip_bridge_ = std::make_shared<TripBridge>(h.controller_, node, h.zsession_);

  h.eta_adapter_ = std::make_shared<vehicle_gateway::ClusterEtaAdapter>(
    h.cluster_bridge_->GetState(), h.cluster_bridge_->GetStateMutex(),
    h.cluster_bridge_->GetRequestId());

  h.loc_adapter_ = std::make_shared<vehicle_gateway::ClusterLocationAdapter>(
    h.cluster_bridge_->GetState(), h.cluster_bridge_->GetStateMutex());

  h.trip_adapter_ = std::make_shared<vehicle_gateway::AutowareControllerTripAdapter>(h.controller_);

  auto stream_client = std::make_shared<vehicle_gateway::VehicleGatewayStreamClient>(
    "192.168.64.7:50051",
    h.trip_adapter_.get(),  // ← now backed by h, lives as long as AppHandles
    h.eta_adapter_.get(), h.loc_adapter_.get(), "ORIN_NANO_001");

  stream_client->set_handlers(
    {.on_trip_init =
       [ctrl = h.controller_](const vehicle_gateway::TripInitRequest& req) {
         autoware_agent::GPSCoordinate start{.latitude = req.start_lat(),
                                             .longitude = req.start_long()};
         autoware_agent::GPSCoordinate goal{.latitude = req.end_lat(), .longitude = req.end_long()};
         ctrl->queryEta(start, goal, [ctrl](autoware_agent::EtaQueryResult r) {
           if (!r.success_) {
             spdlog::error("[AutowareApp] queryEta failed: {}", r.error_message_);
             return;
           }
           ctrl->startTrip([](bool ok) {
             if (!ok)
               spdlog::error("[AutowareApp] startTrip rejected");
           });
         });
       },

     .on_trip_move =
       [ctrl = h.controller_](const vehicle_gateway::TripMoveRequest&) {
         ctrl->move([](bool ok) {
           if (!ok)
             spdlog::error("[AutowareApp] move() rejected");
         });
       }});

  h.controller_->setTripStateCallback([sc = stream_client](TripState, TripState next) {
    if (next == TripState::WAITING_FOR_MOVE) {
      sc->ReportTripInitAck();
      sc->ReportEta();
      sc->ReportStatus("accepted");
    } else if (next == TripState::RUNNING)
      sc->ReportStatus("in_progress");
    else if (next == TripState::COMPLETED) {
      sc->ReportArrive();
      sc->ReportStatus("completed");
    } else if (next == TripState::FAILED)
      sc->ReportStatus("error");
  });

  h.stream_client_ = stream_client;

  // Start ROS spinner in a thread. Caller is responsible for calling rclcpp::shutdown()
  h.ros_thread_ = std::thread([c = h.controller_]() { rclcpp::spin(c); });

  spdlog::info("[AutowareApp] Gateway client -> {}", SERVER_ADDR);

  return h;
}

void stopAutowareApp(AppHandles& h) {
  if (h.planning_bridge_) {
    h.planning_bridge_->shutdown();
  }
  if (h.perception_bridge_) {
    h.perception_bridge_->shutdown();
  }
  if (h.trip_bridge_) {
    h.trip_bridge_->shutdown();
  }
  if (h.cluster_bridge_) {
    h.cluster_bridge_->shutdown();
  }
  if (h.stream_client_)
    h.stream_client_->shutdown();

  // Join the ros thread; caller should have invoked rclcpp::shutdown() so spin exits.
  if (h.ros_thread_.joinable()) {
    h.ros_thread_.join();
  }

  // Clear handles
  h.trip_adapter_.reset();
  h.eta_adapter_.reset();
  h.loc_adapter_.reset();
  h.trip_bridge_.reset();
  h.perception_bridge_.reset();
  h.planning_bridge_.reset();
  h.cluster_bridge_.reset();
  h.controller_.reset();
  h.zsession_.reset();
}

}  // namespace autoware_agent
