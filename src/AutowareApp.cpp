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

#include "Config.h"
#include "VehicleGatewayService.h"
#include "ClusterBridgeProvider.h"
#include "AutowareControllerProvider.h"

#include "cluster_bridge/include/ClusterBridge.h"
#include "planning_bridge/include/PlanningBridge.h"
#include "perception_bridge/include/PerceptionBridge.h"
#include "trip_bridge/include/TripBridge.h"
#include <zenoh.hxx>

#include <spdlog/spdlog.h>

using namespace std::chrono_literals;

namespace autoware_agent {

AppHandles startAutowareApp(const std::string& yaml_path,
                           const std::string& server_addr_in,
                           const std::shared_ptr<zenoh::Session>& zsession_in,
                           double controller_route_search_radius)
{
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

  std::string const SERVER_ADDR = "localhost:50051";

  spdlog::info("[AutowareApp] Yaml configs loaded : {}", yaml_path);

  h.controller_ = std::make_shared<autoware_agent::AutowareController>(yaml_path, controller_route_search_radius);
  h.controller_->initialize();

  // Small delay to let controller come up
  std::this_thread::sleep_for(100ms);

  auto node = std::static_pointer_cast<rclcpp::Node>(h.controller_);

  h.cluster_bridge_ = std::make_shared<ClusterBridge>(node, h.zsession_);
  h.planning_bridge_ = std::make_shared<PlanningBridge>(node, h.zsession_);
  h.perception_bridge_ = std::make_shared<PerceptionBridge>(node, h.zsession_);
  h.trip_bridge_ = std::make_shared<TripBridge>(h.controller_, node, h.zsession_);

  auto eta_adapter = std::make_shared<vehicle_gateway::ClusterEtaAdapter>(
                                h.cluster_bridge_->GetState(),
                                h.cluster_bridge_->GetStateMutex(),
                                h.cluster_bridge_->GetRequestId());

  auto loc_adapter = std::make_shared<vehicle_gateway::ClusterLocationAdapter>(
                                h.cluster_bridge_->GetState(),
                                h.cluster_bridge_->GetStateMutex());

  auto trip_adapter = std::make_shared<vehicle_gateway::AutowareControllerTripAdapter>(h.controller_);

  h.gateway_ = std::make_shared<vehicle_gateway::VehicleGatewayService>(
        SERVER_ADDR, eta_adapter, loc_adapter, trip_adapter, "yaqoud-001", h.cluster_bridge_->GetIoContext());

  // Register trip state callback
  h.controller_->setTripStateCallback(
        [gw = h.gateway_](TripState prev, TripState next) {
            spdlog::info("[AutowareApp] TripState {} → {}", static_cast<int>(prev), static_cast<int>(next));
            if (next == TripState::WAITING_FOR_MOVE) {
                gw->ReportTripInit(); gw->ReportEta(); gw->ReportAccepted();
            } else if (next == TripState::RUNNING) {
                gw->ReportDriving();
            } else if (next == TripState::COMPLETED) {
                gw->ReportArrive(); gw->ReportCompleted();
            } else if (next == TripState::FAILED) {
                gw->ReportStatus(next == TripState::CANCELLED ? "cancelled" : "error");
            }
        });

  // Start ROS spinner in a thread. Caller is responsible for calling rclcpp::shutdown()
  h.ros_thread_ = std::thread([c = h.controller_]() { rclcpp::spin(c); });

  spdlog::info("[AutowareApp] Gateway client -> {}", SERVER_ADDR);

  return h;
}

void stopAutowareApp(AppHandles& h)
{
  if (h.planning_bridge_) { h.planning_bridge_->shutdown(); }
  if (h.perception_bridge_) { h.perception_bridge_->shutdown(); }
  if (h.trip_bridge_) { h.trip_bridge_->shutdown(); }
  if (h.cluster_bridge_) { h.cluster_bridge_->shutdown(); }

  // Join the ros thread; caller should have invoked rclcpp::shutdown() so spin exits.
  if (h.ros_thread_.joinable()) { h.ros_thread_.join(); }

  // Clear handles
  h.gateway_.reset();
  h.trip_bridge_.reset();
  h.perception_bridge_.reset();
  h.planning_bridge_.reset();
  h.cluster_bridge_.reset();
  h.controller_.reset();
  h.zsession_.reset();
}

} // namespace AutowareAgent


