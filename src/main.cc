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

#include "AutowareController.h"
#include "AutowareControllerProvider.h"
#include "ClusterBridgeProvider.h"
#include "Config.h"
#include "VehicleGatewayService.h"
#include "cluster_bridge/include/ClusterBridge.h"
#include "perception_bridge/include/PerceptionBridge.h"
#include "planning_bridge/include/PlanningBridge.h"
#include "trip_bridge/include/TripBridge.h"
#include "AutowareApp.h"

#include <rclcpp/rclcpp.hpp>

#include <csignal>
#include <memory>
#include <string>

#include <spdlog/spdlog.h>
#include <zenoh.hxx>

static std::atomic<bool> g_shutdown_requested{false};

static void signalHandler(int /*signal*/) {
  g_shutdown_requested.store(true);
}

int main(int argc, char** argv) {
  rclcpp::init(argc, argv);

  std::signal(SIGINT, signalHandler);
  std::signal(SIGTERM, signalHandler);

  std::string const YAML_PATH = std::string(autoware_agent::SRC_MAP_DIR) + "/nishishinjuku_routes.yaml";
  autoware_agent::AppHandles app = autoware_agent::startAutowareApp(YAML_PATH);

  while (!g_shutdown_requested.load()) { std::this_thread::sleep_for(std::chrono::milliseconds(100)); }

  rclcpp::shutdown();  // signals rclcpp::spin() to exit
  autoware_agent::stopAutowareApp(app);

  RCLCPP_INFO(rclcpp::get_logger("main"), "[main] Shutting down…");
  spdlog::info("[main] Shutting down…");
  return 0;
}