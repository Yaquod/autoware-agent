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
#include "Config.h"
#include "cluster_bridge/include/ClusterBridge.h"
#include "perception_bridge/include/PerceptionBridge.h"
#include "planning_bridge/include/PlanningBridge.h"
#include "trip_bridge/include/TripBridge.h"

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
  zenoh::init_log_from_env_or("debug");

  auto zconfig = zenoh::Config::create_default();
  zconfig.insert_json5("scouting/multicast/enabled", "false");

  zconfig.insert_json5("transport/shared_memory/enabled", "false");

  zconfig.insert_json5("listen/endpoints", R"(["udp/0.0.0.0:7447"])");

  auto zsession = std::make_shared<zenoh::Session>(zenoh::Session::open(std::move(zconfig)));

  std::string const YAML_PATH =
    std::string(AutowareAgent::SRC_MAP_DIR) + "/nishishinjuku_routes.yaml";

  RCLCPP_INFO(rclcpp::get_logger("main"), "[AutowareAgent] Yaml configs loaded: %s",
              YAML_PATH.c_str());
  spdlog::info("[AutowareAgent] Yaml configs loaded : {}", YAML_PATH);

  // Create controller
  auto controller = std::make_shared<AutowareAgent::AutowareController>(YAML_PATH, 10.0);

  controller->initialize();

  std::this_thread::sleep_for(std::chrono::milliseconds(100));

  RCLCPP_INFO(rclcpp::get_logger("main"), "[AutowareAgent] Controller ready");
  spdlog::info("[AutowareAgent] Controller ready");

  auto node = std::static_pointer_cast<rclcpp::Node>(controller);

  auto cluster_bridge =
    std::make_shared<ClusterBridge>(std::static_pointer_cast<rclcpp::Node>(controller), zsession);

  auto planning_bridge = std::make_shared<PlanningBridge>(node, zsession);

  auto perception_bridge = std::make_shared<PerceptionBridge>(node, zsession);

  auto trip_bridge = std::make_shared<TripBridge>(controller, node, zsession);

  std::thread ros_thread([&controller]() { rclcpp::spin(controller); });

  while (!g_shutdown_requested.load()) {
    std::this_thread::sleep_for(std::chrono::milliseconds(100));
  }

  rclcpp::shutdown();  // signals rclcpp::spin() to exit
  if (ros_thread.joinable()) {
    ros_thread.join();
  }

  planning_bridge->shutdown();
  perception_bridge->shutdown();
  trip_bridge->shutdown();
  cluster_bridge->shutdown();

  RCLCPP_INFO(rclcpp::get_logger("main"), "[main] Shutting down…");
  spdlog::info("[main] Shutting down…");
  return 0;
}