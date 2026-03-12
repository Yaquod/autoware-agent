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
#include "planning_bridge/include/PlanningBridge.h"

#include <rclcpp/rclcpp.hpp>

#include <csignal>
#include <memory>
#include <string>

#include <spdlog/spdlog.h>

// static std::atomic<bool> g_shutdown_requested{false};
//
// void signalHandler(int /*signal*/) {
//   g_shutdown_requested.store(true);
// }

int main(int argc, char** argv) {
  rclcpp::init(argc, argv);

  // std::signal(SIGINT, signalHandler);
  // std::signal(SIGTERM, signalHandler);

  std::string yaml_path = std::string(AutowareAgent::SRC_MAP_DIR) + "/nishishinjuku_routes.yaml";

  RCLCPP_INFO(rclcpp::get_logger("main"), "[AutowareAgent] Yaml configs loaded: %s",
              yaml_path.c_str());
  spdlog::info("[AutowareAgent] Yaml configs loaded : {}", yaml_path);

  // Create controller
  auto controller = std::make_shared<AutowareAgent::AutowareController>(yaml_path, 10.0);

  controller->initialize();

  std::this_thread::sleep_for(std::chrono::milliseconds(100));

  RCLCPP_INFO(rclcpp::get_logger("main"), "[AutowareAgent] Controller ready");
  spdlog::info("[AutowareAgent] Controller ready");

  // Create cluster bridge
  // auto cluster_bridge = std::make_shared<ClusterBridge>(controller, "0.0.0.0:50052");
  auto node = std::static_pointer_cast<rclcpp::Node>(controller);

  auto cluster_bridge = std::make_shared<ClusterBridge>(
    std::static_pointer_cast<rclcpp::Node>(controller), "0.0.0.0:50052");

  cluster_bridge->prepareGrpcServer();
  auto planning_bridge = std::make_shared<PlanningBridge>(           // ← ADD
   node, cluster_bridge->getBuilder());

  std::thread cluster_bridge_thread([&cluster_bridge]() { cluster_bridge->runGrpcServer(); });
  std::thread ros_thread([&controller]() { rclcpp::spin(controller); });

  // while (!g_shutdown_requested.load()) {
  //   std::this_thread::sleep_for(std::chrono::milliseconds(100));
  // }
  ros_thread.join();
  RCLCPP_INFO(rclcpp::get_logger("main"), "[main] Shutting down...");



  // rclcpp::shutdown();  // signals rclcpp::spin() to exit
  // if (ros_thread.joinable())
  //   ros_thread.join();

  planning_bridge->shutdown();
  cluster_bridge->shutdown();

  if (cluster_bridge_thread.joinable()) {
    cluster_bridge_thread.join();
  }

  planning_bridge.reset();
  cluster_bridge.reset();


  RCLCPP_INFO(rclcpp::get_logger("main"), "[main] Shutting down…");
  spdlog::info("[main] Shutting down…");
  return 0;
}