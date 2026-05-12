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
#include "AutowareController.h"
#include "Config.h"
#include "cluster_bridge/include/ClusterBridge.h"

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

<<<<<<< HEAD
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
  auto planning_bridge = std::make_shared<PlanningBridge>(           // ADDED
   node, cluster_bridge->getBuilder());

  auto perception_bridge = std::make_shared<PerceptionBridge>(           // ADDED
   node, cluster_bridge->getBuilder());

  auto trip_bridge = std::make_shared<TripBridge>(                  //ADDED
    controller, node ,
    cluster_bridge->getBuilder()
);



  std::thread cluster_bridge_thread([&cluster_bridge]() { cluster_bridge->runGrpcServer(); });
  std::thread ros_thread([&controller]() { rclcpp::spin(controller); });


  std::this_thread::sleep_for(std::chrono::seconds(2));  // wait for everything to init
  controller->startTrip(
      35.68814679007944,   // ← from test file
      139.69440756809428,
      [](bool success) {
          spdlog::info("[main] startTrip result: {}", success);
      }
  );



=======
  std::string const YAML_PATH = std::string(autoware_agent::SRC_MAP_DIR) + "/lanelet2_map.osm";
  autoware_agent::AppHandles app = autoware_agent::startAutowareApp(YAML_PATH);
>>>>>>> 0713a0b91a362ff643eab8386cfbc53500ef34c6

  while (!g_shutdown_requested.load()) {
    std::this_thread::sleep_for(std::chrono::milliseconds(100));
  }

  rclcpp::shutdown();  // signals rclcpp::spin() to exit
  autoware_agent::stopAutowareApp(app);

  RCLCPP_INFO(rclcpp::get_logger("main"), "[main] Shutting down…");
  spdlog::info("[main] Shutting down…");
  return 0;
}