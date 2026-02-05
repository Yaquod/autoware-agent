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

#include <spdlog/spdlog.h>

#include <csignal>
#include <memory>
#include <rclcpp/rclcpp.hpp>
#include <string>

#include "AutowareController.h"
#include "Config.h"

static std::atomic<bool> g_shutdown_requested{false};

void signalHandler(int /*signal*/) { g_shutdown_requested.store(true); }

int main(int argc, char** argv) {
  rclcpp::init(argc, argv);

  std::signal(SIGINT, signalHandler);
  std::signal(SIGTERM, signalHandler);

  std::string yaml_path =
      std::string(AutowareAgent::SRC_MAP_DIR) + "/nishishinjuku_routes.yaml";

  RCLCPP_INFO(rclcpp::get_logger("main"),
              "[AutowareAgent] Yaml configs loaded: %s", yaml_path.c_str());
  spdlog::info("[AutowareAgent] Yaml configs loaded : {}", yaml_path);

  // Create controller
  auto controller =
      std::make_shared<AutowareAgent::AutowareController>(yaml_path, 10.0);

  controller->initialize();

  std::this_thread::sleep_for(std::chrono::milliseconds(100));

  RCLCPP_INFO(rclcpp::get_logger("main"), "[AutowareAgent] Controller ready");
  spdlog::info("[AutowareAgent] Controller ready");

  // TODO: start the gRPC server on a background thread

  // rclcpp::spin blocks until shutdown is requested
  while (rclcpp::ok() && !g_shutdown_requested.load()) {
    rclcpp::spin_some(controller);
    std::this_thread::sleep_for(std::chrono::milliseconds(10));
  }

  RCLCPP_INFO(rclcpp::get_logger("main"), "[main] Shutting down…");
  spdlog::info("[main] Shutting down…");

  rclcpp::shutdown();
  return 0;
}