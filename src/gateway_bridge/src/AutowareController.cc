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

#include <spdlog/spdlog.h>

namespace AutowareAgent {

AutowareController::AutowareController(const std::string& map_path,
                                       double tick_hz)
    : rclcpp::Node("autoware_controller") {
  RCLCPP_INFO(get_logger(), "[AutowareAgent] Booting..");
  spdlog::info("[AutowareAgent] Booting..");

  RCLCPP_INFO(get_logger(), "[AutowareAgent] Loading Route configs: %s",
              map_path.c_str());
  spdlog::info("[AutowareAgent] Loading Route configs: {}", map_path);

  route_config_ = std::make_unique<RouteConfig>(map_path);
  RCLCPP_INFO(get_logger(),
              "[AutowareAgent] Route config loaded — map \"%s\", %zu lanes, "
              "%zu start(s)",
              route_config_->getMapName().c_str(),
              route_config_->getLanesCount(),
              route_config_->getDefaultStart() ? 1u : 0u);
  spdlog::info(
      "[AutowareAgent] Route config loaded — map \"{}\", {} lanes, {} start(s)",
      route_config_->getMapName(), route_config_->getLanesCount(),
      route_config_->getDefaultStart() ? 1u : 0u);

  trip_ctrl_ = std::make_unique<TripController>(
      std::shared_ptr<rclcpp::Node>(this, [](rclcpp::Node*) {}),
      *route_config_);

  trip_ctrl_->setStateChangeCallback([this](TripState prev, TripState next) {
    onTripStateChanged(prev, next);
  });

  // periodic tick
  int period_timer = static_cast<int>(1000.0 / tick_hz);
  tick_timer_ = create_wall_timer(std::chrono::milliseconds(period_timer),
                                  [this]() { onTickTimer(); });
}

bool AutowareController::startTrip(double latitude, double longitude) {
  return trip_ctrl_->startTrip(GPSCoordinate{latitude, longitude});
}

void AutowareController::cancelTrip() { trip_ctrl_->cancel(); }

TripStatus AutowareController::getTripStatus() const {
  return trip_ctrl_->status();
}

void AutowareController::onTickTimer() { trip_ctrl_->tick(); }

void AutowareController::onTripStateChanged(TripState prev, TripState next) {
  auto name = [](TripState s) -> const char* {
    switch (s) {
      case TripState::IDLE:
        return "IDLE";
      case TripState::PUBLISHING_INITIAL_POSE:
        return "PUBLISHING_INITIAL_POSE";
      case TripState::WAITING_LOCALISATION:
        return "WAITING_LOCALISATION";
      case TripState::PUBLISHING_GOAL:
        return "PUBLISHING_GOAL";
      case TripState::WAITING_ROUTE:
        return "WAITING_ROUTE";
      case TripState::ENGAGING:
        return "ENGAGING";
      case TripState::RUNNING:
        return "RUNNING";
      case TripState::COMPLETED:
        return "COMPLETED";
      case TripState::FAILED:
        return "FAILED";
    }
    return "???";
  };

  RCLCPP_INFO(get_logger(), "[AutowareAgent] %s -> %s", name(prev), name(next));
  spdlog::info("[AutowareAgent] {} -> {}", name(prev), name(next));

  if (next == TripState::FAILED) {
    RCLCPP_ERROR(get_logger(), "[AutowareAgent] FAILURE Trip failed");
    spdlog::error("[AutowareAgent] FAILURE Trip failed");
  }
  if (next == TripState::RUNNING) {
    auto st = trip_ctrl_->status();
    RCLCPP_INFO(get_logger(), "[AutowareAgent] Trip live: lane %s → lane %s",
                st.start_lanelet_id.c_str(), st.goal_lanelet_id.c_str());
    spdlog::info("[AutowareAgent] Trip live: lane {} → lane {}",
                 st.start_lanelet_id, st.goal_lanelet_id);
  }
}
}  // namespace AutowareAgent