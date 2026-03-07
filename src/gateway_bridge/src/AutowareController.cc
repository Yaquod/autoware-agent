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

AutowareController::AutowareController(const std::string& map_path, double tick_hz)
  : rclcpp::Node("autoware_controller")
  , tick_hz_(tick_hz) {
  RCLCPP_INFO(get_logger(), "[AutowareAgent] Booting..");
  spdlog::info("[AutowareAgent] Booting..");

  RCLCPP_INFO(get_logger(), "[AutowareAgent] Loading Route configs: %s", map_path.c_str());
  spdlog::info("[AutowareAgent] Loading Route configs: {}", map_path);

  route_config_ = std::make_unique<RouteConfig>(map_path);
  RCLCPP_INFO(get_logger(),
              "[AutowareAgent] Route config loaded — map \"%s\", %zu lanes, "
              "%zu start(s)",
              route_config_->getMapName().c_str(), route_config_->getLanesCount(),
              route_config_->getDefaultStart() ? 1u : 0u);
  spdlog::info("[AutowareAgent] Route config loaded — map \"{}\", {} lanes, {} start(s)",
               route_config_->getMapName(), route_config_->getLanesCount(),
               (route_config_->getDefaultStart() != nullptr) ? 1u : 0u);

  io_context_ = std::make_shared<boost::asio::io_context>();
  strand_ = std::make_shared<boost::asio::io_context::strand>(*io_context_);
  work_guard_ = std::make_unique<boost::asio::io_context::work>(*io_context_);
}

AutowareController::~AutowareController() {
  if (work_guard_) {
    work_guard_.reset();
  }
  if (io_context_) {
    io_context_->stop();
  }
  if (io_thread_.joinable()) {
    io_thread_.join();
  }
}

void AutowareController::initialize() {
  strand_->post([this]() {
    trip_ctrl_ = std::make_unique<TripController>(shared_from_this(), *route_config_, strand_);

    trip_ctrl_->setStateChangeCallback(
      [this](TripState prev, TripState next) { onTripStateChanged(prev, next); });

    RCLCPP_INFO(get_logger(), "[AutowareAgent] Initialization complete");
    spdlog::info("[AutowareAgent] Initialization complete");
  });

  int period_timer = static_cast<int>(1000.0 / tick_hz_);
  tick_timer_ = create_wall_timer(std::chrono::milliseconds(period_timer),
                                  [this]() { strand_->post([this]() { onTickTimer(); }); });

  io_thread_ = std::thread([this]() { io_context_->run(); });
}

void AutowareController::startTrip(double latitude, double longitude,
                                   std::function<void(bool)> callback) {
  strand_->post(
    [this, latitude, longitude, callback]() { startTripImpl(latitude, longitude, callback); });
}

void AutowareController::startTripImpl(double latitude, double longitude,
                                       std::function<void(bool)> callback) {
  if (!trip_ctrl_) {
    RCLCPP_ERROR(get_logger(),
                 "[AutowareAgent] TripController not initialized. Call "
                 "initialize() first.");
    spdlog::error(
      "[AutowareAgent] TripController not initialized. Call initialize() "
      "first.");
    if (callback) {
      callback(false);
    }
    return;
  }
  bool result = trip_ctrl_->startTrip(GPSCoordinate{latitude, longitude});
  if (callback) {
    callback(result);
  }
}

void AutowareController::cancelTrip() {
  strand_->post([this]() { cancelTripImpl(); });
}

void AutowareController::cancelTripImpl() {
  if (trip_ctrl_) {
    trip_ctrl_->cancel();
  }
}

void AutowareController::getTripStatus(std::function<void(TripStatus)> callback) const {
  strand_->post([this, callback]() { getTripStatusImpl(callback); });
}

void AutowareController::getTripStatusImpl(std::function<void(TripStatus)> callback) const {
  if (!trip_ctrl_) {
    if (callback) {
      callback(TripStatus{});
    }
    return;
  }
  TripStatus status = trip_ctrl_->status();
  if (callback) {
    callback(status);
  }
}

void AutowareController::onTickTimer() {
  if (trip_ctrl_) {
    trip_ctrl_->tick();
  }
}

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
    spdlog::info("[AutowareAgent] Trip live: lane {} → lane {}", st.start_lanelet_id,
                 st.goal_lanelet_id);
  }
}

}  // namespace AutowareAgent