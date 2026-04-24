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

namespace autoware_agent {

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
              "%u start(s)",
              route_config_->getMapName().c_str(), route_config_->getLanesCount(),
              route_config_->getDefaultStart() ? 1u : 0u);
  spdlog::info("[AutowareAgent] Route config loaded — map \"{}\", {} lanes, {} start(s)",
               route_config_->getMapName(), route_config_->getLanesCount(),
               (route_config_->getDefaultStart() != nullptr) ? 1u : 0u);

  io_context_ = std::make_shared<boost::asio::io_context>();
  strand_ = std::make_shared<boost::asio::io_context::strand>(*io_context_);
  work_guard_ = std::make_unique <boost::asio::executor_work_guard<boost::asio::io_context::executor_type>>(
    io_context_->get_executor());
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
  boost::asio::post(*strand_,[this]() {
    trip_ctrl_ = std::make_unique<TripController>(shared_from_this(), *route_config_, strand_);

    trip_ctrl_->setStateChangeCallback(
      [this](TripState prev, TripState next) { onTripStateChanged(prev, next); });

    RCLCPP_INFO(get_logger(), "[AutowareAgent] Initialization complete");
    spdlog::info("[AutowareAgent] Initialization complete");
  });

  int period_timer = static_cast<int>(1000.0 / tick_hz_);
  tick_timer_ = create_wall_timer(std::chrono::milliseconds(period_timer),
                                  [this]() { boost::asio::post(*strand_,[this]() { onTickTimer(); }); });

  io_thread_ = std::thread([this]() { io_context_->run(); });
}

void AutowareController::startTrip(double latitude, double longitude,
                                   std::function<void(bool)> callback) {
  boost::asio::post(*strand_,
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
  bool result = trip_ctrl_->startTrip();
  if (callback) {
    callback(result);
  }
}

void AutowareController::cancelTrip() {
  boost::asio::post(*strand_,[this]() { cancelTripImpl(); });
}

void AutowareController::cancelTripImpl() {
  if (trip_ctrl_) {
    trip_ctrl_->cancel();
  }
}

void AutowareController::getTripStatus(std::function<void(TripStatus)> callback) const {
  boost::asio::post(*strand_,[this, callback]() { getTripStatusImpl(callback); });
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
      case TripState::QUERY_PUBLISHING_INITIAL_POSE:
        return "QUERY_PUBLISHING_INITIAL_POSE";
      case TripState::QUERY_WAITING_LOCALISATION:
        return "QUERY_WAITING_LOCALISATION";
      case TripState::QUERY_PUBLISHING_GOAL:
        return "QUERY_PUBLISHING_GOAL";
      case TripState::QUERY_WAITING_ROUTE:
        return "QUERY_WAITING_ROUTE";
      case TripState::QUERY_READING_ETA:
        return "QUERY_READING_ETA";
      case TripState::ROUTES_READY:
        return "ROUTES_READY";
      case TripState::PUBLISHING_INITIAL_POSE:
        return "PUBLISHING_INITIAL_POSE";
      case TripState::WAITING_LOCALISATION:
        return "WAITING_LOCALISATION";
      case TripState::ENGAGING:
        return "ENGAGING";
      case TripState::DRIVING_TO_PICKUP:
        return "DRIVING_TO_PICKUP";
      case TripState::WAITING_FOR_MOVE:
        return "WAITING_FOR_MOVE";
      case TripState::RUNNING:
        return "RUNNING";
      case TripState::COMPLETED:
        return "COMPLETED";
      case TripState::CANCELLED:
        return "CANCELLED";
      case TripState::FAILED:
        return "FAILED";
    }
    return "UNKNOWN";
  };

  RCLCPP_INFO(get_logger(), "[AutowareAgent] %s -> %s", name(prev), name(next));
  spdlog::info("[AutowareAgent] {} -> {}", name(prev), name(next));

  if (trip_state_callback_) {
    trip_state_callback_(prev, next);
  }

  // Special handling for important transitions
  if (next == TripState::FAILED) {
    RCLCPP_ERROR(get_logger(), "[AutowareAgent] FAILURE Trip failed");
    spdlog::error("[AutowareAgent] FAILURE Trip failed");
  }

  // When the trip becomes RUNNING or the vehicle is driving to pickup, log lane info
  if ((next == TripState::RUNNING || next == TripState::DRIVING_TO_PICKUP) && trip_ctrl_) {
    auto st = trip_ctrl_->status();
    RCLCPP_INFO(get_logger(), "[AutowareAgent] Trip live: lane %s → lane %s",
                st.start_lanelet_id.c_str(), st.goal_lanelet_id.c_str());
    spdlog::info("[AutowareAgent] Trip live: lane {} → lane {}", st.start_lanelet_id,
                 st.goal_lanelet_id);
  }

  // If the routes are ready, provide a short info message about the planned route
  if (next == TripState::ROUTES_READY && trip_ctrl_) {
    auto st = trip_ctrl_->status();
    RCLCPP_INFO(get_logger(), "[AutowareAgent] Routes ready: start=%s goal=%s",
                st.start_lanelet_id.c_str(), st.goal_lanelet_id.c_str());
    spdlog::info("[AutowareAgent] Routes ready: start={} goal={}", st.start_lanelet_id,
                 st.goal_lanelet_id);
  }
}

TripStatus AutowareController::getTripStatusSync() const {
  return trip_ctrl_->status();
}

void AutowareController::setTripStateCallback(std::function<void(TripState, TripState)> cb) {
  // Store the external callback so onTripStateChanged() can forward events.
  trip_state_callback_ = std::move(cb);
}

void AutowareController::queryEta(GPSCoordinate start_gps, GPSCoordinate goal_gps, RouteQueryCallback callback) {
  boost::asio::post(*strand_,[this, start_gps, goal_gps, cb = std::move(callback)]() mutable {
      if (!trip_ctrl_) {
        RCLCPP_ERROR(get_logger(),
                     "[AutowareAgent] queryEta called before initialize()");
        if (cb) cb(RouteQueryResult{ .success = false,
                                     .error_message = "not initialized" });
        return;
      }
      trip_ctrl_->queryEta(start_gps,goal_gps, std::move(cb));
    });
}
}  // namespace AutowareAgent