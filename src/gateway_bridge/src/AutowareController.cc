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

#include "map_routes/MapProjectorInfo.h"

#include <spdlog/spdlog.h>

namespace autoware_agent {

AutowareController::AutowareController(const std::string& map_path, double tick_hz)
  : rclcpp::Node("autoware_controller")
  , tick_hz_(tick_hz) {
  RCLCPP_INFO(get_logger(), "[AutowareAgent] Booting..");
  spdlog::info("[AutowareAgent] Booting..");

  RCLCPP_INFO(get_logger(), "[AutowareAgent] Loading Route configs: %s", map_path.c_str());
  spdlog::info("[AutowareAgent] Loading Route configs: {}", map_path);

  std::string dir_path = map_path;
  std::string osm_path = map_path + "/lanelet2_map.osm";

  if (map_path.length() >= 4 && map_path.substr(map_path.length() - 4) == ".osm") {
    size_t last_slash = map_path.find_last_of('/');
    dir_path = (last_slash != std::string::npos) ? map_path.substr(0, last_slash) : ".";
    osm_path = map_path;
  }

  auto proj_info = MapProjectorInfo::load(dir_path);

  lanelet_map_ = std::make_unique<LaneletMap>(osm_path, proj_info.origin_lat, proj_info.origin_lon,
                                              proj_info.local_offset_x, proj_info.local_offset_y);

  if (proj_info.has_start) {
    lanelet_map_->setDefaultStart(proj_info.start_name,
                                  GPSCoordinate{proj_info.start_lat, proj_info.start_lon});
    const auto* start = lanelet_map_->getDefaultStart();
    RCLCPP_INFO(get_logger(),
                "[AutowareAgent] LaneletMap loaded — %zu lanelets, start '%s' → local (%.2f, %.2f)",
                lanelet_map_->getLaneletCount(), start->name.c_str(), start->local.x,
                start->local.y);
  } else {
    RCLCPP_INFO(get_logger(),
                "[AutowareAgent] LaneletMap loaded — %zu lanelets (no start position configured)",
                lanelet_map_->getLaneletCount());
  }

  io_context_ = std::make_shared<boost::asio::io_context>();
  strand_ = std::make_shared<boost::asio::io_context::strand>(*io_context_);
  work_guard_ =
    std::make_unique<boost::asio::executor_work_guard<boost::asio::io_context::executor_type>>(
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
  boost::asio::post(*strand_, [this]() {
    trip_ctrl_ = std::make_unique<TripController>(shared_from_this(), *lanelet_map_, strand_);

    trip_ctrl_->setStateChangeCallback(
      [this](TripState prev, TripState next) { onTripStateChanged(prev, next); });

    trip_ctrl_->setArrivalCallbacks(
      // Pickup arrived
      [this]() {
        if (arrival_callback_) {
          arrival_callback_(ArrivalKind::PICKUP);
        }
      },
      // Dropoff arrived
      [this]() {
        if (arrival_callback_) {
          arrival_callback_(ArrivalKind::DROPOFF);
        }
      });

    trip_ctrl_->initializeInMap();
    ready_.store(true);
    RCLCPP_INFO(get_logger(), "[AutowareAgent] Initialization complete");
    spdlog::info("[AutowareAgent] Initialization complete");
  });

  const int PERIOD_MS = static_cast<int>(1000.0 / tick_hz_);
  tick_timer_ = create_wall_timer(std::chrono::milliseconds(PERIOD_MS), [this]() {
    boost::asio::post(*strand_, [this]() { onTickTimer(); });
  });

  io_thread_ = std::thread([this]() { io_context_->run(); });
}

void AutowareController::startTrip(const std::function<void(bool)>& callback) {
  boost::asio::post(*strand_, [this, callback]() { startTripImpl(callback); });
}

void AutowareController::move(const std::function<void(bool)>& callback) {
  boost::asio::post(*strand_, [this, callback]() { moveImpl(callback); });
}

void AutowareController::startTripImpl(std::function<void(bool)> callback) {
  if (!trip_ctrl_) {
    RCLCPP_ERROR(get_logger(), "[AutowareAgent] startTrip called before initialize()");
    spdlog::error("[AutowareAgent] startTrip called before initialize()");
    if (callback)
      callback(false);
    return;
  }
  bool ok = trip_ctrl_->startTrip();
  if (callback)
    callback(ok);
}

void AutowareController::cancelTrip() {
  boost::asio::post(*strand_, [this]() { cancelTripImpl(); });
}

void AutowareController::cancelTripImpl() {
  if (trip_ctrl_) {
    trip_ctrl_->cancel();
  }
}

void AutowareController::getTripStatus(std::function<void(TripStatus)> callback) const {
  boost::asio::post(*strand_, [this, callback]() { getTripStatusImpl(callback); });
}

void AutowareController::getTripStatusImpl(std::function<void(TripStatus)> callback) const {
  if (!trip_ctrl_) {
    if (callback)
      callback(TripStatus{});
    return;
  }
  if (callback)
    callback(trip_ctrl_->status());
}

void AutowareController::onTickTimer() {
  if (!trip_ctrl_)
    return;

  auto pose = trip_ctrl_->getCurrentVehiclePose();
  if (pose.has_value()) {
    trip_ctrl_->setCurrentGps(*pose);
  }

  trip_ctrl_->tick();
}

void AutowareController::onTripStateChanged(TripState prev, TripState next) {
  auto name = [](TripState s) -> const char* {
    switch (s) {
      case TripState::IDLE:
        return "IDLE";
      case TripState::INITIALIZING_IN_MAP:
        return "INITIALIZING_IN_MAP";
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
      default:
        return "UNKNOWN";
    }
  };

  RCLCPP_INFO(get_logger(), "[AutowareAgent] %s -> %s", name(prev), name(next));
  spdlog::info("[AutowareAgent] {} -> {}", name(prev), name(next));

  if (trip_state_callback_) {
    trip_state_callback_(prev, next);
  }

  if (next == TripState::FAILED) {
    RCLCPP_ERROR(get_logger(), "[AutowareAgent] Trip FAILED");
    spdlog::error("[AutowareAgent] Trip FAILED");
  }

  if ((next == TripState::RUNNING || next == TripState::DRIVING_TO_PICKUP) && trip_ctrl_) {
    auto st = trip_ctrl_->status();
    RCLCPP_INFO(get_logger(), "[AutowareAgent] Trip live: lane %s → lane %s",
                st.start_lanelet_id_.c_str(), st.goal_lanelet_id_.c_str());
    spdlog::info("[AutowareAgent] Trip live: lane {} → lane {}", st.start_lanelet_id_,
                 st.goal_lanelet_id_);
  }

  if (next == TripState::ROUTES_READY && trip_ctrl_) {
    auto st = trip_ctrl_->status();
    RCLCPP_INFO(get_logger(), "[AutowareAgent] Routes ready: pickup=%s goal=%s",
                st.start_lanelet_id_.c_str(), st.goal_lanelet_id_.c_str());
    spdlog::info("[AutowareAgent] Routes ready: pickup={} goal={}", st.start_lanelet_id_,
                 st.goal_lanelet_id_);
  }
}

TripStatus AutowareController::getTripStatusSync() const {
  return trip_ctrl_ ? trip_ctrl_->status() : TripStatus{};
}

void AutowareController::setTripStateCallback(std::function<void(TripState, TripState)> cb) {
  trip_state_callback_ = std::move(cb);
}

void AutowareController::setArrivalCallback(ArrivalCallback cb) {
  arrival_callback_ = std::move(cb);
}

void AutowareController::queryEta(GPSCoordinate start_gps, GPSCoordinate goal_gps,
                                  RouteQueryCallback callback) {
  if (!ready_.load()) {
    spdlog::warn("[AutowareAgent] queryEta rejected — controller not ready yet");
    if (callback)
      callback(EtaQueryResult{.success_ = false, .error_message_ = "controller not ready"});
    return;
  }
  boost::asio::post(*strand_, [this, start_gps, goal_gps, cb = std::move(callback)]() mutable {
    if (!trip_ctrl_) {
      if (cb)
        cb(EtaQueryResult{.success_ = false, .error_message_ = "not initialized"});
      return;
    }
    trip_ctrl_->queryEta(start_gps, goal_gps, std::move(cb));
  });
}

void AutowareController::moveImpl(std::function<void(bool)> callback) {
  if (!trip_ctrl_) {
    if (callback) {
      callback(false);
    }
    return;
  }
  bool const OK = trip_ctrl_->move();
  if (callback) {
    callback(OK);
  }
}

}  // namespace autoware_agent