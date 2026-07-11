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

#include "LocationBridge.h"

#include <boost/asio/bind_executor.hpp>
#include <nlohmann/json.hpp>


#include <algorithm>
#include <memory>
#include <utility>



LocationBridge::LocationBridge(rclcpp::Node::SharedPtr node,
                                const std::shared_ptr<zenoh::Session>& zsession , GpsProvider gps_provider , TripIdProvider trip_id_provider)
  : ZenohPublisher(zsession, "autoware/location")
  , gps_provider_(std::move(gps_provider))
  ,trip_id_provider_(std::move(trip_id_provider)) 
  , strand_(io_context_)
  , work_guard_(boost::asio::make_work_guard(io_context_))
  , publisher_timer_(io_context_)
  , node_(std::move(node)) {
  RCLCPP_INFO(node_->get_logger(), "[LocationBridge] Booting..");
  io_thread_ = std::thread([this]() { io_context_.run(); });

  auto sensor_qos = rclcpp::SensorDataQoS();



  boost::asio::post(strand_, [this]() { scheduleNextTick(); });
  RCLCPP_INFO(node_->get_logger(), "[LocationBridge] Ready...");
}

LocationBridge::~LocationBridge() {
  shutdown();
}

void LocationBridge::shutdown() {
  publisher_timer_.cancel();
  work_guard_.reset();

  if (io_thread_.joinable()) {
    io_thread_.join();
  }
}

void LocationBridge::scheduleNextTick() {
  publisher_timer_.expires_after(std::chrono::seconds(1));  // 1 hz
  publisher_timer_.async_wait(
    boost::asio::bind_executor(strand_, [this](const boost::system::error_code& ec) {
      if (ec == boost::asio::error::operation_aborted) {
        return;
      }
      onTick();
    }));
}

void LocationBridge::onTick() {

    if (gps_provider_) {
    auto gps = gps_provider_();
    if (gps.has_value()) {
      current_lat_ = gps->first;
      current_lon_ = gps->second;
    }
  }



  auto t0 = std::chrono::steady_clock::now();
if (streaming_enabled_) {
    broadcastFrame(buildFrame());
}
  auto dt = std::chrono::steady_clock::now() - t0;
  if (dt > std::chrono::milliseconds(5)) {
    RCLCPP_WARN(node_->get_logger(), "[LocationBridge] ontick delayed %ldms",
                std::chrono::duration_cast<std::chrono::milliseconds>(dt).count());
  }
  scheduleNextTick();
}

vehicle_frame::LocationFrame LocationBridge::buildFrame() {
  vehicle_frame::LocationFrame frame;
  frame.set_stamp_ns(node_->now().nanoseconds());
  frame.set_seq(frame_seq_++);
    auto* loc = frame.mutable_vehicle_location();
  loc->set_latitude(current_lat_);
  loc->set_longitude(current_lon_);




  return frame;
}

void LocationBridge::broadcastFrame(const vehicle_frame::LocationFrame& frame) {
  nlohmann::json j = {
    {"latitude", current_lat_},
    {"longitude", current_lon_},
     {"tripId",    trip_id_provider_ ? trip_id_provider_() : 0}
};
publish(j.dump());

}

void LocationBridge::setStreamingEnabled(bool enabled)
{
    boost::asio::post(strand_, [this, enabled]()
    {
        streaming_enabled_ = enabled;

    });
}






// TODO: complete all on-##nameImpl functions







