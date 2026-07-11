/*
 * Copyright 2026 alaa
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

#ifndef VEHICLEAUTOWAREAGENT_LOCATIONBRIDGE_H
#define VEHICLEAUTOWAREAGENT_LOCATIONBRIDGE_H

#include "FrameStates.h"
#include "vehicle_frame.pb.h"
#include "zenoh_publisher.h"

#include <nav_msgs/msg/odometry.hpp>
#include <rclcpp/rclcpp.hpp>

#include <boost/asio/executor_work_guard.hpp>
#include <boost/asio/io_context.hpp>
#include <boost/asio/steady_timer.hpp>
#include <boost/asio/strand.hpp>

#include <zenoh.hxx>

class LocationBridge : public autoware_agent::ZenohPublisher {
 public:
  // GPS provider function type, returns optional pair of latitude and longitude
  using GpsProvider = std::function<std::optional<std::pair<double, double>>()>;
  using TripIdProvider = std::function<int64_t()>;

  explicit LocationBridge(rclcpp::Node::SharedPtr node,
                          const std::shared_ptr<zenoh::Session>& zsession, GpsProvider gps_provider,
                          TripIdProvider trip_id_provider);

  ~LocationBridge();

  void setStreamingEnabled(bool enabled);

  void shutdown();

 private:
  LocationFrameState state_;
  uint64_t frame_seq_{0};
  boost::asio::io_context io_context_;
  boost::asio::io_context::strand strand_;
  boost::asio::executor_work_guard<boost::asio::io_context::executor_type> work_guard_;
  std::thread io_thread_;

  GpsProvider gps_provider_;
  TripIdProvider trip_id_provider_;
  double current_lat_{0.0};
  double current_lon_{0.0};
  // asio timer 60Hz
  boost::asio::steady_timer publisher_timer_;

  void scheduleNextTick();
  void onTick();

  // called on strand for grpc clients
  vehicle_frame::LocationFrame buildFrame();

  // called on strand for grpc clients
  void broadcastFrame(const vehicle_frame::LocationFrame& frame);

  // ros
  rclcpp::Node::SharedPtr node_;

  // handle location streaming
  bool streaming_enabled_{false};
};

#endif  // VEHICLEAUTOWAREAGENT_PERCEPTIONBRIDGE_H
