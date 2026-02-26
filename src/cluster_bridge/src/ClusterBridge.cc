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

#include "ClusterBridge.h"

ClusterBridge::ClusterBridge(rclcpp::Node::SharedPtr node, const std::string& grpc_address) : node_(node),work_guard_(boost::asio::make_work_guard(io_context_))
,publisher_timer_(io_context_),strand_(io_context_)
{
  io_thread_ = std::thread([this]() {
    io_context_.run();
  });

  velocity_sub_ = node_->create_subscription<autoware_vehicle_msgs::msg::VelocityReport>("/vehicle/status/velocity_status",rclcpp::SensorDataQoS(),[this](const autoware_vehicle_msgs::msg::VelocityReport::SharedPtr msg) {
    // handling on velcity onVelocity(msg)
  });
}
