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

#ifndef VEHICLEAUTOWAREAGENT_PERCEPTIONBRIDGE_H
#define VEHICLEAUTOWAREAGENT_PERCEPTIONBRIDGE_H

#include "FrameStates.h"
#include "vehicle_frame.pb.h"
#include "zenoh_publisher.h"

#include <autoware_perception_msgs/msg/predicted_objects.hpp>
#include <autoware_perception_msgs/msg/traffic_light_group_array.hpp>
#include <nav_msgs/msg/occupancy_grid.hpp>
#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/point_cloud2.hpp>

#include <boost/asio/executor_work_guard.hpp>
#include <boost/asio/io_context.hpp>
#include <boost/asio/steady_timer.hpp>
#include <boost/asio/strand.hpp>

#include <zenoh.hxx>

class PerceptionBridge : public AutowareAgent::ZenohPublisher {
 public:
  explicit PerceptionBridge(rclcpp::Node::SharedPtr node,
                            const std::shared_ptr<zenoh::Session>& zsession);

  ~PerceptionBridge();

  void shutdown();

 private:
  PerceptionFrameState state_;
  uint64_t frame_seq_{0};
  boost::asio::io_context io_context_;
  boost::asio::io_context::strand strand_;
  boost::asio::executor_work_guard<boost::asio::io_context::executor_type> work_guard_;
  std::thread io_thread_;
  // asio timer 60Hz
  boost::asio::steady_timer publisher_timer_;

  void scheduleNextTick();
  void onTick();

  // called on strand for grpc clients
  vehicle_frame::PerceptionFrame buildFrame();

  // called on strand for grpc clients
  void broadcastFrame(const vehicle_frame::PerceptionFrame& frame);

  // ros
  rclcpp::Node::SharedPtr node_;
  rclcpp::Subscription<autoware_perception_msgs::msg::PredictedObjects>::SharedPtr
    surrounding_object_sub_;
  rclcpp::Subscription<sensor_msgs::msg::PointCloud2>::SharedPtr point_cloud_sub_;
  rclcpp::Subscription<nav_msgs::msg::OccupancyGrid>::SharedPtr occupancy_grid_sub_;
  rclcpp::Subscription<autoware_perception_msgs::msg::TrafficLightGroupArray>::SharedPtr
    traffic_signal_sub_;

  // ROS callbacks that would be posted on strand
  void onSurroundingObject(autoware_perception_msgs::msg::PredictedObjects::SharedPtr msg);
  void onPointCloud(sensor_msgs::msg::PointCloud2::SharedPtr msg);
  void onOccupancyGrid(nav_msgs::msg::OccupancyGrid::SharedPtr msg);
  void onTrafficSignal(autoware_perception_msgs::msg::TrafficLightGroupArray::SharedPtr msg);

  // strand implementation

  void onSurroundingObjectImpl(
    const autoware_perception_msgs::msg::PredictedObjects::SharedPtr& msg);
  void onPointCloudImpl(const sensor_msgs::msg::PointCloud2::SharedPtr& msg);
  void onOccupancyGridImpl(const nav_msgs::msg::OccupancyGrid::SharedPtr& msg);
  void onTrafficSignalImpl(
    const autoware_perception_msgs::msg::TrafficLightGroupArray::SharedPtr& msg);

  static vehicle_frame::ObjectClass toObjectClass(uint8_t v);
  static vehicle_frame::TrafficLightColor toTrafficLightColor(uint8_t v);
};

#endif  // VEHICLEAUTOWAREAGENT_PERCEPTIONBRIDGE_H
