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
#include "Config.h"
#include "RouteConfig.h"
#include "TripController.h"
#include "TripStates.h"
#include "TripStatus.h"
#include "cluster_bridge/include/ClusterBridge.h"

#include <autoware_planning_msgs/msg/lanelet_route.hpp>
#include <geometry_msgs/msg/pose_stamped.hpp>
#include <geometry_msgs/msg/pose_with_covariance_stamped.hpp>
#include <nav_msgs/msg/odometry.hpp>
#include <rclcpp/rclcpp.hpp>

#include <atomic>
#include <chrono>
#include <future>
#include <thread>

#include <gtest/gtest.h>
#include <zenoh.hxx>

using namespace autoware_agent;
using namespace std::chrono_literals;

class FullTripTest : public ::testing::Test {
 protected:
  void SetUp() override {
    if (!rclcpp::ok()) {
      rclcpp::init(0, nullptr);
    }

    auto z_config = zenoh::Config::create_default();
    z_session_ = std::make_shared<zenoh::Session>(zenoh::Session::open(std::move(z_config)));

    yaml_path_ = std::string(SRC_MAP_DIR) + "/nishishinjuku_routes.yaml";
    goal_gps_ = GPSCoordinate{35.68814679007944, 139.69440756809428};
  }

  void TearDown() override {
    if (cluster_bridge_) {
      cluster_bridge_->shutdown();
    }

    z_sub_.reset();

    controller_.reset();
    monitor_node_.reset();
    z_session_.reset();

    if (rclcpp::ok()) {
      rclcpp::shutdown();
    }

    if (controller_spin_thread_.joinable()) {
      controller_spin_thread_.join();
    }
  }

  TripStatus getStatusSync() {
    std::promise<TripStatus> promise;
    auto future = promise.get_future();
    controller_->getTripStatus([&promise](TripStatus status) { promise.set_value(status); });
    return future.get();
  }

  bool startTripSync(double lat, double lon) {
    std::promise<bool> promise;
    auto future = promise.get_future();
    controller_->startTrip(lat, lon, [&promise](bool success) { promise.set_value(success); });
    return future.get();
  }

  void createController() {
    controller_ = std::make_shared<AutowareController>(yaml_path_, 10.0);
    controller_->initialize();

    // Pass the Zenoh Session to ClusterBridge
    auto node_ptr = std::static_pointer_cast<rclcpp::Node>(controller_);
    cluster_bridge_ = std::make_shared<ClusterBridge>(node_ptr, z_session_);

    monitor_node_ = std::make_shared<rclcpp::Node>("trip_monitor");

    controller_spin_thread_ = std::thread([this]() { rclcpp::spin(controller_); });

    auto qos = rclcpp::QoS(rclcpp::KeepLast(10)).reliable();

    initial_pose_sub_ =
      monitor_node_->create_subscription<geometry_msgs::msg::PoseWithCovarianceStamped>(
        "/initialpose", qos,
        [this](const geometry_msgs::msg::PoseWithCovarianceStamped::SharedPtr msg) {
          initial_pose_received_ = true;
          last_initial_pose_ = *msg;
          std::cout << "  [Monitor] Initial pose received at (" << msg->pose.pose.position.x << ", "
                    << msg->pose.pose.position.y << ")\n";
        });

    goal_sub_ = monitor_node_->create_subscription<geometry_msgs::msg::PoseStamped>(
      "/planning/mission_planning/goal", qos,
      [this](const geometry_msgs::msg::PoseStamped::SharedPtr msg) {
        goal_received_ = true;
        last_goal_ = *msg;
        std::cout << "  [Monitor] Goal received at (" << msg->pose.position.x << ", "
                  << msg->pose.position.y << ")\n";
      });

    route_sub_ = monitor_node_->create_subscription<autoware_planning_msgs::msg::LaneletRoute>(
      "/planning/mission_planning/route", qos,
      [this](const autoware_planning_msgs::msg::LaneletRoute::SharedPtr msg) {
        route_received_ = true;
        std::cout << "  [Monitor] Route received! Segments: " << msg->segments.size() << "\n";
      });

    odom_sub_ = monitor_node_->create_subscription<nav_msgs::msg::Odometry>(
      "/localization/kinematic_state", qos, [this](const nav_msgs::msg::Odometry::SharedPtr msg) {
        last_odom_ = *msg;
        odom_received_ = true;

        double speed = std::sqrt(msg->twist.twist.linear.x * msg->twist.twist.linear.x +
                                 msg->twist.twist.linear.y * msg->twist.twist.linear.y);

        if (speed > 0.1 && !vehicle_is_moving_) {
          vehicle_is_moving_ = true;
          std::cout << "  [Monitor] Vehicle started moving! Speed: " << speed << " m/s\n";
        }
      });

    // Listens to wildcard autoware topics to prove bridging is working during motion
    z_sub_ = std::make_unique<zenoh::Subscriber<void>>(z_session_->declare_subscriber(
      "autoware/**", [this](const zenoh::Sample& /*sample*/) { zenoh_msgs_count_++; }, []() {}));

    // Give initialization time
    std::this_thread::sleep_for(500ms);
  }

  void spinFor(std::chrono::milliseconds duration,
               std::function<bool()> break_condition = nullptr) {
    auto start = std::chrono::steady_clock::now();
    while (rclcpp::ok() && std::chrono::steady_clock::now() - start < duration) {
      rclcpp::spin_some(monitor_node_);
      std::this_thread::sleep_for(10ms);
      if (break_condition && break_condition())
        break;
    }
  }

  bool hasArrivedAtGoal(double threshold_m = 2.0) {
    if (!odom_received_)
      return false;

    TripStatus status = getStatusSync();
    double dx = last_odom_.pose.pose.position.x - status.goal_x;
    double dy = last_odom_.pose.pose.position.y - status.goal_y;
    double distance = std::sqrt(dx * dx + dy * dy);

    return distance < threshold_m;
  }

  // Configurations
  std::string yaml_path_;
  GPSCoordinate goal_gps_;

  // Components
  std::shared_ptr<zenoh::Session> z_session_;
  std::shared_ptr<void> z_sub_;
  std::shared_ptr<AutowareController> controller_;
  std::shared_ptr<rclcpp::Node> monitor_node_;
  std::shared_ptr<ClusterBridge> cluster_bridge_;
  std::thread controller_spin_thread_;

  // Subscriptions
  rclcpp::Subscription<geometry_msgs::msg::PoseWithCovarianceStamped>::SharedPtr initial_pose_sub_;
  rclcpp::Subscription<geometry_msgs::msg::PoseStamped>::SharedPtr goal_sub_;
  rclcpp::Subscription<autoware_planning_msgs::msg::LaneletRoute>::SharedPtr route_sub_;
  rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr odom_sub_;

  // State flags
  std::atomic<bool> initial_pose_received_{false};
  std::atomic<bool> goal_received_{false};
  std::atomic<bool> route_received_{false};
  std::atomic<bool> odom_received_{false};
  std::atomic<bool> vehicle_is_moving_{false};
  std::atomic<int> zenoh_msgs_count_{0};

  // Last received states
  geometry_msgs::msg::PoseWithCovarianceStamped last_initial_pose_;
  geometry_msgs::msg::PoseStamped last_goal_;
  nav_msgs::msg::Odometry last_odom_;
};

TEST_F(FullTripTest, CompleteTripLifecycle) {
  createController();

  std::cout << "\n========================================\n";
  std::cout << "Full Trip Integration Test\n";
  std::cout << "========================================\n";

  // Phase 1: Start trip
  std::cout << "\n[Phase 1] Starting trip...\n";
  bool started = startTripSync(goal_gps_.latitude, goal_gps_.longitude);
  ASSERT_TRUE(started) << "Failed to start trip";

  spinFor(200ms);

  TripStatus status = getStatusSync();
  std::cout << "  Start lane: " << status.start_lanelet_id << "\n";
  std::cout << "  Goal lane:  " << status.goal_lanelet_id << "\n";
  std::cout << "  Initial state: " << static_cast<int>(status.state) << "\n";

  // Phase 2: Wait for initial pose
  std::cout << "\n[Phase 2] Waiting for initial pose...\n";
  spinFor(5s, [this]() { return initial_pose_received_.load(); });

  if (!initial_pose_received_) {
    std::cout << "  WARNING: Initial pose not detected by monitor\n";
  }

  status = getStatusSync();
  std::cout << "  Current state: " << static_cast<int>(status.state) << "\n";

  // Phase 3: Wait for goal
  std::cout << "\n[Phase 3] Waiting for goal publication...\n";
  spinFor(10s, [this]() { return goal_received_.load(); });

  if (!goal_received_) {
    std::cout << "  WARNING: Goal not detected by monitor\n";
  }

  status = getStatusSync();
  std::cout << "  Current state: " << static_cast<int>(status.state) << "\n";

  // Phase 4: Wait for route and engage
  std::cout << "\n[Phase 4] Waiting for route planning...\n";
  std::cout << "  Monitoring /planning/mission_planning/route topic...\n";

  TripState last_state = status.state;

  spinFor(35s, [this, &last_state]() {
    TripStatus s = getStatusSync();
    if (s.state != last_state) {
      std::cout << "  State transition: " << static_cast<int>(last_state) << " -> "
                << static_cast<int>(s.state) << "\n";
      last_state = s.state;
    }
    return s.state == TripState::RUNNING || s.state == TripState::FAILED;
  });

  status = getStatusSync();
  std::cout << "  Final state after phase 4: " << static_cast<int>(status.state) << "\n";
  std::cout << "  Monitor route flag: " << route_received_.load() << "\n";

  if (status.state == TripState::FAILED) {
    FAIL() << "Trip failed during route planning. Debugging info:\n"
           << "  - Initial pose received: " << initial_pose_received_ << "\n"
           << "  - Goal received: " << goal_received_ << "\n"
           << "  - Route received: " << route_received_ << "\n";
  }

  // Phase 5: Verify motion and Zenoh streaming
  if (status.state >= TripState::ENGAGING) {
    std::cout << "\n[Phase 5] Verifying vehicle motion & Zenoh streaming...\n";

    int initial_zenoh_count = zenoh_msgs_count_.load();

    spinFor(30s, [this, initial_zenoh_count]() {
      bool is_moving = vehicle_is_moving_.load();
      bool is_streaming = (zenoh_msgs_count_.load() > initial_zenoh_count + 5);
      return is_moving && is_streaming;
    });

    if (vehicle_is_moving_) {
      std::cout << "  ✓ Vehicle motion detected via ROS2.\n";
    } else {
      std::cout << "  ✗ Vehicle not moving yet.\n";
    }

    int received_streams = zenoh_msgs_count_.load() - initial_zenoh_count;
    if (received_streams > 0) {
      std::cout << "  ✓ Zenoh is streaming data (" << received_streams << " samples received).\n";
    } else {
      std::cout << "  ✗ Zenoh stream inactive.\n";
    }
  }

  // Summary
  std::cout << "\n========================================\n";
  std::cout << "Test Summary\n";
  std::cout << "========================================\n";
  std::cout << "Initial pose:     " << (initial_pose_received_ ? "✓" : "✗") << "\n";
  std::cout << "Goal published:   " << (goal_received_ ? "✓" : "✗") << "\n";
  std::cout << "Route received:   " << (route_received_ ? "✓" : "✗") << "\n";
  std::cout << "Final state:      " << static_cast<int>(status.state) << "\n";
  std::cout << "Vehicle moving:   " << (vehicle_is_moving_ ? "✓" : "✗") << "\n";
  std::cout << "Zenoh Streaming:  " << (zenoh_msgs_count_.load() > 0 ? "✓" : "✗") << "\n";
  std::cout << "========================================\n\n";
  std::cout << "Test complete. Press ENTER to exit...\n";
  std::cin.get();
}

// Simpler test for debugging
TEST_F(FullTripTest, BasicPublishTest) {
  createController();

  std::cout << "\n[Test] Basic publish test\n";

  bool started = startTripSync(goal_gps_.latitude, goal_gps_.longitude);
  ASSERT_TRUE(started);

  std::cout << "  Spinning for 10 seconds to observe publications...\n";

  for (int i = 0; i < 100; ++i) {
    rclcpp::spin_some(monitor_node_);
    std::this_thread::sleep_for(100ms);

    TripStatus status = getStatusSync();
    if (i % 10 == 0) {
      std::cout << "  t=" << i / 10 << "s  State=" << static_cast<int>(status.state)
                << "  InitPose=" << initial_pose_received_ << "  Goal=" << goal_received_
                << "  Route=" << route_received_ << "  ZenohMsgs=" << zenoh_msgs_count_.load()
                << "\n";
    }
  }

  std::cout << "  Test complete - check output above\n";
}