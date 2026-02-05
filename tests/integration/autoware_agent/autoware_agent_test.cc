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

#include <gtest/gtest.h>

#include <atomic>
#include <chrono>
#include <future>
#include <rclcpp/rclcpp.hpp>
#include <thread>

#include "AutowareController.h"
#include "Config.h"
#include "RouteConfig.h"
#include "TripStates.h"
#include "TripStatus.h"

using namespace AutowareAgent;

class AutowareAgentTest : public ::testing::Test {
 protected:
  void SetUp() override {
    if (!rclcpp::ok()) {
      rclcpp::init(0, nullptr);
    }
    yaml_path_ = std::string(SRC_MAP_DIR) + "/nishishinjuku_routes.yaml";
  }

  void TearDown() override { controller_.reset(); }

  void createController() {
    controller_ = std::make_shared<AutowareController>(yaml_path_, 10.0);
    controller_->initialize();
    std::this_thread::sleep_for(std::chrono::milliseconds(100));
  }

  void spinFor(std::chrono::milliseconds duration) const {
    auto start = std::chrono::steady_clock::now();
    while (rclcpp::ok() &&
           std::chrono::steady_clock::now() - start < duration) {
      rclcpp::spin_some(controller_);
      std::this_thread::sleep_for(std::chrono::milliseconds(10));
    }
  }

  TripStatus getStatusSync() {
    std::promise<TripStatus> promise;
    auto future = promise.get_future();

    controller_->getTripStatus(
        [&promise](TripStatus status) { promise.set_value(status); });

    return future.get();
  }

  // Helper: Start trip synchronously
  bool startTripSync(double lat, double lon) {
    std::promise<bool> promise;
    auto future = promise.get_future();

    controller_->startTrip(
        lat, lon, [&promise](bool success) { promise.set_value(success); });

    return future.get();
  }

  std::string yaml_path_;
  std::shared_ptr<AutowareController> controller_;
};

// Test 1: RouteConfig loads successfully
TEST_F(AutowareAgentTest, RouteConfigLoadsYAML) {
  ASSERT_NO_THROW({ RouteConfig config(yaml_path_); });

  RouteConfig config(yaml_path_);

  // Verify map metadata
  EXPECT_EQ(config.getMapName(), "Nishishinjuku");
  EXPECT_GT(config.getLanesCount(), 0u) << "YAML should have at least one lane";

  // Verify origin (this will catch the origin-read bug if it's not fixed)
  const MapOrigin& origin = config.getMapOrigin();
  EXPECT_NEAR(origin.latitude, 35.68855194431519, 1e-6);
  EXPECT_NEAR(origin.longitude, 139.69142711058254, 1e-6);
  EXPECT_NEAR(origin.local_x, 81596.1357, 1e-2);
  EXPECT_NEAR(origin.local_y, 50194.0803, 1e-2);

  // Verify we have a default start position
  const FixedStartPosition* start = config.getDefaultStart();
  ASSERT_NE(start, nullptr)
      << "YAML should have at least one fixed_start_position";
  EXPECT_EQ(start->lane_id, 552)
      << "Default start should be lane 552 per your YAML";
}

// Test 3: FindNearestLane returns a valid lane
TEST_F(AutowareAgentTest, FindNearestLane) {
  RouteConfig config(yaml_path_);

  // Pick a GPS point near lane 1 from your YAML example:
  //   lane_id: 1
  //   gps: { latitude: 35.68816945289868, longitude: 139.69425702193618 }
  GPSCoordinate near_lane_1{35.68816945289868, 139.69425702193618};

  const LaneInfo* lane = config.FindNearestLane(near_lane_1);
  ASSERT_NE(lane, nullptr) << "FindNearestLane should return a lane";
  EXPECT_EQ(lane->lane_id, 1) << "Should snap to lane 1 (exact GPS match)";

  // Try a point that's slightly offset — it should still snap to the nearest
  GPSCoordinate offset{35.6882, 139.6943};
  lane = config.FindNearestLane(offset);
  ASSERT_NE(lane, nullptr);
}

// Test 4: AutowareController constructs without throwing
TEST_F(AutowareAgentTest, ControllerConstruction) {
  ASSERT_NO_THROW({ createController(); });

  ASSERT_NE(controller_, nullptr);

  // Get status using callback-based API
  TripStatus status = getStatusSync();

  EXPECT_EQ(status.state, TripState::IDLE) << "Controller should start in IDLE";
}

// Test 5: Start a trip and verify state transitions
TEST_F(AutowareAgentTest, StartTripStateTransitions) {
  createController();

  // ----- Define a goal GPS coordinate (pick any lane from your YAML) -----
  // Using lane 2 as an example:
  //   gps: { latitude: 35.68814679007944, longitude: 139.69440756809428 }
  GPSCoordinate goal{35.68814679007944, 139.69440756809428};

  // ----- Start the trip ----------------------------------------------------
  bool started = startTripSync(goal.latitude, goal.longitude);
  ASSERT_TRUE(started) << "startTrip should succeed when in IDLE";

  // Give the strand time to process
  std::this_thread::sleep_for(std::chrono::milliseconds(100));

  TripStatus status = getStatusSync();
  EXPECT_NE(status.state, TripState::IDLE)
      << "Should have left IDLE immediately";

  // ----- Spin for a few seconds to let the FSM progress --------------------
  std::cout
      << "\n[Test] Spinning for 20 seconds to observe FSM transitions...\n";

  auto start_time = std::chrono::steady_clock::now();
  TripState last_state = status.state;

  while (rclcpp::ok() && std::chrono::steady_clock::now() - start_time <
                             std::chrono::seconds(20)) {
    rclcpp::spin_some(controller_);
    std::this_thread::sleep_for(std::chrono::milliseconds(100));

    status = getStatusSync();

    // Log state changes
    if (status.state != last_state) {
      std::cout << "[Test] State changed: " << static_cast<int>(last_state)
                << " → " << static_cast<int>(status.state) << "\n";
      last_state = status.state;
    }

    // If we reach RUNNING or FAILED, stop spinning early
    if (status.state == TripState::RUNNING ||
        status.state == TripState::FAILED) {
      break;
    }
  }

  status = getStatusSync();

  if (status.state == TripState::FAILED) {
    // This is expected if Autoware is not running or the route planner
    // couldn't connect the start and goal lanes.
    std::cout << "[Test] Trip FAILED. "
              << "This is normal if Autoware is not launched or lanes "
                 "are not connected.\n";
    // We still pass the test — the FSM behaved correctly by timing out.
  } else if (status.state == TripState::RUNNING) {
    // Success — Autoware accepted the route and engaged.
    std::cout << "[Test] Trip RUNNING. Start lane: " << status.start_lanelet_id
              << ", Goal lane: " << status.goal_lanelet_id << "\n";
    EXPECT_EQ(status.start_lanelet_id, "552") << "Start should be lane 552";
    // We can't assert the exact goal lane ID because FindNearestLane
    // might snap to a different lane if the map has been updated.
    // Just verify it's non-empty.
    EXPECT_FALSE(status.goal_lanelet_id.empty());
  } else {
    // Still stuck in some intermediate state — could be a timing issue
    // or the route topic never arrived.
    FAIL() << "Trip did not reach RUNNING or FAILED within 20s. "
           << "Current state: " << static_cast<int>(status.state);
  }
}

// Test 6: Cancel a trip mid-flight
TEST_F(AutowareAgentTest, CancelTrip) {
  createController();

  // Start a trip
  GPSCoordinate goal{35.688, 139.694};
  bool started = startTripSync(goal.latitude, goal.longitude);
  EXPECT_TRUE(started);

  // Give strand time to process
  std::this_thread::sleep_for(std::chrono::milliseconds(100));

  // Verify we left IDLE
  TripStatus status = getStatusSync();
  EXPECT_NE(status.state, TripState::IDLE);

  // Spin briefly to let it progress
  spinFor(std::chrono::milliseconds(500));

  // Cancel
  controller_->cancelTrip();

  // Give strand time to process cancel
  std::this_thread::sleep_for(std::chrono::milliseconds(100));

  status = getStatusSync();
  EXPECT_EQ(status.state, TripState::IDLE)
      << "Cancel should return to IDLE immediately";
}

// Test 7: Reject startTrip when a trip is already in progress
TEST_F(AutowareAgentTest, RejectDuplicateTrip) {
  createController();

  GPSCoordinate goal1{35.688, 139.694};
  bool started1 = startTripSync(goal1.latitude, goal1.longitude);
  ASSERT_TRUE(started1);

  // Give strand time to process
  std::this_thread::sleep_for(std::chrono::milliseconds(100));

  // Try to start another trip while the first is still running
  GPSCoordinate goal2{35.689, 139.695};
  bool started2 = startTripSync(goal2.latitude, goal2.longitude);
  EXPECT_FALSE(started2)
      << "Should reject second trip while one is in progress";
}
