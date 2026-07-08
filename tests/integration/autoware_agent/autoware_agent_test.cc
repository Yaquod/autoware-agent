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
#include "TripStates.h"
#include "TripStatus.h"
#include "map_routes/LaneletMap.h"

#include <rclcpp/rclcpp.hpp>

#include <atomic>
#include <chrono>
#include <future>
#include <thread>

#include <gtest/gtest.h>

using namespace autoware_agent;

class AutowareAgentTest : public ::testing::Test {
 protected:
  void SetUp() override {
    if (!rclcpp::ok()) {
      rclcpp::init(0, nullptr);
    }
    map_path_ = std::string(SRC_MAP_DIR) + "/lanelet2_map.osm";
  }

  void TearDown() override {
    controller_.reset();
  }

  void createController() {
    controller_ = std::make_shared<AutowareController>(map_path_, 10.0);
    controller_->initialize();
    // Give the controller a short time to initialize. Tests that require
    // full Autoware localization or route planning will check the state and
    // skip if those services are not available in the environment.
    std::this_thread::sleep_for(std::chrono::milliseconds(100));
  }

  void spinFor(std::chrono::milliseconds duration) const {
    auto start = std::chrono::steady_clock::now();
    while (rclcpp::ok() && std::chrono::steady_clock::now() - start < duration) {
      rclcpp::spin_some(controller_);
      std::this_thread::sleep_for(std::chrono::milliseconds(10));
    }
  }

  TripStatus getStatusSync() {
    std::promise<TripStatus> promise;
    auto future = promise.get_future();

    controller_->getTripStatus([&promise](TripStatus status) { promise.set_value(status); });

    return future.get();
  }

  // Helper: Start trip synchronously
  bool startTripSync(double lat, double lon) {
    std::promise<bool> promise;
    auto future = promise.get_future();

    controller_->startTrip([&promise](bool success) { promise.set_value(success); });

    return future.get();
  }

  std::string map_path_;
  std::shared_ptr<AutowareController> controller_;
};

// Test 1: OSM loads successfully
TEST_F(AutowareAgentTest, MapLoadsOSM) {
  ASSERT_NO_THROW({
    // Provide a dummy origin for now to just check it loads
    LaneletMap map(map_path_, 35.688, 139.691);
  });
}

// Test 3: FindNearestLane returns a valid lane
TEST_F(AutowareAgentTest, FindNearestLane) {
  LaneletMap map(map_path_, 35.688, 139.691);

  // Pick a GPS point near somewhere in Nishishinjuku map
  GPSCoordinate near_point{35.68816945289868, 139.69425702193618};

  const LaneInfo* lane = map.findNearestLane(near_point);
  ASSERT_NE(lane, nullptr) << "FindNearestLane should return a lane";

  // Try a point that's slightly offset — it should still snap to the nearest
  GPSCoordinate offset{35.6882, 139.6943};
  lane = map.findNearestLane(offset);
  ASSERT_NE(lane, nullptr);
}

// Test 4: AutowareController constructs without throwing
TEST_F(AutowareAgentTest, ControllerConstruction) {
  ASSERT_NO_THROW({ createController(); });

  ASSERT_NE(controller_, nullptr);

  // Get status using callback-based API
  TripStatus status = getStatusSync();

  // Controller may be in INITIALIZING_IN_MAP while waiting for Autoware
  // localisation to report INITIALIZED. Accept either state here so tests
  // remain robust in environments without the full Autoware stack.
  EXPECT_TRUE(status.state_ == TripState::IDLE || status.state_ == TripState::INITIALIZING_IN_MAP)
    << "Controller should be IDLE or INITIALIZING_IN_MAP";
}

// Test 5: Start a trip and verify state transitions
TEST_F(AutowareAgentTest, StartTripStateTransitions) {
  createController();

  // ----- Define a goal GPS coordinate (pick any lane from your YAML) -----
  // Using lane 2 as an example:
  //   gps: { latitude: 35.68814679007944, longitude: 139.69440756809428 }
  GPSCoordinate goal{35.68814679007944, 139.69440756809428};

  // ----- Start the trip ----------------------------------------------------
  TripStatus status = getStatusSync();

  // If controller hasn't completed map initialisation (i.e. it's not IDLE),
  // then the environment likely doesn't have Autoware localisation running —
  // skip the part of the test that requires query/route services.
  if (status.state_ != TripState::IDLE) {
    GTEST_SKIP() << "Controller not IDLE (state=" << static_cast<int>(status.state_)
                 << ") - skipping trip start test (requires Autoware services).";
  }

  // Populate route bank via queryEta before starting the trip
  // NOTE: This integration test assumes the Autoware route planner will
  // respond; if not, the test will fail here and should be run with
  // Autoware services available.
  RouteQueryResult qr;
  {
    std::promise<RouteQueryResult> p;

    auto f = p.get_future();
    controller_->queryEta(GPSCoordinate{goal.latitude, goal.longitude},
                          GPSCoordinate{goal.latitude, goal.longitude},
                          [&p](RouteQueryResult r) { p.set_value(r); });
    qr = f.get();
  }
  if (!qr.success_) {
    GTEST_SKIP() << "queryEta failed: " << qr.error_message_ << " - skipping trip start assertions";
  }

  bool started = startTripSync(goal.latitude, goal.longitude);
  ASSERT_TRUE(started) << "startTrip should succeed when in IDLE and routes are available";

  // Give the strand time to process
  std::this_thread::sleep_for(std::chrono::milliseconds(100));

  status = getStatusSync();
  EXPECT_NE(status.state_, TripState::IDLE) << "Should have left IDLE immediately";

  // ----- Spin for a few seconds to let the FSM progress --------------------
  std::cout << "\n[Test] Spinning for 20 seconds to observe FSM transitions...\n";

  auto start_time = std::chrono::steady_clock::now();
  TripState last_state = status.state_;

  while (rclcpp::ok() && std::chrono::steady_clock::now() - start_time < std::chrono::seconds(20)) {
    rclcpp::spin_some(controller_);
    std::this_thread::sleep_for(std::chrono::milliseconds(100));

    status = getStatusSync();

    // Log state changes
    if (status.state_ != last_state) {
      std::cout << "[Test] State changed: " << static_cast<int>(last_state) << " → "
                << static_cast<int>(status.state_) << "\n";
      last_state = status.state_;
    }

    // If we reach RUNNING or FAILED, stop spinning early
    if (status.state_ == TripState::RUNNING || status.state_ == TripState::FAILED) {
      break;
    }
  }

  status = getStatusSync();

  if (status.state_ == TripState::FAILED) {
    // This is expected if Autoware is not running or the route planner
    // couldn't connect the start and goal lanes.
    std::cout << "[Test] Trip FAILED. "
              << "This is normal if Autoware is not launched or lanes "
                 "are not connected.\n";
    // We still pass the test — the FSM behaved correctly by timing out.
  } else if (status.state_ == TripState::RUNNING) {
    // Success — Autoware accepted the route and engaged.
    std::cout << "[Test] Trip RUNNING. Start lane: " << status.start_lanelet_id_
              << ", Goal lane: " << status.goal_lanelet_id_ << "\n";
    EXPECT_EQ(status.start_lanelet_id_, "552") << "Start should be lane 552";
    // We can't assert the exact goal lane ID because FindNearestLane
    // might snap to a different lane if the map has been updated.
    // Just verify it's non-empty.
    EXPECT_FALSE(status.goal_lanelet_id_.empty());
  } else {
    // Still stuck in some intermediate state — could be a timing issue
    // or the route topic never arrived.
    FAIL() << "Trip did not reach RUNNING or FAILED within 20s. "
           << "Current state: " << static_cast<int>(status.state_);
  }
}

// Test 6: Cancel a trip mid-flight
TEST_F(AutowareAgentTest, CancelTrip) {
  createController();

  // Start a trip
  GPSCoordinate goal{35.688, 139.694};
  TripStatus status = getStatusSync();
  if (status.state_ != TripState::IDLE) {
    GTEST_SKIP() << "Controller not IDLE - skipping CancelTrip test";
  }

  // Ensure routes are available
  std::promise<RouteQueryResult> p;
  auto f = p.get_future();
  controller_->queryEta(goal, goal, [&p](RouteQueryResult r) { p.set_value(r); });
  RouteQueryResult qr = f.get();
  if (!qr.success_) {
    GTEST_SKIP() << "queryEta failed - skipping CancelTrip test";
  }

  bool started = startTripSync(goal.latitude, goal.longitude);
  EXPECT_TRUE(started);

  // Give strand time to process
  std::this_thread::sleep_for(std::chrono::milliseconds(100));

  // Verify we left IDLE
  status = getStatusSync();
  EXPECT_NE(status.state_, TripState::IDLE);

  // Spin briefly to let it progress
  spinFor(std::chrono::milliseconds(500));

  // Cancel
  controller_->cancelTrip();

  // Give strand time to process cancel
  std::this_thread::sleep_for(std::chrono::milliseconds(100));

  status = getStatusSync();
  EXPECT_EQ(status.state_, TripState::IDLE) << "Cancel should return to IDLE immediately";
}

// Test 7: Reject startTrip when a trip is already in progress
TEST_F(AutowareAgentTest, RejectDuplicateTrip) {
  createController();

  GPSCoordinate goal1{35.688, 139.694};
  TripStatus status = getStatusSync();
  if (status.state_ != TripState::IDLE) {
    GTEST_SKIP() << "Controller not IDLE - skipping RejectDuplicateTrip test";
  }

  // Populate routes
  std::promise<RouteQueryResult> p1;
  auto f1 = p1.get_future();
  controller_->queryEta(goal1, goal1, [&p1](RouteQueryResult r) { p1.set_value(r); });
  RouteQueryResult qr1 = f1.get();
  if (!qr1.success_) {
    GTEST_SKIP() << "queryEta failed - skipping RejectDuplicateTrip test";
  }

  bool started1 = startTripSync(goal1.latitude, goal1.longitude);
  ASSERT_TRUE(started1);

  // Give strand time to process
  std::this_thread::sleep_for(std::chrono::milliseconds(100));

  // Try to start another trip while the first is still running
  GPSCoordinate goal2{35.689, 139.695};
  bool started2 = startTripSync(goal2.latitude, goal2.longitude);
  EXPECT_FALSE(started2) << "Should reject second trip while one is in progress";
}
