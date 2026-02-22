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

#include "map_routes/RouteConfig.h"

#include <fstream>

#include <gtest/gtest.h>

using namespace AutowareAgent;

class RouteConfigTest : public ::testing::Test {
 protected:
  RouteConfigTest() : route_config_("test_routes.yaml") {}

  void SetUp() override {
    // Create a dummy yaml file for testing
    std::ofstream test_yaml("test_routes.yaml");
    test_yaml << "map_info:\n";
    test_yaml << "  name: \"TestMap\"\n";
    test_yaml << "origin:\n";
    test_yaml << "  longitude: 139.6917\n";
    test_yaml << "  latitude: 35.6895\n";
    test_yaml << "  local_x: 0.0\n";
    test_yaml << "  local_y: 0.0\n";
    test_yaml << "fixed_start_positions:\n";
    test_yaml << "  - name: \"start1\"\n";
    test_yaml << "    lane_id: 101\n";
    test_yaml << "    gps:\n";
    test_yaml << "      longitude: 139.692\n";
    test_yaml << "      latitude: 35.690\n";
    test_yaml << "    local:\n";
    test_yaml << "      x: 27.3\n";
    test_yaml << "      y: 55.6\n";
    test_yaml << "      z: 0.0\n";
    test_yaml << "    orientation:\n";
    test_yaml << "      x: 0.0\n";
    test_yaml << "      y: 0.0\n";
    test_yaml << "      z: 0.707\n";
    test_yaml << "      w: 0.707\n";
    test_yaml << "lanes:\n";
    test_yaml << "  - lane_id: 101\n";
    test_yaml << "    gps:\n";
    test_yaml << "      longitude: 139.692\n";
    test_yaml << "      latitude: 35.690\n";
    test_yaml << "    local:\n";
    test_yaml << "      x: 27.3\n";
    test_yaml << "      y: 55.6\n";
    test_yaml << "      z: 0.0\n";
    test_yaml << "    orientation:\n";
    test_yaml << "      z: 0.707\n";
    test_yaml << "      w: 0.707\n";
    test_yaml << "      yaw_degrees: 90.0\n";
    test_yaml << "  - lane_id: 102\n";
    test_yaml << "    gps:\n";
    test_yaml << "      longitude: 139.693\n";
    test_yaml << "      latitude: 35.691\n";
    test_yaml << "    local:\n";
    test_yaml << "      x: 118.4\n";
    test_yaml << "      y: 166.8\n";
    test_yaml << "      z: 0.0\n";
    test_yaml << "    orientation:\n";
    test_yaml << "      z: 0.0\n";
    test_yaml << "      w: 1.0\n";
    test_yaml << "      yaw_degrees: 0.0\n";
    test_yaml.close();
  }

  void TearDown() override {
    std::remove("test_routes.yaml");
  }

  RouteConfig route_config_;
};

TEST_F(RouteConfigTest, GetMapName) {
  ASSERT_EQ(route_config_.getMapName(), "TestMap");
}

TEST_F(RouteConfigTest, GetLaneCount) {
  ASSERT_EQ(route_config_.getLanesCount(), 2);
}

TEST_F(RouteConfigTest, GetLaneByID) {
  const LaneInfo* lane = route_config_.getLaneByID(101);
  ASSERT_NE(lane, nullptr);
  EXPECT_EQ(lane->lane_id, 101);
  EXPECT_NEAR(lane->local.x, 27.3, 1e-5);

  const LaneInfo* not_found_lane = route_config_.getLaneByID(999);
  ASSERT_EQ(not_found_lane, nullptr);
}

TEST_F(RouteConfigTest, FindNearestLane) {
  GPSCoordinate gps{35.690, 139.692};  // Close to lane 101
  const LaneInfo* nearest_lane = route_config_.FindNearestLane(gps);
  ASSERT_NE(nearest_lane, nullptr);
  EXPECT_EQ(nearest_lane->lane_id, 101);
}

TEST_F(RouteConfigTest, GetDefaultStart) {
  const FixedStartPosition* start = route_config_.getDefaultStart();
  ASSERT_NE(start, nullptr);
  EXPECT_EQ(start->name, "start1");
  EXPECT_EQ(start->lane_id, 101);
}
