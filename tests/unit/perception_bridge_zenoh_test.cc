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

#include "perception_bridge/include/PerceptionBridge.h"

#include <rclcpp/rclcpp.hpp>

#include <gtest/gtest.h>
#include <zenoh.hxx>

// This is a basic test skeleton for PerceptionBridge zenoh streaming
// You may need to mock ROS messages and zenoh session for full coverage

class PerceptionBridgeZenohTest : public ::testing::Test {
 protected:
  void SetUp() override {
    if (!rclcpp::ok())
      rclcpp::init(0, nullptr);
    node_ = std::make_shared<rclcpp::Node>("test_perception_bridge");
    zsession_ = zenoh::open();  // You may want to mock or use a test zenoh session
    bridge_ = std::make_unique<PerceptionBridge>(node_, zsession_);
  }
  void TearDown() override {
    bridge_->shutdown();
    bridge_.reset();
    zsession_.reset();
    node_.reset();
  }
  std::shared_ptr<rclcpp::Node> node_;
  std::shared_ptr<zenoh::Session> zsession_;
  std::unique_ptr<PerceptionBridge> bridge_;
};

TEST_F(PerceptionBridgeZenohTest, StartupAndShutdown) {
  ASSERT_TRUE(bridge_);
  // Optionally, publish a dummy message and check zenoh for output
}

// Add more tests for streaming, publishing, and message conversion as needed
