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

#include "AutowareApp.h"
#include "AutowareController.h"
#include "AutowareControllerProvider.h"
#include "ClusterBridgeProvider.h"
#include "FrameStates.h"
#include "VehicleGatewayClient.h"
#include "VehicleGatewayService.h"
#include "map_routes/RouteConfig.h"
#include "vehicle_gateway.grpc.pb.h"

#include <rclcpp/rclcpp.hpp>

#include <atomic>
#include <chrono>
#include <condition_variable>
#include <mutex>
#include <random>
#include <thread>

#include <Config.h>
#include <grpcpp/grpcpp.h>
#include <grpcpp/server_builder.h>
#include <gtest/gtest.h>

using namespace std::chrono_literals;

class RecordingGatewayServer final : public vehicle_gateway::VehicleGateway::Service {
 public:
  vehicle_gateway::EtaRequest last_eta;
  vehicle_gateway::ArriveRequest last_arrive;
  vehicle_gateway::StatusRequest last_status;
  vehicle_gateway::TripInitRequest last_trip_init;
  vehicle_gateway::TripMoveRequest last_trip_move;
  vehicle_gateway::UpdateVehicleLocationRequest last_location;

  std::atomic<int> eta_count{0};
  std::atomic<int> arrive_count{0};
  std::atomic<int> status_count{0};
  std::atomic<int> trip_init_count{0};
  std::atomic<int> trip_move_count{0};
  std::atomic<int> location_count{0};

  std::mutex mtx;
  std::condition_variable cv;

  // Block until at least n calls of a given type have arrived.
  bool wait_eta(int n, std::chrono::milliseconds t = 5000ms) {
    return wait_(eta_count, n, t);
  }
  bool wait_arrive(int n, std::chrono::milliseconds t = 5000ms) {
    return wait_(arrive_count, n, t);
  }
  bool wait_status(int n, std::chrono::milliseconds t = 5000ms) {
    return wait_(status_count, n, t);
  }
  bool wait_trip_init(int n, std::chrono::milliseconds t = 5000ms) {
    return wait_(trip_init_count, n, t);
  }
  bool wait_trip_move(int n, std::chrono::milliseconds t = 5000ms) {
    return wait_(trip_move_count, n, t);
  }
  bool wait_location(int n, std::chrono::milliseconds t = 5000ms) {
    return wait_(location_count, n, t);
  }

  grpc::Status SendEta(grpc::ServerContext*, const vehicle_gateway::EtaRequest* req,
                       vehicle_gateway::EtaResponse*) override {
    std::lock_guard lk(mtx);
    last_eta = *req;
    ++eta_count;
    cv.notify_all();
    return grpc::Status::OK;
  }

  grpc::Status SendArrive(grpc::ServerContext*, const vehicle_gateway::ArriveRequest* req,
                          vehicle_gateway::ArriveResponse*) override {
    std::lock_guard lk(mtx);
    last_arrive = *req;
    ++arrive_count;
    cv.notify_all();
    return grpc::Status::OK;
  }

  grpc::Status SendStatus(grpc::ServerContext*, const vehicle_gateway::StatusRequest* req,
                          vehicle_gateway::StatusResponse*) override {
    std::lock_guard lk(mtx);
    last_status = *req;
    ++status_count;
    cv.notify_all();
    return grpc::Status::OK;
  }

  grpc::Status TripInit(grpc::ServerContext*, const vehicle_gateway::TripInitRequest* req,
                        vehicle_gateway::TripInitResponse*) override {
    std::lock_guard lk(mtx);
    last_trip_init = *req;
    ++trip_init_count;
    cv.notify_all();
    return grpc::Status::OK;
  }

  grpc::Status TripMove(grpc::ServerContext*, const vehicle_gateway::TripMoveRequest* req,
                        vehicle_gateway::TripMoveResponse*) override {
    std::lock_guard lk(mtx);
    last_trip_move = *req;
    ++trip_move_count;
    cv.notify_all();
    return grpc::Status::OK;
  }

  grpc::Status UpdateVehicleLocation(grpc::ServerContext*,
                                     const vehicle_gateway::UpdateVehicleLocationRequest* req,
                                     vehicle_gateway::UpdateVehicleLocationResponse*) override {
    std::lock_guard lk(mtx);
    last_location = *req;
    ++location_count;
    cv.notify_all();
    return grpc::Status::OK;
  }

 private:
  bool wait_(std::atomic<int>& counter, int n, std::chrono::milliseconds t) {
    std::unique_lock lk(mtx);
    return cv.wait_for(lk, t, [&] { return counter.load() >= n; });
  }
};

class GatewayTest : public ::testing::Test {
 protected:
  RecordingGatewayServer server_impl_;
  std::unique_ptr<grpc::Server> grpc_server_;
  std::string server_addr_;

  boost::asio::io_context retry_io_;
  std::thread retry_thread_;
  std::shared_ptr<autoware_agent::AutowareController> controller_;
  std::thread controller_spin_thread_;

  void SetUp() override {
    std::cerr << "[GatewayTest::SetUp] rclcpp::ok() = " << rclcpp::ok() << "\n";
    if (!rclcpp::ok()) {
      std::cerr << "[GatewayTest::SetUp] calling rclcpp::init()\n";
      rclcpp::init(0, nullptr);
    }
    int port = 0;
    grpc::ServerBuilder builder;
    builder.AddListeningPort("127.0.0.1:0", grpc::InsecureServerCredentials(), &port);
    builder.RegisterService(&server_impl_);
    grpc_server_ = builder.BuildAndStart();
    ASSERT_GT(port, 0) << "gRPC server failed to bind";
    server_addr_ = "127.0.0.1:" + std::to_string(port);

    retry_thread_ = std::thread([this] { retry_io_.run(); });
  }

  void TearDown() override {
    grpc_server_->Shutdown();
    // Shutdown ROS and stop any controller spin thread
    rclcpp::shutdown();
    if (controller_spin_thread_.joinable()) {
      controller_spin_thread_.join();
    }
    controller_.reset();
    retry_io_.stop();
    if (retry_thread_.joinable()) {
      retry_thread_.join();
    }
  }
};

TEST_F(GatewayTest, ClientConnectsTest) {
  vehicle_gateway::VehicleGatewayClient client(server_addr_);

  vehicle_gateway::EtaRequest req;
  req.set_vin_number("MY-VIN");
  req.set_request_id(1);
  req.set_time(60.0);

  std::atomic<bool> done{false};
  client.SendEta(req, [&](bool ok, const vehicle_gateway::EtaResponse&) {
    EXPECT_TRUE(ok) << "RPC failed — channel or server problem";
    done = true;
  });

  ASSERT_TRUE(server_impl_.wait_eta(1)) << "Server never received the call";
  EXPECT_EQ(server_impl_.last_eta.vin_number(), "MY-VIN");
}

TEST_F(GatewayTest, ReportLocationTest) {
  FrameState frame{};
  frame.latitude = 35.68814679007944;
  frame.longitude = 139.69440756809428;

  std::mutex frame_mtx;
  int64_t request_id = 0;
  std::string const YAML_PATH =
    std::string(autoware_agent::SRC_MAP_DIR) + "/nishishinjuku_routes.yaml";
  // AutowareAgent::GPSCoordinate goal_gps = AutowareAgent::GPSCoordinate{35.68814679007944,
  // 139.69440756809428};

  std::cerr << "[Test] before creating controller, rclcpp::ok() = " << rclcpp::ok() << "\n";
  // Create a controller instance but do NOT call initialize(). The
  // ReportLocation test only needs the controller object to construct the
  // AutowareControllerTripAdapter; initializing the controller kicks off
  // async work that creates subscriptions and can throw in minimal
  // environments, which would abort the whole test run. By not calling
  // initialize() we keep the test robust.
  controller_ = std::make_shared<autoware_agent::AutowareController>(YAML_PATH, 10.0);

  auto location_provider =
    std::make_shared<vehicle_gateway::ClusterLocationAdapter>(frame, frame_mtx);
  auto eta_provider =
    std::make_shared<vehicle_gateway::ClusterEtaAdapter>(frame, frame_mtx, request_id);

  auto trip_manager_provider =
    std::make_shared<vehicle_gateway::AutowareControllerTripAdapter>(controller_);

  vehicle_gateway::VehicleGatewayService gateway(server_addr_, eta_provider, location_provider,
                                                 trip_manager_provider, "MY-VIN", retry_io_);

  gateway.ReportLocation();

  ASSERT_TRUE(server_impl_.wait_location(1));
  {
    std::scoped_lock const lk(server_impl_.mtx);
    EXPECT_DOUBLE_EQ(server_impl_.last_location.latitude(), frame.latitude);
    EXPECT_DOUBLE_EQ(server_impl_.last_location.longitude(), frame.longitude);
  }
}

TEST_F(GatewayTest, TripInitRpcRandomPoints) {
  // Start the full Autoware app so tests exercise the public wiring.
  if (!rclcpp::ok())
    rclcpp::init(0, nullptr);
  std::string const YAML_PATH =
    std::string(autoware_agent::SRC_MAP_DIR) + "/nishishinjuku_routes.yaml";
  autoware_agent::AppHandles app = autoware_agent::startAutowareApp(YAML_PATH, server_addr_);

  // Load lanes from yaml and pick two random distinct lanes as start/end
  autoware_agent::RouteConfig rc(YAML_PATH);
  const auto& lanes = rc.getAllLanes();
  ASSERT_GE(lanes.size(), 2u) << "Need at least two lanes in YAML to pick random start/end";

  // pick two distinct random indices (fixed-seed for deterministic tests)
  std::mt19937 gen(12345);
  std::uniform_int_distribution<size_t> dist(0, lanes.size() - 1);
  const size_t i = dist(gen);
  size_t j = dist(gen);
  while (j == i && lanes.size() > 1)
    j = dist(gen);

  auto start = lanes[i].gps;
  auto end = lanes[j].gps;

  // Use plain VehicleGatewayClient to issue the TripInit RPC to the in-test server.
  vehicle_gateway::VehicleGatewayClient client(server_addr_);

  vehicle_gateway::TripInitRequest req;
  req.set_vin_number("MY-VIN");
  req.set_request_id(42);
  req.set_start_long(start.longitude);
  req.set_start_lat(start.latitude);
  req.set_end_long(end.longitude);
  req.set_end_lat(end.latitude);

  std::atomic<bool> done{false};
  client.TripInit(req, [&](bool ok, const vehicle_gateway::TripInitResponse&) {
    EXPECT_TRUE(ok) << "TripInit RPC failed";
    done = true;
  });

  ASSERT_TRUE(server_impl_.wait_trip_init(1)) << "Server never received TripInit";
  {
    std::scoped_lock const lk(server_impl_.mtx);
    EXPECT_DOUBLE_EQ(server_impl_.last_trip_init.start_lat(), start.latitude);
    EXPECT_DOUBLE_EQ(server_impl_.last_trip_init.start_long(), start.longitude);
    EXPECT_DOUBLE_EQ(server_impl_.last_trip_init.end_lat(), end.latitude);
    EXPECT_DOUBLE_EQ(server_impl_.last_trip_init.end_long(), end.longitude);
  }

  // Clean up the app
  rclcpp::shutdown();
  autoware_agent::stopAutowareApp(app);
}
