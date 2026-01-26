#pragma once

#include <rclcpp/rclcpp.hpp>
#include <grpcpp/grpcpp.h>
#include <atomic>
#include <memory>
#include <string>
#include <functional>

#include "vehicle_gateway.grpc.pb.h"
#include "vehicle_control.grpc.pb.h"
#include "bridge_metrics.hpp"
#include "gateway_config.hpp"

using grpc::Channel;
using grpc::ClientContext;
using grpc::Status;

/**
 * @class GatewayClient
 * @brief Manages all gRPC communication with the backend gateway.
 *
 * This class handles channel creation, keepalives, login, retries,
 * and provides a clean API for sending vehicle data. It is designed
 * to be thread-safe and resilient to network issues.
 */
class GatewayClient {
private:
    std::shared_ptr<Channel> channel_;
    std::unique_ptr<vehicle_gateway::VehicleGateway::Stub> data_stub_;
    std::unique_ptr<vehicle_control::VehicleControl::Stub> control_stub_; // For receiving commands

    GatewayConfig config_;
    std::shared_ptr<BridgeMetrics> metrics_;
    rclcpp::Logger logger_;

    std::atomic<bool> logged_in_{false};
    std::atomic<bool> connected_{false};

    // Generic RPC retry mechanism
    template<typename Stub, typename Request, typename Response>
    bool retry_rpc(
        const std::string& method_name,
        std::function<Status(Stub*, ClientContext*, const Request&, Response*)> rpc_func,
        Stub* stub,
        const Request& request,
        Response& response)
    {
        auto start_time = std::chrono::steady_clock::now();

        for (int attempt = 0; attempt < config_.max_retry_attempts; ++attempt) {
            ClientContext context;
            context.set_deadline(std::chrono::system_clock::now() + std::chrono::seconds(config_.connection_timeout_sec));

            Status status = rpc_func(stub, &context, request, response);

            if (status.ok()) {
                connected_ = true;
                auto end_time = std::chrono::steady_clock::now();
                double latency_ms = std::chrono::duration<double, std::milli>(end_time - start_time).count();
                metrics_->record_latency(latency_ms);

                RCLCPP_DEBUG(logger_, "%s succeeded in %.2fms", method_name.c_str(), latency_ms);
                return true;
            }

            RCLCPP_WARN(logger_, "%s failed (attempt %d/%d): [%d] %s - %s",
                        method_name.c_str(), attempt + 1, config_.max_retry_attempts,
                        status.error_code(), status.error_message().c_str(), "gRPC call failed.");

            metrics_->retry_count++;

            if (attempt < config_.max_retry_attempts - 1) {
                int delay_ms = config_.retry_delay_ms * (1 << attempt);
                std::this_thread::sleep_for(std::chrono::milliseconds(delay_ms));
            }
        }

        connected_ = false;
        metrics_->connection_failures++;
        RCLCPP_ERROR(logger_, "%s failed after %d attempts", method_name.c_str(), config_.max_retry_attempts);
        return false;
    }

public:
    GatewayClient(const GatewayConfig& config, std::shared_ptr<BridgeMetrics> metrics, rclcpp::Logger logger)
        : config_(config), metrics_(metrics), logger_(logger) {

        grpc::ChannelArguments args;
        args.SetInt(GRPC_ARG_KEEPALIVE_TIME_MS, config_.keepalive_time_ms);
        args.SetInt(GRPC_ARG_KEEPALIVE_TIMEOUT_MS, config_.keepalive_timeout_ms);
        args.SetInt(GRPC_ARG_KEEPALIVE_PERMIT_WITHOUT_CALLS, 1);
        args.SetInt(GRPC_ARG_MAX_RECEIVE_MESSAGE_LENGTH, config_.max_message_size);
        args.SetInt(GRPC_ARG_MAX_SEND_MESSAGE_LENGTH, config_.max_message_size);

        channel_ = grpc::CreateCustomChannel(config_.gateway_address, grpc::InsecureChannelCredentials(), args);
        data_stub_ = vehicle_gateway::VehicleGateway::NewStub(channel_);
        control_stub_ = vehicle_control::VehicleControl::NewStub(channel_);

        RCLCPP_INFO(logger_, "Gateway client initialized for address: %s", config_.gateway_address.c_str());
    }

    bool login() {
        if (!config_.enable_login) {
            logged_in_ = true;
            return true;
        }

        vehicle_gateway::LoginRequest request;
        request.set_vin_number(config_.vin_number);
        request.set_trip_id(config_.trip_id);

        vehicle_gateway::LoginResponse response; // Response object
        auto rpc_func = &vehicle_gateway::VehicleGateway::Stub::VehicleLogin;

        bool success = retry_rpc("VehicleLogin", rpc_func, data_stub_.get(), request, response);
        logged_in_ = success;

        if (success) {
            RCLCPP_INFO(logger_, "Vehicle logged in successfully: VIN %s", config_.vin_number.c_str());
        } else {
            RCLCPP_ERROR(logger_, "Vehicle login RPC failed for VIN %s", config_.vin_number.c_str());
        }
        return success;
    }

    bool send_position(double lat, double lon) {
        if (!logged_in_.load() && config_.enable_login) {
            RCLCPP_WARN_THROTTLE(logger_, *rclcpp::Clock::make_shared(), 5000, "Not logged in, skipping position update.");
            return false;
        }

        vehicle_gateway::ArriveRequest request;
        request.set_vin_number(config_.vin_number);
        request.set_trip_id(config_.trip_id);
        request.set_lat(lat);
        request.set_long_(lon);

        vehicle_gateway::ArriveResponse response;
        auto rpc_func = &vehicle_gateway::VehicleGateway::Stub::SendArrive;

        bool success = retry_rpc("SendArrive", rpc_func, data_stub_.get(), request, response);
        success ? metrics_->messages_sent++ : metrics_->messages_failed++;
        return success;
    }

    bool send_velocity(float speed) {
        if (!logged_in_.load() && config_.enable_login) {
            RCLCPP_WARN_THROTTLE(logger_, *rclcpp::Clock::make_shared(), 5000, "Not logged in, skipping velocity update.");
            return false;
        }

        vehicle_gateway::VelocityRequest request;
        request.set_vin_number(config_.vin_number);
        request.set_trip_id(config_.trip_id);
        request.set_velocity_mps(speed);

        vehicle_gateway::VelocityResponse response;
        auto rpc_func = &vehicle_gateway::VehicleGateway::Stub::SendVelocity;

        bool success = retry_rpc("SendVelocity", rpc_func, data_stub_.get(), request, response);
        success ? metrics_->messages_sent++ : metrics_->messages_failed++;
        return success;
    }

    bool send_status(const std::string& status_msg) {
        if (!config_.enable_status_updates) return true;

        vehicle_gateway::StatusRequest request;
        request.set_vin_number(config_.vin_number);
        request.set_trip_id(config_.trip_id);
        request.set_status(status_msg);

        vehicle_gateway::StatusResponse response;
        auto rpc_func = &vehicle_gateway::VehicleGateway::Stub::SendStatus;

        return retry_rpc("SendStatus", rpc_func, data_stub_.get(), request, response);
    }


    bool is_connected() {
        auto state = channel_->GetState(true);
        connected_ = (state == GRPC_CHANNEL_READY);
        return connected_.load();
    }

    bool is_logged_in() const {
        return logged_in_.load();
    }

    vehicle_control::VehicleControl::Stub* get_control_stub() {
        return control_stub_.get();
    }
};