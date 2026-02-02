#pragma once
#include <grpcpp/grpcpp.h>
#include <rclcpp/rclcpp.hpp>
#include <memory>
#include "bridge_context.hpp"
#include "vehicle_gateway.grpc.pb.h"

class SensorServiceImpl final : public vehicle_gateway::SensorService::Service {
private:
    std::shared_ptr<BridgeContext> context_;
    rclcpp::Logger logger_;

public:
    SensorServiceImpl(std::shared_ptr<BridgeContext> context, rclcpp::Logger logger);

    grpc::Status StreamSensorData(
        grpc::ServerContext* context,
        const vehicle_gateway::SensorSubscription* request,
        grpc::ServerWriter<vehicle_gateway::SensorData>* writer) override;
};