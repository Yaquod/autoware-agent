#pragma once
#include <grpcpp/grpcpp.h>
#include <rclcpp/rclcpp.hpp>
#include <memory>
#include "bridge_context.hpp"
#include "vehicle_gateway.grpc.pb.h"

class VehicleServiceImpl final : public vehicle_gateway::VehicleService::Service {
private:
    std::shared_ptr<BridgeContext> context_;
    rclcpp::Logger logger_;

public:
    VehicleServiceImpl(std::shared_ptr<BridgeContext> context, rclcpp::Logger logger);

    grpc::Status StreamVehicleState(
        grpc::ServerContext* context,
        const vehicle_gateway::VehicleStreamRequest* request,
        grpc::ServerWriter<vehicle_gateway::VehicleState>* writer) override;

    grpc::Status SendControlCommand(
        grpc::ServerContext* context,
        const vehicle_gateway::VehicleControl* request,
        google::protobuf::Empty* response) override;
};