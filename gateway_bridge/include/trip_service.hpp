#pragma once
#include <grpcpp/grpcpp.h>
#include <rclcpp/rclcpp.hpp>
#include <memory>
#include "bridge_context.hpp"
#include "vehicle_gateway.grpc.pb.h"

class TripServiceImpl final : public vehicle_gateway::TripService::Service {
private:
    std::shared_ptr<BridgeContext> context_;
    rclcpp::Logger logger_;

public:
    TripServiceImpl(std::shared_ptr<BridgeContext> context, rclcpp::Logger logger);

    grpc::Status VehicleLogin(
        grpc::ServerContext* context,
        const vehicle_gateway::LoginRequest* request,
        vehicle_gateway::LoginResponse* response) override;

    grpc::Status SendEta(
        grpc::ServerContext* context,
        const vehicle_gateway::EtaRequest* request,
        vehicle_gateway::EtaResponse* response) override;

    grpc::Status SendStatus(
        grpc::ServerContext* context,
        const vehicle_gateway::StatusRequest* request,
        vehicle_gateway::StatusResponse* response) override;

    grpc::Status SendArrive(
        grpc::ServerContext* context,
        const vehicle_gateway::ArriveRequest* request,
        vehicle_gateway::ArriveResponse* response) override;

    grpc::Status StartTrip(
        grpc::ServerContext* context,
        const vehicle_gateway::StartTripRequest* request,
        vehicle_gateway::StartTripResponse* response) override;

    grpc::Status CancelTrip(
        grpc::ServerContext* context,
        const vehicle_gateway::TripCommand* request,
        vehicle_gateway::CancelTripResponse* response) override;

    grpc::Status GetTripStatus(
        grpc::ServerContext* context,
        const vehicle_gateway::TripStatusRequest* request,
        vehicle_gateway::TripStatusResponse* response) override;

    grpc::Status StreamTripUpdates(
        grpc::ServerContext* context,
        grpc::ServerReader<vehicle_gateway::TripUpdate>* reader,
        google::protobuf::Empty* response) override;
};