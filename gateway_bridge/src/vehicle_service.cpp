#include "vehicle_service.hpp"

VehicleServiceImpl::VehicleServiceImpl(std::shared_ptr<BridgeContext> context, rclcpp::Logger logger)
    : context_(context), logger_(logger) {
    RCLCPP_INFO(logger_, "VehicleServiceImpl initialized");
}

grpc::Status VehicleServiceImpl::StreamVehicleState(
    grpc::ServerContext* context,
    const vehicle_gateway::VehicleStreamRequest* request,
    grpc::ServerWriter<vehicle_gateway::VehicleState>* writer) {

    RCLCPP_INFO(logger_, "StreamVehicleState called for VIN: %s", request->vin_number().c_str());

    // TODO: Implement actual vehicle state streaming
    return grpc::Status::OK;
}

grpc::Status VehicleServiceImpl::SendControlCommand(
    grpc::ServerContext* context,
    const vehicle_gateway::VehicleControl* request,
    google::protobuf::Empty* response) {

    RCLCPP_INFO(logger_, "SendControlCommand: throttle=%.2f, steer=%.2f, brake=%.2f",
                request->throttle(), request->steer(), request->brake());

    // TODO: Send control commands to vehicle
    return grpc::Status::OK;
}