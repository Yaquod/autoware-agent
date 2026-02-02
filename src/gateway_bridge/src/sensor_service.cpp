#include "sensor_service.hpp"

SensorServiceImpl::SensorServiceImpl(std::shared_ptr<BridgeContext> context, rclcpp::Logger logger)
    : context_(context), logger_(logger) {
    RCLCPP_INFO(logger_, "SensorServiceImpl initialized");
}

grpc::Status SensorServiceImpl::StreamSensorData(
    grpc::ServerContext* context,
    const vehicle_gateway::SensorSubscription* request,
    grpc::ServerWriter<vehicle_gateway::SensorData>* writer) {

    RCLCPP_INFO(logger_, "StreamSensorData called for sensor: %s", request->sensor_id().c_str());

    // TODO: Implement actual sensor streaming
    // For now, just return OK
    return grpc::Status::OK;
}