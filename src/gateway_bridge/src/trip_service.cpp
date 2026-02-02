#include "trip_service.hpp"

TripServiceImpl::TripServiceImpl(std::shared_ptr<BridgeContext> context, rclcpp::Logger logger)
    : context_(context), logger_(logger) {
    RCLCPP_INFO(logger_, "TripServiceImpl initialized");
}

grpc::Status TripServiceImpl::VehicleLogin(
    grpc::ServerContext* context,
    const vehicle_gateway::LoginRequest* request,
    vehicle_gateway::LoginResponse* response) {

    RCLCPP_INFO(logger_, "VehicleLogin: VIN=%s, TripID=%s",
                request->vin_number().c_str(), request->trip_id().c_str());

    response->set_success(true);
    response->set_message("Login successful");
    return grpc::Status::OK;
}

grpc::Status TripServiceImpl::SendEta(
    grpc::ServerContext* context,
    const vehicle_gateway::EtaRequest* request,
    vehicle_gateway::EtaResponse* response) {

    RCLCPP_INFO(logger_, "SendEta: TripID=%s, ETA=%.2f min",
                request->trip_id().c_str(), request->time());

    response->set_success(true);
    response->set_message("ETA updated");
    return grpc::Status::OK;
}

grpc::Status TripServiceImpl::SendStatus(
    grpc::ServerContext* context,
    const vehicle_gateway::StatusRequest* request,
    vehicle_gateway::StatusResponse* response) {

    RCLCPP_INFO(logger_, "SendStatus: TripID=%s, Status=%s",
                request->trip_id().c_str(), request->status().c_str());

    response->set_success(true);
    response->set_message("Status updated");
    return grpc::Status::OK;
}

grpc::Status TripServiceImpl::SendArrive(
    grpc::ServerContext* context,
    const vehicle_gateway::ArriveRequest* request,
    vehicle_gateway::ArriveResponse* response) {

    RCLCPP_INFO(logger_, "SendArrive: TripID=%s at (%.6f, %.6f)",
                request->trip_id().c_str(), request->lat(), request->long_());

    response->set_success(true);
    response->set_message("Arrival confirmed");
    return grpc::Status::OK;
}

grpc::Status TripServiceImpl::StartTrip(
    grpc::ServerContext* context,
    const vehicle_gateway::StartTripRequest* request,
    vehicle_gateway::StartTripResponse* response) {

    RCLCPP_INFO(logger_, "StartTrip: TripID=%s, VIN=%s, Passenger=%s",
                request->trip_id().c_str(),
                request->vin_number().c_str(),
                request->passenger_id().c_str());

    response->set_success(true);
    response->set_message("Trip started");
    response->set_trip_id(request->trip_id());
    response->set_trip_state(vehicle_gateway::TRIP_STATE_EN_ROUTE_PICKUP);
    response->set_progress_percent(0.0);

    return grpc::Status::OK;
}

grpc::Status TripServiceImpl::CancelTrip(
    grpc::ServerContext* context,
    const vehicle_gateway::TripCommand* request,
    vehicle_gateway::CancelTripResponse* response) {

    RCLCPP_INFO(logger_, "CancelTrip: TripID=%s", request->trip_id().c_str());

    response->set_success(true);
    response->set_message("Trip cancelled");
    return grpc::Status::OK;
}

grpc::Status TripServiceImpl::GetTripStatus(
    grpc::ServerContext* context,
    const vehicle_gateway::TripStatusRequest* request,
    vehicle_gateway::TripStatusResponse* response) {

    RCLCPP_INFO(logger_, "GetTripStatus: TripID=%s", request->trip_id().c_str());

    response->set_success(true);
    response->set_message("Status retrieved");
    response->set_state(vehicle_gateway::TRIP_STATE_EN_ROUTE_DESTINATION);
    response->set_progress_percent(45.0);

    return grpc::Status::OK;
}

grpc::Status TripServiceImpl::StreamTripUpdates(
    grpc::ServerContext* context,
    grpc::ServerReader<vehicle_gateway::TripUpdate>* reader,
    google::protobuf::Empty* response) {

    vehicle_gateway::TripUpdate update;
    while (reader->Read(&update)) {
        RCLCPP_INFO(logger_, "TripUpdate received: TripID=%s, State=%d",
                    update.trip_id().c_str(), update.trip_state());
    }

    return grpc::Status::OK;
}