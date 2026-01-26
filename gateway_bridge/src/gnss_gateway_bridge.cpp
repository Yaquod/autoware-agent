#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/nav_sat_fix.hpp>
#include <grpcpp/grpcpp.h>
#include <thread>
#include <memory>

// #include "carla/client/Client.h"

// Service Implementations
#include "sensor_service.hpp"
#include "vehicle_service.hpp"
#include "trip_service.hpp"

// Shared data context between ROS and gRPC
#include "bridge_context.hpp"


class GatewayNode : public rclcpp::Node {
private:
    // gRPC
    std::unique_ptr<grpc::Server> grpc_server_;
    std::thread grpc_thread_;
    std::string server_address_;

    // CARLA
    // std::unique_ptr<carla::client::Client> carla_client_;

    // ROS2
    rclcpp::Subscription<sensor_msgs::msg::NavSatFix>::SharedPtr gnss_sub_;

    // Shared data context
    std::shared_ptr<BridgeContext> context_;

    // Service implementations
    std::unique_ptr<SensorServiceImpl> sensor_service_;
    std::unique_ptr<VehicleServiceImpl> vehicle_service_;
    std::unique_ptr<TripServiceImpl> trip_service_;


    void start_grpc_server() {
        grpc::ServerBuilder builder;
        builder.AddListeningPort(server_address_, grpc::InsecureServerCredentials());

        sensor_service_ = std::make_unique<SensorServiceImpl>(context_, this->get_logger());
        vehicle_service_ = std::make_unique<VehicleServiceImpl>(context_, this->get_logger());
        trip_service_ = std::make_unique<TripServiceImpl>(context_, this->get_logger());

        builder.RegisterService(sensor_service_.get());
        builder.RegisterService(vehicle_service_.get());
        builder.RegisterService(trip_service_.get());

        grpc_server_ = builder.BuildAndStart();
        RCLCPP_INFO(this->get_logger(), "gRPC server listening on %s", server_address_.c_str());
        grpc_server_->Wait();
    }

    // void connect_to_carla() {
    //     try {
    //         std::string carla_host = this->get_parameter("carla_host").as_string();
    //         int carla_port = this->get_parameter("carla_port").as_int();
    //         RCLCPP_INFO(this->get_logger(), "Connecting to CARLA at %s:%d...", carla_host.c_str(), carla_port);
    //         // carla_client_ = std::make_unique<carla::client::Client>(carla_host, carla_port);
    //         // carla_client_->SetTimeout(std::chrono::seconds(10));
    //         // context_->carla_client = carla_client_.get();
    //         RCLCPP_INFO(this->get_logger(), "Successfully connected to CARLA Server. Version: %s", carla_client_->GetServerVersion().c_str());
    //     } catch (const std::exception& e) {
    //         RCLCPP_ERROR(this->get_logger(), "Failed to connect to CARLA: %s", e.what());
    //         // Decide if this is a fatal error
    //     }
    // }

    void gnss_callback(const sensor_msgs::msg::NavSatFix::SharedPtr msg) {
        context_->gnss_queue->push(msg);
    }

public:
    GatewayNode() : Node("gateway_bridge_node") {
        this->declare_parameter<std::string>("grpc_listen_address", "0.0.0.0:50051");
        this->declare_parameter<std::string>("carla_host", "localhost");
        this->declare_parameter<int>("carla_port", 2000);

        server_address_ = this->get_parameter("grpc_listen_address").as_string();
        context_ = std::make_shared<BridgeContext>();

        // connect_to_carla();

        grpc_thread_ = std::thread([this]() { this->start_grpc_server(); });

        gnss_sub_ = this->create_subscription<sensor_msgs::msg::NavSatFix>(
            "/sensing/gnss/fix",
            rclcpp::QoS(10).best_effort(),
            std::bind(&GatewayNode::gnss_callback, this, std::placeholders::_1)
        );

        RCLCPP_INFO(this->get_logger(), "ROS2 to gRPC Gateway Node has started.");
    }

    ~GatewayNode() {
        if (grpc_server_) {
            grpc_server_->Shutdown();
        }
        if (grpc_thread_.joinable()) {
            grpc_thread_.join();
        }
    }
};

int main(int argc, char** argv) {
    rclcpp::init(argc, argv);
    try {
        auto node = std::make_shared<GatewayNode>();
        rclcpp::spin(node);
    } catch (const std::exception& e) {
        RCLCPP_FATAL(rclcpp::get_logger("main"), "Fatal error: %s", e.what());
        return 1;
    }
    rclcpp::shutdown();
    return 0;
}