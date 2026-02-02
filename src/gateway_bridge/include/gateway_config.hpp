#pragma once

#include <string>

/**
 * @struct GatewayConfig
 * @brief Holds all configuration parameters for the gRPC bridge server.
 */
struct GatewayConfig {
    std::string gateway_address;
    size_t max_message_size = 4 * 1024 * 1024;

    std::string carla_host;
    int carla_port;
    int carla_timeout_ms = 10000;

    std::string vin_number;
    std::string trip_id;

    double position_rate_hz = 50.0;
    size_t sensor_queue_size = 100;
};