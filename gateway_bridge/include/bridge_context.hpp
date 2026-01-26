#pragma once

#include <memory>
#include <atomic>
#include "thread_safe_queue.hpp"
#include <sensor_msgs/msg/nav_sat_fix.hpp>

// Forward declare carla client
// namespace carla {
namespace client {
    class Client;
}
// }

struct BridgeContext {
    // CARLA client instance, owned by the main GatewayNode
    // carla::client::Client* carla_client = nullptr;

    // A counter for all dropped GNSS messages across the application.
    std::atomic<uint64_t> dropped_gnss_counter{0};

    // Queue for GNSS data.
    std::shared_ptr<ThreadSafeQueue<sensor_msgs::msg::NavSatFix>> gnss_queue;

    BridgeContext() {
        gnss_queue = std::make_shared<ThreadSafeQueue<sensor_msgs::msg::NavSatFix>>(100, 10.0, dropped_gnss_counter);
    }
};