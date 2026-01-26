#pragma once

#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/nav_sat_fix.hpp>
#include <autoware_auto_vehicle_msgs/msg/velocity_report.hpp>

#include "gateway_client.hpp"
#include "thread_safe_queue.hpp"

/**
 * @class IDataExposer
 * @brief Interface for classes that subscribe to ROS topics and expose data to the gateway.
 */
class IDataExposer {
public:
    virtual ~IDataExposer() = default;
    // Each exposer is responsible for its own setup (e.g., subscriptions)
    virtual void initialize() = 0;
};

/**
 * @class GnssExposer
 * @brief Subscribes to GNSS data, queues it, and sends it via the GatewayClient.
 */
class GnssExposer : public IDataExposer, public std::enable_shared_from_this<GnssExposer> {
private:
    rclcpp::Node* node_; // Raw pointer to the main node
    std::shared_ptr<GatewayClient> gateway_client_;
    std::unique_ptr<ThreadSafeQueue<sensor_msgs::msg::NavSatFix>> queue_;
    rclcpp::Subscription<sensor_msgs::msg::NavSatFix>::SharedPtr gnss_sub_;
    std::thread worker_thread_;
    std::atomic<bool> running_{true};

    void gnss_callback(const sensor_msgs::msg::NavSatFix::SharedPtr msg) {
        if (msg->status.status < 0) {
            RCLCPP_WARN_THROTTLE(node_->get_logger(), *node_->get_clock(), 5000, "GNSS fix not available.");
            return;
        }
        queue_->push(std::make_shared<sensor_msgs::msg::NavSatFix>(*msg));
    }

    void worker_loop() {
        RCLCPP_INFO(node_->get_logger(), "GNSS Exposer worker thread started.");
        while (running_) {
            auto msg = queue_->pop();
            if (msg) {
                gateway_client_->send_position(msg->latitude, msg->longitude);
            }
        }
        RCLCPP_INFO(node_->get_logger(), "GNSS Exposer worker thread stopped.");
    }

public:
    GnssExposer(rclcpp::Node* node, std::shared_ptr<GatewayClient> client, const GatewayConfig& config)
        : node_(node), gateway_client_(client)
    {
        // The queue is now created in initialize() to access shared metrics from the node
    }

    ~GnssExposer() {
        running_ = false;
        if (worker_thread_.joinable()) {
            worker_thread_.join();
        }
    }

    void initialize() override {
        // This allows access to the node's members after construction
        auto parent_node = dynamic_cast<GatewayNode*>(node_);
        if (!parent_node) throw std::runtime_error("Node is not a GatewayNode");

        queue_ = std::make_unique<ThreadSafeQueue<sensor_msgs::msg::NavSatFix>>(parent_node->get_config().queue_size, parent_node->get_config().position_rate_hz, parent_node->get_metrics()->messages_dropped);
        auto qos = rclcpp::QoS(parent_node->get_config().qos_depth).best_effort();
        gnss_sub_ = node_->create_subscription<sensor_msgs::msg::NavSatFix>("/sensing/gnss/fix", qos, std::bind(&GnssExposer::gnss_callback, this, std::placeholders::_1));
        worker_thread_ = std::thread(&GnssExposer::worker_loop, this);
    }
};