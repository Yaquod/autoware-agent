#pragma once

#include <atomic>
#include <chrono>
#include <rclcpp/rclcpp.hpp>

/**
 * @struct BridgeMetrics
 * @brief A thread-safe container for collecting and reporting bridge performance metrics.
 */
struct BridgeMetrics {
    std::atomic<uint64_t> messages_received{0};
    std::atomic<uint64_t> messages_sent{0};
    std::atomic<uint64_t> messages_failed{0};
    std::atomic<uint64_t> messages_dropped{0};
    std::atomic<uint64_t> retry_count{0};
    std::atomic<uint64_t> connection_failures{0};

    std::atomic<double> avg_latency_ms{0.0};
    std::atomic<double> max_latency_ms{0.0};

    std::chrono::steady_clock::time_point start_time;

    BridgeMetrics() : start_time(std::chrono::steady_clock::now()) {}

    void record_latency(double latency_ms) {
        // Update max latency
        double current_max = max_latency_ms.load();
        while (latency_ms > current_max && !max_latency_ms.compare_exchange_weak(current_max, latency_ms)) {
            // Loop in case of concurrent update
        }

        // Update average latency using an exponential moving average
        double alpha = 0.2;
        double current_avg = avg_latency_ms.load();
        double new_avg = alpha * latency_ms + (1.0 - alpha) * current_avg;
        avg_latency_ms.store(new_avg);
    }

    double uptime_seconds() const {
        auto now = std::chrono::steady_clock::now();
        return std::chrono::duration<double>(now - start_time).count();
    }

    double success_rate() const {
        uint64_t total = messages_sent.load() + messages_failed.load();
        return total > 0 ? (100.0 * messages_sent.load() / total) : 100.0;
    }

    double throughput_hz() const {
        double uptime = uptime_seconds();
        return uptime > 0 ? (messages_sent.load() / uptime) : 0.0;
    }

    void print(rclcpp::Logger logger) const {
        RCLCPP_INFO(logger, "--- Bridge Statistics (Uptime: %.0fs) ---", uptime_seconds());
        RCLCPP_INFO(logger, "  [Messages] Received: %lu | Sent: %lu (%.1f%%) | Failed: %lu | Dropped: %lu",
                    messages_received.load(),
                    messages_sent.load(),
                    success_rate(),
                    messages_failed.load(),
                    messages_dropped.load());

        RCLCPP_INFO(logger, "  [Performance] Throughput: %.2f Hz | Avg Latency: %.2f ms | Max Latency: %.2f ms",
                    throughput_hz(),
                    avg_latency_ms.load(),
                    max_latency_ms.load());

        RCLCPP_INFO(logger, "  [Connection] gRPC Retries: %lu | Connection Drops: %lu",
                    retry_count.load(),
                    connection_failures.load());
        RCLCPP_INFO(logger, "------------------------------------------");
    }
};