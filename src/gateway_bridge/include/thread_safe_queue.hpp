#pragma once

#include <queue>
#include <mutex>
#include <condition_variable>
#include <memory>
#include <chrono>

/**
 * @class ThreadSafeQueue
 * @brief A generic, thread-safe queue with a maximum size and rate limiting on pops.
 */
template<typename T>
class ThreadSafeQueue {
private:
    std::queue<std::shared_ptr<T>> queue_;
    mutable std::mutex mutex_;
    std::condition_variable cv_;
    size_t max_size_;
    std::chrono::milliseconds min_interval_;
    std::chrono::steady_clock::time_point last_pop_;
    std::atomic<uint64_t>& dropped_count_ref_;

public:
    ThreadSafeQueue(size_t max_size, double rate_hz, std::atomic<uint64_t>& dropped_counter)
        : max_size_(max_size),
          min_interval_(rate_hz > 0 ? static_cast<int>(1000.0 / rate_hz) : 0),
          last_pop_(std::chrono::steady_clock::now()),
          dropped_count_ref_(dropped_counter) {}

    void push(std::shared_ptr<T> item) {
        std::lock_guard<std::mutex> lock(mutex_);
        if (queue_.size() >= max_size_) {
            queue_.pop(); // Drop the oldest item
            dropped_count_ref_++;
        }
        queue_.push(item);
        cv_.notify_one();
    }

    std::shared_ptr<T> pop(std::chrono::milliseconds timeout = std::chrono::milliseconds(100)) {
        std::unique_lock<std::mutex> lock(mutex_);

        // Rate limiting
        if (min_interval_.count() > 0) {
            auto now = std::chrono::steady_clock::now();
            auto elapsed = now - last_pop_;
            if (elapsed < min_interval_) {
                auto wait_time = min_interval_ - elapsed;
                cv_.wait_for(lock, wait_time);
            }
        }

        if (!cv_.wait_for(lock, timeout, [this] { return !queue_.empty(); })) {
            return nullptr;
        }

        std::shared_ptr<T> item = queue_.front();
        queue_.pop();
        last_pop_ = std::chrono::steady_clock::now();
        return item;
    }
};