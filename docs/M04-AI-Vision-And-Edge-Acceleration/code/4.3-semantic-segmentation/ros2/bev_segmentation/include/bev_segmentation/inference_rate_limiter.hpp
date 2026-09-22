#pragma once

#include <chrono>
#include <stdexcept>

namespace bev_segmentation {

// Keeps the shared camera stream untouched while bounding how often this
// subscriber starts the expensive TensorRT path.  The caller supplies a
// steady-clock timestamp, so camera/ROS timestamps are never rewritten.
class InferenceRateLimiter {
public:
    using Clock = std::chrono::steady_clock;

    explicit InferenceRateLimiter(double max_fps = 30.0) {
        set_max_fps(max_fps);
    }

    void set_max_fps(double max_fps) {
        if (max_fps < 1.0 || max_fps > 30.0) {
            throw std::invalid_argument("max_inference_fps must be within [1, 30]");
        }
        interval_ = std::chrono::duration<double>(1.0 / max_fps);
        has_last_ = false;
    }

    bool should_process(Clock::time_point now) {
        if (!has_last_ || now - last_ >= interval_) {
            last_ = now;
            has_last_ = true;
            return true;
        }
        return false;
    }

private:
    std::chrono::duration<double> interval_{1.0 / 30.0};
    Clock::time_point last_{};
    bool has_last_{false};
};

}  // namespace bev_segmentation
