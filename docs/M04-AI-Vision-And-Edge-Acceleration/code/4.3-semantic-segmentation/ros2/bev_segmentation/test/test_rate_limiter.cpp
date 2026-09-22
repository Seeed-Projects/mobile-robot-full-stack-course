#include <gtest/gtest.h>

#include "bev_segmentation/inference_rate_limiter.hpp"

using bev_segmentation::InferenceRateLimiter;

TEST(InferenceRateLimiterTest, BoundsAndSkipsWithoutChangingClockSource) {
    EXPECT_THROW(InferenceRateLimiter(0.0), std::invalid_argument);
    EXPECT_THROW(InferenceRateLimiter(30.1), std::invalid_argument);

    InferenceRateLimiter limiter(10.0);
    const auto t0 = InferenceRateLimiter::Clock::time_point{};
    EXPECT_TRUE(limiter.should_process(t0));
    EXPECT_FALSE(limiter.should_process(t0 + std::chrono::milliseconds(99)));
    EXPECT_TRUE(limiter.should_process(t0 + std::chrono::milliseconds(100)));
}

TEST(InferenceRateLimiterTest, ThirtyFpsAllowsThirtyFpsCadence) {
    InferenceRateLimiter limiter(30.0);
    const auto t0 = InferenceRateLimiter::Clock::time_point{};
    EXPECT_TRUE(limiter.should_process(t0));
    EXPECT_TRUE(limiter.should_process(t0 + std::chrono::milliseconds(34)));
}
