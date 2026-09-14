#include <gtest/gtest.h>

#include <cstdint>
#include <vector>

#include "bev_preprocessor/gpu_image_stager.hpp"

TEST(GpuImageStager, bgrToPlanarLayoutIsExact)
{
  const std::size_t N = 2, H = 8, W = 10;
  bev::preprocessor::GpuImageStager stager(N, H, W);

  std::vector<std::vector<std::uint8_t>> raw(N, std::vector<std::uint8_t>(H * W * 3));
  for (std::size_t i = 0; i < N; ++i) {
    for (std::size_t p = 0; p < H * W; ++p) {
      // interleaved BGR with a recognizable ramp
      raw[i][p * 3 + 0] = static_cast<std::uint8_t>(p);        // B
      raw[i][p * 3 + 1] = static_cast<std::uint8_t>(255 - p);  // G
      raw[i][p * 3 + 2] = static_cast<std::uint8_t>(i);        // R
    }
  }
  std::vector<const std::uint8_t *> ptrs;
  for (auto & r : raw) ptrs.push_back(r.data());

  // default stream (nullptr)
  EXPECT_TRUE(stager.verifyLayout(ptrs, nullptr));
  EXPECT_EQ(stager.bytes(), N * H * W * 3);
  EXPECT_NE(stager.devicePtr(), nullptr);
}

TEST(GpuImageStager, rejectsWrongImageCount)
{
  bev::preprocessor::GpuImageStager stager(6, 4, 4);
  std::vector<std::uint8_t> one(4 * 4 * 3, 0);
  std::vector<const std::uint8_t *> ptrs{one.data()};
  EXPECT_NE(stager.stage(ptrs, nullptr), cudaSuccess);
}