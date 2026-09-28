// test/test_postprocessing.cpp
//
// 单元测试: postprocess (argmax + nearest-neighbor resize)
//
// 不需要 TensorRT / CUDA; 只需 bev_segmentation_core (postprocess.cpp).
// 复用 M4.1 test_letterbox 的同款 testing style.

#include <gtest/gtest.h>
#include "bev_segmentation/postprocess.hpp"
#include "bev_segmentation/types.hpp"

using namespace bev_segmentation;

TEST(PostprocessTest, ArgmaxSimpleKnownLogits) {
    // 2x2 spatial, 3 classes:
    //   logits[c, h, w] (CHW)
    //   c=0: [[0,1],[1,1]]  c=1: [[5,2],[2,3]]  c=2: [[0,0],[0,0]]
    //   expected: [[1,1],[1,1]] (all class 1; (1,1) tie broken by setting c=1=3)
    std::vector<float> logits = {
        // c=0
        0.f, 1.f, 1.f, 1.f,
        // c=1
        5.f, 2.f, 2.f, 3.f,
        // c=2
        0.f, 0.f, 0.f, 0.f,
    };
    auto m = argmax_classes(logits, 2, 2, 3);
    ASSERT_EQ(m.size(), 4u);
    EXPECT_EQ(m[0], 1); EXPECT_EQ(m[1], 1);
    EXPECT_EQ(m[2], 1); EXPECT_EQ(m[3], 1);
}

TEST(PostprocessTest, ArgmaxClassOrder) {
    // 1x1 spatial, 3 classes: c=2 wins
    std::vector<float> logits = {0.f, 1.f, 5.f};
    auto m = argmax_classes(logits, 1, 1, 3);
    ASSERT_EQ(m.size(), 1u);
    EXPECT_EQ(m[0], 2);
}

TEST(PostprocessTest, ArgmaxOutputShape) {
    // 128x256 spatial, 19 classes — output length = 128*256 = 32768
    const int H = 128, W = 256, C = 19;
    std::vector<float> logits(static_cast<size_t>(C) * H * W, 0.f);
    auto m = argmax_classes(logits, H, W, C);
    EXPECT_EQ(m.size(), static_cast<size_t>(H) * W);
    for (auto v : m) { EXPECT_GE(v, 0); EXPECT_LE(v, 18); }
}

TEST(PostprocessTest, NearestNeighborResizeUpsample) {
    // 1x1 -> 2x2 (all same value)
    std::vector<uint8_t> src = {7};
    auto dst = nearest_neighbor_resize(src, 1, 1, 2, 2);
    ASSERT_EQ(dst.size(), 4u);
    EXPECT_EQ(dst[0], 7); EXPECT_EQ(dst[1], 7);
    EXPECT_EQ(dst[2], 7); EXPECT_EQ(dst[3], 7);
}

TEST(PostprocessTest, NearestNeighborResizeIdentity) {
    // 2x3 -> 2x3 (no rescale)
    std::vector<uint8_t> src = {0,1,2, 3,4,5};
    auto dst = nearest_neighbor_resize(src, 2, 3, 2, 3);
    ASSERT_EQ(dst.size(), 6u);
    EXPECT_EQ(dst[0], 0); EXPECT_EQ(dst[5], 5);
}

TEST(PostprocessTest, NearestNeighborResizeLogitToOrig) {
    // 128x256 -> 480x640 (typical camera resolution)
    const int src_h = 128, src_w = 256;
    std::vector<uint8_t> src(static_cast<size_t>(src_h) * src_w, 0);
    src[0] = 9;  // top-left pixel
    auto dst = nearest_neighbor_resize(src, src_h, src_w, 480, 640);
    EXPECT_EQ(dst.size(), static_cast<size_t>(480) * 640u);
    EXPECT_EQ(dst[0], 9);  // top-left block stays 9
}
