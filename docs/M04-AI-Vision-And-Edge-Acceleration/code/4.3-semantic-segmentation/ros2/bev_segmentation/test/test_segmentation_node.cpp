// test/test_segmentation_node.cpp
//
// 节点级 smoke test: 构造合成图像 -> segmentation_node -> 验证 mask shape.
//
// 不依赖真实 engine (使用 mock 替换, 或在缺少 engine 时 GTEST_SKIP).
// 此处简化: 直接调用 preprocess + postprocess, 不通过 TensorRT engine.

#include <gtest/gtest.h>
#include "bev_segmentation/postprocess.hpp"
#include "bev_segmentation/preprocess.hpp"
#include "bev_segmentation/types.hpp"

#include <vector>

using namespace bev_segmentation;

TEST(SegmentationNodeTest, PreprocessOutputShape) {
    // 合成 64x128 RGB image (任意分辨率)
    std::vector<uint8_t> bgr(64 * 128 * 3, 128);
    LetterboxMeta meta;
    auto in = letterbox_to_chw_float(
        bgr.data(), 64, 128, 128 * 3, "bgr8",
        kModelInputH, kModelInputW, meta);
    EXPECT_EQ(in.size(), static_cast<size_t>(3 * 512 * 1024));

    // 验证 scale / padding 元信息
    EXPECT_GT(meta.scale, 0.f);
    EXPECT_GT(meta.scale, 1.f);  // 64x128 -> 512x1024: isotropic scale = 8 (放大)
    EXPECT_GE(meta.pad_x, 0);
    EXPECT_GE(meta.pad_y, 0);
}

TEST(SegmentationNodeTest, PostprocessEndToEnd) {
    // 构造确定性的 logits: 全 0 类 (class 0 = road) 概率最高
    // shape [1, 19, 128, 256]
    const int H = 128, W = 256, C = 19;
    std::vector<float> logits(static_cast<size_t>(C) * H * W, 0.f);
    // 设 class 0 logits = 10, 其余 = 0  → argmax 必为 class 0
    for (int h = 0; h < H; ++h) {
        for (int w = 0; w < W; ++w) {
            logits[0 * H * W + h * W + w] = 10.0f;
        }
    }
    auto class_mask = argmax_classes(logits, H, W, C);
    EXPECT_EQ(class_mask.size(), static_cast<size_t>(H) * W);
    for (auto v : class_mask) EXPECT_EQ(v, 0);  // 全 road

    // resize 到 240x320 (任意 orig resolution)
    auto semantic = nearest_neighbor_resize(class_mask, H, W, 240, 320);
    EXPECT_EQ(semantic.size(), static_cast<size_t>(240) * 320u);
    for (auto v : semantic) EXPECT_EQ(v, 0);

    // drivable (默认 [0]) → 全 255
    auto drivable = make_drivable_mask(semantic, {0});
    EXPECT_EQ(drivable.size(), static_cast<size_t>(240) * 320u);
    for (auto v : drivable) EXPECT_EQ(v, 255);
}

TEST(SegmentationNodeTest, SegmentationMaskValid) {
    SegmentationMask m;
    m.orig_w = 100; m.orig_h = 50;
    m.semantic.assign(100 * 50, 0);
    m.drivable.assign(100 * 50, 255);
    EXPECT_TRUE(m.valid());
}
