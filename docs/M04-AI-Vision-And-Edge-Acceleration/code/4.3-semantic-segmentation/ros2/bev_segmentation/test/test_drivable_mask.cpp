// test/test_drivable_mask.cpp
//
// 单元测试: drivable mask generation (默认 drivable_class_ids = [0])
//
// ⚠️ 验证默认 [0] (road only) 是 road-only, 而非 road+sidewalk.
// ⚠️ 验证 [0,1] 配置可同时把 sidewalk 视为 drivable candidate.

#include <gtest/gtest.h>
#include <bev_segmentation/postprocess.hpp>

#include <vector>

using namespace bev_segmentation;

TEST(DrivableMaskTest, DefaultRoadOnly) {
    // semantic mask: [road=0, sidewalk=1, building=2]
    std::vector<uint8_t> semantic = {0, 1, 2, 0};
    auto drivable = make_drivable_mask(semantic, {0});
    ASSERT_EQ(drivable.size(), 4u);
    EXPECT_EQ(drivable[0], 255);  // road -> 255
    EXPECT_EQ(drivable[1], 0);    // sidewalk -> 0 (NOT drivable by default)
    EXPECT_EQ(drivable[2], 0);    // building -> 0
    EXPECT_EQ(drivable[3], 255);  // road -> 255
}

TEST(DrivableMaskTest, RoadPlusSidewalkConfig) {
    std::vector<uint8_t> semantic = {0, 1, 2};
    auto drivable = make_drivable_mask(semantic, {0, 1});
    EXPECT_EQ(drivable[0], 255);
    EXPECT_EQ(drivable[1], 255);  // sidewalk included
    EXPECT_EQ(drivable[2], 0);
}

TEST(DrivableMaskTest, EmptyConfig) {
    // No drivable class -> all zero
    std::vector<uint8_t> semantic = {0, 1, 2, 3};
    auto drivable = make_drivable_mask(semantic, {});
    for (auto v : drivable) EXPECT_EQ(v, 0);
}

TEST(DrivableMaskTest, MaskSizePreserved) {
    std::vector<uint8_t> semantic(1000, 0);
    auto drivable = make_drivable_mask(semantic, {0});
    EXPECT_EQ(drivable.size(), 1000u);
}

TEST(DrivableMaskTest, AllRoad) {
    std::vector<uint8_t> semantic(100, 0);
    auto drivable = make_drivable_mask(semantic, {0});
    for (auto v : drivable) EXPECT_EQ(v, 255);
}
