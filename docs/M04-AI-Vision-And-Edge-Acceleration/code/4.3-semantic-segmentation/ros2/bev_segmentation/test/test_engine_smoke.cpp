// test/test_engine_smoke.cpp
//
// Engine smoke test: 加载真实的 .engine, 跑一次推理, 验证输出 shape.
//
// ⚠️ 默认期望 SKIP (engine 文件可能尚未构建), 由 GTEST_SKIP 宏控制.
//    跑全测试请确保 models/m4/segmentation/engines/segformer_b0_fp16.engine 存在.

#include <gtest/gtest.h>
#include "bev_segmentation/segmentation_engine.hpp"

#include <cstdlib>
#include <fstream>
#include <vector>

using namespace bev_segmentation;

static std::string engine_path() {
    if (const char* p = std::getenv("SEG_ENGINE_PATH")) return p;
    return "/home/seeed/workspace/ros2_bev/modules/m04-ai-vision-and-edge-acceleration/models/m4/segmentation/engines/segformer_b0_fp16.engine";
}

TEST(EngineSmokeTest, LoadEngineFile) {
    const std::string p = engine_path();
    if (!std::ifstream(p).good()) {
        GTEST_SKIP() << "engine file not found: " << p
                     << " (build it via scripts/m4/build_segformer_engine.sh)";
        return;
    }
    SegmentationEngine eng;
    EXPECT_NO_THROW(eng.load(p));
    EXPECT_TRUE(eng.is_loaded());
    EXPECT_EQ(eng.input_size(),  1 * 3 * 512 * 1024);
    EXPECT_EQ(eng.output_size(), 1 * 19 * 128 * 256);
    EXPECT_EQ(eng.num_classes(), 19);
}

TEST(EngineSmokeTest, InferOnceRandomInput) {
    const std::string p = engine_path();
    if (!std::ifstream(p).good()) {
        GTEST_SKIP() << "engine file not found: " << p;
        return;
    }
    SegmentationEngine eng;
    eng.load(p);

    std::vector<float> in(eng.input_size(), 0.5f);
    std::vector<float> out;
    EXPECT_NO_THROW(eng.infer(in, out));
    EXPECT_EQ(out.size(), static_cast<size_t>(eng.output_size()));

    // Sanity: argmax logits 应该产生非全 0 (因为 input 不全相同)
    bool any_nonzero = false;
    for (float v : out) {
        if (v != 0.f) { any_nonzero = true; break; }
    }
    EXPECT_TRUE(any_nonzero);
}
