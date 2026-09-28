// include/bev_segmentation/types.hpp
//
// M4.3 Semantic Segmentation — shared types.
// 模型契约: input [1,3,512,1024] (RGB float32 NCHW) → logits [1,19,128,256]
// 输出 mask: argmax → uint8 class mask [128,256] → nearest-neighbor → 原图分辨率
//
// ⚠️ 默认 drivable_class_ids 配置为 [0] (road only); sidewalk (id=1) 可选加入.
//    drivable_mask 是 candidate-drivable semantic mask，不等价于 collision-free space.

#pragma once

#include <cstdint>
#include <string>
#include <unordered_map>
#include <vector>

namespace bev_segmentation {

// Cityscapes 19 类固定 class 数（来自 nvidia/segformer-b0-finetuned-cityscapes-512-1024）
constexpr int kNumClasses = 19;

// 模型固定输入尺寸（H × W），与 checkpoint 训练 contract 一致
constexpr int kModelInputH = 512;
constexpr int kModelInputW = 1024;

// SegFormer 输出 stride=4，因此 logits spatial 尺寸为 model_input / 4
constexpr int kLogitsH = kModelInputH / 4;  // 128
constexpr int kLogitsW = kModelInputW / 4;  // 256

// ImageNet 归一化参数（与 SegFormer 训练配置一致）
constexpr float kImageNetMean[3] = {0.485f, 0.456f, 0.406f};
constexpr float kImageNetStd[3]  = {0.229f, 0.224f, 0.225f};

// SegmentationMask: 单帧推理结果
struct SegmentationMask {
    std::vector<uint8_t> semantic;          // 长度 = orig_H * orig_W, class ID ∈ [0, 18]
    std::vector<uint8_t> drivable;          // 长度 = orig_H * orig_W, 0/255
    int orig_w = 0;
    int orig_h = 0;
    int64_t stamp_ns = 0;                   // source image.stamp (秒*1e9)
    std::string frame_id;                   // source image.frame_id

    bool valid() const {
        return orig_w > 0 && orig_h > 0
            && semantic.size() == static_cast<size_t>(orig_w) * static_cast<size_t>(orig_h)
            && drivable.size() == static_cast<size_t>(orig_w) * static_cast<size_t>(orig_h);
    }
};

// Label map: id (string) -> name (string). 来自 labels.json.
using LabelMap = std::unordered_map<int, std::string>;

}  // namespace bev_segmentation
