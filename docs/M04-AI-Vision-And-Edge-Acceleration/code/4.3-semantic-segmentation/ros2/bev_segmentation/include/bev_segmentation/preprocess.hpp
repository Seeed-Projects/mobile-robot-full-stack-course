// include/bev_segmentation/preprocess.hpp
//
// Letterbox resize + ImageNet normalize + CHW float32 layout
//   input:  sensor_msgs::msg::Image (bgr8 / rgb8 / bgra8) - 任意原图分辨率
//   output: [1, 3, 512, 1024] float32 buffer (NCHW, RGB, ImageNet-normalized)
//
// 设计: 简化 letterbox - 仅 isotropic resize + bottom-right pad, 不做 center crop.
//       与 M4.1 letterbox (preprocessing.hpp) 行为对齐.

#pragma once

#include <cstdint>
#include <string>
#include <vector>

namespace bev_segmentation {

struct LetterboxMeta {
    int orig_w = 0;
    int orig_h = 0;
    int dst_w  = 0;     // model input W (固定 1024)
    int dst_h  = 0;     // model input H (固定 512)
    float scale = 1.0f; // isotropic resize ratio
    int pad_x   = 0;    // left padding (像素)
    int pad_y   = 0;    // top  padding (像素)
};

// 从 sensor image data + meta 生成 [1,3,512,1024] NCHW float32 RGB ImageNet-normalized
//   支持 encoding: rgb8 / bgr8 / bgra8 / rgba8
//   输出长度 = 1*3*512*1024
std::vector<float> letterbox_to_chw_float(
    const uint8_t* data, int orig_h, int orig_w, int orig_stride_bytes,
    const std::string& encoding,
    int dst_h, int dst_w,
    LetterboxMeta& meta_out);

// 计算 isotropic scale + padding (与 letterbox_to_chw_float 行为对齐)
void compute_letterbox(int orig_h, int orig_w,
                       int dst_h, int dst_w,
                       LetterboxMeta& meta);

}  // namespace bev_segmentation
