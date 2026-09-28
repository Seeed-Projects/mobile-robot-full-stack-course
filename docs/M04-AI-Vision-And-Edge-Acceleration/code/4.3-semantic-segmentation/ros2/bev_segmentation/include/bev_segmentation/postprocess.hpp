// include/bev_segmentation/postprocess.hpp
//
// M4.3 后处理:
//  1) argmax over 19 classes: logits [1,19,128,256] -> class mask [128,256] (uint8)
//  2) 反向 letterbox: 把 class mask 从网络输入坐标系还原回原图分辨率,
//     先去掉 letterbox padding 再采样到 orig 分辨率 (见 unletterbox_mask)
//  3) drivable mask: candidate-drivable semantic mask based on drivable_class_ids
//
// ⚠️ drivable_mask 是 candidate-drivable semantic mask, 不等价于 collision-free space.

#pragma once

#include "bev_segmentation/preprocess.hpp"   // LetterboxMeta
#include "bev_segmentation/types.hpp"

#include <string>
#include <vector>

namespace bev_segmentation {

// argmax over classes (axis=1) for a [1, 19, 128, 256] logits buffer.
// 假设 logits layout 是 NHWC-like (CHW here per ONNX export): index = n*C*H*W + c*H*W + h*W + w
// 输出长度 = H*W (uint8, class id ∈ [0, 18])
std::vector<uint8_t> argmax_classes(const std::vector<float>& logits,
                                    int H, int W, int num_classes);

// 最近邻上采样: 将 [H, W] class mask 放大到 [out_h, out_w]
// 使用 int64 索引 + 边界 clamp; scale = max(out_h/H, out_w/W) per dim
//
// ⚠️ 注意: 这个函数**不知道** letterbox 的存在. 直接把网络分辨率(low-res)
//    class mask resize 到原图会保留 padding 并产生几何错误 —— 语义分割请用
//    unletterbox_mask() 而不是这个函数.
std::vector<uint8_t> nearest_neighbor_resize(const std::vector<uint8_t>& src,
                                             int src_h, int src_w,
                                             int dst_h, int dst_w);

// 反向 letterbox: 把 [kLogitsH, kLogitsW] 的 class mask 还原成 [orig_h, orig_w].
//
// 正确顺序是: argmax -> 还原到网络输入坐标系 -> 去掉 padding -> 采样回原图.
// 这里把三步合成一次遍历: 对每个输出像素直接算出它在网络输入坐标系里的位置,
// 减掉 pad, 再除以 stride 得到 mask 索引. 既避免两次 resize 的逐帧开销,
// 也避免与 compute_letterbox() 的取整方式不一致而引入系统性半像素偏移.
//
// meta 必须来自产生这帧输入的那次预处理 (含其 scale / pad_x / pad_y).
std::vector<uint8_t> unletterbox_mask(const std::vector<uint8_t>& class_mask,
                                      const LetterboxMeta& meta,
                                      int orig_h, int orig_w);

// 由 class mask + drivable_class_ids 生成 drivable mask (0 / 255)
std::vector<uint8_t> make_drivable_mask(const std::vector<uint8_t>& class_mask,
                                        const std::vector<int>& drivable_class_ids);

}  // namespace bev_segmentation
