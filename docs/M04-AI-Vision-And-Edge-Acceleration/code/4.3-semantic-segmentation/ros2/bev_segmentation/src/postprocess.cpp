// src/postprocess.cpp

#include "bev_segmentation/postprocess.hpp"

#include <algorithm>
#include <stdexcept>

namespace bev_segmentation {

std::vector<uint8_t> argmax_classes(const std::vector<float>& logits,
                                    int H, int W, int num_classes) {
    if (num_classes <= 0 || H <= 0 || W <= 0) {
        throw std::invalid_argument("argmax_classes: invalid dims");
    }
    const size_t expected = static_cast<size_t>(num_classes) * H * W;
    if (logits.size() != expected) {
        throw std::invalid_argument("argmax_classes: logits size mismatch");
    }

    std::vector<uint8_t> out(static_cast<size_t>(H) * W, 0);

    // CHW layout: for each (h,w), iterate over c
    // index(c, h, w) = c * H * W + h * W + w
    const int hw = H * W;
    for (int h = 0; h < H; ++h) {
        for (int w = 0; w < W; ++w) {
            int best_c = 0;
            float best_v = logits[0 * hw + h * W + w];
            for (int c = 1; c < num_classes; ++c) {
                const float v = logits[c * hw + h * W + w];
                if (v > best_v) {
                    best_v = v;
                    best_c = c;
                }
            }
            out[h * W + w] = static_cast<uint8_t>(best_c);
        }
    }
    return out;
}

std::vector<uint8_t> nearest_neighbor_resize(const std::vector<uint8_t>& src,
                                             int src_h, int src_w,
                                             int dst_h, int dst_w) {
    if (src_h <= 0 || src_w <= 0 || dst_h <= 0 || dst_w <= 0) {
        throw std::invalid_argument("nearest_neighbor_resize: invalid dims");
    }
    if (src.size() != static_cast<size_t>(src_h) * src_w) {
        throw std::invalid_argument("nearest_neighbor_resize: src size mismatch");
    }

    std::vector<uint8_t> dst(static_cast<size_t>(dst_h) * dst_w, 0);
    // 使用整数 floor((i * src) / dst) 映射到源坐标; 与 cv2.INTER_NEAREST 行为一致.
    for (int y = 0; y < dst_h; ++y) {
        const int sy = std::min(src_h - 1, (y * src_h) / dst_h);
        for (int x = 0; x < dst_w; ++x) {
            const int sx = std::min(src_w - 1, (x * src_w) / dst_w);
            dst[y * dst_w + x] = src[sy * src_w + sx];
        }
    }
    return dst;
}

std::vector<uint8_t> unletterbox_mask(const std::vector<uint8_t>& class_mask,
                                      const LetterboxMeta& meta,
                                      int orig_h, int orig_w) {
    if (orig_h <= 0 || orig_w <= 0) {
        throw std::invalid_argument("unletterbox_mask: invalid output dims");
    }
    if (class_mask.size() != static_cast<size_t>(kLogitsH) * kLogitsW) {
        throw std::invalid_argument("unletterbox_mask: class_mask must be kLogitsH x kLogitsW");
    }
    if (meta.dst_h != kModelInputH || meta.dst_w != kModelInputW) {
        throw std::invalid_argument(
            "unletterbox_mask: meta canvas does not match the model input size");
    }
    if (meta.scale <= 0.0f) {
        throw std::invalid_argument("unletterbox_mask: non-positive letterbox scale");
    }

    static_assert(kModelInputH % kLogitsH == 0, "logits height must divide the network input");
    static_assert(kModelInputW % kLogitsW == 0, "logits width must divide the network input");
    constexpr int kStrideH = kModelInputH / kLogitsH;  // 4
    constexpr int kStrideW = kModelInputW / kLogitsW;  // 4

    std::vector<uint8_t> out(static_cast<size_t>(orig_h) * orig_w, 0);

    for (int oy = 0; oy < orig_h; ++oy) {
        // Invert the y mapping in letterbox_to_chw_float(): that code walks
        // destination rows `dy` and samples source row int(dy * inv_scale +
        // 0.5f), so the canvas row representing source row `oy` is the one
        // nearest oy * scale -- the centre of dy's preimage. The +0.5f matches
        // compute_letterbox()'s own rounding so the two are exact inverses; the
        // clamp catches the last row, where that rounding lands on the edge
        // (e.g. 1920x1080: row 1079 -> 0.474074*1079 + 0.5 = 512).
        const int canvas_y = std::min(kModelInputH - 1,
            meta.pad_y + static_cast<int>(oy * meta.scale + 0.5f));
        const int mask_y = canvas_y / kStrideH;

        const uint8_t* mask_row =
            class_mask.data() + static_cast<size_t>(mask_y) * kLogitsW;
        uint8_t* out_row = out.data() + static_cast<size_t>(oy) * orig_w;

        for (int ox = 0; ox < orig_w; ++ox) {
            const int canvas_x = std::min(kModelInputW - 1,
                meta.pad_x + static_cast<int>(ox * meta.scale + 0.5f));
            out_row[ox] = mask_row[canvas_x / kStrideW];
        }
    }
    return out;
}

std::vector<uint8_t> make_drivable_mask(const std::vector<uint8_t>& class_mask,
                                        const std::vector<int>& drivable_class_ids) {
    // Build a lookup table for fast membership test.
    // 默认 drivable_class_ids = [0]; 如配置 sidewalk (id=1) 等也可一并合并.
    constexpr int kMaxClass = 256;
    bool lut[kMaxClass] = {false};
    for (int id : drivable_class_ids) {
        if (id >= 0 && id < kMaxClass) {
            lut[id] = true;
        }
    }

    std::vector<uint8_t> out(class_mask.size(), 0);
    for (size_t i = 0; i < class_mask.size(); ++i) {
        out[i] = lut[class_mask[i]] ? 255 : 0;
    }
    return out;
}

}  // namespace bev_segmentation
