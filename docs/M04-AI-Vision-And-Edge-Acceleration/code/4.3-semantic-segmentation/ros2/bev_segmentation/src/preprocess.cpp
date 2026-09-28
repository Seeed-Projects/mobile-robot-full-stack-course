// src/preprocess.cpp

#include "bev_segmentation/preprocess.hpp"
#include "bev_segmentation/types.hpp"

#include <algorithm>
#include <cstring>
#include <stdexcept>
#include <string>

namespace bev_segmentation {

void compute_letterbox(int orig_h, int orig_w,
                       int dst_h, int dst_w,
                       LetterboxMeta& meta) {
    meta.orig_h = orig_h;
    meta.orig_w = orig_w;
    meta.dst_h  = dst_h;
    meta.dst_w  = dst_w;

    // isotropic scale = min(dst/orig)
    const float sh = static_cast<float>(dst_h) / static_cast<float>(orig_h);
    const float sw = static_cast<float>(dst_w) / static_cast<float>(orig_w);
    meta.scale = std::min(sh, sw);

    const int new_w = static_cast<int>(orig_w * meta.scale + 0.5f);
    const int new_h = static_cast<int>(orig_h * meta.scale + 0.5f);

    meta.pad_x = (dst_w - new_w) / 2;  // 简化: left pad; 对语义分割 OK
    meta.pad_y = (dst_h - new_h) / 2;
}

std::vector<float> letterbox_to_chw_float(
    const uint8_t* data, int orig_h, int orig_w, int orig_stride_bytes,
    const std::string& encoding,
    int dst_h, int dst_w,
    LetterboxMeta& meta_out) {

    if (!data || orig_h <= 0 || orig_w <= 0 || orig_stride_bytes <= 0) {
        throw std::invalid_argument("letterbox_to_chw_float: invalid input");
    }

    compute_letterbox(orig_h, orig_w, dst_h, dst_w, meta_out);

    const int new_w = static_cast<int>(orig_w * meta_out.scale + 0.5f);
    const int new_h = static_cast<int>(orig_h * meta_out.scale + 0.5f);

    // 通道数
    int ch = 3;
    bool is_rgba = false;
    if (encoding == "rgb8")       { ch = 3; is_rgba = false; }
    else if (encoding == "bgr8")   { ch = 3; is_rgba = false; }
    else if (encoding == "rgba8")  { ch = 4; is_rgba = true;  }
    else if (encoding == "bgra8")  { ch = 4; is_rgba = true;  }
    else {
        throw std::invalid_argument("letterbox_to_chw_float: unsupported encoding " + encoding);
    }
    if (orig_stride_bytes < orig_w * ch) {
        throw std::invalid_argument("letterbox_to_chw_float: stride too small");
    }

    // 输出 buffer (NCHW float32)
    std::vector<float> out(static_cast<size_t>(3) * dst_h * dst_w, 0.0f);
    // 用 pad value = ImageNet-normalized gray (114/255 - mean) / std
    constexpr float pad_pixel = 114.0f / 255.0f;
    auto norm_pad = [](int c) {
        return (pad_pixel - kImageNetMean[c]) / kImageNetStd[c];
    };
    const float pad_v[3] = { norm_pad(0), norm_pad(1), norm_pad(2) };
    for (int y = 0; y < dst_h; ++y) {
        for (int x = 0; x < dst_w; ++x) {
            for (int c = 0; c < 3; ++c) {
                out[c * dst_h * dst_w + y * dst_w + x] = pad_v[c];
            }
        }
    }

    // 计算源坐标 (在原图中)
    const float inv_scale = (meta_out.scale > 0.f) ? (1.0f / meta_out.scale) : 0.0f;
    const int x0 = meta_out.pad_x;
    const int y0 = meta_out.pad_y;

    for (int dy = 0; dy < new_h; ++dy) {
        const int oy = std::min(orig_h - 1, static_cast<int>(dy * inv_scale + 0.5f));
        const int sy = dy + y0;
        if (sy < 0 || sy >= dst_h) continue;
        const uint8_t* row = data + static_cast<size_t>(oy) * orig_stride_bytes;
        for (int dx = 0; dx < new_w; ++dx) {
            const int ox = std::min(orig_w - 1, static_cast<int>(dx * inv_scale + 0.5f));
            const int sx = dx + x0;
            if (sx < 0 || sx >= dst_w) continue;

            // 根据 encoding & channel 索引提取 BGR/RGB 顺序
            uint8_t b_or_r, g, r_or_b, a = 255;
            if (ch == 3) {
                if (encoding == "rgb8") {
                    r_or_b = row[ox * 3 + 0];
                    g      = row[ox * 3 + 1];
                    b_or_r = row[ox * 3 + 2];
                } else { // bgr8
                    b_or_r = row[ox * 3 + 0];
                    g      = row[ox * 3 + 1];
                    r_or_b = row[ox * 3 + 2];
                }
            } else { // rgba / bgra
                if (encoding == "rgba8") {
                    r_or_b = row[ox * 4 + 0];
                    g      = row[ox * 4 + 1];
                    b_or_r = row[ox * 4 + 2];
                    a      = row[ox * 4 + 3];
                } else { // bgra8
                    b_or_r = row[ox * 4 + 0];
                    g      = row[ox * 4 + 1];
                    r_or_b = row[ox * 4 + 2];
                    a      = row[ox * 4 + 3];
                }
            }

            // 统一为 RGB float normalized (按 a 做 alpha-premultiplication 简化)
            const float af = static_cast<float>(a) / 255.0f;
            const float r  = static_cast<float>(r_or_b) / 255.0f * af;
            const float gg = static_cast<float>(g)      / 255.0f * af;
            const float bb = static_cast<float>(b_or_r) / 255.0f * af;

            // 写入 NCHW (按 RGB 顺序: c=0->R, c=1->G, c=2->B)
            out[0 * dst_h * dst_w + sy * dst_w + sx] = (r  - kImageNetMean[0]) / kImageNetStd[0];
            out[1 * dst_h * dst_w + sy * dst_w + sx] = (gg - kImageNetMean[1]) / kImageNetStd[1];
            out[2 * dst_h * dst_w + sy * dst_w + sx] = (bb - kImageNetMean[2]) / kImageNetStd[2];
        }
    }

    return out;
}

}  // namespace bev_segmentation
