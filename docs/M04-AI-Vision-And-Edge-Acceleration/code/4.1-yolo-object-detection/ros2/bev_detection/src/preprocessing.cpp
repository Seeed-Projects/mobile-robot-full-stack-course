// Letterbox preprocessing implementation.

#include "bev_detection/types.hpp"
#include <algorithm>
#include <cmath>
#include <cstring>

namespace bev::detection
{

LetterBox computeLetterBox(int src_h, int src_w, int target_h, int target_w)
{
  LetterBox lb;

  // Compute scaling factor to fit within target while preserving aspect ratio
  float scale_h = static_cast<float>(target_h) / src_h;
  float scale_w = static_cast<float>(target_w) / src_w;
  lb.scale = std::min(scale_h, scale_w);

  // Compute resized dimensions
  int resized_h = static_cast<int>(std::round(src_h * lb.scale));
  int resized_w = static_cast<int>(std::round(src_w * lb.scale));

  // Compute padding (letterbox bars)
  lb.pad_h = (target_h - resized_h) / 2;
  lb.pad_w = (target_w - resized_w) / 2;

  return lb;
}

void letterboxPreprocess(
  const uint8_t * h_src, int src_h, int src_w,
  float * h_dst, int target_h, int target_w,
  LetterBox * letterbox)
{
  LetterBox lb = computeLetterBox(src_h, src_w, target_h, target_w);
  if (letterbox) *letterbox = lb;

  const int total_elements = target_h * target_w * 3;
  std::memset(h_dst, 0, total_elements * sizeof(float));

  const float inv_255 = 1.f / 255.f;

  int resized_h = static_cast<int>(std::round(src_h * lb.scale));
  int resized_w = static_cast<int>(std::round(src_w * lb.scale));

  // BGR -> RGB and resize with bilinear approximation (nearest neighbor for simplicity)
  for (int y = 0; y < resized_h; ++y) {
    // Map output y to source y (nearest neighbor)
    int src_y = static_cast<int>(y / lb.scale);
    if (src_y >= src_h) src_y = src_h - 1;

    for (int x = 0; x < resized_w; ++x) {
      int src_x = static_cast<int>(x / lb.scale);
      if (src_x >= src_w) src_x = src_w - 1;

      // Get BGR from source
      const uint8_t * src_row = h_src + src_y * src_w * 3;
      float b = src_row[src_x * 3 + 0] * inv_255;
      float g = src_row[src_x * 3 + 1] * inv_255;
      float r = src_row[src_x * 3 + 2] * inv_255;

      // Write to CHW layout with padding offset
      int dst_y = lb.pad_h + y;
      int dst_x = lb.pad_w + x;

      // Channel 0 = R, Channel 1 = G, Channel 2 = B (RGB output)
      h_dst[0 * target_h * target_w + dst_y * target_w + dst_x] = r;
      h_dst[1 * target_h * target_w + dst_y * target_w + dst_x] = g;
      h_dst[2 * target_h * target_w + dst_y * target_w + dst_x] = b;
    }
  }
}

}  // namespace bev::detection
