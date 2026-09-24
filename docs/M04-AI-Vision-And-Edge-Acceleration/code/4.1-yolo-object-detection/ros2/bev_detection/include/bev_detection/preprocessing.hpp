#pragma once
// Letterbox preprocessing for YOLO detection.

#include <cstdint>
#include <cmath>

namespace bev::detection
{

// Forward declaration (defined in types.hpp)
struct LetterBox;

/// Compute letterbox parameters for fitting source image to target size.
LetterBox computeLetterBox(int src_h, int src_w, int target_h, int target_w);

/// Apply letterbox preprocessing: resize + pad + normalize.
/// h_src: source image in BGR format (h*w*3 bytes)
/// h_dst: output buffer (CHW, RGB, normalized [0,1])
/// dst_size: expected output size (target_h * target_w * 3)
/// letterbox: output letterbox parameters for coordinate restoration
void letterboxPreprocess(
  const uint8_t * h_src, int src_h, int src_w,
  float * h_dst, int target_h, int target_w,
  LetterBox * letterbox);

}  // namespace bev::detection
