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


}  // namespace bev::detection
