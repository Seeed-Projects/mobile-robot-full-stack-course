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


}  // namespace bev::detection
