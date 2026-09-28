#pragma once
// Internal data structures for YOLO detection.

#include <cstdint>
#include <vector>
#include <string>
#include <memory>

namespace bev::detection
{

/// Single bounding box detection in image space.
struct BBox
{
  float x1{0.f};      // top-left x (image coords)
  float y1{0.f};      // top-left y
  float x2{0.f};      // bottom-right x
  float y2{0.f};      // bottom-right y
  float confidence{0.f};
  int class_id{0};
  std::string class_name;
};

/// Letterbox info: describes how the image was padded for letterbox resize.
struct LetterBox
{
  float scale{1.f};
  int pad_w{0};
  int pad_h{0};

  /// Restore bbox from model space (640x640) to original image space.
  BBox restore(const BBox & bbox) const
  {
    BBox out = bbox;
    out.x1 = (bbox.x1 - pad_w) / scale;
    out.y1 = (bbox.y1 - pad_h) / scale;
    out.x2 = (bbox.x2 - pad_w) / scale;
    out.y2 = (bbox.y2 - pad_h) / scale;
    return out;
  }
};

/// Detection result container.
struct YoloResult
{
  std::vector<BBox> boxes;
  LetterBox letterbox;
  float inference_ms{0.f};
};

/// Class names loaded from file.
struct ClassNames
{
  std::vector<std::string> names;

  bool load(const std::string & path);
  const std::string & get(int id) const;
};

}  // namespace bev::detection
