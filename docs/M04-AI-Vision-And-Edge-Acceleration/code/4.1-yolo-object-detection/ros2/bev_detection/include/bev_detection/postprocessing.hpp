#pragma once
// NMS and bbox parsing for YOLO detection output.

#include "types.hpp"
#include <vector>

namespace bev::detection
{

/// Parse YOLO model output into bounding boxes.
/// For YOLO11n: output shape [1, 84, 8400] where 84 = 4 bbox + 80 classes.
/// output: flat float array of model output
/// output_size: total number of floats in output
/// num_classes: number of object classes (80 for COCO)
/// conf_thresh: minimum confidence threshold
/// nms_thresh: NMS IoU threshold
std::vector<BBox> parseYoloOutput(
  const float * output, int output_size,
  int num_classes, float conf_thresh, float nms_thresh,
  const LetterBox & letterbox);

/// Restore bounding box coordinates from model space to original image space.
BBox restoreBBox(const BBox & box, const LetterBox & letterbox);

/// Compute IoU (Intersection over Union) between two boxes.
float computeIoU(const BBox & a, const BBox & b);

/// Apply Non-Maximum Suppression to a list of boxes.
std::vector<BBox> applyNMS(
  const std::vector<BBox> & boxes, float nms_thresh);

}  // namespace bev::detection
