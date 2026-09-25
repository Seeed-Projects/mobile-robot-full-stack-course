#pragma once
// YOLO detection-output parsing, bbox restoration and NMS.

#include "types.hpp"
#include <vector>

namespace bev::detection
{

/// YOLO11n output parser + per-class NMS.
///
/// This is the SINGLE implementation of "raw TensorRT output -> detections".
/// It used to exist twice: yolo_engine.cpp carried its own copy that had drifted
/// from the free function below, and the unit tests only ever exercised the copy
/// that nothing called. The node and the tests now both go through this class.
///
/// Layout is CHANNEL-MAJOR [1, 4 + num_classes, num_anchors]: channel c of
/// anchor i lives at c * num_anchors + i. An anchor-major walk reads box
/// coordinates where the class scores live and floods the UI with hundreds of
/// bogus boxes.
class YoloPostprocess
{
public:
  YoloPostprocess(int num_classes, float conf_thresh, float nms_thresh);

  /// Parse raw TensorRT output into detections, in ORIGINAL image coordinates.
  std::vector<BBox> parse(
    const float * output, int output_size,
    int img_h, int img_w, const LetterBox & letterbox) const;

  int numClasses() const { return num_classes_; }
  float confThresh() const { return conf_thresh_; }
  float nmsThresh() const { return nms_thresh_; }

private:
  int num_classes_;
  float conf_thresh_;
  float nms_thresh_;
};

/// Restore bounding box coordinates from model space to original image space.
BBox restoreBBox(const BBox & box, const LetterBox & letterbox);

/// Compute IoU (Intersection over Union) between two model-space boxes.
float computeIoU(const BBox & a, const BBox & b);

/// Apply Non-Maximum Suppression to a list of model-space boxes.
std::vector<BBox> applyNMS(
  const std::vector<BBox> & boxes, float nms_thresh);

}  // namespace bev::detection
