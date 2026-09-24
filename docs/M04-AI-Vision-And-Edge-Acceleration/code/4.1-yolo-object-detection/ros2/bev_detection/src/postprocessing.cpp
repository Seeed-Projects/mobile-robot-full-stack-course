// NMS and bbox parsing implementation.

#include "bev_detection/postprocessing.hpp"
#include "bev_detection/types.hpp"
#include <algorithm>
#include <cmath>

namespace bev::detection
{

BBox restoreBBox(const BBox & box, const LetterBox & letterbox)
{
  BBox restored = box;
  restored.x1 = (box.x1 - letterbox.pad_w) / letterbox.scale;
  restored.y1 = (box.y1 - letterbox.pad_h) / letterbox.scale;
  restored.x2 = (box.x2 - letterbox.pad_w) / letterbox.scale;
  restored.y2 = (box.y2 - letterbox.pad_h) / letterbox.scale;
  return restored;
}

float computeIoU(const BBox & a, const BBox & b)
{
  // Clip boxes to be within [0, 1] range (model space)
  float a_x1 = std::max(0.f, std::min(a.x1, 640.f));
  float a_y1 = std::max(0.f, std::min(a.y1, 640.f));
  float a_x2 = std::max(0.f, std::min(a.x2, 640.f));
  float a_y2 = std::max(0.f, std::min(a.y2, 640.f));

  float b_x1 = std::max(0.f, std::min(b.x1, 640.f));
  float b_y1 = std::max(0.f, std::min(b.y1, 640.f));
  float b_x2 = std::max(0.f, std::min(b.x2, 640.f));
  float b_y2 = std::max(0.f, std::min(b.y2, 640.f));

  float inter_x1 = std::max(a_x1, b_x1);
  float inter_y1 = std::max(a_y1, b_y1);
  float inter_x2 = std::min(a_x2, b_x2);
  float inter_y2 = std::min(a_y2, b_y2);

  float inter_w = std::max(0.f, inter_x2 - inter_x1);
  float inter_h = std::max(0.f, inter_y2 - inter_y1);
  float inter_area = inter_w * inter_h;

  float a_area = (a_x2 - a_x1) * (a_y2 - a_y1);
  float b_area = (b_x2 - b_x1) * (b_y2 - b_y1);
  float union_area = a_area + b_area - inter_area;

  return (union_area > 0.f) ? (inter_area / union_area) : 0.f;
}

std::vector<BBox> applyNMS(const std::vector<BBox> & boxes, float nms_thresh)
{
  if (boxes.empty()) return {};

  // Sort by confidence (descending)
  std::vector<BBox> sorted = boxes;
  std::sort(sorted.begin(), sorted.end(),
    [](const BBox & a, const BBox & b) { return a.confidence > b.confidence; });

  std::vector<bool> suppress(sorted.size(), false);
  std::vector<BBox> result;

  for (size_t i = 0; i < sorted.size(); ++i) {
    if (suppress[i]) continue;

    const BBox & box = sorted[i];
    result.push_back(restoreBBox(box, LetterBox{1.f, 0, 0}));  // Already in model space

    // Suppress overlapping boxes of same class with lower confidence
    for (size_t j = i + 1; j < sorted.size(); ++j) {
      if (suppress[j]) continue;
      if (sorted[j].class_id != box.class_id) continue;

      float iou = computeIoU(box, sorted[j]);
      if (iou > nms_thresh) {
        suppress[j] = true;
      }
    }
  }

  return result;
}

std::vector<BBox> parseYoloOutput(
  const float * output, int output_size,
  int num_classes, float conf_thresh, float nms_thresh,
  const LetterBox & letterbox)
{
  // YOLO11n output: [1, 84, 8400]
  // 84 = 4 (bbox: cx, cy, w, h in model space) + 80 (class scores)
  const int num_anchors = 8400;
  const int channels = 4 + num_classes;   // bbox(4) + class scores(nc)
  if (output_size < channels * num_anchors) { return {}; }

  std::vector<BBox> candidates;
  candidates.reserve(num_anchors);

  // CHANNEL-MAJOR [1, channels, num_anchors]: element (c, i) is at
  // c * num_anchors + i. The old anchor-major walk (output + i*stride) read box
  // coordinates as class scores, flooding the UI with hundreds of boxes
  // labelled "69696%", "620505%" etc.
  for (int i = 0; i < num_anchors; ++i) {
    // Find class with maximum score
    float max_score = 0.f;
    int max_class = 0;
    for (int c = 0; c < num_classes; ++c) {
      const float score = output[(4 + c) * num_anchors + i];
      if (score > max_score) {
        max_score = score;
        max_class = c;
      }
    }

    // Apply confidence threshold
    if (max_score < conf_thresh) continue;

    // Get bbox in model space [0, 640]
    const float cx = output[0 * num_anchors + i];
    const float cy = output[1 * num_anchors + i];
    const float w  = output[2 * num_anchors + i];
    const float h  = output[3 * num_anchors + i];

    // Convert from center format to corner format
    float x1 = cx - w * 0.5f;
    float y1 = cy - h * 0.5f;
    float x2 = cx + w * 0.5f;
    float y2 = cy + h * 0.5f;

    // Clip to model bounds
    x1 = std::max(0.f, std::min(640.f, x1));
    y1 = std::max(0.f, std::min(640.f, y1));
    x2 = std::max(0.f, std::min(640.f, x2));
    y2 = std::max(0.f, std::min(640.f, y2));

    // Skip invalid boxes
    if (x2 <= x1 || y2 <= y1) continue;

    BBox box;
    box.x1 = x1;
    box.y1 = y1;
    box.x2 = x2;
    box.y2 = y2;
    box.confidence = max_score;
    box.class_id = max_class;

    candidates.push_back(box);
  }

  // Apply NMS
  return applyNMS(candidates, nms_thresh);
}

}  // namespace bev::detection
