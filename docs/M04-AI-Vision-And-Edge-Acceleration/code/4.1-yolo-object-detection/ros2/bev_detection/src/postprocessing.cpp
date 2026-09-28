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

// ============================================================================
// YoloPostprocess -- the single parser
// ============================================================================

YoloPostprocess::YoloPostprocess(int num_classes, float conf_thresh, float nms_thresh)
  : num_classes_(num_classes), conf_thresh_(conf_thresh), nms_thresh_(nms_thresh)
{}

std::vector<BBox> YoloPostprocess::parse(
  const float * output, int output_size,
  int img_h, int img_w, const LetterBox & letterbox) const
{
  // YOLO11n output: [1, 4 + num_classes, 8400]
  const int num_anchors = 8400;
  const int channels = 4 + num_classes_;

  // img_h / img_w are not needed: boxes are clipped to the model input box here
  // and mapped back through `letterbox`, which already carries the original
  // geometry.
  (void)img_h;
  (void)img_w;

  if (output == nullptr || output_size < channels * num_anchors) { return {}; }

  std::vector<BBox> candidates;
  candidates.reserve(num_anchors);

  for (int i = 0; i < num_anchors; ++i) {
    // Best class score for this anchor.
    float max_score = 0.f;
    int max_class = 0;
    for (int c = 0; c < num_classes_; ++c) {
      const float score = output[(4 + c) * num_anchors + i];
      if (score > max_score) {
        max_score = score;
        max_class = c;
      }
    }
    if (max_score < conf_thresh_) { continue; }

    // Box in model space [0, 640]: center format -> corner format.
    const float cx = output[0 * num_anchors + i];
    const float cy = output[1 * num_anchors + i];
    const float w  = output[2 * num_anchors + i];
    const float h  = output[3 * num_anchors + i];

    BBox box;
    box.x1 = std::max(0.f, std::min(640.f, cx - w * 0.5f));
    box.y1 = std::max(0.f, std::min(640.f, cy - h * 0.5f));
    box.x2 = std::max(0.f, std::min(640.f, cx + w * 0.5f));
    box.y2 = std::max(0.f, std::min(640.f, cy + h * 0.5f));

    // A degenerate box has no area: it can never suppress anything, and it
    // would be published as a zero-size detection.
    if (box.x2 <= box.x1 || box.y2 <= box.y1) { continue; }

    box.confidence = max_score;
    box.class_id = max_class;
    candidates.push_back(box);
  }

  // applyNMS works in model space; map the survivors back afterwards.
  std::vector<BBox> kept = applyNMS(candidates, nms_thresh_);
  for (BBox & box : kept) {
    box = letterbox.restore(box);
  }
  return kept;
}

}  // namespace bev::detection
