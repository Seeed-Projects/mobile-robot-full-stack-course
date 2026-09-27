// Unit tests for NMS, IoU, score-gating and bbox parsing.

#include <gtest/gtest.h>

#include <cmath>
#include <vector>

#include "bev_detection/postprocessing.hpp"
#include "bev_detection/types.hpp"

namespace {

constexpr int kAnchors = 8400;
constexpr int kClasses = 80;

/// Write one anchor using the REAL output layout, channel-major:
/// channel c of anchor i lives at c * kAnchors + i.
void setAnchor(
  std::vector<float> & out, int anchor,
  float cx, float cy, float w, float h, int class_id, float score)
{
  out[0 * kAnchors + anchor] = cx;
  out[1 * kAnchors + anchor] = cy;
  out[2 * kAnchors + anchor] = w;
  out[3 * kAnchors + anchor] = h;
  out[(4 + class_id) * kAnchors + anchor] = score;
}

bev::detection::LetterBox identityLetterbox()
{
  bev::detection::LetterBox lb;
  lb.scale = 1.0f;
  lb.pad_w = 0;
  lb.pad_h = 0;
  return lb;
}

}  // namespace

TEST(IoU, NoOverlap)
{
  bev::detection::BBox a, b;
  a.x1 = 0; a.y1 = 0; a.x2 = 100; a.y2 = 100;
  b.x1 = 200; b.y1 = 200; b.x2 = 300; b.y2 = 300;
  EXPECT_FLOAT_EQ(bev::detection::computeIoU(a, b), 0.f);
}

TEST(IoU, PerfectOverlap)
{
  bev::detection::BBox a, b;
  a.x1 = 0; a.y1 = 0; a.x2 = 100; a.y2 = 100;
  b.x1 = 0; b.y1 = 0; b.x2 = 100; b.y2 = 100;
  EXPECT_FLOAT_EQ(bev::detection::computeIoU(a, b), 1.f);
}

TEST(IoU, PartialOverlap)
{
  bev::detection::BBox a, b;
  a.x1 = 0; a.y1 = 0; a.x2 = 100; a.y2 = 100;
  b.x1 = 50; b.y1 = 50; b.x2 = 150; b.y2 = 150;
  // inter = 50*50 = 2500, union = 10000 + 10000 - 2500 = 17500
  // IoU = 2500 / 17500 ~= 0.142857
  float expected = 2500.0f / 17500.0f;
  float iou = bev::detection::computeIoU(a, b);
  EXPECT_NEAR(iou, expected, 0.01f);
}

TEST(NMS, DifferentClassesKeepBoth)
{
  // Same location, different class => both survive NMS.
  std::vector<bev::detection::BBox> boxes;
  bev::detection::BBox box1, box2;
  box1.x1 = 0; box1.y1 = 0; box1.x2 = 100; box1.y2 = 100;
  box1.class_id = 0; box1.confidence = 0.9f;
  box2.x1 = 0; box2.y1 = 0; box2.x2 = 100; box2.y2 = 100;
  box2.class_id = 1; box2.confidence = 0.8f;
  boxes.push_back(box1);
  boxes.push_back(box2);

  auto result = bev::detection::applyNMS(boxes, 0.5f);
  EXPECT_EQ(result.size(), 2u);
}

TEST(NMS, SameClassOverlapSuppresses)
{
  std::vector<bev::detection::BBox> boxes;
  bev::detection::BBox box1, box2;
  box1.x1 = 0; box1.y1 = 0; box1.x2 = 100; box1.y2 = 100;
  box1.class_id = 0; box1.confidence = 0.9f;
  box2.x1 = 10; box2.y1 = 10; box2.x2 = 110; box2.y2 = 110;  // overlaps
  box2.class_id = 0; box2.confidence = 0.8f;
  boxes.push_back(box1);
  boxes.push_back(box2);

  auto result = bev::detection::applyNMS(boxes, 0.5f);
  EXPECT_EQ(result.size(), 1u);
}

// ---------------------------------------------------------------------------
// Parsing. The layout test below is the regression guard: the parser used to be
// duplicated, one copy read the output anchor-major, and nothing exercised the
// copy the node actually called.
// ---------------------------------------------------------------------------

TEST(YoloPostprocess, LayoutIsChannelMajor)
{
  bev::detection::YoloPostprocess parser(kClasses, 0.5f, 0.45f);
  const auto lb = identityLetterbox();

  // The classic anchor-major tuple (cx, cy, w, h, score at indices 0..4) must
  // NOT be accepted: under the real layout those indices are channel 0 (cx) of
  // anchors 0..4, so every class score stays zero.
  std::vector<float> anchor_major(84 * kAnchors, 0.0f);
  anchor_major[0] = 320.0f;
  anchor_major[1] = 320.0f;
  anchor_major[2] = 100.0f;
  anchor_major[3] = 100.0f;
  anchor_major[4] = 0.9f;
  EXPECT_TRUE(parser.parse(anchor_major.data(), static_cast<int>(anchor_major.size()),
                           640, 640, lb).empty())
    << "anchor-major data produced a detection - the parser reads the wrong layout";

  // The same detection, written channel-major, IS found with the right geometry.
  std::vector<float> channel_major(84 * kAnchors, 0.0f);
  setAnchor(channel_major, 0, 320.0f, 320.0f, 100.0f, 100.0f, 0, 0.9f);

  auto boxes = parser.parse(channel_major.data(), static_cast<int>(channel_major.size()),
                            640, 640, lb);
  ASSERT_EQ(boxes.size(), 1u);
  EXPECT_EQ(boxes[0].class_id, 0);
  EXPECT_NEAR(boxes[0].confidence, 0.9f, 1e-4f);
  EXPECT_NEAR(boxes[0].x1, 270.0f, 1e-3f);  // 320 - 100/2
  EXPECT_NEAR(boxes[0].y1, 270.0f, 1e-3f);
  EXPECT_NEAR(boxes[0].x2, 370.0f, 1e-3f);
  EXPECT_NEAR(boxes[0].y2, 370.0f, 1e-3f);
}

TEST(YoloPostprocess, ConfidenceFilterGatesBelowThreshold)
{
  std::vector<float> output(84 * kAnchors, 0.0f);
  setAnchor(output, 0, 320.0f, 320.0f, 100.0f, 100.0f, 0, 0.9f);

  bev::detection::YoloPostprocess parser(kClasses, 0.5f, 0.45f);
  auto boxes = parser.parse(output.data(), static_cast<int>(output.size()),
                            640, 640, identityLetterbox());

  ASSERT_FALSE(boxes.empty());
  EXPECT_EQ(boxes[0].class_id, 0);
  EXPECT_NEAR(boxes[0].confidence, 0.9f, 1e-4f);
}

TEST(YoloPostprocess, ScoreBelowThresholdIsDropped)
{
  std::vector<float> output(84 * kAnchors, 0.0f);
  setAnchor(output, 0, 320.0f, 320.0f, 100.0f, 100.0f, 0, 0.1f);  // < 0.5

  bev::detection::YoloPostprocess parser(kClasses, 0.5f, 0.45f);
  auto boxes = parser.parse(output.data(), static_cast<int>(output.size()),
                            640, 640, identityLetterbox());
  EXPECT_TRUE(boxes.empty());
}

TEST(YoloPostprocess, BestClassWinsOverOthers)
{
  std::vector<float> output(84 * kAnchors, 0.0f);
  setAnchor(output, 7, 100.0f, 100.0f, 40.0f, 40.0f, 3, 0.7f);
  output[(4 + 5) * kAnchors + 7] = 0.95f;  // class 5 is higher

  bev::detection::YoloPostprocess parser(kClasses, 0.5f, 0.45f);
  auto boxes = parser.parse(output.data(), static_cast<int>(output.size()),
                            640, 640, identityLetterbox());
  ASSERT_EQ(boxes.size(), 1u);
  EXPECT_EQ(boxes[0].class_id, 5);
  EXPECT_NEAR(boxes[0].confidence, 0.95f, 1e-4f);
}

TEST(YoloPostprocess, ZeroAreaBoxIsDropped)
{
  std::vector<float> output(84 * kAnchors, 0.0f);
  setAnchor(output, 0, 320.0f, 320.0f, 0.0f, 0.0f, 0, 0.9f);  // w = h = 0

  bev::detection::YoloPostprocess parser(kClasses, 0.5f, 0.45f);
  auto boxes = parser.parse(output.data(), static_cast<int>(output.size()),
                            640, 640, identityLetterbox());
  EXPECT_TRUE(boxes.empty()) << "a zero-area box would be published as a detection";
}

TEST(YoloPostprocess, RestoresThroughLetterbox)
{
  // 1920x1080 letterboxed into 640x640: scale 1/3, no vertical padding is
  // impossible, so use the real numbers from computeLetterBox: scale = 640/1920
  // = 0.3333, canvas 640x360, pad_h = 140. Model-space box (320,320)-(400,400)
  // maps back to ((320-140)/0.3333, ...) in the original frame.
  std::vector<float> output(84 * kAnchors, 0.0f);
  setAnchor(output, 0, 360.0f, 180.0f, 60.0f, 60.0f, 0, 0.9f);

  bev::detection::LetterBox lb;
  lb.scale = 640.0f / 1920.0f;
  lb.pad_w = 0;
  lb.pad_h = 140;

  bev::detection::YoloPostprocess parser(kClasses, 0.5f, 0.45f);
  auto boxes = parser.parse(output.data(), static_cast<int>(output.size()),
                            1080, 1920, lb);
  ASSERT_EQ(boxes.size(), 1u);
  EXPECT_NEAR(boxes[0].x1, (330.0f - 0.0f) / lb.scale, 1e-2f);
  EXPECT_NEAR(boxes[0].y1, (150.0f - 140.0f) / lb.scale, 1e-2f);
  EXPECT_NEAR(boxes[0].x2, (390.0f - 0.0f) / lb.scale, 1e-2f);
  EXPECT_NEAR(boxes[0].y2, (210.0f - 140.0f) / lb.scale, 1e-2f);
}
