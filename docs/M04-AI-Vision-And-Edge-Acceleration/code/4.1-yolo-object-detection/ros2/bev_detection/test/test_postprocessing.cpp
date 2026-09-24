// Unit tests for NMS, IoU, score-gating and bbox parsing.

#include <gtest/gtest.h>

#include <cmath>

#include "bev_detection/postprocessing.hpp"
#include "bev_detection/types.hpp"

static bool float_eq(float a, float b, float eps = 1e-4f)
{
  return std::fabs(a - b) < eps;
}

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
  // IoU = 2500 / 17500 ≈ 0.142857
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

TEST(ParseYoloOutput, ConfidenceFilterGatesBelowThreshold)
{
  // YOLO11n output: [1, 84, 8400] = 705600 floats
  std::vector<float> output(84 * 8400, 0.0f);

  // Anchor 0: cx=320, cy=320, w=100, h=100, class 0 score = 0.9
  output[0] = 320.0f;
  output[1] = 320.0f;
  output[2] = 100.0f;
  output[3] = 100.0f;
  output[4] = 0.9f;

  bev::detection::LetterBox lb;
  lb.scale = 1.0f;
  lb.pad_w = 0;
  lb.pad_h = 0;

  auto boxes = bev::detection::parseYoloOutput(
    output.data(), static_cast<int>(output.size()),
    80, 0.5f, 0.45f, lb);

  ASSERT_FALSE(boxes.empty());
  EXPECT_EQ(boxes[0].class_id, 0);
  EXPECT_NEAR(boxes[0].confidence, 0.9f, 1e-4f);
}

TEST(ParseYoloOutput, ScoreBelowThresholdIsDropped)
{
  std::vector<float> output(84 * 8400, 0.0f);
  output[0] = 320.0f;
  output[1] = 320.0f;
  output[2] = 100.0f;
  output[3] = 100.0f;
  output[4] = 0.1f;  // below 0.5 threshold

  bev::detection::LetterBox lb;
  lb.scale = 1.0f;
  lb.pad_w = 0;
  lb.pad_h = 0;

  auto boxes = bev::detection::parseYoloOutput(
    output.data(), static_cast<int>(output.size()),
    80, 0.5f, 0.45f, lb);

  EXPECT_TRUE(boxes.empty());
}
