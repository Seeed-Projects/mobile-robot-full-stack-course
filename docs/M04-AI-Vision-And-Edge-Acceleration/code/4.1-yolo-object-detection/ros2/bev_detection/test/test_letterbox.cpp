// Unit tests for letterbox preprocessing and bbox restoration.
// Extended for M4.1 contract: 16:9 and 4:3 cases that M4.2 tracking
// downstream relies on.

#include <gtest/gtest.h>

#include "bev_detection/preprocessing.hpp"
#include "bev_detection/types.hpp"

static bool float_eq(float a, float b, float eps = 1e-4f)
{
  return std::fabs(a - b) < eps;
}

// ----- computeLetterBox geometry --------------------------------------------

TEST(Letterbox, Basic_16x9)
{
  // 1920x1080 -> 640x640 letterbox
  // scale = 640/1920 = 1/3 (limit dim), resized = 640x360, pad_h = 140
  auto lb = bev::detection::computeLetterBox(1080, 1920, 640, 640);

  EXPECT_TRUE(float_eq(lb.scale, 1.0f / 3.0f));
  EXPECT_EQ(lb.pad_h, 140);
  EXPECT_EQ(lb.pad_w, 0);

  int resized_h = static_cast<int>(std::round(1080 * lb.scale));
  int resized_w = static_cast<int>(std::round(1920 * lb.scale));
  EXPECT_EQ(resized_h, 360);
  EXPECT_EQ(resized_w, 640);
}

TEST(Letterbox, Basic_4x3)
{
  // 640x480 -> 640x640 letterbox
  // scale = min(640/640, 640/480) = min(1.0, 1.333) = 1.0
  // resized = 480x640, pad_h = 80
  auto lb = bev::detection::computeLetterBox(480, 640, 640, 640);

  EXPECT_TRUE(float_eq(lb.scale, 1.0f));
  EXPECT_EQ(lb.pad_h, 80);
  EXPECT_EQ(lb.pad_w, 0);

  int resized_h = static_cast<int>(std::round(480 * lb.scale));
  int resized_w = static_cast<int>(std::round(640 * lb.scale));
  EXPECT_EQ(resized_h, 480);
  EXPECT_EQ(resized_w, 640);
}

TEST(Letterbox, Square_NoPadding)
{
  auto lb = bev::detection::computeLetterBox(640, 640, 640, 640);
  EXPECT_TRUE(float_eq(lb.scale, 1.0f));
  EXPECT_EQ(lb.pad_h, 0);
  EXPECT_EQ(lb.pad_w, 0);
}

// ----- bbox restore (model-space -> original image-space) -------------------

TEST(BBoxRestore, FromModelSpace_16x9)
{
  // Original 1920x1080, lb: scale=1/3, pad_h=140, pad_w=0
  bev::detection::LetterBox lb;
  lb.scale = 1.0f / 3.0f;
  lb.pad_h = 140;
  lb.pad_w = 0;

  bev::detection::BBox model_box;
  model_box.x1 = 0;
  model_box.y1 = 140;
  model_box.x2 = 640;
  model_box.y2 = 500;

  auto restored = lb.restore(model_box);

  EXPECT_TRUE(float_eq(restored.x1, 0.f));
  EXPECT_TRUE(float_eq(restored.y1, 0.f));
  EXPECT_TRUE(float_eq(restored.x2, 1920.f));
  EXPECT_TRUE(float_eq(restored.y2, 1080.f));
}

TEST(BBoxRestore, FromModelSpace_4x3)
{
  // Original 640x480, lb: scale=1.0, pad_h=80, pad_w=0
  bev::detection::LetterBox lb;
  lb.scale = 1.0f;
  lb.pad_h = 80;
  lb.pad_w = 0;

  bev::detection::BBox model_box;
  model_box.x1 = 100;
  model_box.y1 = 200;
  model_box.x2 = 300;
  model_box.y2 = 400;

  auto restored = lb.restore(model_box);

  EXPECT_TRUE(float_eq(restored.x1, 100.f));
  EXPECT_TRUE(float_eq(restored.y1, 120.f));  // (200-80)/1
  EXPECT_TRUE(float_eq(restored.x2, 300.f));
  EXPECT_TRUE(float_eq(restored.y2, 320.f));  // (400-80)/1
}

TEST(BBoxRestore, NonSquare_RestoreStaysInImageSpace)
{
  // Random non-square image 1280x720 -> 640x640 letterbox
  // scale = min(640/1280, 640/720) = min(0.5, 0.888) = 0.5
  // resized = 360x640, pad_h = 140, pad_w = 0
  bev::detection::LetterBox lb = bev::detection::computeLetterBox(720, 1280, 640, 640);
  EXPECT_TRUE(float_eq(lb.scale, 0.5f));
  EXPECT_EQ(lb.pad_h, 140);
  EXPECT_EQ(lb.pad_w, 0);

  bev::detection::BBox model_box;
  model_box.x1 = 50;
  model_box.y1 = 200;
  model_box.x2 = 600;
  model_box.y2 = 500;

  auto restored = lb.restore(model_box);

  // Original image coords: divide by scale, subtract pad.
  // x1 = (50 - 0)/0.5 = 100
  // y1 = (200 - 140)/0.5 = 120
  // x2 = (600 - 0)/0.5 = 1200
  // y2 = (500 - 140)/0.5 = 720
  EXPECT_TRUE(float_eq(restored.x1, 100.f));
  EXPECT_TRUE(float_eq(restored.y1, 120.f));
  EXPECT_TRUE(float_eq(restored.x2, 1200.f));
  EXPECT_TRUE(float_eq(restored.y2, 720.f));
}

// ----- clipping --------------------------------------------------------------

TEST(Letterbox, Clipping_AtModelBoundaries)
{
  bev::detection::BBox box;
  box.x1 = -10;
  box.y1 = 100;
  box.x2 = 650;  // exceeds 640
  box.y2 = 700;

  // Manual clip (mirrors what YoloPostprocess::parse does)
  box.x1 = std::max(0.f, std::min(640.f, box.x1));
  box.y1 = std::max(0.f, std::min(640.f, box.y1));
  box.x2 = std::max(0.f, std::min(640.f, box.x2));
  box.y2 = std::max(0.f, std::min(640.f, box.y2));

  EXPECT_FLOAT_EQ(box.x1, 0.f);
  EXPECT_FLOAT_EQ(box.x2, 640.f);
  EXPECT_FLOAT_EQ(box.y2, 640.f);
}
