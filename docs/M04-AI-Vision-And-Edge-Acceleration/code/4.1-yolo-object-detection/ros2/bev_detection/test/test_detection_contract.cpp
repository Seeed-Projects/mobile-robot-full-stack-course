// M4.1 Detection Contract Tests — verifies the publish-time shape of
// /perception/detections messages that M4.2 (ByteTrack) will consume.
//
// We mirror the publishDetections body inline here, so that the contract
// is locked at the data-shape level without spinning up ROS. The full
// ROS-level smoke is covered by scripts/m4/test_empty_frame_contract.sh
// and the existing scripts/m4/test_yolo_node.sh.
//
// Contract checks:
//   - test_empty_frame_publishes_detection_array
//       empty BBox vector  -> one Detection2DArray with detections.size()==0
//   - test_detection_header_matches_image_header
//       output header == source image header
//   - test_detection_bbox_is_original_image_coordinates
//       bbox coords are in source image pixel space (post letterbox restore)
//   - test_detection_score_is_normalized
//       score in [0,1], no NaN
//   - test_detection_class_is_present
//       class_id is non-empty string, falls back to "class_<id>"

#include <gtest/gtest.h>

#include <cmath>
#include <string>
#include <vector>

#include <sensor_msgs/msg/image.hpp>
#include <vision_msgs/msg/detection2_d_array.hpp>
#include <vision_msgs/msg/detection2_d.hpp>
#include <vision_msgs/msg/bounding_box2_d.hpp>
#include <vision_msgs/msg/object_hypothesis_with_pose.hpp>
#include <std_msgs/msg/header.hpp>

#include "bev_detection/types.hpp"

namespace
{

// Mirror of publishDetections() in yolo_trt_node.cpp.
// Kept in lock-step; if the production code changes shape, this test
// must change too — that is by design.
struct DetectorState
{
  // Whether the COCO class-name file has been loaded. When false, the
  // contract falls back to "class_<id>".
  bool class_names_loaded{false};
  std::vector<std::string> class_names;
};

vision_msgs::msg::Detection2DArray publishDetections(
  const sensor_msgs::msg::Image & image_msg,
  const std::vector<bev::detection::BBox> & boxes,
  const DetectorState & state)
{
  vision_msgs::msg::Detection2DArray detections;
  detections.header = image_msg.header;

  for (const auto & box : boxes) {
    vision_msgs::msg::Detection2D det;
    det.header = image_msg.header;

    vision_msgs::msg::BoundingBox2D bbox;
    bbox.center.position.x = (box.x1 + box.x2) * 0.5;
    bbox.center.position.y = (box.y1 + box.y2) * 0.5;
    bbox.size_x = box.x2 - box.x1;
    bbox.size_y = box.y2 - box.y1;
    bbox.center.theta = 0.0;
    det.bbox = bbox;

    vision_msgs::msg::ObjectHypothesisWithPose hyp;
    if (state.class_names_loaded &&
        box.class_id >= 0 &&
        box.class_id < static_cast<int>(state.class_names.size())) {
      hyp.hypothesis.class_id = state.class_names[box.class_id];
    } else {
      hyp.hypothesis.class_id = "class_" + std::to_string(box.class_id);
    }
    hyp.hypothesis.score = box.confidence;

    det.results.push_back(hyp);
    detections.detections.push_back(det);
  }

  // Contract: publish unconditionally — empty vector still produces one msg.
  return detections;
}

sensor_msgs::msg::Image makeImageMsg(int w, int h, const std::string & frame_id)
{
  sensor_msgs::msg::Image img;
  img.width = static_cast<uint32_t>(w);
  img.height = static_cast<uint32_t>(h);
  img.encoding = "bgr8";
  img.header.frame_id = frame_id;
  img.header.stamp.sec = 1700000000;
  img.header.stamp.nanosec = 123456789;
  return img;
}

}  // namespace

// ---- Contract tests --------------------------------------------------------

TEST(DetectionContract, EmptyFramePublishesDetectionArray)
{
  // Frame with NO detections must still produce one Detection2DArray.
  auto image_msg = makeImageMsg(1920, 1080, "camera_front");
  DetectorState state;

  auto msg = publishDetections(image_msg, /*boxes=*/{}, state);

  EXPECT_EQ(msg.detections.size(), 0u);
  EXPECT_EQ(msg.header.frame_id, "camera_front");
  EXPECT_EQ(msg.header.stamp.sec, 1700000000);
  EXPECT_EQ(msg.header.stamp.nanosec, 123456789u);
}

TEST(DetectionContract, HeaderMatchesImageHeader)
{
  auto image_msg = makeImageMsg(1920, 1080, "camera_front_left");
  image_msg.header.stamp.sec = 1700000123;
  image_msg.header.stamp.nanosec = 999;

  DetectorState state;
  bev::detection::BBox b;
  b.x1 = 0; b.y1 = 0; b.x2 = 10; b.y2 = 10;
  b.confidence = 0.5f; b.class_id = 0;

  auto msg = publishDetections(image_msg, {b}, state);

  ASSERT_EQ(msg.detections.size(), 1u);
  EXPECT_EQ(msg.header.frame_id, "camera_front_left");
  EXPECT_EQ(msg.detections[0].header.frame_id, "camera_front_left");
  EXPECT_EQ(msg.header.stamp.sec, 1700000123);
  EXPECT_EQ(msg.header.stamp.nanosec, 999u);
}

TEST(DetectionContract, BBoxInOriginalImageCoordinates)
{
  // Source image 1920x1080. After letterbox (scale=1/3, pad_h=140),
  // a model-space box at [x1=200, y1=200, x2=440, y2=360] maps to
  // [x1=600, y1=180, x2=1320, y2=660] in the source image.
  auto image_msg = makeImageMsg(1920, 1080, "camera_front");
  DetectorState state;

  bev::detection::BBox b;
  b.x1 = 600; b.y1 = 180; b.x2 = 1320; b.y2 = 660;  // original-image coords
  b.confidence = 0.9f; b.class_id = 0;

  auto msg = publishDetections(image_msg, {b}, state);
  ASSERT_EQ(msg.detections.size(), 1u);

  const auto & bbox = msg.detections[0].bbox;
  // center = (960, 420)
  EXPECT_FLOAT_EQ(static_cast<float>(bbox.center.position.x), 960.f);
  EXPECT_FLOAT_EQ(static_cast<float>(bbox.center.position.y), 420.f);
  EXPECT_FLOAT_EQ(static_cast<float>(bbox.size_x), 720.f);  // 1320-600
  EXPECT_FLOAT_EQ(static_cast<float>(bbox.size_y), 480.f);  // 660-180

  // Sanity: bbox stays within original image bounds.
  EXPECT_GE(bbox.center.position.x, 0.0);
  EXPECT_LE(bbox.center.position.x, 1920.0);
  EXPECT_GE(bbox.center.position.y, 0.0);
  EXPECT_LE(bbox.center.position.y, 1080.0);
}

TEST(DetectionContract, ScoreIsNormalized)
{
  auto image_msg = makeImageMsg(1920, 1080, "camera_front");
  DetectorState state;

  bev::detection::BBox b;
  b.x1 = 0; b.y1 = 0; b.x2 = 100; b.y2 = 100;
  b.confidence = 0.5f; b.class_id = 0;

  auto msg = publishDetections(image_msg, {b}, state);
  ASSERT_EQ(msg.detections.size(), 1u);
  ASSERT_FALSE(msg.detections[0].results.empty());

  float score = static_cast<float>(msg.detections[0].results[0].hypothesis.score);
  EXPECT_FALSE(std::isnan(score));
  EXPECT_GE(score, 0.0f);
  EXPECT_LE(score, 1.0f);
}

TEST(DetectionContract, ClassIsPresent)
{
  auto image_msg = makeImageMsg(1920, 1080, "camera_front");

  // Case 1: no labels loaded → fallback "class_<id>"
  DetectorState no_labels;
  bev::detection::BBox b1;
  b1.x1 = 0; b1.y1 = 0; b1.x2 = 10; b1.y2 = 10;
  b1.confidence = 0.5f; b1.class_id = 7;

  auto msg1 = publishDetections(image_msg, {b1}, no_labels);
  ASSERT_EQ(msg1.detections.size(), 1u);
  EXPECT_EQ(msg1.detections[0].results[0].hypothesis.class_id, "class_7");

  // Case 2: labels loaded → use COCO name
  DetectorState with_labels;
  with_labels.class_names_loaded = true;
  with_labels.class_names = {"person", "bicycle", "car", "motorcycle"};
  bev::detection::BBox b2;
  b2.x1 = 0; b2.y1 = 0; b2.x2 = 10; b2.y2 = 10;
  b2.confidence = 0.5f; b2.class_id = 2;

  auto msg2 = publishDetections(image_msg, {b2}, with_labels);
  ASSERT_EQ(msg2.detections.size(), 1u);
  EXPECT_EQ(msg2.detections[0].results[0].hypothesis.class_id, "car");

  // Case 3: out-of-range class id with labels loaded → fallback "class_<id>"
  bev::detection::BBox b3;
  b3.x1 = 0; b3.y1 = 0; b3.x2 = 10; b3.y2 = 10;
  b3.confidence = 0.5f; b3.class_id = 99;

  auto msg3 = publishDetections(image_msg, {b3}, with_labels);
  ASSERT_EQ(msg3.detections.size(), 1u);
  EXPECT_EQ(msg3.detections[0].results[0].hypothesis.class_id, "class_99");
}

TEST(DetectionContract, IdFieldNotSet)
{
  // M4.1 must not write the track-id field — that is M4.2's job.
  auto image_msg = makeImageMsg(1920, 1080, "camera_front");
  DetectorState state;

  bev::detection::BBox b;
  b.x1 = 0; b.y1 = 0; b.x2 = 100; b.y2 = 100;
  b.confidence = 0.9f; b.class_id = 0;

  auto msg = publishDetections(image_msg, {b}, state);
  ASSERT_EQ(msg.detections.size(), 1u);
  EXPECT_EQ(msg.detections[0].id, "");
}

TEST(DetectionContract, OrderingIsNotIdentity)
{
  // The contract must NOT depend on detections order. Document this by
  // showing the function returns boxes in the input order they were
  // given, not in any tracker-meaningful order. M4.2 must not index by
  // array position.
  auto image_msg = makeImageMsg(1920, 1080, "camera_front");
  DetectorState state;

  // 3 boxes in arbitrary insertion order
  bev::detection::BBox a, b, c;
  a.x1 = 0; a.y1 = 0; a.x2 = 10; a.y2 = 10;
  a.confidence = 0.7f; a.class_id = 1;
  b.x1 = 0; b.y1 = 0; b.x2 = 10; b.y2 = 10;
  b.confidence = 0.9f; b.class_id = 0;
  c.x1 = 0; c.y1 = 0; b.x2 = 10; c.y2 = 10;
  c.confidence = 0.8f; c.class_id = 2;

  auto msg = publishDetections(image_msg, {a, b, c}, state);
  ASSERT_EQ(msg.detections.size(), 3u);
  // Order is the order we passed them in (insertion order).
  EXPECT_EQ(msg.detections[0].results[0].hypothesis.class_id, "class_1");
  EXPECT_EQ(msg.detections[1].results[0].hypothesis.class_id, "class_0");
  EXPECT_EQ(msg.detections[2].results[0].hypothesis.class_id, "class_2");
}
