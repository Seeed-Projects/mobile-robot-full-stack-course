// Unit tests: FrameSetAssembler policies (missing camera / stale / order).
#include <gtest/gtest.h>

#include <string>
#include <vector>

#include "bev_camera_sync/frame_set_assembler.hpp"

using bev::camera_sync::FrameSetAssembler;

namespace
{
sensor_msgs::msg::Image makeImage(uint64_t ns, const std::string & frame_id)
{
  sensor_msgs::msg::Image m;
  m.header.stamp.sec = 0;
  m.header.stamp.nanosec = ns;
  m.header.frame_id = frame_id;
  m.height = 900;
  m.width = 1600;
  return m;
}

sensor_msgs::msg::CameraInfo makeInfo(uint64_t ns)
{
  sensor_msgs::msg::CameraInfo m;
  m.header.stamp.sec = 0;
  m.header.stamp.nanosec = ns;
  return m;
}

const std::vector<std::string> CAMS{"front_left", "front", "front_right", "back_left", "back", "back_right"};

uint64_t t(int ms) { return static_cast<uint64_t>(ms) * 1000000ULL; }
}  // namespace

TEST(FrameSetAssembler, completeSetPublishesInEngineOrder)
{
  FrameSetAssembler a(CAMS, 5.0);
  bev_interfaces::msg::FrameSet fs;
  int published = 0;
  // feed in shuffled order
  const std::vector<std::string> order{
    "back", "front_left", "back_right", "front_right", "back_left", "front"};
  for (std::size_t i = 0; i < order.size(); ++i) {
    const bool ok = a.update(order[i], makeImage(t(10), "camera_" + order[i]), makeInfo(t(10)), &fs);
    if (ok) published++;
  }
  EXPECT_EQ(published, 1);
  EXPECT_EQ(fs.camera_ids.size(), 6u);
  // engine order
  EXPECT_EQ(fs.camera_ids[0], "front_left");
  EXPECT_EQ(fs.camera_ids[1], "front");
  EXPECT_EQ(fs.camera_ids[2], "front_right");
  EXPECT_EQ(fs.camera_ids[3], "back_left");
  EXPECT_EQ(fs.camera_ids[4], "back");
  EXPECT_EQ(fs.camera_ids[5], "back_right");
  EXPECT_EQ(a.stats().published_framesets, 1u);
}

TEST(FrameSetAssembler, missingCameraNeverPublishes)
{
  FrameSetAssembler a(CAMS, 5.0);
  bev_interfaces::msg::FrameSet fs;
  for (const std::string & id : {"front_left", "front", "front_right", "back_left", "back"}) {
    EXPECT_FALSE(a.update(id, makeImage(t(10), "camera_" + id), makeInfo(t(10)), &fs));
  }
  EXPECT_EQ(a.stats().published_framesets, 0u);
}

TEST(FrameSetAssembler, timestampDeltaViolationDropsWholeSet)
{
  FrameSetAssembler a(CAMS, 5.0);
  bev_interfaces::msg::FrameSet fs;
  std::size_t i = 0;
  for (const auto & id : CAMS) {
    // one camera is 40 ms off
    const uint64_t stamp = (id == "back") ? t(140) : t(100);
    const bool ok = a.update(id, makeImage(stamp, "camera_" + id), makeInfo(stamp), &fs);
    if (ok) GTEST_FAIL() << "set with 40ms spread must not publish, camera " << i;
    i++;
  }
  EXPECT_EQ(a.stats().published_framesets, 0u);
  EXPECT_EQ(a.stats().stale_framesets, 1u);
  // assembler cleaned the pending set; same 6 can form a valid set next
  i = 0;
  for (const auto & id : CAMS) {
    const bool ok = a.update(id, makeImage(t(200), "camera_" + id), makeInfo(t(200)), &fs);
    if (ok && i != 5) GTEST_FAIL() << "published too early";
    i++;
  }
  EXPECT_EQ(a.stats().published_framesets, 1u);
}

TEST(FrameSetAssembler, purgeStaleCountsIncomplete)
{
  FrameSetAssembler a(CAMS, 5.0);
  bev_interfaces::msg::FrameSet fs;
  for (const std::string & id : {"front_left", "front", "front_right"}) {
    a.update(id, makeImage(t(10), "camera_" + id), makeInfo(t(10)), &fs);
  }
  rclcpp::Time now(0, t(1000), RCL_ROS_TIME);  // 990 ms later
  const auto purged = a.purgeStale(now, 150.0);
  EXPECT_EQ(purged, 3u);
  EXPECT_EQ(a.stats().incomplete_framesets, 3u);
}

TEST(FrameSetAssembler, unknownCameraIgnored)
{
  FrameSetAssembler a(CAMS, 5.0);
  bev_interfaces::msg::FrameSet fs;
  EXPECT_FALSE(a.update("top", makeImage(t(10), "camera_top"), makeInfo(t(10)), &fs));
  EXPECT_EQ(a.stats().dropped_frames, 0u);
}

TEST(FrameSetAssembler, newerFrameDropsUnconsumedOld)
{
  FrameSetAssembler a(CAMS, 5.0);
  bev_interfaces::msg::FrameSet fs;
  a.update("front", makeImage(t(10), "camera_front"), makeInfo(t(10)), &fs);
  a.update("front", makeImage(t(20), "camera_front"), makeInfo(t(20)), &fs);
  EXPECT_EQ(a.stats().dropped_frames, 1u);
}