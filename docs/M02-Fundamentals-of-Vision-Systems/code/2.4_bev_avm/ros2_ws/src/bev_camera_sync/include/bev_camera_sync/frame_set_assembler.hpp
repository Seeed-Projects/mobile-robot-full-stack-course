#pragma once
// FrameSetAssembler: 6-camera synchronization policy, unit-testable.
//
// Policy (master spec §7):
//  - group per camera_id, keep only the LATEST message per camera
//  - a frame set is complete when all 6 cameras deliver and
//    max(stamp) - min(stamp) <= allowed_max_delta
//  - violated delta or stale input when completeness was expected => counts
//    toward incomplete/dropped stats; the frame set is NOT published
//    (default policy: drop entire frame set, no fill-in)
#include <cstdint>
#include <map>
#include <memory>
#include <string>
#include <vector>

#include "bev_interfaces/msg/frame_set.hpp"
#include "rclcpp/time.hpp"
#include "sensor_msgs/msg/camera_info.hpp"
#include "sensor_msgs/msg/image.hpp"

namespace bev::camera_sync
{

struct PendingImage
{
  std::shared_ptr<sensor_msgs::msg::Image> image;
  std::shared_ptr<sensor_msgs::msg::CameraInfo> info;
};

struct AssemblerStats
{
  std::uint32_t dropped_frames{0};        // new message replaced an un-consumed one
  std::uint32_t incomplete_framesets{0};  // purge while a set was expected
  std::uint32_t stale_framesets{0};       // delta between cameras too large
  std::uint32_t published_framesets{0};
};

class FrameSetAssembler
{
public:
  explicit FrameSetAssembler(
    std::vector<std::string> camera_ids, double allowed_max_delta_ms = 5.0)
  : camera_ids_(std::move(camera_ids)), allowed_max_delta_ms_(allowed_max_delta_ms)
  {
  }

  /// Store newest image+info for camera_id. Returns true if a frame set became complete.
  bool update(
    const std::string & camera_id, const sensor_msgs::msg::Image & image,
    const sensor_msgs::msg::CameraInfo & info, bev_interfaces::msg::FrameSet * out_frameset)
  {
    if (!cameraKnown(camera_id)) return false;
    if (pending_.count(camera_id)) {
      stats_.dropped_frames++;
    }
    pending_[camera_id] = PendingImage{
      std::make_shared<sensor_msgs::msg::Image>(image),
      std::make_shared<sensor_msgs::msg::CameraInfo>(info)};
    return tryAssemble(out_frameset);
  }

  /// Explicit completeness attempt (used by the node timer as a backstop).
  bool tryAssemble(bev_interfaces::msg::FrameSet * out)
  {
    if (pending_.size() < camera_ids_.size()) return false;

    rclcpp::Time tmin, tmax;
    bool first = true;
    for (const auto & [id, p] : pending_) {
      const auto t = rclcpp::Time(p.image->header.stamp);
      if (first) { tmin = tmax = t; first = false; }
      else { if (t < tmin) tmin = t; if (t > tmax) tmax = t; }
    }
    const double delta_ms = (tmax - tmin).seconds() * 1e3;
    if (delta_ms > allowed_max_delta_ms_) {
      stats_.stale_framesets++;
      pending_.clear();  // drop entire frame set, no fill-in
      return false;
    }

    if (out) {
      out->header.stamp = tmax;
      out->header.frame_id = "base_link";
      // bounded sequences: resize before indexed assignment (exactly 6)
      out->camera_ids.resize(camera_ids_.size());
      out->images.resize(camera_ids_.size());
      out->camera_infos.resize(camera_ids_.size());
      for (std::size_t i = 0; i < camera_ids_.size(); ++i) {
        const auto & p = pending_.at(camera_ids_[i]);
        out->camera_ids[i] = camera_ids_[i];
        out->images[i] = *p.image;
        out->camera_infos[i] = *p.info;
      }
    }
    pending_.clear();
    stats_.published_framesets++;
    return true;
  }

  /// Purge cameras whose stored stamp is older than (ref_stamp - stale_ms).
  /// The reference is message-relative (the newest stamp seen), NOT wall clock:
  /// wall-clock comparison breaks with rosbag playback stamps and with any
  /// clock-domain mismatch. Returns number of purged cameras.
  std::size_t purgeStale(const rclcpp::Time & ref_stamp, double stale_ms = 150.0)
  {
    std::size_t purged = 0;
    for (auto it = pending_.begin(); it != pending_.end();) {
      const double age_ms =
        (ref_stamp - rclcpp::Time(it->second.image->header.stamp)).seconds() * 1e3;
      if (age_ms > stale_ms) {
        it = pending_.erase(it);
        purged++;
        stats_.incomplete_framesets++;
      } else {
        ++it;
      }
    }
    return purged;
  }

  /// Drop all pending state (playback restart / stream reset).
  void resetScene() { pending_.clear(); }

  const AssemblerStats & stats() const { return stats_; }
  const std::vector<std::string> & cameraIds() const { return camera_ids_; }

  bool cameraKnown(const std::string & id) const
  {
    for (const auto & c : camera_ids_) {
      if (c == id) return true;
    }
    return false;
  }

private:
  std::vector<std::string> camera_ids_;
  double allowed_max_delta_ms_;
  std::map<std::string, PendingImage> pending_;
  AssemblerStats stats_;
};

}  // namespace bev::camera_sync