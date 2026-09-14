// camera_sync_node: assembles 6 camera streams into FrameSet messages.
#include <atomic>
#include <chrono>
#include <cstdio>
#include <cstdlib>
#include <fstream>
#include <map>
#include <memory>
#include <string>
#include <vector>

#include <opencv2/imgcodecs.hpp>

#include "bev_camera_sync/frame_set_assembler.hpp"
#include "bev_interfaces/msg/frame_set.hpp"
#include "bev_interfaces/msg/sync_stats.hpp"
#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/camera_info.hpp"
#include "sensor_msgs/msg/compressed_image.hpp"
#include "sensor_msgs/msg/image.hpp"

namespace bev::camera_sync
{

class CameraSyncNode : public rclcpp::Node
{
public:
  CameraSyncNode() : Node("camera_sync_node")
  {
    // Engine camera order (nuScenes-derived); also used to label FrameSet output
    camera_ids_ = declare_parameter(
      "camera_ids",
      std::vector<std::string>{"front_left", "front", "front_right", "back_left", "back", "back_right"});
    const double allowed_delta = declare_parameter("allowed_max_delta_ms", 5.0);
    const double stale_ms = declare_parameter("stale_drop_ms", 150.0);
    const bool compressed = declare_parameter("compressed", true);

    assembler_ = std::make_unique<FrameSetAssembler>(camera_ids_, allowed_delta);

    // RELIABLE depth-5: a 6-image frameset is ~26 MB; over best-effort DDS
    // large payloads are delivered corrupted (SHM zero-copy races / chunked
    // fragmentation). Reliable keeps integrity at the cost of ACK overhead.
    frameset_pub_ = create_publisher<bev_interfaces::msg::FrameSet>(
      "/bev/frameset", rclcpp::QoS(5).reliable());
    stats_pub_ = create_publisher<bev_interfaces::msg::SyncStats>("/camera/sync_stats", rclcpp::QoS(2));

    const auto img_qos = rclcpp::SensorDataQoS();
    for (const auto & id : camera_ids_) {
      if (compressed) {
        compressed_subs_.push_back(create_subscription<sensor_msgs::msg::CompressedImage>(
          "/camera/" + id + "/image_raw/compressed", img_qos,
          [this, id](const sensor_msgs::msg::CompressedImage::SharedPtr m) { onCompressed(id, m); }));
      } else {
        img_subs_.push_back(create_subscription<sensor_msgs::msg::Image>(
          "/camera/" + id + "/image_raw", img_qos,
          [this, id](const sensor_msgs::msg::Image::SharedPtr m) { onImage(id, m); }));
      }
      info_subs_.push_back(create_subscription<sensor_msgs::msg::CameraInfo>(
        "/camera/" + id + "/camera_info", img_qos,
        [this, id](const sensor_msgs::msg::CameraInfo::SharedPtr m) { onInfo(id, m); }));
      RCLCPP_INFO(get_logger(), "subscribing /camera/%s/%s", id.c_str(),
        compressed ? "image_raw/compressed" : "image_raw");
    }
    // backstop: purge stale partial sets at 20 Hz. Reference is the newest
    // message stamp (message-relative), so wall clock is never compared
    // against playback stamps.
    timer_ = create_wall_timer(std::chrono::milliseconds(50), [this, stale_ms]() {
      if (newest_stamp_valid_) {
        assembler_->purgeStale(newest_stamp_, stale_ms);
      }
    });
    stats_timer_ = create_wall_timer(std::chrono::seconds(1), [this]() { publishStats(); });
    RCLCPP_INFO(get_logger(), "sync node ready (allowed delta %.1f ms)", allowed_delta);
  }

private:
  void onCompressed(const std::string & id, const sensor_msgs::msg::CompressedImage::SharedPtr m)
  {
    noteStamp(m->header.stamp);
    // copy the payload before decoding: the subscription's serialized buffer
    // must not alias the decode input (middleware buffer reuse races)
    auto raw = decompress(id, m);
    if (!raw) return;
    auto it = pending_info_.find(id);
    if (it == pending_info_.end() || !it->second) {
      last_img_[id] = raw;
      return;
    }
    auto info = it->second;
    it->second.reset();
    processPair(id, *raw, *info);
  }

  sensor_msgs::msg::Image::SharedPtr decompress(
    const std::string & id, const sensor_msgs::msg::CompressedImage::SharedPtr m)
  {
    try {
      // defensive copy: never decode straight out of the subscription payload
      std::vector<unsigned char> jpeg_copy(m->data.begin(), m->data.end());
      const cv::Mat mat = cv::imdecode(
        cv::Mat(1, static_cast<int>(jpeg_copy.size()), CV_8UC1, jpeg_copy.data()),
        cv::IMREAD_COLOR);
      if (mat.empty()) {
        RCLCPP_WARN_THROTTLE(get_logger(), *get_clock(), 2000,
          "jpeg decode failed for %s", id.c_str());
        return nullptr;
      }
      auto raw = std::make_shared<sensor_msgs::msg::Image>();
      raw->header = m->header;
      raw->height = mat.rows;
      raw->width = mat.cols;
      raw->encoding = "bgr8";
      raw->is_bigendian = false;
      raw->step = static_cast<uint32_t>(mat.step[0]);
      raw->data.assign(mat.data, mat.data + mat.total() * mat.elemSize());
      if (const char * dbg = getenv("SYNC_DEBUG_DUMP")) {
        static std::atomic<int> dump_count{0};
        if (dump_count.fetch_add(1) < 36) {
          char stamp_s[64];
          snprintf(stamp_s, sizeof(stamp_s), "%u.%09u",
                   m->header.stamp.sec, m->header.stamp.nanosec);
          std::ofstream f(std::string(dbg) + "/t" + stamp_s + "_" + id + ".bin",
                          std::ios::binary);
          f.write(reinterpret_cast<const char *>(raw->data.data()),
                  static_cast<std::streamsize>(raw->data.size()));
        }
      }
      return raw;
    } catch (const cv::Exception & e) {
      RCLCPP_WARN_THROTTLE(get_logger(), *get_clock(), 2000, "cv error: %s", e.what());
      return nullptr;
    }
  }
  void onImage(const std::string & id, const sensor_msgs::msg::Image::SharedPtr m)
  {
    noteStamp(m->header.stamp);
    auto it = pending_info_.find(id);
    if (it == pending_info_.end() || !it->second) {
      last_img_[id] = m;
      return;
    }
    auto info = it->second;
    it->second.reset();
    processPair(id, *m, *info);
  }

  void onInfo(const std::string & id, const sensor_msgs::msg::CameraInfo::SharedPtr m)
  {
    noteStamp(m->header.stamp);
    auto it = last_img_.find(id);
    if (it == last_img_.end() || !it->second) {
      pending_info_[id] = m;
      return;
    }
    auto img = it->second;
    it->second.reset();
    processPair(id, *img, *m);
  }

  /// Track the newest stamp seen; detect rollback (playback loop restart)
  /// and reset the assembler so stale tail entries cannot contaminate the
  /// next pass.
  void noteStamp(const builtin_interfaces::msg::Time & stamp)
  {
    const rclcpp::Time t(stamp);
    if (!newest_stamp_valid_) {
      newest_stamp_ = t;
      newest_stamp_valid_ = true;
      return;
    }
    if ((newest_stamp_ - t).seconds() > 1.0) {
      RCLCPP_INFO(get_logger(), "stamp rollback detected (%.3f -> %.3f): resetting assembler",
        newest_stamp_.seconds(), t.seconds());
      assembler_->resetScene();
      newest_stamp_ = t;
      return;
    }
    if (t > newest_stamp_) newest_stamp_ = t;
  }

  void processPair(
    const std::string & id, const sensor_msgs::msg::Image & img,
    const sensor_msgs::msg::CameraInfo & info)
  {
    bev_interfaces::msg::FrameSet fs;
    fs.header.frame_id = "base_link";
    if (assembler_->update(id, img, info, &fs)) {
      last_delta_ms_ = computeDeltaMs(fs);
      frameset_pub_->publish(fs);
    }
  }

  static double computeDeltaMs(const bev_interfaces::msg::FrameSet & fs)
  {
    if (fs.images.empty()) return 0.0;
    rclcpp::Time tmin(fs.images[0].header.stamp), tmax(tmin);
    for (const auto & img : fs.images) {
      const rclcpp::Time t(img.header.stamp);
      if (t < tmin) tmin = t;
      if (t > tmax) tmax = t;
    }
    return (tmax - tmin).seconds() * 1e3;
  }

  void publishStats()
  {
    static double last_delta = 0.0;
    const auto & s = assembler_->stats();
    bev_interfaces::msg::SyncStats msg;
    msg.header.stamp = this->now();
    msg.max_timestamp_delta_ms = last_delta_ms_ >= 0.0 ? last_delta_ms_ : last_delta;
    msg.mean_timestamp_delta_ms = msg.max_timestamp_delta_ms * 0.5;  // 2-point approx
    msg.dropped_frames = s.dropped_frames;
    msg.incomplete_framesets = s.incomplete_framesets;
    msg.stale_framesets = s.stale_framesets;
    msg.published_framesets = s.published_framesets;
    stats_pub_->publish(msg);
    last_delta = last_delta_ms_ >= 0.0 ? last_delta_ms_ : last_delta;
  }

  std::vector<std::string> camera_ids_;
  std::unique_ptr<FrameSetAssembler> assembler_;
  rclcpp::Publisher<bev_interfaces::msg::FrameSet>::SharedPtr frameset_pub_;
  rclcpp::Publisher<bev_interfaces::msg::SyncStats>::SharedPtr stats_pub_;
  std::vector<rclcpp::Subscription<sensor_msgs::msg::Image>::SharedPtr> img_subs_;
  std::vector<rclcpp::Subscription<sensor_msgs::msg::CompressedImage>::SharedPtr> compressed_subs_;
  std::vector<rclcpp::Subscription<sensor_msgs::msg::CameraInfo>::SharedPtr> info_subs_;
  std::map<std::string, sensor_msgs::msg::Image::SharedPtr> last_img_;
  std::map<std::string, sensor_msgs::msg::CameraInfo::SharedPtr> pending_info_;
  rclcpp::TimerBase::SharedPtr timer_;
  rclcpp::TimerBase::SharedPtr stats_timer_;
  double last_delta_ms_ = -1.0;
  rclcpp::Time newest_stamp_{0, 0, RCL_ROS_TIME};
  bool newest_stamp_valid_{false};
};

}  // namespace bev::camera_sync

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<bev::camera_sync::CameraSyncNode>());
  rclcpp::shutdown();
  return 0;
}