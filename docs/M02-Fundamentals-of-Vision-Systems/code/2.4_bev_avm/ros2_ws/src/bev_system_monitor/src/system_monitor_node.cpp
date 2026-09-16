// bev_system_monitor: aggregates pipeline health into /bev/system_status.
#include <chrono>
#include <memory>
#include <string>

#include "autoware_perception_msgs/msg/detected_objects.hpp"
#include "bev_interfaces/msg/sync_stats.hpp"
#include "bev_interfaces/msg/system_status.hpp"
#include "rclcpp/rclcpp.hpp"

namespace bev::monitor
{

class SystemMonitorNode : public rclcpp::Node
{
public:
  SystemMonitorNode() : Node("bev_system_monitor")
  {
    objects_sub_ = create_subscription<autoware_perception_msgs::msg::DetectedObjects>(
      "/bev/objects", rclcpp::SensorDataQoS(),
      [this](const autoware_perception_msgs::msg::DetectedObjects::SharedPtr m) {
        onObjects(m);
      });
    sync_sub_ = create_subscription<bev_interfaces::msg::SyncStats>(
      "/camera/sync_stats", rclcpp::QoS(2),
      [this](const bev_interfaces::msg::SyncStats::SharedPtr m) { last_sync_ = *m; });

    status_pub_ = create_publisher<bev_interfaces::msg::SystemStatus>("/bev/system_status", rclcpp::QoS(2));
    timer_ = create_wall_timer(std::chrono::milliseconds(500), [this]() { tick(); });
    RCLCPP_INFO(get_logger(), "system monitor running");
  }

private:
  void onObjects(const autoware_perception_msgs::msg::DetectedObjects::SharedPtr m)
  {
    const auto now = get_clock()->now();
    double dt_ms = 1000.0;
    if (last_obj_time_.nanoseconds() > 0) {
      const double dt = (now - last_obj_time_).seconds();
      if (dt > 0.001) {
        objects_fps_ = 0.8 * objects_fps_ + 0.2 * (1.0 / dt);
        dt_ms = dt * 1000.0;
      }
    }
    last_obj_time_ = now;
    last_obj_count_ = m->objects.size();
    last_obj_stamp_ = m->header.stamp;
    // inter-arrival time as latency proxy: avoids system-clock vs playback-
    // stamp domain mismatch when validating from rosbags
    latency_ms_ = static_cast<float>(dt_ms);
  }

  void tick()
  {
    bev_interfaces::msg::SystemStatus msg;
    const auto now_t = get_clock()->now();
    const double since_obj = (now_t - last_obj_time_).seconds();

    if (last_obj_time_.nanoseconds() == 0) {
      msg.state = bev_interfaces::msg::SystemStatus::INITIALIZING;
      msg.message = "waiting for perception output";
    } else if (since_obj > 2.0) {
      msg.state = bev_interfaces::msg::SystemStatus::ERROR;
      msg.message = "perception timeout — no /bev/objects for >2s";
    } else if (since_obj > 0.5 || latency_ms_ > 150.0 ||
               last_sync_.dropped_frames > 0 || last_sync_.incomplete_framesets > 0 ||
               last_sync_.stale_framesets > 0) {
      msg.state = bev_interfaces::msg::SystemStatus::DEGRADED;
      msg.message = "latency or camera-sync degradation";
    } else {
      msg.state = bev_interfaces::msg::SystemStatus::READY;
      msg.message = "system ready";
    }

    msg.camera_fps = 0.0f;
    msg.camera_sync_delta_ms = last_sync_.max_timestamp_delta_ms;
    msg.inference_fps = objects_fps_;
    msg.inference_latency_ms = latency_ms_;
    msg.total_latency_ms = latency_ms_;
    msg.detected_objects = last_obj_count_;
    msg.gpu_usage = 0.0f;   // populated in Phase 3+ (nvml)
    msg.gpu_memory = 0.0f;
    status_pub_->publish(msg);
  }

  rclcpp::Subscription<autoware_perception_msgs::msg::DetectedObjects>::SharedPtr objects_sub_;
  rclcpp::Subscription<bev_interfaces::msg::SyncStats>::SharedPtr sync_sub_;
  rclcpp::Publisher<bev_interfaces::msg::SystemStatus>::SharedPtr status_pub_;
  rclcpp::TimerBase::SharedPtr timer_;

  rclcpp::Time last_obj_time_{0, 0, RCL_ROS_TIME};
  rclcpp::Time last_obj_stamp_{0, 0, RCL_ROS_TIME};
  std::size_t last_obj_count_{0};
  double objects_fps_{0.0};
  float latency_ms_{0.0f};
  bev_interfaces::msg::SyncStats last_sync_;
};

}  // namespace bev::monitor

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<bev::monitor::SystemMonitorNode>());
  rclcpp::shutdown();
  return 0;
}