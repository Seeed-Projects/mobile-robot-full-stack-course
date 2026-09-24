// Camera adapter node: extracts front camera from FrameSet and publishes as Image.
// This is a thin adapter - YOLO subscribes to standard Image topic.
#include <memory>
#include <string>
#include <vector>

#include "bev_interfaces/msg/frame_set.hpp"
#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/image.hpp"

namespace bev::detection
{

class CameraAdapterNode : public rclcpp::Node
{
public:
  CameraAdapterNode()
    : Node("camera_adapter_node")
  {
    // Parameters
    frameset_topic_ = declare_parameter<std::string>("frameset_topic", "/bev/frameset");
    front_image_topic_ = declare_parameter<std::string>("front_image_topic", "/perception/cameras/front/image");
    camera_id_ = declare_parameter<std::string>("camera_id", "front");

    // Subscribe to FrameSet
    frameset_sub_ = create_subscription<bev_interfaces::msg::FrameSet>(
      frameset_topic_, rclcpp::QoS(5).reliable(),
      [this](const bev_interfaces::msg::FrameSet::SharedPtr fs) { onFrameSet(fs); });

    // Publisher for front camera image
    front_pub_ = create_publisher<sensor_msgs::msg::Image>(
      front_image_topic_, rclcpp::SensorDataQoS());

    RCLCPP_INFO(get_logger(), "camera_adapter_node ready. "
      "Subscribing to %s, publishing front camera to %s",
      frameset_topic_.c_str(), front_image_topic_.c_str());
  }

private:
  void onFrameSet(const bev_interfaces::msg::FrameSet::SharedPtr & fs)
  {
    // Find front camera in the FrameSet
    int front_idx = -1;
    for (std::size_t i = 0; i < fs->camera_ids.size(); ++i) {
      if (fs->camera_ids[i] == camera_id_) {
        front_idx = static_cast<int>(i);
        break;
      }
    }

    if (front_idx < 0 || front_idx >= static_cast<int>(fs->images.size())) {
      RCLCPP_WARN_THROTTLE(get_logger(), *get_clock(), 3000,
        "Camera '%s' not found in FrameSet", camera_id_.c_str());
      return;
    }

    // Publish front camera image with original timestamp
    const auto & img = fs->images[front_idx];
    sensor_msgs::msg::Image out_img;
    out_img.header = img.header;
    out_img.header.frame_id = "camera_" + camera_id_;  // Keep frame_id
    out_img.height = img.height;
    out_img.width = img.width;
    out_img.encoding = img.encoding;
    out_img.is_bigendian = img.is_bigendian;
    out_img.step = img.step;
    out_img.data = img.data;

    front_pub_->publish(out_img);
  }

  rclcpp::Subscription<bev_interfaces::msg::FrameSet>::SharedPtr frameset_sub_;
  rclcpp::Publisher<sensor_msgs::msg::Image>::SharedPtr front_pub_;

  std::string frameset_topic_;
  std::string front_image_topic_;
  std::string camera_id_;
};

}  // namespace bev::detection

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<bev::detection::CameraAdapterNode>());
  rclcpp::shutdown();
  return 0;
}
