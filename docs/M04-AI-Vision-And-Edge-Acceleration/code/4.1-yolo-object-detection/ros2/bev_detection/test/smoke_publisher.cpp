// YOLO ROS smoke test: publishes synthetic image, verifies detection output.
// Uses standalone executables to verify the full ROS2 pipeline.

#include <memory>
#include <string>
#include <chrono>
#include <thread>

#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/image.hpp>
#include <vision_msgs/msg/detection2_d_array.hpp>
#include <cv_bridge/cv_bridge.h>

#include <opencv2/opencv.hpp>

using namespace std::chrono_literals;

class SmokePublisher : public rclcpp::Node
{
public:
  SmokePublisher() : Node("smoke_publisher")
  {
    image_pub_ = create_publisher<sensor_msgs::msg::Image>(
      "/perception/cameras/front/image", rclcpp::SensorDataQoS());

    detections_sub_ = create_subscription<vision_msgs::msg::Detection2DArray>(
      "/perception/detections", rclcpp::SensorDataQoS(),
      [this](const vision_msgs::msg::Detection2DArray::SharedPtr msg) {
        received_count_++;
        last_received_size_ = msg->detections.size();
        if (!msg->detections.empty()) {
          auto & det = msg->detections[0];
          auto & bbox = det.bbox;
          RCLCPP_INFO(get_logger(), "Got detection: center=(%.1f, %.1f), size=(%.1f, %.1f), conf=%.2f",
                      bbox.center.position.x, bbox.center.position.y,
                      bbox.size_x, bbox.size_y,
                      det.results.empty() ? 0.f : det.results[0].hypothesis.score);
        }
      });

    timer_ = create_wall_timer(100ms, [this]() { publishFrame(); });
    RCLCPP_INFO(get_logger(), "Smoke publisher ready");
  }

  void publishFrame()
  {
    static int counter = 0;
    counter++;

    // Create test image
    cv::Mat img = cv::Mat::zeros(900, 1600, CV_8UC3);
    cv::rectangle(img, cv::Point(200, 200), cv::Point(800, 800),
                  cv::Scalar(255, 0, 0), -1);
    cv::rectangle(img, cv::Point(900, 300), cv::Point(1400, 800),
                  cv::Scalar(0, 255, 0), -1);
    cv::putText(img, "FRAME " + std::to_string(counter),
                cv::Point(50, 100), cv::FONT_HERSHEY_SIMPLEX, 3,
                cv::Scalar(255, 255, 255), 5);

    auto msg = cv_bridge::CvImage(std_msgs::msg::Header(), "bgr8", img).toImageMsg();
    msg->header.stamp = now();
    msg->header.frame_id = "camera_front";
    image_pub_->publish(*msg);

    if (counter % 10 == 0) {
      RCLCPP_INFO(get_logger(), "Published %d frames, received %zu detections",
                  counter, last_received_size_);
    }
  }

  int received_count_ = 0;
  size_t last_received_size_ = 0;

private:
  rclcpp::Publisher<sensor_msgs::msg::Image>::SharedPtr image_pub_;
  rclcpp::Subscription<vision_msgs::msg::Detection2DArray>::SharedPtr detections_sub_;
  rclcpp::TimerBase::SharedPtr timer_;
};

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  auto node = std::make_shared<SmokePublisher>();
  rclcpp::spin(node);
  rclcpp::shutdown();
  return 0;
}
