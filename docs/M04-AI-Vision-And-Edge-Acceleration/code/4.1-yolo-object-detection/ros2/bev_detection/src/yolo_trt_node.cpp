// YOLO TensorRT ROS2 detection node.
// Subscribes to camera images and publishes Detection2DArray.

#include <algorithm>
#include <chrono>
#include <cmath>
#include <cstdio>
#include <memory>
#include <string>
#include <vector>

#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/image.hpp>
#include <vision_msgs/msg/detection2_d_array.hpp>
#include <vision_msgs/msg/detection2_d.hpp>
#include <vision_msgs/msg/bounding_box2_d.hpp>
#include <vision_msgs/msg/object_hypothesis_with_pose.hpp>
#include <cv_bridge/cv_bridge.h>

#include "bev_detection/yolo_engine.hpp"
#include "bev_detection/preprocessing.hpp"
#include "bev_detection/postprocessing.hpp"
#include "bev_detection/types.hpp"

namespace bev::detection
{

class YoloTrtNode : public rclcpp::Node
{
public:
  YoloTrtNode()
    : Node("yolo_trt_node")
  {
    // Declare parameters
    model_path_ = declare_parameter<std::string>("model_path", "");
    class_names_path_ = declare_parameter<std::string>("class_names_path", "");
    image_topic_ = declare_parameter<std::string>("image_topic", "/perception/cameras/front/image");
    detections_topic_ = declare_parameter<std::string>("detections_topic", "/perception/detections");
    debug_image_topic_ = declare_parameter<std::string>("debug_image_topic", "/perception/debug/detection_image");
    input_width_ = declare_parameter<int>("input_width", 640);
    input_height_ = declare_parameter<int>("input_height", 640);
    num_classes_ = declare_parameter<int>("num_classes", 80);
    confidence_threshold_ = declare_parameter<float>("confidence_threshold", 0.25f);
    nms_threshold_ = declare_parameter<float>("nms_threshold", 0.45f);
    publish_debug_image_ = declare_parameter<bool>("publish_debug_image", true);
    expected_trt_version_ = declare_parameter<std::string>("expected_trt_version", "10.3");

    // Load class names
    if (!class_names_path_.empty()) {
      if (!class_names_.load(class_names_path_)) {
        RCLCPP_WARN(get_logger(), "Failed to load class names from %s, using IDs only",
                    class_names_path_.c_str());
      }
    }

    // Load TensorRT engine
    RCLCPP_INFO(get_logger(), "Loading YOLO engine from: %s", model_path_.c_str());
    inferencer_ = std::make_unique<YoloInferencer>();

    YoloInferencer::Config config;
    config.engine_path = model_path_;
    config.input_width = input_width_;
    config.input_height = input_height_;
    config.num_classes = num_classes_;
    config.conf_thresh = confidence_threshold_;
    config.nms_thresh = nms_threshold_;

    if (!inferencer_->init(config, logger_)) {
      RCLCPP_ERROR(get_logger(), "Failed to initialize YOLO inferencer");
      startup_error_ = "Engine load failed";
      return;
    }

    RCLCPP_INFO(get_logger(), "YOLO engine loaded successfully");

    // Subscribe to image topic (use rclcpp subscription since image_transport API differs)
    image_sub_ = create_subscription<sensor_msgs::msg::Image>(
      image_topic_, rclcpp::SensorDataQoS(),
      [this](const sensor_msgs::msg::Image::SharedPtr msg) { onImage(msg); });

    // Publishers
    detections_pub_ = create_publisher<vision_msgs::msg::Detection2DArray>(
      detections_topic_, rclcpp::SensorDataQoS());

    if (publish_debug_image_) {
      debug_pub_ = create_publisher<sensor_msgs::msg::Image>(
        debug_image_topic_, rclcpp::SensorDataQoS());
    }

    // Stats timer
    stats_timer_ = create_wall_timer(std::chrono::seconds(1),
      [this]() { publishStats(); });

    RCLCPP_INFO(get_logger(), "yolo_trt_node ready. Subscribing to: %s", image_topic_.c_str());
  }

private:
  void onImage(const sensor_msgs::msg::Image::SharedPtr & msg)
  {
    const auto t_receipt = std::chrono::steady_clock::now();

    if (!inferencer_->ready()) {
      RCLCPP_WARN_THROTTLE(get_logger(), *get_clock(), 2000,
        "Engine not ready, skipping frame");
      return;
    }

    // Convert ROS image to OpenCV
    cv_bridge::CvImageConstPtr cv_ptr;
    try {
      cv_ptr = cv_bridge::toCvShare(msg, "bgr8");
    } catch (const cv_bridge::Exception & e) {
      RCLCPP_ERROR_THROTTLE(get_logger(), *get_clock(), 2000,
        "cv_bridge exception: %s", e.what());
      return;
    }

    const cv::Mat & img = cv_ptr->image;
    int img_h = img.rows;
    int img_w = img.cols;

    // Run inference
    YoloResult result = inferencer_->detect(img.data, img_h, img_w);

    // Update stats
    frames_processed_++;
    frames_in_window_++;
    last_inference_ms_ = result.inference_ms;
    last_detection_count_ = result.boxes.size();
    total_inference_ms_ += result.inference_ms;

    // Publish detections
    publishDetections(msg, result.boxes);

    // Publish debug image
    if (publish_debug_image_) {
      publishDebugImage(msg, result.boxes, img);
    }

    // End-to-end latency
    const auto t_now = std::chrono::steady_clock::now();
    const double e2e_ms = std::chrono::duration<double, std::milli>(t_now - t_receipt).count();
    last_e2e_ms_ = e2e_ms;
  }

  void publishDetections(
    const sensor_msgs::msg::Image::SharedPtr & msg,
    const std::vector<BBox> & boxes)
  {
    vision_msgs::msg::Detection2DArray detections;
    detections.header = msg->header;

    for (const auto & box : boxes) {
      vision_msgs::msg::Detection2D det;
      det.header = msg->header;

      // Bounding box (image coordinates)
      vision_msgs::msg::BoundingBox2D bbox;
      bbox.center.position.x = (box.x1 + box.x2) * 0.5;
      bbox.center.position.y = (box.y1 + box.y2) * 0.5;
      bbox.size_x = box.x2 - box.x1;
      bbox.size_y = box.y2 - box.y1;
      bbox.center.theta = 0.0;
      det.bbox = bbox;

      // Hypothesis (class + confidence)
      // Contract (M4.1 → M4.2): class_id is ALWAYS a non-empty string.
      //   - With COCO labels loaded: e.g. "person", "car"
      //   - Without labels:               "class_0", "class_1", ...
      // M4.2 must not depend on numeric encoding; the "class_" prefix
      // makes the fallback unambiguous.
      vision_msgs::msg::ObjectHypothesisWithPose hyp;
      if (!class_names_.names.empty() && box.class_id >= 0 &&
          box.class_id < static_cast<int>(class_names_.names.size())) {
        hyp.hypothesis.class_id = class_names_.names[box.class_id];
      } else {
        hyp.hypothesis.class_id = "class_" + std::to_string(box.class_id);
      }
      hyp.hypothesis.score = box.confidence;

      det.results.push_back(hyp);

      detections.detections.push_back(det);
    }

    // Contract: publish exactly one Detection2DArray per processed frame,
    // including frames with zero detections. M4.2 relies on this to
    // advance its track buffer / lost-count on empty frames.
    detections_pub_->publish(detections);
  }

  // ---- debug overlay style ------------------------------------------------
  //
  // The previous overlay used a 12-entry fixed palette that included near
  // black entries -- (0,0,128) and (128,0,0) -- a fixed 2 px line, a 0.5 font
  // scale and a label chip drawn unconditionally ABOVE the box. On a bright
  // fisheye scene the boxes were almost invisible, and any box touching the
  // top or right border had its label clipped away.
  //
  // Brightness + distinctness is now guaranteed by walking the hue circle in
  // golden-angle steps at S=215/V=255, so every class gets a saturated,
  // clearly separable colour. Geometry adapts to the frame size and the label
  // chip is clamped inside the image on all four sides.

  static cv::Scalar hsvToBgr(double hue_deg, int sat, int val)
  {
    cv::Mat hsv(1, 1, CV_8UC3, cv::Scalar(hue_deg / 2.0, sat, val));
    cv::Mat bgr;
    cv::cvtColor(hsv, bgr, cv::COLOR_HSV2BGR);
    const cv::Vec3b p = bgr.at<cv::Vec3b>(0, 0);
    return cv::Scalar(p[0], p[1], p[2]);
  }

  static const std::vector<cv::Scalar> & classPalette()
  {
    static const std::vector<cv::Scalar> palette = [] {
      std::vector<cv::Scalar> out;
      out.reserve(64);
      for (int i = 0; i < 64; ++i) {
        // 137.508 deg is the golden angle: successive hues stay far apart,
        // so adjacent class ids never look alike.
        out.push_back(hsvToBgr(std::fmod(i * 137.508, 360.0), 215, 255));
      }
      return out;
    }();
    return palette;
  }

  static cv::Scalar colorForClass(int class_id)
  {
    const auto & palette = classPalette();
    const int idx = ((class_id % static_cast<int>(palette.size())) +
                     static_cast<int>(palette.size())) %
                    static_cast<int>(palette.size());
    return palette[static_cast<std::size_t>(idx)];
  }

  // Four thick L-shaped corners: prominent without covering the object.
  static void drawCornerBrackets(
    cv::Mat & im, const cv::Rect & r, const cv::Scalar & color,
    int thickness, int len)
  {
    const int x1 = r.x;
    const int y1 = r.y;
    const int x2 = r.x + r.width - 1;
    const int y2 = r.y + r.height - 1;
    const int lx = std::max(4, std::min(len, r.width / 2));
    const int ly = std::max(4, std::min(len, r.height / 2));

    cv::line(im, cv::Point(x1, y1), cv::Point(x1 + lx, y1), color, thickness, cv::LINE_AA);
    cv::line(im, cv::Point(x1, y1), cv::Point(x1, y1 + ly), color, thickness, cv::LINE_AA);
    cv::line(im, cv::Point(x2, y1), cv::Point(x2 - lx, y1), color, thickness, cv::LINE_AA);
    cv::line(im, cv::Point(x2, y1), cv::Point(x2, y1 + ly), color, thickness, cv::LINE_AA);
    cv::line(im, cv::Point(x1, y2), cv::Point(x1 + lx, y2), color, thickness, cv::LINE_AA);
    cv::line(im, cv::Point(x1, y2), cv::Point(x1, y2 - ly), color, thickness, cv::LINE_AA);
    cv::line(im, cv::Point(x2, y2), cv::Point(x2 - lx, y2), color, thickness, cv::LINE_AA);
    cv::line(im, cv::Point(x2, y2), cv::Point(x2, y2 - ly), color, thickness, cv::LINE_AA);
  }

  // Filled label chip, kept fully inside the frame.
  static void drawLabelChip(
    cv::Mat & im, const std::string & text, const cv::Rect & r,
    const cv::Scalar & color, double font_scale)
  {
    const int font = cv::FONT_HERSHEY_DUPLEX;
    int baseline = 0;
    const cv::Size ts = cv::getTextSize(text, font, font_scale, 1, &baseline);
    const int pad_x = std::max(4, static_cast<int>(std::lround(font_scale * 8.0)));
    const int pad_y = std::max(3, static_cast<int>(std::lround(font_scale * 5.0)));
    const int chip_w = ts.width + 2 * pad_x;
    const int chip_h = ts.height + baseline + 2 * pad_y;

    // Above the box when there is room, otherwise below it.
    int y = r.y - chip_h;
    if (y < 0) {
      y = r.y + r.height;
    }
    // Clamp on all four sides -- this is what stops the border-clipped labels.
    y = std::max(0, std::min(y, std::max(0, im.rows - chip_h)));
    const int x = std::max(0, std::min(r.x, std::max(0, im.cols - chip_w)));

    // LINE_8 so the chip fill exactly covers its rect (LINE_AA + FILLED
    // leaves a partially transparent inset edge that looks like a seam).
    cv::rectangle(im, cv::Rect(x, y, chip_w, chip_h), color, cv::FILLED);
    cv::putText(
      im, text, cv::Point(x + pad_x, y + pad_y + ts.height),
      font, font_scale, cv::Scalar(255, 255, 255), 1, cv::LINE_AA);
  }

  // Translucent top-left HUD so students can see throughput without a shell.
  void drawHudPanel(cv::Mat & im, std::size_t n_boxes)
  {
    const int w = im.cols;
    const int h = im.rows;
    const double fs = std::max(0.5, std::min(w, h) / 1600.0);
    const int font = cv::FONT_HERSHEY_DUPLEX;
    const int pad = std::max(6, static_cast<int>(std::lround(14.0 * fs)));
    const int gap = std::max(2, static_cast<int>(std::lround(6.0 * fs)));

    char line1[96];
    char line2[128];
    std::snprintf(line1, sizeof(line1), "M4.1  YOLO TensorRT");
    std::snprintf(line2, sizeof(line2), "FPS %.1f    boxes %zu", hud_fps_, n_boxes);

    int b1 = 0;
    int b2 = 0;
    const cv::Size t1 = cv::getTextSize(line1, font, fs, 1, &b1);
    const cv::Size t2 = cv::getTextSize(line2, font, fs, 1, &b2);
    const int text_w = std::max(t1.width, t2.width);
    const int text_h = t1.height + b1 + gap + t2.height + b2;

    cv::Rect panel(0, 0, text_w + 2 * pad, text_h + 2 * pad);
    panel &= cv::Rect(0, 0, w, h);
    if (panel.width < 4 || panel.height < 4) {
      return;
    }

    // Opaque backing. A translucent panel let a saturated label chip from a
    // detection underneath bleed through, which made the HUD unreadable
    // (e.g. "FPS 23.9 boxes 3" with a green "traffic light 35%" showing
    // through between the two). The HUD is drawn last, so it always wins.
    // LINE_8 (not LINE_AA) so the fill exactly covers `panel`; anti-aliased
    // FILLED rectangles leave a partially transparent inset edge.
    cv::rectangle(im, panel, cv::Scalar(24, 28, 34), cv::FILLED);
    cv::line(im, cv::Point(panel.x, panel.y + panel.height - 1),
             cv::Point(panel.x + panel.width - 1, panel.y + panel.height - 1),
             cv::Scalar(60, 160, 255), 2, cv::LINE_AA);

    int y = panel.y + pad + t1.height;
    cv::putText(im, line1, cv::Point(panel.x + pad, y), font, fs,
                cv::Scalar(255, 255, 255), 1, cv::LINE_AA);
    y += b1 + gap + t2.height;
    cv::putText(im, line2, cv::Point(panel.x + pad, y), font, fs,
                cv::Scalar(120, 230, 255), 1, cv::LINE_AA);
  }

  void updateHudFps()
  {
    const auto now = std::chrono::steady_clock::now();
    if (last_draw_t_.time_since_epoch().count() != 0) {
      const double dt = std::chrono::duration<double>(now - last_draw_t_).count();
      if (dt > 1e-6) {
        const double inst = 1.0 / dt;
        hud_fps_ = (hud_fps_ <= 0.0) ? inst : (0.9 * hud_fps_ + 0.1 * inst);
      }
    }
    last_draw_t_ = now;
  }

  void publishDebugImage(
    const sensor_msgs::msg::Image::SharedPtr & msg,
    const std::vector<BBox> & boxes,
    const cv::Mat & img)
  {
    cv::Mat vis_img = img.clone();
    const int w = vis_img.cols;
    const int h = vis_img.rows;
    const cv::Rect frame_rect(0, 0, w, h);

    // Scale the overlay with the frame: at 1920x1080 this gives a 4 px line
    // and a ~0.77 font instead of the old fixed 2 px / 0.5.
    const int thickness =
      std::max(2, static_cast<int>(std::lround(std::min(w, h) / 300.0)));
    const int inner_thickness = std::max(1, thickness / 3);
    const int corner_len =
      std::max(12, static_cast<int>(std::lround(std::min(w, h) / 45.0)));
    const double font_scale = std::max(0.55, std::min(w, h) / 1400.0);

    for (const auto & box : boxes) {
      cv::Rect roi(
        static_cast<int>(std::lround(box.x1)),
        static_cast<int>(std::lround(box.y1)),
        static_cast<int>(std::lround(box.x2 - box.x1)),
        static_cast<int>(std::lround(box.y2 - box.y1)));
      // Clamp to the image: the old code drew outside the frame and the
      // label of a border box was cut off with it.
      roi &= frame_rect;
      if (roi.width < 2 || roi.height < 2) {
        continue;
      }

      const cv::Scalar color = colorForClass(box.class_id);

      // Thin full outline for extent + thick bright corners for prominence.
      cv::rectangle(vis_img, roi, color, inner_thickness, cv::LINE_AA);
      drawCornerBrackets(vis_img, roi, color, thickness, corner_len);

      // Mirror the publishDetections contract for the class string.
      std::string label;
      if (!class_names_.names.empty() && box.class_id >= 0 &&
          box.class_id < static_cast<int>(class_names_.names.size())) {
        label = class_names_.names[box.class_id];
      } else {
        label = "class_" + std::to_string(box.class_id);
      }
      label += " " + std::to_string(static_cast<int>(box.confidence * 100)) + "%";

      drawLabelChip(vis_img, label, roi, color, font_scale);
    }

    updateHudFps();
    drawHudPanel(vis_img, boxes.size());

    cv_bridge::CvImage out_img;
    out_img.header = msg->header;
    out_img.encoding = "bgr8";
    out_img.image = vis_img;

    sensor_msgs::msg::Image::SharedPtr img_msg = out_img.toImageMsg();
    debug_pub_->publish(*img_msg);
  }

  void publishStats()
  {
    // FPS
    double dt = 1.0;
    double fps = frames_in_window_ / dt;
    frames_in_window_ = 0;

    RCLCPP_INFO_THROTTLE(get_logger(), *get_clock(), 1000,
      "YOLO: fps=%.1f, last_inf=%.2fms, last_e2e=%.2fms, boxes=%zu",
      fps, last_inference_ms_, last_e2e_ms_,
      last_detection_count_);

    (void)total_inference_ms_;
    (void)last_detection_count_;
  }

  // ROS
  rclcpp::Subscription<sensor_msgs::msg::Image>::SharedPtr image_sub_;
  rclcpp::Publisher<vision_msgs::msg::Detection2DArray>::SharedPtr detections_pub_;
  rclcpp::Publisher<sensor_msgs::msg::Image>::SharedPtr debug_pub_;
  rclcpp::TimerBase::SharedPtr stats_timer_;

  // Parameters
  std::string model_path_;
  std::string class_names_path_;
  std::string image_topic_;
  std::string detections_topic_;
  std::string debug_image_topic_;
  int input_width_;
  int input_height_;
  int num_classes_;
  float confidence_threshold_;
  float nms_threshold_;
  bool publish_debug_image_;
  std::string expected_trt_version_;

  // Engine
  Logger logger_;
  std::string startup_error_;
  std::unique_ptr<YoloInferencer> inferencer_;
  ClassNames class_names_;

  // Stats
  std::uint64_t frames_processed_{0};
  std::size_t frames_in_window_{0};
  float last_inference_ms_{0.f};
  double last_e2e_ms_{0.f};
  double total_inference_ms_{0.f};
  std::size_t last_detection_count_{0};

  // Debug-overlay HUD state (smoothed draw rate, independent of the
  // 1 Hz publishStats() window which resets frames_in_window_).
  double hud_fps_{0.0};
  std::chrono::steady_clock::time_point last_draw_t_{};
};

}  // namespace bev::detection

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<bev::detection::YoloTrtNode>());
  rclcpp::shutdown();
  return 0;
}
