// bevdet_node: FrameSet -> TensorRT BEVDet FP16 -> DetectedObjects (base_link).
//
// Startup validation (§35): engine file, TRT version, IO contract, camera
// count, resolution. Hard errors, no silent fallback.
#include <chrono>
#include <cstdio>
#include <cstdlib>
#include <fstream>
#include <map>
#include <memory>
#include <set>
#include <string>
#include <vector>

#include <Eigen/Core>
#include <Eigen/Geometry>

#include <optional>
#include "autoware_perception_msgs/msg/detected_objects.hpp"
#include "bev_interfaces/msg/frame_set.hpp"
#include "bev_perception/engine_loader.hpp"
#include "bev_preprocessor/gpu_image_stager.hpp"
#include "bevdet.h"
#include "data.h"
#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/camera_info.hpp"
#include "sensor_msgs/msg/image.hpp"
#include "std_msgs/msg/float32.hpp"
#include "tf2/convert.h"
#include "tf2_ros/buffer.h"
#include "tf2_ros/transform_listener.h"

namespace bev::perception
{

using bev_interfaces::msg::FrameSet;

class BevdetNode : public rclcpp::Node
{
public:
  BevdetNode() : Node("bevdet_node"), tf_buffer_(get_clock()), tf_listener_(tf_buffer_)
  {
    engine_path_ = declare_parameter("engine_path", std::string(""));
    onnx_path_ = declare_parameter("onnx_path", std::string(""));
    model_config_ = declare_parameter("model_config", std::string(""));
    precision_ = declare_parameter("precision", std::string("fp16"));
    expected_trt_ = declare_parameter("expected_trt_version", std::string("10.3"));
    score_threshold_ = declare_parameter("score_threshold", 0.25);
    scene_gap_s_ = declare_parameter("scene_gap_seconds", 2.0);
    frameset_topic_ = declare_parameter("frameset_topic", std::string("/bev/frameset"));
    camera_ids_ = declare_parameter(
      "camera_ids",
      std::vector<std::string>{"front_left", "front", "front_right", "back_left", "back", "back_right"});
    auto_build_engine_ = declare_parameter("auto_build_engine", false);
    const bool verbose = declare_parameter("verbose", false);

    // ---- startup validation gate --------------------------------------------
    engine_lifetime_ = loadEngine(engine_path_, logger_, &startup_error_);
    if (!engine_lifetime_) {
      RCLCPP_ERROR(get_logger(), "ENGINE LOAD FAILED: %s", startup_error_.c_str());
      return;  // node keeps running in INITIALIZING (no data published)
    }
    engine_info_ = inspectEngine(*engine_lifetime_->engine);
    if (!validateEngine(engine_info_, expected_trt_, &startup_error_)) {
      RCLCPP_ERROR(get_logger(), "ENGINE VALIDATION FAILED: %s", startup_error_.c_str());
      engine_lifetime_.reset();
      return;
    }
    RCLCPP_INFO(get_logger(), "engine OK: TRT %s, %d tensors, images=%dx%d INT%d-carrier",
      engine_info_.trt_version.c_str(), engine_info_.nb_tensors,
      static_cast<int>(engine_info_.images_dims.d[2]),
      static_cast<int>(engine_info_.images_dims.d[3]),
      static_cast<int>(engine_info_.images_dtype));

    frameset_sub_ = create_subscription<FrameSet>(
      frameset_topic_, rclcpp::QoS(5).reliable(),
      [this](FrameSet::SharedPtr fs) { onFrameSet(std::move(fs)); });

    objects_pub_ = create_publisher<autoware_perception_msgs::msg::DetectedObjects>(
      "/bev/objects", rclcpp::SensorDataQoS());
    fps_pub_ = create_publisher<std_msgs::msg::Float32>("/bev/debug/fps", rclcpp::QoS(2));
    latency_pub_ = create_publisher<std_msgs::msg::Float32>("/bev/debug/latency", rclcpp::QoS(2));

    stats_timer_ = create_wall_timer(std::chrono::seconds(1), [this]() { publishDebugStats(); });
    (void)verbose;
    RCLCPP_INFO(get_logger(), "bevdet_node: waiting for framesets on %s", frameset_topic_.c_str());
  }

private:
  void onFrameSet(FrameSet::SharedPtr fs)
  {
    const auto ts_receipt = std::chrono::steady_clock::now();
    if (!engine_lifetime_) return;  // failed startup: never publish

    if (fs->images.size() != 6 || fs->camera_infos.size() != 6) {
      RCLCPP_WARN_THROTTLE(get_logger(), *get_clock(), 2000,
        "frameset with %zu images ignored (expected 6)", fs->images.size());
      return;
    }

    // -------- one-time init: calib anchor + rank precompute ----------------
    if (!bevdet_) {
      if (!initializeBevdet(fs)) return;
    }

    // -------- scene change detection -> temporal buffer reset ---------------
    const auto stamp = rclcpp::Time(fs->header.stamp);
    const double gap = (stamp - last_frame_stamp_).seconds();
    if (!last_frame_stamp_valid_ || gap > scene_gap_s_) {
      scene_index_++;
    }
    last_frame_stamp_ = stamp;
    last_frame_stamp_valid_ = true;

    const auto t0 = std::chrono::steady_clock::now();

    // -------- per-frame camera parameters (K from CameraInfo, sensor2ego + ego pose from TF)
    camsData cd;
    cd.param = camParams(cams_intrin_anchor_, cams2ego_rot_anchor_, cams2ego_trans_anchor_);
    if (!fillPerFrameParams(fs, &cd.param)) return;

    // -------- images -> planar BGR GPU staging -------------------------------
    std::vector<const std::uint8_t *> ptrs;
    ptrs.reserve(6);
    for (const auto & img : fs->images) {
      if (img.encoding != "bgr8" || img.height != 900 || img.width != 1600) {
        RCLCPP_WARN_THROTTLE(get_logger(), *get_clock(), 3000,
          "unsupported image encoding/size %s %dx%d (need bgr8 1600x900)",
          img.encoding.c_str(), img.width, img.height);
        return;
      }
      ptrs.push_back(img.data.data());
    }
    if (getenv("BEV_DEBUG_DUMP") && frames_processed_ < 1) {
      dumpDebugImages(frames_processed_, fs);
    }
    if (stager_->stage(ptrs, nullptr) != cudaSuccess) {
      RCLCPP_ERROR_THROTTLE(get_logger(), *get_clock(), 2000, "GPU staging failed");
      return;
    }
    cd.imgs_dev = stager_->devicePtr();

    // -------- inference ------------------------------------------------------
    std::vector<Box> boxes;
    float infer_ms = 0.f;
    bevdet_->DoInfer(cd, boxes, infer_ms, static_cast<int>(frames_processed_));
    frames_processed_++;

    // -------- publish --------------------------------------------------------
    publishObjects(fs->header, boxes);

    const auto t1 = std::chrono::steady_clock::now();
    const double total_ms = std::chrono::duration<double, std::milli>(t1 - t0).count();
    // end-to-end measured from frameset receipt (avoids clock-domain mismatch
    // between bag playback stamps and the system clock); includes queue wait
    const double e2e_ms = std::chrono::duration<double, std::milli>(t1 - ts_receipt).count();
    last_infer_ms_ = infer_ms;
    last_total_ms_ = total_ms;
    last_e2e_ms_ = e2e_ms;
    frames_in_window_++;
  }

  /// Debug hook: write the first received frameset's images to BEV_DEBUG_DUMP
  /// (PPM; no OpenCV dependency). Verifies decode + camera-slot ordering.
  static void dumpDebugImages(int frame_idx, const FrameSet::SharedPtr & fs)
  {
    const char * dir = getenv("BEV_DEBUG_DUMP");
    if (!dir) return;
    char stamp_s[64];
    snprintf(stamp_s, sizeof(stamp_s), "%u.%09u",
             fs->header.stamp.sec, fs->header.stamp.nanosec);
    for (std::size_t i = 0; i < fs->images.size(); ++i) {
      const auto & img = fs->images[i];
      std::string p = std::string(dir) + "/t" + stamp_s + "_slot" + std::to_string(i) +
        "_" + fs->camera_ids[i] + ".ppm";
      std::ofstream f(p, std::ios::binary);
      if (!f) continue;
      f << "P6\n" << img.width << " " << img.height << "\n255\n";
      f.write(reinterpret_cast<const char *>(img.data.data()),
              static_cast<std::streamsize>(img.data.size()));
    }
  }

  bool initializeBevdet(const FrameSet::SharedPtr & fs)
  {
    // intrinsics from CameraInfo K
    cams_intrin_anchor_.clear();
    for (std::size_t i = 0; i < 6; ++i) {
      const auto & ci = fs->camera_infos[i];
      if (ci.k.size() != 9) {
        RCLCPP_ERROR(get_logger(), "camera_info K invalid for %s", fs->camera_ids[i].c_str());
        return false;
      }
      Eigen::Matrix3f K;
      K << ci.k[0], ci.k[1], ci.k[2],
           ci.k[3], ci.k[4], ci.k[5],
           ci.k[6], ci.k[7], ci.k[8];
      cams_intrin_anchor_.push_back(K);
    }

    // extrinsics from TF base_link -> camera_<id> at the frameset stamp
    const auto stamp = rclcpp::Time(fs->header.stamp);
    cams2ego_rot_anchor_.clear();
    cams2ego_trans_anchor_.clear();
    for (std::size_t i = 0; i < 6; ++i) {
      const std::string child = "camera_" + fs->camera_ids[i];
      geometry_msgs::msg::TransformStamped tf;
      try {
        tf = tf_buffer_.lookupTransform("base_link", child, stamp);
      } catch (const tf2::TransformException & e) {
        RCLCPP_ERROR(get_logger(), "startup TF lookup base_link->%s failed: %s",
          child.c_str(), e.what());
        return false;
      }
      Eigen::Quaternionf q(
        tf.transform.rotation.w, tf.transform.rotation.x,
        tf.transform.rotation.y, tf.transform.rotation.z);
      cams2ego_rot_anchor_.push_back(q);
      cams2ego_trans_anchor_.push_back(Eigen::Translation3f(
        tf.transform.translation.x, tf.transform.translation.y, tf.transform.translation.z));
    }

    // camera identity / order validation against deployed contract
    for (std::size_t i = 0; i < 6; ++i) {
      // frameset carries ids in engine order; verify they are distinct and known
      if (std::find(camera_ids_.begin(), camera_ids_.end(), fs->camera_ids[i]) ==
          camera_ids_.end()) {
        RCLCPP_ERROR(get_logger(), "unknown camera id %s at slot %zu",
          fs->camera_ids[i].c_str(), i);
        return false;
      }
    }
    if (std::set<std::string>(fs->camera_ids.begin(), fs->camera_ids.end()).size() != 6) {
      RCLCPP_ERROR(get_logger(), "duplicate camera ids in frameset");
      return false;
    }

    stager_ = std::make_unique<bev::preprocessor::GpuImageStager>(6, 900, 1600);

    RCLCPP_INFO(get_logger(), "initializing BEVDet (calib anchor from first frameset)");
    bevdet_ = std::make_unique<BEVDet>(
      model_config_, 6,
      cams_intrin_anchor_, cams2ego_rot_anchor_, cams2ego_trans_anchor_,
      onnx_path_, engine_path_, precision_);
    bevdet_->setVerbose(false);
    return true;
  }

  bool fillPerFrameParams(const FrameSet::SharedPtr & fs, camParams * param)
  {
    const auto stamp = rclcpp::Time(fs->header.stamp);
    param->scene_token = "scene_" + std::to_string(scene_index_);
    param->timestamp = static_cast<unsigned long long>(
      fs->header.stamp.sec) * 1000000ULL +
      static_cast<unsigned long long>(fs->header.stamp.nanosec) / 1000ULL;

    // per-frame intrinsics (may drift across scenes; ranks stay anchored)
    param->cams_intrin.clear();
    for (std::size_t i = 0; i < 6; ++i) {
      const auto & ci = fs->camera_infos[i];
      Eigen::Matrix3f K;
      K << ci.k[0], ci.k[1], ci.k[2], ci.k[3], ci.k[4], ci.k[5], ci.k[6], ci.k[7], ci.k[8];
      param->cams_intrin.push_back(K);
    }
    // per-frame extrinsics from TF
    param->cams2ego_rot.clear();
    param->cams2ego_trans.clear();
    for (std::size_t i = 0; i < 6; ++i) {
      geometry_msgs::msg::TransformStamped tf;
      try {
        tf = tf_buffer_.lookupTransform("base_link", "camera_" + fs->camera_ids[i], stamp);
      } catch (const tf2::TransformException & e) {
        RCLCPP_WARN_THROTTLE(get_logger(), *get_clock(), 3000,
          "TF lookup failed for %s: %s", fs->camera_ids[i].c_str(), e.what());
        return false;
      }
      param->cams2ego_rot.push_back(Eigen::Quaternionf(
        tf.transform.rotation.w, tf.transform.rotation.x,
        tf.transform.rotation.y, tf.transform.rotation.z));
      param->cams2ego_trans.push_back(Eigen::Translation3f(
        tf.transform.translation.x, tf.transform.translation.y, tf.transform.translation.z));
    }
    // ego pose for temporal fusion (map -> base_link)
    try {
      const auto tf = tf_buffer_.lookupTransform("map", "base_link", stamp);
      param->ego2global_rot = Eigen::Quaternionf(
        tf.transform.rotation.w, tf.transform.rotation.x,
        tf.transform.rotation.y, tf.transform.rotation.z);
      param->ego2global_trans = Eigen::Translation3f(
        tf.transform.translation.x, tf.transform.translation.y, tf.transform.translation.z);
    } catch (const tf2::TransformException &) {
      static bool warned = false;
      if (!warned) {
        RCLCPP_WARN(get_logger(),
          "map->base_link TF unavailable; temporal alignment degraded (ego2global=identity)");
        warned = true;
      }
      param->ego2global_rot = Eigen::Quaternionf::Identity();
      param->ego2global_trans = Eigen::Translation3f(0.f, 0.f, 0.f);
    }
    return true;
  }

  static std::uint8_t mapLabel(int model_label)
  {
    // nuScenes 10-class -> autoware ObjectClassification label
    switch (model_label) {
      case 0: return autoware_perception_msgs::msg::ObjectClassification::CAR;       // car
      case 1: return autoware_perception_msgs::msg::ObjectClassification::TRUCK;     // truck
      case 2: return autoware_perception_msgs::msg::ObjectClassification::TRUCK;     // construction_vehicle
      case 3: return autoware_perception_msgs::msg::ObjectClassification::BUS;       // bus
      case 4: return autoware_perception_msgs::msg::ObjectClassification::TRAILER;   // trailer
      case 5: return autoware_perception_msgs::msg::ObjectClassification::UNKNOWN;   // barrier
      case 6: return autoware_perception_msgs::msg::ObjectClassification::MOTORCYCLE;
      case 7: return autoware_perception_msgs::msg::ObjectClassification::BICYCLE;
      case 8: return autoware_perception_msgs::msg::ObjectClassification::PEDESTRIAN;
      case 9: return autoware_perception_msgs::msg::ObjectClassification::UNKNOWN;   // traffic_cone
      default: return autoware_perception_msgs::msg::ObjectClassification::UNKNOWN;
    }
  }

  void publishObjects(const std_msgs::msg::Header & header, const std::vector<Box> & boxes)
  {
    autoware_perception_msgs::msg::DetectedObjects msg;
    msg.header = header;
    msg.header.frame_id = "base_link";

    for (const auto & b : boxes) {
      if (b.score < score_threshold_) continue;
      if (!std::isfinite(b.x) || !std::isfinite(b.y) || !std::isfinite(b.z) ||
          !std::isfinite(b.r)) {
        RCLCPP_WARN_THROTTLE(get_logger(), *get_clock(), 3000, "NaN box dropped");
        continue;
      }
      autoware_perception_msgs::msg::DetectedObject obj;
      obj.existence_probability = b.score;

      autoware_perception_msgs::msg::ObjectClassification cls;
      cls.label = mapLabel(b.label);
      cls.probability = b.score;
      obj.classification.push_back(cls);

      obj.kinematics.pose_with_covariance.pose.position.x = b.x;
      obj.kinematics.pose_with_covariance.pose.position.y = b.y;
      obj.kinematics.pose_with_covariance.pose.position.z = b.z;
      tf2::Quaternion q;
      q.setRPY(0.0, 0.0, b.r);
      obj.kinematics.pose_with_covariance.pose.orientation.x = q.x();
      obj.kinematics.pose_with_covariance.pose.orientation.y = q.y();
      obj.kinematics.pose_with_covariance.pose.orientation.z = q.z();
      obj.kinematics.pose_with_covariance.pose.orientation.w = q.w();
      obj.kinematics.has_position_covariance = false;
      obj.kinematics.orientation_availability =
        autoware_perception_msgs::msg::DetectedObjectKinematics::AVAILABLE;

      obj.kinematics.twist_with_covariance.twist.linear.x = b.vx;
      obj.kinematics.twist_with_covariance.twist.linear.y = b.vy;
      obj.kinematics.has_twist = true;
      obj.kinematics.has_twist_covariance = false;

      obj.shape.type = autoware_perception_msgs::msg::Shape::BOUNDING_BOX;
      obj.shape.dimensions.x = b.l;
      obj.shape.dimensions.y = b.w;
      obj.shape.dimensions.z = b.h;

      msg.objects.push_back(obj);
    }
    last_object_count_ = msg.objects.size();
    objects_pub_->publish(msg);
  }

  void publishDebugStats()
  {
    const auto now = std::chrono::steady_clock::now();
    double dt_s = 0.0;
    if (last_stats_ts_) {
      dt_s = std::chrono::duration<double>(now - *last_stats_ts_).count();
    }
    last_stats_ts_ = now;
    std_msgs::msg::Float32 fps;
    fps.data = dt_s > 0.0 ? static_cast<float>(frames_in_window_ / dt_s) : 0.0f;
    frames_in_window_ = 0;
    fps_pub_->publish(fps);

    std_msgs::msg::Float32 lat;
    lat.data = static_cast<float>(last_e2e_ms_);
    latency_pub_->publish(lat);
    (void)last_infer_ms_;
    (void)last_total_ms_;
  }

  // ROS
  rclcpp::Subscription<FrameSet>::SharedPtr frameset_sub_;
  rclcpp::Publisher<autoware_perception_msgs::msg::DetectedObjects>::SharedPtr objects_pub_;
  rclcpp::Publisher<std_msgs::msg::Float32>::SharedPtr fps_pub_;
  rclcpp::Publisher<std_msgs::msg::Float32>::SharedPtr latency_pub_;
  rclcpp::TimerBase::SharedPtr stats_timer_;
  tf2_ros::Buffer tf_buffer_;
  tf2_ros::TransformListener tf_listener_;

  // params
  std::string engine_path_, onnx_path_, model_config_, precision_, expected_trt_;
  double score_threshold_, scene_gap_s_;
  std::string frameset_topic_;
  std::vector<std::string> camera_ids_;
  bool auto_build_engine_;

  // engine
  Logger logger_;
  std::string startup_error_;
  std::shared_ptr<EngineLifetime> engine_lifetime_;
  EngineInfo engine_info_;

  // BEVDet runtime
  std::unique_ptr<BEVDet> bevdet_;
  std::unique_ptr<bev::preprocessor::GpuImageStager> stager_;
  std::vector<Eigen::Matrix3f> cams_intrin_anchor_;
  std::vector<Eigen::Quaternion<float>> cams2ego_rot_anchor_;
  std::vector<Eigen::Translation3f> cams2ego_trans_anchor_;

  // frame accounting
  std::uint64_t frames_processed_{0};
  std::size_t frames_in_window_{0};
  std::optional<std::chrono::steady_clock::time_point> last_stats_ts_;
  rclcpp::Time last_frame_stamp_{0, 0, RCL_ROS_TIME};
  bool last_frame_stamp_valid_{false};
  int scene_index_{0};
  double last_infer_ms_{0}, last_total_ms_{0}, last_e2e_ms_{0};
  std::size_t last_object_count_{0};
};

}  // namespace bev::perception

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<bev::perception::BevdetNode>());
  rclcpp::shutdown();
  return 0;
}