// bev_visualization: DetectedObjects -> BEV 3D markers (boxes + id/class/distance text).
#include <chrono>
#include <memory>
#include <string>
#include <sstream>
#include <iomanip>

#include "autoware_perception_msgs/msg/detected_objects.hpp"
#include "rclcpp/rclcpp.hpp"
#include "visualization_msgs/msg/marker.hpp"
#include "visualization_msgs/msg/marker_array.hpp"
#include "tf2/utils.h"

namespace bev::viz
{

class VisualizationNode : public rclcpp::Node
{
public:
  VisualizationNode() : Node("bev_visualization")
  {
    objects_sub_ = create_subscription<autoware_perception_msgs::msg::DetectedObjects>(
      "/bev/objects", rclcpp::SensorDataQoS(),
      [this](const autoware_perception_msgs::msg::DetectedObjects::SharedPtr m) {
        onObjects(m);
      });
    markers_pub_ = create_publisher<visualization_msgs::msg::MarkerArray>(
      "/bev/debug/markers", rclcpp::QoS(2));
    RCLCPP_INFO(get_logger(), "visualization running");
  }

private:
  static const char * labelName(std::uint8_t label)
  {
    switch (label) {
      case autoware_perception_msgs::msg::ObjectClassification::CAR: return "car";
      case autoware_perception_msgs::msg::ObjectClassification::TRUCK: return "truck";
      case autoware_perception_msgs::msg::ObjectClassification::BUS: return "bus";
      case autoware_perception_msgs::msg::ObjectClassification::TRAILER: return "trailer";
      case autoware_perception_msgs::msg::ObjectClassification::MOTORCYCLE: return "motorcycle";
      case autoware_perception_msgs::msg::ObjectClassification::BICYCLE: return "bicycle";
      case autoware_perception_msgs::msg::ObjectClassification::PEDESTRIAN: return "pedestrian";
      case autoware_perception_msgs::msg::ObjectClassification::ANIMAL: return "animal";
      default: return "object";
    }
  }

  void onObjects(const autoware_perception_msgs::msg::DetectedObjects::SharedPtr msg)
  {
    visualization_msgs::msg::MarkerArray arr;
    int id = 0;
    for (const auto & obj : msg->objects) {
      visualization_msgs::msg::Marker m;
      m.header = msg->header;
      m.ns = "bev_boxes";
      m.id = id++;
      m.type = visualization_msgs::msg::Marker::CUBE;
      m.action = visualization_msgs::msg::Marker::ADD;
      m.pose = obj.kinematics.pose_with_covariance.pose;
      m.scale.x = std::max(static_cast<double>(obj.shape.dimensions.x), 0.2);
      m.scale.y = std::max(static_cast<double>(obj.shape.dimensions.y), 0.2);
      m.scale.z = std::max(static_cast<double>(obj.shape.dimensions.z), 0.2);
      m.color.a = 0.35f;
      m.color.r = (obj.classification.empty() ||
        obj.classification[0].label == autoware_perception_msgs::msg::ObjectClassification::PEDESTRIAN) ? 1.0f : 0.3f;
      m.color.g = 0.6f;
      m.color.b = (obj.classification.empty() ||
        obj.classification[0].label == autoware_perception_msgs::msg::ObjectClassification::PEDESTRIAN) ? 0.3f : 1.0f;
      arr.markers.push_back(m);

      visualization_msgs::msg::Marker t;
      t.header = msg->header;
      t.ns = "bev_labels";
      t.id = id++;
      t.type = visualization_msgs::msg::Marker::TEXT_VIEW_FACING;
      t.action = visualization_msgs::msg::Marker::ADD;
      t.pose = obj.kinematics.pose_with_covariance.pose;
      t.pose.position.z += obj.shape.dimensions.z + 0.4;
      t.scale.z = 0.7;
      t.color.a = 1.0f;
      t.color.r = t.color.g = t.color.b = 1.0f;
      const double dist = std::hypot(
        obj.kinematics.pose_with_covariance.pose.position.x,
        obj.kinematics.pose_with_covariance.pose.position.y);
      std::ostringstream oss;
      oss << std::fixed << std::setprecision(2) << dist << "m";
      if (!obj.classification.empty()) {
        oss << " " << labelName(obj.classification[0].label);
      }
      t.text = oss.str();
      arr.markers.push_back(t);
    }
    markers_pub_->publish(arr);
  }

  rclcpp::Subscription<autoware_perception_msgs::msg::DetectedObjects>::SharedPtr objects_sub_;
  rclcpp::Publisher<visualization_msgs::msg::MarkerArray>::SharedPtr markers_pub_;
};

}  // namespace bev::viz

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<bev::viz::VisualizationNode>());
  rclcpp::shutdown();
  return 0;
}