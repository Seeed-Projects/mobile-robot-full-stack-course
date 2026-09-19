// src/segmentation_node.cpp
//
// ROS2 node: 订阅相机图像 -> TensorRT 推理 -> 发布 semantic_mask + drivable_mask
//
// 默认 drivable_class_ids = [0] (road only); sidewalk (id=1) 可选加入.
// drivable_mask 是 candidate-drivable semantic mask, 不等价于 collision-free space.

#include "bev_segmentation/config.hpp"
#include "bev_segmentation/postprocess.hpp"
#include "bev_segmentation/preprocess.hpp"
#include "bev_segmentation/segmentation_engine.hpp"
#include "bev_segmentation/types.hpp"

#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/image.hpp>

#include <cctype>
#include <fstream>
#include <memory>
#include <sstream>
#include <string>
#include <vector>

namespace bev_segmentation {

namespace {

// 极简 JSON 解析: 仅支持 labels.json 的 {"0":"road","1":"sidewalk",...}
// 体积小, 不引入 nlohmann_json / rapidjson 等额外依赖.
// 这是 P0 阶段的简化方案; P1 可替换为成熟库.
std::string slurp(const std::string& path) {
    std::ifstream in(path);
    if (!in) return {};
    std::ostringstream ss; ss << in.rdbuf();
    return ss.str();
}

bool parse_labels_json(const std::string& text, LabelMap& out) {
    out.clear();
    // 期望形如 {"0":"road","1":"sidewalk",...}
    // 状态机: 跳过空白, 在 "key":"value" 之间切换
    size_t i = 0;
    const size_t n = text.size();
    while (i < n) {
        // 找下一个 "
        size_t k_start = text.find('"', i);
        if (k_start == std::string::npos) break;
        size_t k_end = text.find('"', k_start + 1);
        if (k_end == std::string::npos) break;
        // ":"
        size_t colon = text.find(':', k_end);
        if (colon == std::string::npos) break;
        size_t v_start = text.find('"', colon);
        if (v_start == std::string::npos) break;
        size_t v_end = text.find('"', v_start + 1);
        if (v_end == std::string::npos) break;

        std::string key = text.substr(k_start + 1, k_end - k_start - 1);
        std::string val = text.substr(v_start + 1, v_end - v_start - 1);
        try {
            int id = std::stoi(key);
            out[id] = val;
        } catch (...) {
            // skip malformed
        }
        i = v_end + 1;
    }
    return !out.empty();
}

}  // namespace

class SegmentationNode : public rclcpp::Node {
public:
    explicit SegmentationNode(const rclcpp::NodeOptions& opts = rclcpp::NodeOptions())
        : Node("segmentation_node", opts) {
        // ---- 参数 ----
        cfg_.engine_path = declare_parameter<std::string>(
            "engine_path", cfg_.engine_path);
        cfg_.labels_path = declare_parameter<std::string>(
            "labels_path", cfg_.labels_path);
        cfg_.input_image_topic = declare_parameter<std::string>(
            "input_image_topic", cfg_.input_image_topic);
        cfg_.semantic_mask_topic = declare_parameter<std::string>(
            "semantic_mask_topic", cfg_.semantic_mask_topic);
        cfg_.drivable_mask_topic = declare_parameter<std::string>(
            "drivable_mask_topic", cfg_.drivable_mask_topic);
        // ROS param vector<int64_t> -> vector<int>
        const auto ids64 = declare_parameter<std::vector<int64_t>>(
            "drivable_class_ids", std::vector<int64_t>{0});
        cfg_.drivable_class_ids.clear();
        for (auto v : ids64) cfg_.drivable_class_ids.push_back(static_cast<int>(v));
        // (publish_debug / debug_topic were declared here but never read by any
        // code path. Removed rather than implemented: colourising a mask is
        // visualisation, and M4 visualisation belongs to m4_demo_bringup
        // (segmentation_visualizer.py), not to the algorithm package.)

        // ---- 加载 labels.json ----
        const std::string text = slurp(cfg_.labels_path);
        if (text.empty()) {
            RCLCPP_WARN(this->get_logger(),
                        "labels.json not readable: %s (continuing with empty map)", cfg_.labels_path.c_str());
        } else if (!parse_labels_json(text, labels_)) {
            RCLCPP_WARN(this->get_logger(),
                        "labels.json parse error: %s (continuing with empty map)", cfg_.labels_path.c_str());
        }

        // ---- 加载 TensorRT engine ----
        engine_ = std::make_unique<SegmentationEngine>();
        engine_->load(cfg_.engine_path);

        // ---- ROS 接口 ----
        using std::placeholders::_1;
        image_sub_ = this->create_subscription<sensor_msgs::msg::Image>(
            cfg_.input_image_topic, rclcpp::SensorDataQoS(),
            std::bind(&SegmentationNode::on_image, this, _1));

        // Publisher QoS follows the house convention used by 4.1/4.2 (and by
        // this node's own subscription): SensorDataQoS, i.e. BEST_EFFORT. A
        // bare depth of 10 would mean RELIABLE, which is the odd one out and
        // makes a slow consumer block the publisher.
        semantic_pub_ = this->create_publisher<sensor_msgs::msg::Image>(
            cfg_.semantic_mask_topic, rclcpp::SensorDataQoS());
        drivable_pub_ = this->create_publisher<sensor_msgs::msg::Image>(
            cfg_.drivable_mask_topic, rclcpp::SensorDataQoS());

        RCLCPP_INFO(this->get_logger(),
                    "segmentation_node ready | engine=%s | topic=%s | drivable_class_ids=[%s] | labels=%zu",
                    cfg_.engine_path.c_str(),
                    cfg_.input_image_topic.c_str(),
                    join_ids(cfg_.drivable_class_ids).c_str(),
                    labels_.size());
    }

private:
    static std::string join_ids(const std::vector<int>& v) {
        std::string s;
        for (size_t i = 0; i < v.size(); ++i) {
            if (i) s += ",";
            s += std::to_string(v[i]);
        }
        return s;
    }

    void on_image(const sensor_msgs::msg::Image::SharedPtr msg) {
        if (!engine_->is_loaded()) return;

        const int orig_h = static_cast<int>(msg->height);
        const int orig_w = static_cast<int>(msg->width);
        if (orig_h <= 0 || orig_w <= 0) return;

        // ---- 预处理: 原图 -> [1,3,512,1024] float32 ----
        LetterboxMeta meta;
        std::vector<float> input_tensor;
        try {
            input_tensor = letterbox_to_chw_float(
                msg->data.data(), orig_h, orig_w,
                static_cast<int>(msg->step),
                msg->encoding,
                kModelInputH, kModelInputW,
                meta);
        } catch (const std::exception& e) {
            RCLCPP_WARN(this->get_logger(),
                        "preprocess failed: %s", e.what());
            return;
        }

        // Log the geometry once: the physical acceptance check for M4.3 is
        // that mask boundaries land on real object edges, which is impossible
        // to interpret without knowing the letterbox actually applied.
        if (!logged_geometry_) {
            logged_geometry_ = true;
            RCLCPP_INFO(this->get_logger(),
                        "geometry: %dx%d -> canvas %dx%d | scale=%.6f pad=(%d,%d) | "
                        "masks restored via unletterbox_mask",
                        orig_w, orig_h, kModelInputW, kModelInputH,
                        meta.scale, meta.pad_x, meta.pad_y);
        }

        // ---- 推理 ----
        std::vector<float> logits;
        try {
            engine_->infer(input_tensor, logits);
        } catch (const std::exception& e) {
            RCLCPP_ERROR(this->get_logger(),
                         "infer failed: %s", e.what());
            return;
        }

        // ---- 后处理 ----
        // argmax, then undo the letterbox. Resizing the padded mask straight to
        // the original size (what this used to do) smears the pad prediction
        // into the image: 1920x1080 letterboxes to 910x512 with 57 px of pad
        // each side, so 14 mask columns of padding were landing on real pixels.
        std::vector<uint8_t> class_mask_logit = argmax_classes(
            logits, kLogitsH, kLogitsW, kNumClasses);
        std::vector<uint8_t> semantic_mask = unletterbox_mask(
            class_mask_logit, meta, orig_h, orig_w);
        std::vector<uint8_t> drivable_mask = make_drivable_mask(
            semantic_mask, cfg_.drivable_class_ids);

        // ---- 发布 ----
        publish_mask(semantic_pub_, msg, semantic_mask, orig_w, orig_h, "mono8");
        publish_mask(drivable_pub_, msg, drivable_mask, orig_w, orig_h, "mono8");
    }

    static void publish_mask(const rclcpp::Publisher<sensor_msgs::msg::Image>::SharedPtr& pub,
                             const sensor_msgs::msg::Image::SharedPtr& src,
                             const std::vector<uint8_t>& data,
                             int w, int h,
                             const std::string& encoding) {
        sensor_msgs::msg::Image out;
        out.header = src->header;
        out.height = h;
        out.width  = w;
        out.encoding = encoding;
        out.is_bigendian = false;
        out.step = static_cast<uint32_t>(w);
        out.data = data;
        pub->publish(out);
    }

    SegmentationConfig cfg_;
    std::unique_ptr<SegmentationEngine> engine_;
    LabelMap labels_;
    bool logged_geometry_ = false;   // one-shot letterbox geometry log

    rclcpp::Subscription<sensor_msgs::msg::Image>::SharedPtr image_sub_;
    rclcpp::Publisher<sensor_msgs::msg::Image>::SharedPtr semantic_pub_;
    rclcpp::Publisher<sensor_msgs::msg::Image>::SharedPtr drivable_pub_;
};

}  // namespace bev_segmentation

int main(int argc, char** argv) {
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<bev_segmentation::SegmentationNode>());
    rclcpp::shutdown();
    return 0;
}
