// include/bev_segmentation/config.hpp
//
// M4.3 Segmentation 节点配置.
// drivable_class_ids 默认 [0] (road only); sidewalk (id=1) 可选加入.

#pragma once

#include <string>
#include <vector>

namespace bev_segmentation {

struct SegmentationConfig {
    // 模型文件
    std::string engine_path;
    std::string labels_path;

    // ROS topics (共享 M4.1/M4.2/M4.3 camera contract)
    std::string input_image_topic  = "/perception/cameras/front/image";
    std::string semantic_mask_topic = "/perception/semantic_mask";
    std::string drivable_mask_topic = "/perception/drivable_mask";

    // 默认 drivable_class_ids = [0] (road only)
    // sidewalk (id=1) 可选加入, 需用户显式配置
    // 该 mask 是 candidate-drivable semantic mask, 不等价于 collision-free space
    std::vector<int> drivable_class_ids = {0};

    // 推理 precision 标签 (仅用于日志/benchmark 报告)
    std::string precision = "fp16";

    // 注: 这里原本声明了 publish_debug / debug_topic, 但从未被任何代码路径读取.
    // 已移除而不是补实现 —— 给 mask 上色属于可视化, 按模块分层应放在
    // m4_demo_bringup (segmentation_visualizer.py), 不在算法包内.
};

}  // namespace bev_segmentation
