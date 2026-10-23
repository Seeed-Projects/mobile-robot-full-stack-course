# 5.4 空间语义地图构建：从几何空间到环境理解

> **实机环境**：reComputer J501（AGX Orin 32GB），L4T 36.4.4 / JetPack 6.2.1
> **软件栈**：ROS 2 Humble、OpenCV 4.10（CUDA）、PCL / Open3D、TensorRT 10.x、RViz2

通过之前的内容可以得到一张几何地图：哪里被占据、哪里可通行、哪里有障碍。但机器人还看不懂世界——它不知道"那团障碍是一把椅子还是一扇门"，更不知道"会议室在哪"。这一课把 M4 的目标检测与语义分割接进三维地图，产出带物体类别和空间位置的语义地图：机器人不只知道这里有东西，还知道那是什么、能不能走、值不值得记住。

![语义地图系统数据流：M4 三路输出（detections / tracks / 语义掩膜）与深度图汇入 semantic_map 节点，分层输出静态、可通行、动态三层](./images/m5-4-semantic-map-dataflow.jpg)

## 一、学完后，你能做到什么

- 说清几何地图与语义地图的区别，以及为什么语义图层是叠加在几何层之上、而不是替换它；
- 把 2D 语义标签（检测框、分割掩膜）结合深度数据，投影到 3D 地图坐标；
- 区分语义地图的三种粒度：语义点云、对象级地图、空间语义地标，以及各自服务的下游任务；
- 把静态物体、动态物体、可通行区域分三层表达，不让动态目标污染长期地图；
- 交付一份融合 AI 感知与三维地图的语义地图，并在 RViz2 中可叠加显示。

## 二、硬件与软件清单

**硬件：** J501（MAXN 模式）、RGB-D 深度相机（Orbbec Gemini 2，M2.2/5.2/5.3 已接入并完成 D2C 对齐）、激光雷达（M3.1）、移动底盘。

**软件：**

- M4 产物（三路输出）：`/perception/detections`（YOLO，`vision_msgs/Detection2DArray`）、`/perception/tracks`（ByteTrack，带 `id`）、`/perception/semantic_mask`（19 类 uint8）+ `/perception/drivable_mask`（0/255 candidate-drivable）；
- 三维地图：5.3 的 Nvblox 三维几何地图（Mesh / ESDF / 占据栅格）；
- 投影工具：Open3D / PCL、`tf2`、`cv_bridge`；
- 工作目录沿用 `/home/seeed/workspace/ros2_bev`，语义节点放 `ros2_ws/src/semantic_map/`。

## 三、前置基础

- M4.1：目标检测链路已通，`/perception/detections` 持续发布；
- M4.2：多目标跟踪已通，`/perception/tracks` 持续发布，每个框带稳定 `id`；
- M4.3：语义分割两路输出在跑——`/perception/semantic_mask`（Cityscapes 19 类 uint8，0–18）与 `/perception/drivable_mask`（由 `drivable_class_ids` 派生的 candidate-drivable 掩膜）；
- M2.2：深度相机与彩色图完成 D2C 对齐（registration），深度图与彩色图同分辨率、`camera_info` 一致，16UC1 深度按毫米读入后需除以 1000；
- M3.3：相机→雷达→底盘外参已知；
- 5.3：Nvblox 三维几何地图在跑，TF 树（map→odom→base_link→camera_link）连续。

---

## 四、先读懂：几何地图与语义地图差在哪

### 4.1 几何地图回答「能不能走」，语义地图回答「那是什么」

| 维度 | 几何地图（5.3） | 语义地图（本节） |
|-|-|-|
| 数据 | 栅格占用概率、点云坐标 | 坐标 + 类别标签 + 置信度 + （对象）ID |
| 典型问题 | 「这里能不能过？」 | 「那是门还是墙？桌子在哪？」 |
| 下游 | 路径规划、代价地图 | 物体检索、空间位置查询 |
| 更新频率 | 长期静态层，缓慢变化 | 静态层慢变 + 动态层实时 |

关键认知：**语义地图不是另一张地图，而是几何地图上叠加的语义图层**。几何层管通行性，语义层管「是什么」，两层共用同一坐标系，通过 TF 对齐。

### 4.2 2D 语义标签怎么落到 3D 空间

AI 感知给你的是图像坐标系下的东西：YOLO 给像素框、分割给像素掩膜。要进地图，必须经过三步投影：

1. **取深度**：对掩膜/框内每个像素 (u, v)，从深度图读出该点距离 d；
2. **反投影到相机坐标系**：用相机内参把像素还原为三维点：$X_c = (u - c_x) \cdot d / f_x$，Y、Z 同理；
3. **TF 变换到 map**：沿 `camera_link → base_link → odom → map` 把点变换到地图坐标系，落到对应栅格/点云。

![2D 像素到 3D 地图栅格的投影链路：取深度 d → 相机内参反投影到相机坐标系 → 经 TF 落到 map 栅格](./images/m5-4-2d-to-3d-projection.jpg)

完整反投影需要四个内参（来自 `camera_info`，M2.2 标定已给出）：`fx, fy, cx, cy`。对深度图上每个有效像素 `(u, v)`、深度值 `d`（单位米）：

$$\begin{bmatrix} x_c \\ y_c \\ z_c \end{bmatrix} = \begin{bmatrix} (u - c_x)\,d / f_x \\ (v - c_y)\,d / f_y \\ d \end{bmatrix}$$

再用 TF 把相机坐标点变换到地图：$p_{map} = T_{map}^{camera} \cdot [x_c,\, y_c,\, z_c,\, 1]^\top$，其中 `T` 由 `tf2` 在线查询。工程上注意三点：

- 深度图 `d` 是 16UC1 毫米读数，必须先 `d / 1000.0` 转米，否则点会缩到相机附近；
- `d <= 0` 或超出相机量程的像素直接跳过（Gemini 2 官方量程 0.15–10 m、理想范围 0.2–5 m，室内场景常用 0.4–10 m 保守窗口）；
- 掩膜与深度图分辨率、光心必须一致，不一致就用 `cv_bridge` + `image_proc` 先做对齐。

这条链上最容易错的不是数学，而是**对齐**：掩膜分辨率和深度图是否同尺寸、时间戳是否同一帧、相机外参是否新鲜。任何一步错位，标签就会整体偏移——见排障。

### 4.3 三种语义粒度，对应三种下游

| 粒度 | 是什么 | 服务谁 |
|-|-|-|
| **语义点云** | 每个点附类别标签（地面绿、障碍红、人黄） | 可视化、可通行区域分析 |
| **对象级地图** | 每个物体是一条记录：ID、类别、3D 位置/包围盒、置信度 | 物体检索、抓取定位 |
| **空间语义地标** | 持久物体（门、桌、电器）注册为带名称的路标 | 按名称或类别查询位置，支撑高层任务指令 |

![三种语义粒度对比：左为语义点云（按类别着色）、中为对象级地图（3D 包围盒）、右为空间语义地标（拓扑节点）](./images/m5-4-semantic-granularity.png)

### 4.4 静态、动态、可通行区域，分三层表达

- **静态物体**（墙、门、桌椅、电器）：多帧稳定出现，写入长期语义地图；
- **动态物体**（行人、移动推车）：实时图层，**绝不写进静态地图**——否则地图会被人「污染」，下一次建图全是鬼影；
- **可通行区域**：直接消费 M4.3 的 `/perception/drivable_mask`（candidate-drivable，默认仅 road 类），投影到地图 navigable 层。注意它是「候选可行驶」，**不等于 collision-free**——必须再与 5.3 的几何障碍层叠加，才是真正可走的区域。

![三层语义地图叠加：底层几何 Occupancy Grid、中层静态语义物体（桌/门/障碍）、上层动态行人（黄色虚线轮廓）](./images/m5-4-three-layer-map.png)

**一句话记忆：**几何层管「能不能走」，语义层管「是什么」；动态目标永远活在实时图层里，静态地图只装持久物体。

---

## 五、动手：从感知到语义地图
Comming...


## 参考资料

1. [ROS 2 TF2 坐标变换文档](https://index.ros.org/p/tf2/)
2. [ROS 2 cv_bridge：图像与 OpenCV 转换](https://index.ros.org/p/cv_bridge/)
3. [ROS 2 message_filters：时间同步（ApproximateTimeSynchronizer）](https://index.ros.org/p/message_filters/)
4. [vision_msgs：Detection2DArray 等消息定义](https://index.ros.org/p/vision_msgs/)
5. [Orbbec ROS 2 Driver（Gemini 2 驱动与话题）](https://github.com/orbbec/OrbbecSDK_ROS2)
6. [Cityscapes 数据集：语义分割 19 类标签定义](https://www.cityscapes-dataset.com/)
7. [PCL 点云库](https://pointclouds.org/) / [Open3D](https://www.open3d.org/)
8. [NVIDIA Isaac ROS Nvblox：5.3 三维几何地图上游](https://nvidia-isaac-ros.github.io/v/release-3.2/repositories_and_packages/isaac_ros_nvblox/index.html)
