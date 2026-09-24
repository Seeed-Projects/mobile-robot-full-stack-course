# 4.5 原生 NVlabs FoundationPose：从 RGB-D 到 6D 位姿

## 本章目标

本章直接使用 NVlabs/PyTorch FoundationPose，把一个已知物体的 RGB-D 图像、相机内参、实例掩膜和 CAD 网格转换成物体在相机坐标系中的 6D 位姿。课程代码保留 `bev_pose` ROS 2 封装，同时提供一个更容易复现的 standalone MVP，用来先理解算法，再接入 ROS 话题。

完成本章后，你应能：

- 用平移、旋转、齐次矩阵和四元数解释 6D 位姿。
- 说明 RGB、depth、CameraInfo、实例掩膜和 CAD mesh 的分工。
- 区分 FoundationPose 的 `register` 首帧初始化与 `track_one` 后续跟踪。
- 在 Jetson 上准备 NVlabs 仓库、权重、CUDA 扩展和 Mustard 示例数据。
- 运行最小 MVP，读取 `report.json` 和首末帧位姿图。
- 看懂 `bev_pose` 如何把模型结果发布为 `PoseStamped`、`Detection3DArray` 和 TF。

## 1. 6D 位姿基础

刚体的 6D 位姿由三维平移和三维旋转组成：

```text
t = (x, y, z)
R = 3×3 rotation matrix
T_cam_obj = [[R, t], [0, 0, 0, 1]]
```

物体坐标系中的点 `p_obj` 经过位姿变换后位于相机坐标系：

```text
p_cam = R · p_obj + t
```

ROS 2 通常用 `geometry_msgs/Pose` 表示位姿，其中旋转使用 `x,y,z,w` 顺序的四元数。四元数应保持有限且归一化；`q` 与 `-q` 表示同一个旋转。位姿消息的 `header.frame_id` 同样重要：相机坐标系中的位置必须通过 TF 和相机外参转换后，才能用于机器人基座或机械臂规划。

### 检测框、语义掩膜和实例掩膜

| 输入 | 表示什么 | FoundationPose 中的用途 |
| --- | --- | --- |
| 检测框 | 目标在图像中的矩形范围 | 提供搜索区域或初始化提示 |
| 语义掩膜 | 每个像素属于哪一类 | 说明“这是路面/墙/桌子”等类别 |
| 实例掩膜 | 某一个具体物体的像素 | 从深度中提取目标点，作为 `register` 的 `ob_mask` |

4.3 的道路语义掩膜不能直接作为一个路由器或瓶子的实例掩膜。目标实例掩膜应尽量只覆盖一个物体，并且与 RGB、depth 使用相同的分辨率和坐标对齐关系。

### RGB、深度和 CameraInfo 的分工

- RGB 提供纹理和外观，用于网络比较观测图像与渲染模型。
- depth 提供每个像素的距离，使二维区域能够恢复为三维点。
- `CameraInfo.K` 提供焦距和主点，决定像素如何反投影到相机坐标系。
- CAD mesh 提供物体的几何尺寸、表面法线和可渲染模型。

没有深度，系统很难从单个二维区域直接确定尺度和距离；没有内参，深度点无法正确变换到相机坐标系；没有实例掩膜，背景和邻近物体会干扰注册。

## 2. FoundationPose 的算法流程

FoundationPose 将“物体模型”和“当前观测”放进同一个几何与学习混合的流程中。论文同时讨论了 model-based 和 model-free 两种输入方式：model-based 使用已知 CAD/mesh，model-free 可以从参考图像建立物体表示。本课程的 MVP 使用 model-based 路线，目标 mesh 是已准备好的 `textured_simple.obj`。

![FoundationPose 论文 Figure 2：姿态假设、细化、评分与跟踪](../4.4_Isaac_ROS_FoundationPose_and_Acceleration/images/foundationpose_paper_pipeline.png)

*图：FoundationPose 的统一估计与跟踪流程。来源：[FoundationPose 论文，arXiv 2312.08344](https://arxiv.org/abs/2312.08344)，Figure 2。首帧从全局姿态候选开始，后续帧从上一帧姿态开始。*

### 首帧 `register`

`register` 接收：

```text
RGB + aligned depth + camera K + object instance mask + mesh
```

核心步骤可以概括为：

1. 根据实例掩膜和深度估计目标的大致三维中心。
2. 在 icosphere 采样的多个视角和面内旋转上生成姿态假设。
3. refine 网络比较渲染模型与真实 RGB-D 观测，迭代更新平移和旋转。
4. score 网络为候选姿态打分并排序，选出当前帧结果。

因此，检测框只是二维提示，最终输出是一个包含尺度、距离和朝向的 `T_cam_obj`。

### 后续 `track_one`

跟踪阶段不再从完整的全局姿态网格开始，而是使用上一帧的姿态作为热启动，重点做局部更新。首帧注册通常需要更多候选和更多 refine 迭代，后续 tracking 的计算量更小。课程脚本分别记录 `register_ms` 和 `track_ms`，避免把两个阶段混成一个帧率。

官方接口可在 [NVlabs `estimater.py`](https://github.com/NVlabs/FoundationPose/blob/main/estimater.py) 和 [官方 `run_demo.py`](https://github.com/NVlabs/FoundationPose/blob/main/run_demo.py) 中核对：

```python
pose = est.register(K=K, rgb=rgb, depth=depth, ob_mask=mask, iteration=5)
pose = est.track_one(rgb=rgb, depth=depth, K=K, iteration=2)
```

### 常见误差来源

- 物体有对称结构时，多个旋转可能产生相近的外观。
- 目标被遮挡或掩膜包含背景时，候选排序更容易出错。
- RGB/depth 没有对齐、深度单位错误或 CameraInfo 不匹配时，会出现系统性的三维偏移。
- CAD mesh 的单位或姿态与真实物体不一致时，位姿方向可能看似合理，但距离和尺寸会整体错误。

## 3. Jetson MVP 环境

当前课程目标平台是 Seeed reComputer Robotics J501，Jetson AGX Orin 32 GB，JetPack 6.2.1 / L4T R36.4.4，CUDA 12.6，Python 3.10。NVlabs 仓库固定在课程验证过的 commit：

```text
a1b694b83e633c2cb6115b9063d940a687759392
```

MVP 使用以下目录：

```text
/home/seeed/workspace/third_party/FoundationPose/
├── demo_data/mustard0/
├── weights/2023-10-28-18-33-37/model_best.pth
├── weights/2024-01-11-20-02-45/model_best.pth
└── mycpp/build/mycpp*.so
```

其中 refiner 和 scorer 分别从各自的 `config.yml` 和 `model_best.pth` 加载。`mycpp` 用于姿态候选聚类；`nvdiffrast` 用于 GPU 光栅化；PyTorch 负责网络推理和张量计算。

## 4. 运行最小 MVP

课程代码位于 `code/scripts/m4/`。在 Jetson 的 M4 模块根目录执行：

```bash
cd /home/seeed/workspace/ros2_bev/modules/m04-ai-vision-and-edge-acceleration

# 语法兼容入口，实际转发到 MVP runner
bash scripts/m4/phase0_foundationpose_verify.sh --frames 8

# 推荐入口
bash scripts/m4/run_m4_5_native_mvp.sh --frames 8
```

脚本默认使用 NVlabs 官方 Mustard 录制序列，执行一次 `register`，再执行若干次 `track_one`。输出目录为：

```text
output/m4/m45_native_mvp/
├── report.json
├── pose_0000.txt
├── pose_0001.txt
├── ...
├── frame_0000_pose.png
├── frame_0007_pose.png
└── debug/
```

`report.json` 的关键字段如下：

| 字段 | 含义 |
| --- | --- |
| `register_ms` | 首帧全局姿态初始化耗时 |
| `track_ms` | 每个后续帧的跟踪耗时列表 |
| `track_fps` | 仅由 tracking 阶段平均耗时换算的参考值 |
| `first_pose` / `last_pose` | 首帧和末帧的 4×4 相机到物体变换 |

MVP 的价值在于把算法生命周期和输出证据固定下来：先看注册如何从候选中选择姿态，再看 tracking 如何沿用上一帧结果。

### Jetson Mustard 运行结果

在 J501 / AGX Orin 32 GB、JetPack 6.2.1、CUDA 12.6 上，使用官方 `mustard0` RGB-D 序列运行 8 帧，结果如下：

| 项目 | 实测结果 |
| --- | --- |
| 首帧 `register` | 11050.90 ms |
| 后续 `track_one` | 89.37–237.11 ms/帧，平均约 120.43 ms/帧 |
| tracking 参考换算 | 8.30 FPS，仅表示这组 tracking 阶段的平均耗时 |
| 输出 | 8 个有限值 4×4 位姿矩阵、`report.json`、首帧和末帧标注图 |

![Mustard 首帧 register 结果](images/m45_native_mustard_register.png)

*图：官方 Mustard 序列首帧的注册结果。来源：NVlabs FoundationPose `demo_data/mustard0`，本课程脚本在 Jetson 上生成。*

![Mustard 后续 tracking 结果](images/m45_native_mustard_tracking.png)

*图：同一序列第 8 帧的 tracking 结果。首帧注册和后续 tracking 的耗时应分别阅读，不能把 8.30 FPS 解释为完整实时 RGB-D 相机帧率。*

脚本默认导出 `PYTHONNOUSERSITE=1`，避免用户目录中的 NumPy 覆盖 conda 环境。JetPack 6.2.1 的 CUDA 12.6 与当前 PyTorch wheel 在 3×3 逆矩阵符号上存在版本差异，runner 对 FoundationPose 使用到的相机/裁剪 3×3 矩阵采用局部解析逆矩阵兼容路径；上游 FoundationPose 仓库本身没有修改。

## 5. CAD mesh 与尺度

课程中的真实目标模型来自 GL.iNet GL-SFT1200 Opal CAD。原始 STEP 使用毫米单位，预处理脚本将 mesh 缩放到米，并在 `object.yaml` 中记录来源和尺寸。运行时不需要 CAD 内核，只加载已经三角化的 OBJ 或 `.npz`。

```bash
python3 scripts/m4/step_to_foundationpose_mesh.py models/m4/pose
bash scripts/m4/preprocess_mesh.sh
```

检查 mesh 时至少确认：

1. 顶点和法线形状为 `(N,3)`。
2. 面索引没有越界。
3. 包围盒尺寸与实物数量级一致。
4. mesh 的天线、接口和外壳方向与真实物体坐标系一致。

## 6. `bev_pose` ROS 2 封装

standalone MVP 解决“原生 FoundationPose 能否读取 RGB-D 并输出位姿”。ROS 2 封装进一步解决“如何把它放进移动机器人感知链”。主要节点关系是：

```text
RGB ───────────────┐
aligned depth ─────┼─> foundationpose_node ─> PoseStamped
CameraInfo ────────┤                         ├─> Detection3DArray
object instance mask┘                         └─> camera_front → object TF
```

### 话题契约

| 方向 | 话题 | 类型 |
| --- | --- | --- |
| 输入 | `/perception/cameras/front/image` | `sensor_msgs/Image` |
| 输入 | `/perception/cameras/front/depth` | `sensor_msgs/Image`，米制 depth |
| 输入 | `/perception/cameras/front/camera_info` | `sensor_msgs/CameraInfo` |
| 输入 | `/perception/object_mask` | `sensor_msgs/Image`，单目标二值掩膜 |
| 输出 | `/perception/object_pose` | `geometry_msgs/PoseStamped` |
| 输出 | `/perception/object_poses_3d` | `vision_msgs/Detection3DArray` |
| 输出 | `/tf` | `camera_front → object` |

`foundationpose_engine.py` 将生命周期拆成 `load_object_model`、`register` 和 `track`；`foundationpose_node.py` 负责消息同步、图像转换、内参提取和位姿发布。这样可以分别测试算法、输入校验和 ROS 通信，而不是把所有逻辑塞进一个回调。

## 7. 应用案例

| 场景 | 需要的输入 | 位姿的作用 |
| --- | --- | --- |
| 机械臂抓取 | RGB-D、目标实例掩膜、CAD mesh、手眼标定 | 将 `T_cam_obj` 转成 `T_base_obj`，生成抓取姿态 |
| 移动机器人 | 对齐深度、CameraInfo、目标 mesh | 判断目标距离、朝向和与底盘的空间关系 |
| AR/MR 叠加 | RGB、相机内参、目标 mesh | 在真实物体表面叠加坐标轴或虚拟模型 |
| 盘点与检视 | 目标实例掩膜、参考 mesh、连续 tracking | 判断物体是否出现、姿态是否变化、视角是否覆盖 |

四类应用都依赖同一条链路：正确的 RGB-D、可靠的实例掩膜、尺度正确的模型和坐标标定。算法输出的相机系位姿不能跳过 TF 直接用于机器人基座。

## 8. 思考题与延伸阅读

1. 为什么首帧需要 icosphere 姿态假设，而后续 tracking 可以使用上一帧结果？
2. 如果 depth 单位从米误读成毫米，平移向量会出现什么现象？
3. 为什么语义分割图不能直接代替一个物体的实例掩膜？
4. `T_base_obj = T_base_cam · T_cam_obj` 中，哪一部分来自标定，哪一部分来自 FoundationPose？

参考资料：

- [FoundationPose 论文，arXiv 2312.08344](https://arxiv.org/abs/2312.08344)
- [NVlabs FoundationPose](https://github.com/NVlabs/FoundationPose)
- [Isaac ROS FoundationPose](https://github.com/NVIDIA-ISAAC-ROS/isaac_ros_pose_estimation)
- [FoundationPose 官方示例 `run_demo.py`](https://github.com/NVlabs/FoundationPose/blob/main/run_demo.py)
