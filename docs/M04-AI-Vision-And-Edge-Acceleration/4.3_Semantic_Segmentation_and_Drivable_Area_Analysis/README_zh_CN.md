# 4.3 语义分割与可通行区域分析

**状态：VERIFIED。** Jetson A 已完成 SegFormer-B0 FP16 TensorRT 端到端推理、几何恢复、限流与 unit / engine gate / full 测试。模型二进制只保留在 Jetson，不进入课程快照；完整证据与环境以 [`code/PROJECT_STATUS.md`](../code/PROJECT_STATUS.md) 为准。

## 课程概述

4.1 让机器人知道「画面里有什么」，4.2 让同一个目标在连续帧里保持同一个编号。但导航真正要回答的是第三个问题：**哪里能走**。语义分割做的就是这件事：给画面里每一个像素贴一个类别标签，再把「哪些类别算能走」这张表贴上去，得到一张可行驶掩膜。

实机的接口定义了**两路输出**：一路是 19 类的原始语义图 `/perception/semantic_mask`，一路是映射后的可行驶掩膜 `/perception/drivable_mask`。分成两路是有意的：上层如果想换一套「什么算能走」的定义，改的是映射，不用重新过一遍网络。

本章实机侧的包是 `bev_segmentation`。节点、预处理、后处理、engine 封装、launch 与测试已经形成可运行链路；课程仓库只同步源码、元数据与验收方法，不分发模型二进制。

### 先知道：这节课会带你完成什么

| 阶段 | 你会理解什么 | 最终能做什么 |
| --- | --- | --- |
| 读懂 | 语义分割与实例分割的区别，以及导航为什么只需要前者 | 判断一个任务该用哪种分割 |
| 看透 | 19 类语义怎么变成一张 0/255 的可行驶掩膜 | 读懂 `drivable_class_ids` 这类映射配置 |
| 接入 | 两路掩膜各自的分工与消费方式 | 按正确语义接住 `/perception/semantic_mask` 与 `/perception/drivable_mask` |
| 验收 | 如何证明几何恢复、engine 对齐和运行限流正确 | 用 unit、engine gate 与 full 三层测试复核当前实现 |

### 学完后，你能做到什么

- 说清语义分割、实例分割、全景分割的判据，并说明导航为什么选语义分割。

- 说清 mIoU、Pixel Accuracy、FW-IoU 各回答什么问题，以及小型类别会怎样影响 mIoU。

- 说清 SegFormer 与 DeepLabV3+ 在编码器、多尺度上下文、解码器上的差异，以及为什么部署侧更偏向 SegFormer-B0。

- 读懂 `config/segmentation.yaml` 的每一项，说清 `drivable_class_ids` 的取值意味着什么。

- 复述「candidate-drivable 不等于 collision-free」这条边界，并说明它为什么必须保留。

- 独立启动 `bev_segmentation`，核对两路输出，并用三层测试复核当前已验证实现。

### 硬件与软件清单

| ![硬件与软件清单](./images/R29abyTLHooQaSx8cBYcPRoVncj.png) | ![硬件与软件清单](./images/R45DbADhNoekD2xJmSBch3hLn5c.png) | ![硬件与软件清单](./images/Rti6b89EuoWbfIxA4P5cb3usnLb.png) |
| --- | --- | --- |
| reCImputer mini J501  + GMSL拓展板 |   | GMSL摄像头 / USB 摄像头 |

### 前置基础

- 4.1 / 4.2：检测与跟踪链路已跑通。本章与它们共用同一路相机图像输入。

- [2.1 GMSL2 ：车载级多相机接入](https://seeedstudio.feishu.cn/docx/Takhd7wo5oPx3mx0ljhcfyxDnEe)：确认图像通路与驱动。

- [2.2 深度相机与 3D 视觉感知](https://seeedstudio.feishu.cn/docx/Jj53dSSudoKyEdxk6jHcYnMHnOc)：使用 Orbbec Gemini 2 时需要。

- 会用 `ros2 topic` 看话题与消息。本章不要求会写节点，也不要求会训练模型。

## 先读懂：让每个像素带上「能不能走」的标签

### 三种分割任务：谁区分实例，谁只认类别

区分这三种任务的判据只有一条：同一类里的两个物体，输出能不能把它们分开。分不开的是语义分割，分得开的是实例分割，而全景分割把「分不开的 stuff」与「分得开的 thing」放进同一张输出。

| 任务 | 输出 | 同类实例是否分开 | 本页的位置 |
| --- | --- | --- | --- |
| 语义分割 | 每个像素一个类别 ID | 不分开，两个行人都是 person | **主线**：产出两路掩膜 |
| 实例分割 | 每个实例一副二值掩膜，另带实例 ID | 分开，person_1 与 person_2 各一副掩膜 | 本页不做 |
| 全景分割 | stuff 的语义图 + thing 的实例图统一输出 | thing 分开，stuff 不分开 | 本页不做：两套后处理叠加，端侧要并发跑两个 engine |

导航关心的是「哪片区域能走」。地面、墙体、植被属于 stuff，没有实例概念，硬做实例分割只会多花算力；反过来，抓取任务要的是某个具体物体的掩膜，语义分割给不出。选哪一条只看产物去向：产物要进代价地图就用语义分割，产物要进抓取规划的位姿估计才需要实例分割。

### 三个指标各自回答什么问题

指标选错，会把模型问题误判成数据集问题。

| 指标 | 回答的问题 | 会被什么误导 |
| --- | --- | --- |
| mIoU | 每一类各自分对了多少 | 小类（行人、杆件）拉低整体，看不出「大类其实已经够用」 |
| Pixel Accuracy | 全体像素里分对的比例 | 地面占 60% 时，全部预测成地面也能拿到 0.6 |
| FW-IoU | 按像素占比加权后的 IoU | 与 Pixel Accuracy 同源，大类好它就好看 |

三者的口径差别在于「谁说话响」：mIoU 对每个类别等权，一个小类分不好就明显压低总分；Pixel Accuracy 与 FW-IoU 都按像素数加权，谁像素多谁说话响。

读到数字后的动作是固定的：**mIoU 与 FW-IoU 的差值超过 0.15，说明小类基本没学出来**，此时先补样本或加类别权重，不要先去试量化；如果 FW-IoU 高而 mIoU 低，去混淆矩阵里看小类被并进了哪个大类。部署侧还要单独看两个二分类召回率：**地面召回率低意味着把能走的地方判成不能走，障碍物召回率低意味着有碰撞风险**，它们比 mIoU 更接近「机器人会不会撞」。

**一句话记忆：**mIoU 问「每一类都分对了吗」，Pixel Accuracy 问「整体对了多少」；前者看公平，后者看体量。

### 编码器-解码器：SegFormer 与 DeepLabV3+ 的差异

两者的大框架相同，都是「先降采样提语义、再升采样还分辨率」；差别在编码器用什么算子、多尺度上下文怎么拿、解码器多重。

| 对比项 | SegFormer（MiT-B0） | DeepLabV3+（ResNet-101 / MobileNetV2） |
| --- | --- | --- |
| 编码器 | 分层 Transformer（MiT），4 个 stage 依次输出 1/4、1/8、1/16、1/32 特征；4×4 重叠 patch 嵌入，不使用位置编码 | CNN 主干加空洞卷积，输出 stride 16 的特征图 |
| 多尺度上下文 | 自注意力本身覆盖全局，不需要额外模块 | ASPP：膨胀率 6、12、18 的空洞卷积加图像级池化，5 个分支拼接 |
| 解码器 | 全 MLP：把 4 个 stage 特征统一到 256 通道，上采样后拼接，输出 1/4 分辨率 logits | 深度可分离卷积，融合 stride 4 的低层特征，输出 1/4 分辨率 logits |
| 端侧友好度 | 算子以 MatMul、LayerNorm、GELU 为主，FP16 下 kernel 选择明确 | 大膨胀率空洞卷积在 FP16 下显存占用随膨胀率上升，ASPP 分支多、层数多 |

选型判据按部署走：本页主线用 **SegFormer-B0**，理由很直接：它的解码器输出就是 H/4 × W/4 的 logits，后处理只需一次最近邻上采样；DeepLabV3+ 同样是 1/4 输出，但 ASPP 的 5 个分支让 engine 层数和显存都更大。已有 CNN 训练管线、或需要 ASPP 那种显式多尺度上下文时，再考虑换过去。

**一句话记忆：**SegFormer 用注意力换掉了 ASPP；DeepLabV3+ 用空洞卷积换掉了自注意力。

### 类别映射：19 类语义怎么变成一张可行驶掩膜

分割网络输出的是数据集类别，机器人要的是「能不能走」。这一步是纯查表，但表定错，后面全是错的。

实机的模型是 Cityscapes 19 类的 SegFormer-B0，输出 `/perception/semantic_mask` 是一张 19 类 uint8 图（取值 0 到 18）。可行驶掩膜由 `config/segmentation.yaml` 里的 `drivable_class_ids` 派生：

| 来源类别 | 归入哪一档 | 理由与判据 |
| --- | --- | --- |
| road（trainId 0） | 可通行 | **实机默认只取这一类** |
| sidewalk（trainId 1） | 可选加入 | 配置注释明确写出：`drivable_class_ids` 默认 `[0]`（仅 road），sidewalk 为 id=1，可选 |
| terrain（trainId 9） | 条件可通行 | 草地、泥地取决于底盘：履带式可过、轮式需实测，默认按障碍处理更保守 |
| building、wall、fence、pole、traffic light、traffic sign、vegetation（trainId 2 到 8） | 障碍（静态） | 几何障碍 |
| person、rider、car、truck、bus、train、motorcycle、bicycle（trainId 11 到 18） | 障碍（动态） | 只标出当前位置；轨迹预测需要 4.2 的跟踪结果，本页不承担 |
| sky（trainId 10） | 忽略 | 深度图在该区域无效，或不在机器人本体高度范围内 |

映射在代码里写成一张长度等于类别数的常量数组，不要写成 if-else 链：类别顺序一变，if-else 会静默错位，而常量数组会直接报长度不匹配。

**注意 `drivable_class_ids: [0]` 是个保守的默认值。** 它意味着人行道不进可行驶区域。换场景前先确认这张表，而不是直接调推理参数。

### 两路掩膜：semantic_mask 与 drivable_mask 的分工

实机发布两路掩膜，而不是把映射结果合成一路：

| 话题 | 类型与取值 | 它的角色 |
| --- | --- | --- |
| `/perception/semantic_mask` | `sensor_msgs/Image`，19 类 uint8（0–18） | 原始语义结果，保留全部类别信息 |
| `/perception/drivable_mask` | `sensor_msgs/Image`，0/255 uint8 | 由 `drivable_class_ids` 派生，供下游直接消费 |

分成两路的好处是**映射与推理解耦**：上层要改「什么算能走」，改的是配置里的 `drivable_class_ids`，不需要重新推一遍网络；想调试映射对不对，两路掩膜并排看就能定位问题出在模型还是出在表。

这里有一条必须原样保留的边界，它写在实机配置的注释里：

> **`drivable_mask` 是 candidate-drivable semantic mask，不等价于 collision-free space（候选可行驶语义掩膜，不等价于无碰撞空间）。**

差别在哪：掩膜只回答「这个像素的类别看起来能不能走」，它不知道前方有没有一个未被分类的悬空障碍、不知道地面承重、不知道动态目标下一秒在哪。把它直接当「可通行」用，等于把一张语义图当成了安全凭证。

**一句话记忆：**`semantic_mask` 回答「这是什么」，`drivable_mask` 回答「按当前定义算不算能走」；两者都不回答「走过去安不安全」。

### 深度反投影：这一章为什么不做点云

源稿的后续是「把带标签的像素反投影成 3D 障碍物点云」。这条链路在数学上很短：

$\begin{bmatrix} X \\ Y \\ Z \end{bmatrix}=d\,K^{-1}\begin{bmatrix} u \\ v \\ 1 \end{bmatrix}$

展开就是 $X=(u-c_x)d/f_x$、$Y=(v-c_y)d/f_y$、$Z=d$，其中 $(u,v)$ 是像素坐标、$d$ 是该像素深度（米）、$f_x$、$f_y$、$c_x$、$c_y$ 取自 `camera_info` 的 K 矩阵。

**但实机没有点云话题。** `bev_segmentation` 只发布上面那两路掩膜，不发布任何 `PointCloud2`。所以本节只保留公式作为延伸知识，不作为本章的动手内容。

要真正做点云，还需要四个前置条件同时成立：深度图做过 D2C 对齐（深度像素与彩色像素一一对应）；深度图与彩色图分辨率相同且 `camera_info` 一致；K 来自与图像同一条标定线（本页用 2.3 主线的针孔 `plumb_bob` K；如果手上是鱼眼 `equidistant` 的 K，代入本式会得到系统性偏移）；深度单位换算正确（16UC1 编码一般以毫米存储，读进来要除以 1000）。这四条缺一条，点云就会和图像错位。

## 动手：运行分割链路并复核验收门

三步。以下 Jetson A 命令都从 `/home/seeed/workspace/ros2_bev` 执行；课程快照中的路径位于本章 `code/` 目录。模型与 engine 属于设备本地资产，不随课程仓库分发。

### 步骤 8：核对源码、安装结果与设备本地资产

先看清代码到哪一步、模型到哪一步。

```bash
cd /home/seeed/workspace/ros2_bev

# 源码：节点 / 前后处理 / engine 封装 / launch / 测试
find modules/m04-ai-vision-and-edge-acceleration/ros2/bev_segmentation -type f \
  \( -name "*.cpp" -o -name "*.hpp" -o -name "*.py" \) | sort

# 设备本地模型资产（课程快照不会包含二进制）
find modules/m04-ai-vision-and-edge-acceleration/models/m4/segmentation -type f | sort

# 已安装节点
source install/setup.bash
ros2 pkg executables bev_segmentation
```

代码与安装侧应能看到 `segmentation_node`、`segmentation_engine`、`preprocess`、`postprocess`、`config/segmentation.yaml`、launch 与测试。Jetson A 当前还持有 SegFormer-B0 ONNX、FP16 engine 与标签资产；这些大文件被排除在课程快照之外。

仓库提供三个脚本作为实施入口，按顺序：

```bash
scripts/m4/export_segformer.sh        # 导出 ONNX
scripts/m4/build_segformer_engine.sh  # 建 TensorRT engine
scripts/m4/generate_labels_json.sh    # 生成 labels.json
```

这三个脚本是重建设备本地资产的入口。正常学习与验收直接复用 Jetson A 已验证的产物；只有替换模型或重建环境时才需要重新生成。

### 步骤 9：核对两路输出接口

确认下游该按什么语义消费。

```bash
cat modules/m04-ai-vision-and-edge-acceleration/ros2/bev_segmentation/config/segmentation.yaml
```

逐项核对：

- **输入**：`/perception/cameras/front/image`（彩色图，本章不消费深度）。

- **两路输出**：`/perception/semantic_mask`（19 类 uint8）与 `/perception/drivable_mask`（0/255 uint8）。

- **`drivable_class_ids: [0]`**：默认只有 road 算可行驶，sidewalk（id=1）需要显式加入。

- **`publish_debug`**：配置中存在该项，默认 `false`；调试输出不属于当前规范验收接口。

这里有两条使用边界：`drivable_mask` 是候选可行驶掩膜，不等价于无碰撞空间；映射改了之后两路掩膜要一起复核。

### 步骤 10：启动节点并重新运行三层验收

先独立启动 M4.3，再观察两路掩膜是否持续发布：

```bash
./modules/m04-ai-vision-and-edge-acceleration/scripts/m4/run_m4_3_demo.sh

# 新终端
source /home/seeed/workspace/ros2_bev/install/setup.bash
ros2 topic hz /perception/semantic_mask
ros2 topic hz /perception/drivable_mask
```

随后重新运行三层测试；不得只复用旧结果：

```bash
colcon test --packages-select bev_segmentation --ctest-args -L unit
colcon test-result --all --verbose

colcon test --packages-select bev_segmentation --ctest-args -L engine_gate
colcon test-result --all --verbose

colcon test --packages-select bev_segmentation
colcon test-result --all --verbose
```

| 门 | 判据 | Jetson A 基线 |
| --- | --- | --- |
| Unit | 预处理、后处理、几何恢复与限流逻辑通过 | 已通过 |
| Engine Gate | PyTorch 与 TensorRT 输出对齐 | cosine `0.9966`，mIoU `0.9898` |
| Full | 全部 CTest / GTest 目标通过 | 7 个 CTest 目标、29 项 GTest 已通过 |
| 几何专项 | letterbox 还原后与参考掩膜一致 | 修复后 mIoU `1.0000`；旧实现 `0.8101` |

Hub 模式默认把 M4.3 限制在 `10 FPS`，独立模式允许按配置运行到 `25 FPS` 以上。限流是调度策略，不得误判为 engine 性能下降。

## 产出物与验收标准

### 交付清单

**当前可交付**

1. 可安装的 `bev_segmentation` 节点、配置、launch、预处理与后处理实现。

2. 一份接口契约记录：两路掩膜的话题、类型、取值范围，以及 `drivable_class_ids` 的当前取值。

3. Unit / Engine Gate / Full 三层回归，以及几何恢复和限流专项验证。

4. Jetson A 上的 ONNX、FP16 engine 与标签资产；课程快照只保留元数据和许可证，不分发二进制。

**运行产物**：`/perception/semantic_mask` 与 `/perception/drivable_mask` 两路实际图像。每次更换模型或 engine 后，都要重新运行 engine gate 和几何专项验证。

### 验收标准

| 检查项 | 通过标准 | 不通过时优先检查 |
| --- | --- | --- |
| 运行链路 | 节点可启动，两路掩膜持续发布且时间戳随输入更新 | engine 路径、输入话题和当前输入源 |
| 测试门 | unit、engine gate、full 均通过 | 是否复用了旧结果，或加载了不同 engine |
| 接口核对 | 两路掩膜的话题名、类型、取值范围与配置一致；能解释 `drivable_class_ids` 的作用 | 是否只记住了 0/255 而忘了 19 类那一路 |
| 安全边界 | 能复述 candidate-drivable ≠ collision-free，并举例说明为什么 | — |
| 运行限流 | Hub 与独立模式的目标 FPS 符合配置，无重复推理节点 | 是否同时启动了两个分割进程 |

## 常见问题与排障

### 照着步骤做，但一帧图都出不来

- **现象**：节点起不来，或者起来了但没有输出。

- **原因**：通常是课程快照未包含模型二进制、配置仍指向旧 engine，或输入话题没有图像。

- **处理**：先在 Jetson A 检查设备本地 engine 与 labels，再确认 `/perception/cameras/front/image` 有数据；如果更换过模型，按步骤 8 重建资产并重新跑三层验收。

### 地面类 IoU 明显低于其他类

- **现象**：整图看着还行，但地面（road）这一类分得很差。

- **原因**：地面在画面里占比大、纹理弱、受光照和反光影响明显；也可能是训练集里地面样本的多样性不足。

- **处理**：先看混淆矩阵确认地面被并进了哪一类（常见是并进 sidewalk 或 terrain）；再检查训练集的机位与光照分布。本章不训练模型，所以这条排障的方向是「换上游权重或补数据」，不是「调推理参数」。

### 两路掩膜对不上

- **现象**：`semantic_mask` 里明明是 road，`drivable_mask` 却把它标成 0。

- **原因**：`drivable_class_ids` 的取值与预期不一致，或者两路掩膜来自不同的两帧（时间戳没对齐）。

- **处理**：先 `ros2 topic echo` 两路掩膜的 `header.stamp`，确认是同一帧；再回到 `config/segmentation.yaml` 核对 `drivable_class_ids`。这条排障的价值在于：它把「模型问题」和「映射问题」分开了。

> **下一步：**4.4 姿态估计需要同时消费彩色与深度，但当前仍为 `BLOCKED`；4.5 只保留集成规划。4.3 的两路掩膜已经可运行，后续章节应按这里定义的接口消费，不能把规划描述成已完成实现。
