# 2.4 多相机 BEV 拼接与环视感知基础

## 课程概述

标定回答的是"每台相机怎样看、装在哪、朝向哪"；拼接回答的是"怎么把四台相机看到的，铺成同一张地图"。2.3 结束时，`calib_results/` 里已经有四路内参文件和一份 `extrinsics.json`。本章把这些参数真正用起来：先把投影与拼接的几何原理讲透，再在 Jetson 上把实时鸟瞰图跑起来。

### 先知道：这节课会带你完成什么

| 阶段 | 你会理解什么 | 最终能做什么 |
| --- | --- | --- |
| 读懂 | 单应矩阵 $H$ 怎样把"地面上的点"与"画面里的像素"对应起来 | 看懂 BEV 投影的每一步，不再把拼接当黑盒 |
| 看透 | 四路画面为什么能共处一张画布而不打架 | 能诊断接缝错位、亮度差异、覆盖缺口 |
| 运行 | `avm_ros2` 怎样消费标定结果、输出哪些话题 | 在 RViz2 里得到实时 BEV 并逐项验证 |

整个环视感知可分为六个阶段：4 路鱼眼输入 → 相机标定 → 鱼眼去畸变 → 透视变换 / BEV 投影 → 图像拼接与融合 → 输出 360° 环视图。**2.3 进阶已经产出** `calib_results/{front,back,left,right}.json`、`extrinsics.json` 和 4 份 `equidistant` YAML。本章不再重做标定，只消费这些文件，把后四个阶段跑成实时 BEV。

![六阶段流程](./images/SadCb2VUrocSAPxunAKcS5b1nNV.png)

### 学完后，你能做到什么

- 理解单应矩阵 $H$ 的几何含义，说清它与 2.3 求出的内外参是什么关系。
- 理解逆透视投影（IPM）的"逆"在哪里：从 BEV 画布的每个像素，一步步推回鱼眼原图的采样位置。
- 理解多路拼接的融合策略：归属硬选、窄过渡带、亮度增益对齐与接缝精修，以及各自防住什么问题。
- 在 Jetson 上运行 `avm_ros2` 实时 BEV 管线，用 RViz2 查看并验证输出。
- 知道平面假设的失效边界，能判断一张 BEV 图里哪些内容可信、哪些只是投影假象。
- 能区分 metric BEV、`valid_mask` 与 surround/bowl 观察图，并说出未知区为什么不能填色。

### 硬件与软件清单

| 类别 | 说明 |
| --- | --- |
| 计算平台 | reComputer J501（Jetson AGX Orin 32GB，JetPack 6.2.1，ROS 2 Humble） |
| 相机 | 4 × GMSL 3G 鱼眼摄像头（Hov 198），固定支架间隔 90°，已按 2.3 完成内外参标定 |
| 标定板 | A4 棋盘格（8×6 内角点、格宽 25 mm；2.3 外参摆位用） |
| ROS 2 包 | `avm_ros2`（BEV 渲染与话题输出）、`j501_avm_calib`（相机驱动与标定配置）、RViz2 |
| 标定结果 | `calib_results/` 下四路内参 json 与 `extrinsics.json`（2.3 的产出，本章的直接输入） |

### 前置基础

完成 2.3 进阶：`~/workspace/ros2_bev/calib_results/` 已有四路 json 与 `extrinsics.json`，以及 `camera_info/<dir>.yaml`（`distortion_model: equidistant`）。主线 `plumb_bob` YAML 不能作为本章输入。

- 齐次坐标与矩阵乘法（2.3 前置基础的延续；本章还会大量用到矩阵的逆）。
- ROS 2 基础：launch、话题订阅、RViz2 显示配置。

## 先读懂：从标定结果到一张鸟瞰图

### 为什么需要 BEV？

BEV（Bird's Eye View，鸟瞰图）是把摄像头看到的世界，转换成一张从上往下看的地图。普通摄像头拍到的是透视画面，近处大、远处小，很难直接判断物体之间的真实位置；而 BEV 会利用相机的标定信息，把路面上的内容统一投影到同一个平面上，让车辆、行人、障碍物都像出现在地图上一样。这样不仅人看起来更直观，机器也更容易计算距离、判断相对位置和规划运动路线。所以，无论是汽车的 360° 环视、自动泊车，还是机器人的环境感知，BEV 的核心作用都是：把"看到的图像"变成"可理解的空间地图"。

![BEV 作用](./images/E657bSIT5oAl7jxX2VjcOAWYnNg.png)

### 从标定结果到地图：单应矩阵 H

BEV 拼接的几何，几乎全部压在一个假设上：**车辆周围的地面近似为一张平面**。在这个假设下，三维世界退化成二维，地面上的每个点只用两个坐标 $(X, Y)$ 就能说清（坐标系沿用 2.3 的车体系：以 `base_link` 为原点）。

对每台相机，2.3 外参标定求出的 $H$，描述的正是"地面点"与"画面像素"之间的对应：

$s \begin{bmatrix} u \\ v \\ 1 \end{bmatrix} = H \begin{bmatrix} X \\ Y \\ 1 \end{bmatrix}$

其中 $(X, Y)$ 是地面点在车体系中的位置，$(u, v)$ 是它在这台相机画面里的像素坐标，$s$ 是随点变化的缩放因子；"同一个矩阵乘出来还要再除一次"正是射影变换的标志。$H$ 是 3×3 矩阵、8 个自由度，4 对"地面点 ↔ 像素点"对应就足够解出它。

$H$ 不是新的魔法参数，它就是 2.3 整条成像链在地面平面上的压缩。对针孔模型，把地面取 $Z=0$，完整投影公式会塌缩成一个矩阵：

$H = K \begin{bmatrix} r_1 & r_2 & t \end{bmatrix}$

即内参 $K$ 乘上外参旋转的前两列与平移。但本项目没有从 $K, R, t$ 去推 $H$，而是**直接测量**：2.3 外参标定时把棋盘格平贴地面、用卷尺量好摆位（近边距车心 0.35 m），角点的地面坐标是量出来的，像素坐标是检测出来的，`findHomography` 从这批对应直接解出 $H$。这样做绕开了模型误差：$H$ 把"这台相机怎样看这块地面"整体封装，标定时什么样，运行时就按什么样用。

**一句话记忆：**$T_{base_camera}$ 回答"相机装在车的哪里、朝向哪"；$H$ 回答"地面上的一个点，落在它画面的哪里"。前者给 RViz 的 TF 用，后者给 BEV 投影用。

### 逆透视投影：BEV 画布的每个像素从哪里取色

有了 $H$，最直接的想象是"把原图投影到地面"。但实现要反过来问：**BEV 画布上这个像素，对应原图哪个像素？**逐像素回答"你从哪里来"，一次遍历就能填满整张画布，不产生洞，也不需要处理一对多。这就是逆透视投影（IPM）里"逆"的含义。

教科书路线是两段式：先把鱼眼去畸变成普通透视图，再对去畸变图做透视变换。它直观、可中途检查，但去畸变画布尺寸有限，大视场角的鱼眼画面会损失边缘。本项目走**直接反向映射**：跳过中间画布，从 BEV 像素一路推回原始鱼眼像素。以默认参数（600×600 画布、4×4 m 地面范围）为例，每个像素走五步：

**第一步：画布像素 → 地面坐标。**画布中心是车心，比例尺 $s_{px} = \text{canvas_px} / \text{view_m} = 600 / 4 = 150$ 像素/米：

$X = \frac{u - c}{s_{px}}, \qquad Y = -\frac{v - c}{s_{px}}$

$Y$ 取负号，是因为图像坐标向下增长，而画布上方是车头方向。

**第二步：地面 → 去畸变图像坐标。**标定时保存的 $H$ 把去畸变图像的像素映到标定画布（1000×1000、100 像素/米）。把它与一个视图变换 $T_{view}$（把标定画布坐标缩放平移到当前画布）复合成 $H_s = T_{view} \cdot H$，再取逆，就把当前画布像素送回了去畸变图像：

$\begin{bmatrix} u_d \\ v_d \\ 1 \end{bmatrix} \sim H_s^{-1} \begin{bmatrix} u \\ v \\ 1 \end{bmatrix}$

**第三步：去畸变坐标 → 归一化坐标。**用去畸变内参的逆 $K_{new}^{-1}$ 左乘齐次坐标，再除以第三个分量，得到 $(x, y)$。2.3 的"归一化图像坐标"一节讲的正是它。$K_{new}$ 在标定时按 balance=0.8 生成（0 = 裁掉全部无效区，1 = 保留全部视场，0.8 偏向保留）。

**第四步：归一化坐标 → 鱼眼原图像素。**这一步最反直觉也最妙：2.3 里畸变 $D$ 是要被消除的误差，这里却要**正向使用**它。`cv2.fisheye.distortPoints` 把理想射线位置 $(x, y)$ 按鱼眼模型扭曲，得到真实镜头实际把它拍到的位置 $(u_{raw}, v_{raw})$。

2.3 的成像链是"世界 → 像素"的正向行驶；本章这条链是同一条路的**逆行**：

$(u, v)_{bev} \xrightarrow{\ \div s_{px}\ } (X, Y) \xrightarrow{\ H_s^{-1}\ } (u, v)_d \xrightarrow{\ K_{new}^{-1}\ } (x, y) \xrightarrow{\ \text{鱼眼 } D\ } (u, v)_{raw}$

**第五步：查表采样。**五步合起来，对每个画布像素算出一个原图采样坐标。整张采样表（mapx / mapy）只构建一次，运行时 `cv2.remap` 查表插值，没有逐帧矩阵求解，这就是本章末尾"实时性"验证项的底气。

这条链的全程走向：主链自左向右是五次变换，下方三道关卡汇成有效支撑，才是真正可信的采样。

### 一次映射，三道关卡

反向映射算出的坐标，数学上成立不代表物理上该采。直接采样会引入三类"假像素"，代码里各设一道关卡：

| 关卡 | 拦住什么 | 判据 |
| --- | --- | --- |
| 界内检查 | 采样坐标落在原图之外 | $0 \le u_{raw} < w$ 且 $0 \le v_{raw} < h$ |
| 地面正分支 | 单应的逆映射有两条分支：相机前方的真实地面，与相机身后的翻折像（后者会把画面翻到车体对侧） | 与标定时保存的棋盘参考点做同侧判定（对 $H$ 整体变号不敏感） |
| 角度扇区 | 单应外推：把某路画面延伸到它视线扇区之外 | 每路以自身视线为轴，半角 50° 内满权重、75° 外归零的软扇区 |

三道关卡分别挡住三件事：界外检查拦住"采到图外"；单应正分支拦住"翻折到车体对侧"；角度扇区拦住"把一路画面外推到它看不见的扇区"。它们的交集才是这台相机的**有效支撑**。四路取并集，就是哪些像素被真实观测到，发布为 `/avm/bev/valid_mask`。没有任何相机支撑的像素是 unknown：`owner` 只在正向扇区得分里选，不借用邻路填色。未知区一旦被涂上，局部地图会把洞当成可通行地面。车体区域（0.46×0.46 m）以外的未观测比例低于 5%，是管线的健康线。

### 多路拼接与融合

四路相机间隔 90°，鱼眼视场又远超 90°，相邻画面必然重叠。重叠是拼接质量的来源（2.3 的接缝精修就发生在重叠带），但也带来一个问题：重叠区的像素听谁的？

朴素答案是加权平均。但平均的代价是**双影**：同一个物体在相邻两路画面里的投影总有像素级偏差，宽区域平均会把偏差糊成一片模糊重影。

本项目的方案是"先硬后软"：

**归属硬选（owner）**：每个画布像素只属于一路相机，取"角度扇区权重 × 有效支撑"得分最高的那路。没有任何相机支撑的像素保持 unknown，不借用邻路填色。

1. **窄过渡带**：只在归属边界两侧 4 cm（`transition_m` = 0.04）内做高斯平滑过渡。硬边界会在帧间闪跳，宽过渡会双影，4 cm 是两者的折中。
2. **亮度增益对齐**：每路乘一个标量增益，从重叠区中值亮度比估计（对数域最小二乘、front 锚定 1.0、限幅 0.85–1.15），把四路曝光差异拉平。
3. **接缝精修（可选）**：场景静止时一次性运行：用图割（graph cut）在稳定归属边界两侧约 6 cm 的窄带内，沿画面差异最小的路径微调接缝，把缝"绕开"差异大的画面内容。

整条融合管线：主链是每帧都在跑的渲染路径，增益与接缝精修是标定后一次性的优化支路。

最终合成是一行加权叠加：

$I_{bev} = \frac{\sum_{d} g_d \, w_d \, \mathcal{W}_d(I_d)}{\sum_{d} w_d}$

其中 $\mathcal{W}_d$ 是第 $d$ 路的查表投影，$w_d$ 是过渡带权重，$g_d$ 是增益。车体自身覆盖的 0.46×0.46 m 区域被遮罩排除，四台相机谁也看不到自己底盘下面。

**一句话记忆：**拼接的矛盾是"接缝要平滑"与"物体不能被切成两半"，解法是窄过渡带加低差异路径走缝，而不是大范围平均。

### BEV 与 AVM

BEV（Bird's Eye View）是一种"从上往下看的空间地图"，而 AVM（Around View Monitor）是一种"给人看的 360° 环视功能"。BEV 的重点是把多个摄像头的画面统一到同一个坐标系中，让系统能够理解物体的位置、距离和运动关系；**AVM 则是在此基础上，把这些画面拼接和融合成一张直观的鸟瞰图**，方便驾驶员观察车辆周围环境。**BEV 负责"理解空间"，AVM 负责"展示空间"**——前者更偏机器感知，后者更偏人机交互，而现代汽车的环视系统通常就是建立在 BEV 技术之上的。

现网把这两件事拆开了：`/avm/bev/metric/image` 是 4×4 m 可测距地面图，只能和 `valid_mask` 一起进局部地图；`/avm/bev/surround/image` 是给人看的 bowl 观察图，几何不可当尺子。coverage 的正式名是 `valid_mask`：未知保持未知。

![BEV 与 AVM](./images/DKD5b3gWZo4ozaxLxcoch6L3nuf.png)

### 平面假设的边界

单应成立的唯一前提是"点在地面上"。离开地面的内容，在 BEV 图里看到的不是它的真实形状，而是它与地面接触关系的投影：

- **离地物体被拉伸**：行人、桌椅在 BEV 上呈现为朝相机方向的拖影：接触点位置是对的，"身高"变成了沿视线方向的条纹。判断障碍物时信接触点，不信拖影轮廓。
- **地面起伏**：坡道、路缘让平面假设失效，投影随之漂移；本课程的平整室内地面可接受。
- **光照差异**：增益对齐能补小差异；四路自动曝光差异大时，要在源头固定曝光与白平衡（呼应 2.3 的标定前检查）。
- **时间相位**：`avm_bev` 对四路各自取最新帧（0.7 s 超时判失效），不做硬件同步。车辆静止时无感；移动中拼接会出现瞬间错位。

还有一条与 2.3 章末尾的提醒呼应：本节的 BEV 是**地面平面**的投影，不是三维重建。它为占据栅格和导航提供"地面视角的可靠输入"，但不预测离地物体的形状。项目里另有一条神经 BEV 主线（BEVDet 从多路图像直接回归三维感知），与本章的经典几何拼接是两条路线，不在本章展开。

## 动手：把标定结果跑成实时鸟瞰图

步骤编号承接 2.3 进阶：标定台已经交出文件。本章**不再重做标定**。启动前先停标定台（V4L2 互斥），再在 Jetson 上跑 `avm_ros2`。网页 `/bev` 只是预览，正式验收看 ROS 话题。

### 步骤 5：构建 AVM ROS 2 包

`avm_ros2` 只订阅 ROS 2 图像话题、渲染 BEV 并发布结果，不直接打开 `/dev/video*`，相机设备的归属仍在驱动与标定台。

```bash
cd /home/seeed/workspace/ros2_bev/ros2_ws
source /opt/ros/humble/setup.bash
source /home/seeed/ros2_ws/install/setup.bash
colcon build --packages-select avm_ros2 --symlink-install
source install/setup.bash
```

### 步骤 6：启动实时 BEV 拼接

```bash
ros2 launch avm_ros2 avm_rviz.launch.py start_camera_driver:=true
```

若四路 `/cameras/{front,back,left,right}/image_raw` 已有发布者（录包回放），可去掉 `start_camera_driver`，只启动渲染。课程实操默认把驱动一起拉起。标定台不关会 device busy，抓不到帧。

```bash
ros2 launch avm_ros2 avm_rviz.launch.py
```

> **提示：**标定台 `calib_web.py` 与相机驱动互斥占用 V4L2 设备：启动 `start_camera_driver` 前先停掉标定台，否则会因 device busy 抓不到帧。

RViz2 自动打开预配置视图，默认显示 `/avm/bev/image_annotated`（带健康状态叠加）。测量请勾选 `/avm/bev/metric/image` 与 `/avm/bev/valid_mask`；`/avm/bev/surround/image` 只观察，几何不可当尺子。旧名 `/avm/bev/image` / `/avm/bev/coverage` 仅兼容。BEV 渲染节点 `avm_bev` 的常用参数：

| 参数 | 默认值 | 含义 |
| --- | --- | --- |
| `view_m` | 4.0 | 地面覆盖范围（米），画布中心为车心 |
| `canvas_px` | 600 | 画布边长（像素），可 400–1000；细节与延迟的权衡 |
| `rate_hz` | 5.0 | 渲染频率 |
| `transition_m` | 0.04 | 接缝过渡带宽度（米） |
| `require_all_cameras` | true | 四路缺一即不出图（排障时可关） |

主要输出话题：

| 话题 | 内容 |
| --- | --- |
| `/avm/bev/metric/image` | 4×4 m 可测距地面图。只能和 `valid_mask` 一起进局部地图 |
| `/avm/bev/surround/image` | 给人看的 bowl 观察图，几何不可当尺子 |
| `/avm/bev/valid_mask` | mono8 可信地面像素；未知保持未知，禁止填色 |
| `/avm/bev/camera_mask` | mono8 最终归属：0 为 unknown，1–4 为四路相机 |
| `/avm/local_costmap` | 4×4 m、5 cm/格占据栅格（-1 未知 / 0 自由 / 50 候选 / 100 持续候选） |
| `/avm/bev/diagnostics` | 渲染后端（cuda/cpu）、覆盖率、四路在线状态 |

RViz 还显示 `/avm/bev/image_annotated`（带健康叠加，不当尺子）。旧名 `/avm/bev/image`、`/avm/bev/coverage` 只是 metric / valid_mask 的兼容别名。输入既可以是标定分辨率 1920×1536，也可以是驱动的 960×768 DDS 流；渲染器按同一比例缩放采样表，几何不变形。

![实时 BEV](./images/XcDkb0TrxoTVGTx64agcUG4rnkg.png)

### 步骤 7：验证与调优

- **拼接缝对齐**：观察相邻相机重叠区域，棋盘格应在接缝处连续、无错位；有明显错位时重新标定对应相机的外参。
- **距离准确性**：把已知尺寸的物体放在不同位置，在 `/avm/bev/metric/image` 上量，同时看 `/avm/bev/valid_mask` 确认观测覆盖。surround/bowl 图不能当尺子。
- **亮度一致性**：各相机曝光/白平衡差异大时，先固定曝光与白平衡（见 2.3 标定前检查），再依赖增益对齐。
- **实时性**：优先用 remap 查找表替代逐帧矩阵运算。本项目已按此实现；看 `/avm/bev/diagnostics` 确认渲染后端与覆盖是否健康。未知像素必须保持未知，不要在图上填色。

### 运行代码（Run the code）

> **说明**：下文命令中的 `<Jetson IP>` 请替换为你 Jetson 的实际 IP。可在 Jetson 终端运行 `hostname -I` 查询；请勿直接照抄固定地址。

本章代码在本仓库 `code/2.4_bev_avm/`，核心是 `ros2_ws/src/avm_ros2`。`ros2_ws/src/` 下的其余 `bev_*` / `bevdet_vendor` 是神经 BEV 主线，推理依赖 CUDA/TensorRT 与未入库的模型文件，这里仅作源码参考。标定工具共享 2.3，不在这里重复放：`j501_avm_calib`、`calib_web.py` 见 [2.3](../2.3_Camera_Calibration/README_zh_CN.md) 与 `code/2.3_camera_calibration/`。

**clone / 拿到代码**（从本仓库 checkout 推到 Jetson）

```bash
scp -r code/2.4_bev_avm/ros2_ws/src/avm_ros2 \
      seeed@<Jetson IP>:~/workspace/ros2_bev/ros2_ws/src/
```

`avm_ros2` 依赖 2.3 已构建的 `j501_avm_calib`：launch 里 `start_camera_driver:=true` 会拉起它的 `camera_driver` 节点。部署前先完成 2.3 的运行代码小节，确保 `~/ros2_ws` 里能 source 到该包。

**configure（构建）**

```bash
cd ~/workspace/ros2_bev/ros2_ws
source /opt/ros/humble/setup.bash
source ~/ros2_ws/install/setup.bash        # 提供 j501_avm_calib 的接口与驱动
colcon build --packages-select avm_ros2 --symlink-install
source install/setup.bash
```

**run（启动）**

```bash
# 标定台 calib_web.py 与相机驱动互斥占用 V4L2，先停掉
pkill -f calib_web.py || true
ros2 launch avm_ros2 avm_rviz.launch.py start_camera_driver:=true
```

**验证话题**：确认 `/avm/bev/metric/image`、`/avm/bev/valid_mask`、`/avm/bev/surround/image`、`/avm/bev/diagnostics` 持续发布；RViz 看 `/avm/bev/image_annotated`。

## 产出物与验收标准

### 交付清单

1. 运行中的实时 BEV 管线：`/avm/bev/metric/image` 与 `/avm/bev/valid_mask` 持续发布。
2. RViz2 截图：metric BEV 与 valid_mask，接缝处放置棋盘格或已知尺寸物体。surround 图可附一张，但不当测量依据。
3. （进阶）一次接缝精修的前后对比截图（标定台 Web 页面提供 before/after 对比）。

### 验收标准

| 检查项 | 通过标准 | 不通过时优先检查 |
| --- | --- | --- |
| 拼接几何 | 地面线条在接缝处连续，无明显错位 | 外参摆位测量、内参是否 stale、四路是否在线。接缝错位回 2.3 `/extrinsics` 对应方向，不回内参页，除非内参本身已 stale |
| 覆盖健康 | 车体区域外未观测比例 < 5%；未知区保持未知，没有被邻路填色 | 相机掉线（diagnostics）、扇区与分支关卡 |
| 亮度 | 重叠区无可见明暗跳变 | 四路曝光固定、增益对齐是否生效 |
| 实时性 | `/avm/bev/metric/image` 持续发布，diagnostics 无 ERROR | 渲染后端、`canvas_px`、输入分辨率 |

## 常见问题与排障

### BEV 完全不出图

- **原因**：`require_all_cameras` 为 true 且某路话题缺失或超时（0.7 s）。
- **解决**：`ros2 topic hz /cameras/front/image_raw` 逐路检查；排障期可临时关闭 `require_all_cameras`，看部分拼接定位是哪一路。

### 接缝错位

- **原因**：外参摆位量错、棋盘格规格不符、相机支架形变。
- **解决**：停 `avm_ros2`，回到 2.3 进阶 `/extrinsics` 重标对应方向。不要先回内参页，除非该路内参已被标 stale。错位固定出现在某一对相邻相机时，优先查那一路的 near_m 与 180° 角点序。

### 重叠区明暗跳变

- **原因**：四路自动曝光不一致，超出增益对齐的 0.85–1.15 限幅。
- **解决**：固定曝光与白平衡后重新标定外参，让增益从新的重叠区估计。

### 启动报设备占用（device busy）

- **原因**：`calib_web.py` 与相机驱动互斥占用 V4L2 设备。
- **解决**：停掉标定台再用 `start_camera_driver:=true`；或维持默认模式，由已有话题源供图。

> **下一步：**完成本节后，360° 环视图可作为上层算法的输入：接车道线/障碍物检测（M4）或视觉 SLAM（M3）时，把 2.3 进阶的标定结果（`equidistant` camera_info、单应矩阵 H）与本章的 BEV 话题（`/avm/bev/metric/image`、`/avm/bev/valid_mask`、`/avm/local_costmap`）一起接入对应节点。surround/bowl 图只给人看。