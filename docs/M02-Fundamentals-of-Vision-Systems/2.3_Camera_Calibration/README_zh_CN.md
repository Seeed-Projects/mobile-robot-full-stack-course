# 2.3 相机标定：从看懂内外参到完成双目标定

## 课程概述

相机能拍到画面，却不天然知道画面里的一个像素在真实世界的哪里。标定就是把这层缺失的几何关系补齐：图像去畸变、SLAM 定位、双目测距、三维重建和机械臂抓取，用的都是同一套标定结果。本课带你把一组抽象参数，变成可以被 ROS2 节点直接加载和验证的 `camera_info` 配置。

### 先知道：这节课会带你完成什么

| 阶段 | 你会理解什么 | 最终能做什么 |
| --- | --- | --- |
| 看懂 | 像素如何对应到空间、内参和外参分别回答什么问题 | 读懂标定结果，不再把参数当作黑盒 |
| 采集 | 为什么棋盘格要覆盖不同位置、距离和倾角 | 采到足够多样、可用于求解的数据 |
| 验证 | 重投影误差和极线对齐分别说明什么 | 判断一份标定文件是否真的可用 |

### 学完后，你能做到什么

- 理解针孔相机模型与畸变模型的物理含义，掌握内参矩阵 $K$ 与畸变系数 $(k_1, k_2, p_1, p_2, k_3)$ 的求解方法。
- 理解外参（旋转 $R$ 与平移 $t$）的几何意义，能够建立相机坐标系与世界坐标系的变换关系。
- 完成单目与双目相机的标定流程，输出可用的 $camera\_info$ YAML 文件。
- 通过重投影误差定量评估标定精度，识别并排除低质量标定数据。
- 了解手眼标定的基本概念（Eye-in-hand / Eye-to-hand），为 M9 机械臂视觉模块做铺垫。
- 进阶路径：说清为何 198° 鱼眼不用 `plumb_bob`、为何多样性比堆帧重要、为何新内参要重做外参、为何棋盘有 180° 错解；并产出 2.4 直接消费的 `equidistant` YAML + JSON。

### 硬件与软件清单

| 类别 | 说明 |
| --- | --- |
| 计算平台 | J501 开发板（或等效 x86/ARM 主机，运行 Ubuntu 22.04） |
| 相机 | 1–2 路相机（GMSL / USB / MIPI，本课程以 GMSL 相机为例），单目标定用 1 路，双目标定用 2 路已同步相机 |
| 标定板 | 棋盘格（Chessboard）或 ChArUco。主线 ROS 示例用 9×7 的 A4 板；**进阶四鱼眼必须用 8×6 内角点、25 mm**，与 2.4 现网一致。格距已知且精确，打印时不缩放，两套板不要混用。 |
| 标定工具 | 主线：ROS `camera_calibration`，或 Kalibr。进阶：`python3 tools/calib_web.py`（浏览器 `http://<jetson-ip>:8090`）。不要用 `fisheye-avm-calib` 的 GPU hub `:8787`。 |
| 辅助工具 | 卷尺（测量标定板格距）、三脚架或固定夹具、均匀照明环境 |

### 前置基础

- M2.1–M2.3：相机驱动正常工作，能够通过 ROS 话题或 SDK 稳定获取图像流。
- 线性代数基础：矩阵乘法、齐次坐标、旋转矩阵与平移向量的基本运算。
- ROS 基础：能够发布/订阅图像话题，理解 $sensor\_msgs/Image$ 与 $sensor\_msgs/CameraInfo$ 消息格式。

> **标定前检查：**必须锁定焦距或关闭自动对焦；否则采集过程中内参会变化，结果不可复用。镜头应清洁、标定板应平整。曝光和白平衡以“角点清晰、无过曝和强反光”为目标；是否关闭自动曝光取决于相机和现场光照，不必机械地一刀切。

## 先读懂：相机怎样把世界变成像素

先把相机想成一台“把三维世界压到二维照片上”的测量仪器。标定不要求你先掌握复杂数学；你只需要依次回答三个问题：

1. 这台相机怎样把光线变成像素？
2. 镜头让直线偏了多少？
3. 相机与另一个坐标系之间相隔多远、朝向哪里？

下面的公式都在回答这三个问题。

### 针孔相机模型与畸变

相机标定的核心，是用一组“已知在哪里”的点，反推出**相机怎样把三维世界投到二维像素**上：同一个空间点投到哪里、看起来有多大、边缘会偏多少，都由这套模型决定。我们先用针孔相机模型描述这一过程（最理想的情况）。

#### 针孔成像：最简单的“相机”

![针孔成像示意](./images/ILTUbTCaJo627QxZcM9cIPZKnie.gif)

在进入公式之前，不妨先直观地理解针孔成像。想象一个完全密闭的黑盒子，盒壁上只开一个针尖大的小孔。物体表面的每一点都在向四面八方反射光线，但只有恰好穿过小孔的那一束能进入盒内；又因为光沿直线传播，物体顶部的光穿过小孔后会落在盒内后壁的**下方**，底部的光反而落在**上方**。于是后壁上浮现出一幅上下颠倒、左右反转的影像。这就是最原始的相机：暗箱（camera obscura），“camera” 一词正是源于拉丁语中的“房间”。

小孔的大小决定了一对绕不开的矛盾：孔越小，每个物点对应的光束越细，影像越清晰，但进入盒内的光也越少，画面越暗；孔开大，画面变亮，但每个物点的光斑随之扩散，影像就模糊了。真实相机用镜头（凸透镜）化解了这对矛盾——大口径收集更多光线保证亮度，同时把光重新汇聚到一点保证清晰；而承接影像的那面“后壁”，则由图像传感器取代。

针孔相机模型就是把这套几何关系理想化：所有光线汇聚于一个没有体积的“光心”，穿过光心沿直线前进，投射到成像平面上。下文用相似三角形，把空间点在相机坐标系中的位置换算到成像平面上；真实镜头与这个理想模型的偏差，留给后文“镜头畸变”一节解释。

![针孔模型与外参](./images/Uxllb8S0joMT8cxfzCmcHLsinqf.png)

设空间中的一点 $P$ 在世界坐标系中的三维坐标为：

$P_w = \begin{bmatrix} X_w \\ Y_w \\ Z_w \end{bmatrix}$

首先需要通过**相机外参**，将该点从世界坐标系转换到相机坐标系：

$P_c = R P_w + t$

其中，**$R$ 表示两个坐标系的朝向差，$t$ 表示它们的原点位置差**；二者统称外参（Extrinsic Parameters）。重要的是：外参一定要说清“相对于谁”。单目采集时，求出的通常是相机相对每一张标定板的临时位姿；双目或多相机系统中，我们真正关心的是相机之间固定不变的相对位姿。

![归一化成像平面](./images/OGRrbXTGvoZ03Px04vxc9tIXnDe.png)

在相机坐标系中，相机光心位于坐标原点：

$O_c = (0, 0, 0)$

对于 OpenCV 中常用的相机坐标系约定，$X_c$ 轴指向图像右侧，$Y_c$ 轴指向图像下方，$Z_c$ 轴沿相机光轴指向相机前方。

空间点 $P_c=(X_c, Y_c, Z_c)$ 发出的光线经过相机光心后，与成像平面相交。根据相似三角形关系，可以得到该点在归一化成像平面上的坐标：

$x = \frac{X_c}{Z_c}, \qquad y = \frac{Y_c}{Z_c}$

这里的 $(x, y)$ 称为**归一化图像坐标（Normalized Image Coordinates）**。它们还不是最终图像中的像素坐标，而是描述空间点相对于相机光轴的位置。

---

### 相机内参与像素坐标

归一化图像坐标还需要通过相机内参转换为最终的像素坐标。

相机内参矩阵通常表示为：

$K = \begin{bmatrix} f_x & 0 & c_x \\ 0 & f_y & c_y \\ 0 & 0 & 1 \end{bmatrix}$

其中：

- $f_x$：水平方向上以像素为单位的等效焦距；
- $f_y$：垂直方向上以像素为单位的等效焦距；
- $(c_x, c_y)$：相机主点（Principal Point）在图像中的像素坐标；
- $K$：相机内参矩阵（Intrinsic Matrix）。

![内参矩阵](./images/Zh6UbJquxo3EzExc4GscxxTXn4d.png)

理想针孔模型下，归一化坐标与像素坐标之间满足：

$u = f_x x + c_x$

$v = f_y y + c_y$

因此：

$\begin{bmatrix} u \\ v \\ 1 \end{bmatrix} = K \begin{bmatrix} x \\ y \\ 1 \end{bmatrix}$

结合 $x = \frac{X_c}{Z_c}, \qquad y = \frac{Y_c}{Z_c}$，

可以得到常见的针孔相机投影公式：

$\begin{bmatrix} u \\ v \\ 1 \end{bmatrix} = \frac{1}{Z_c} \begin{bmatrix} f_x & 0 & c_x \\ 0 & f_y & c_y \\ 0 & 0 & 1 \end{bmatrix} \begin{bmatrix} X_c \\ Y_c \\ Z_c \end{bmatrix}$

镜头上标注的物理焦距 $f$ 以 mm 为单位，而标定得到的 $(f_x, f_y)$ 以 pixel 为单位，两者不是同一个物理量。

如果传感器在两个方向上的单像素尺寸分别为 $s_x$ 和 $s_y$，则可以近似表示为：

$f_x = \frac{f}{s_x}, \qquad f_y = \frac{f}{s_y}$

因此，在实际相机标定中，我们通常直接使用 $f_x$ 和 $f_y$，而不是直接使用镜头的毫米焦距 $f$。

---

### 镜头畸变

理想针孔模型假设光线经过一个理想的小孔完成投影，但真实镜头由多个光学镜片组成，因此实际图像通常会产生一定程度的几何畸变。常见镜头畸变主要包括**径向畸变（Radial Distortion）**和**切向畸变（Tangential Distortion）**。畸变模型作用的对象是归一化图像坐标 $(x, y)$，而不是最终的像素坐标 $(u, v)$。

定义：设归一化图像坐标为 $(x, y)$，则 $r$ 表示该点到图像中心（主点）的径向距离，满足：

$r^2 = x^2 + y^2$

在常用的 Brown-Conrady 畸变模型中，径向畸变将归一化坐标修正为：

$x_r = x(1 + k_1 r^2 + k_2 r^4 + k_3 r^6)$

$y_r = y(1 + k_1 r^2 + k_2 r^4 + k_3 r^6)$

其中 $(k_1, k_2, k_3)$ 为径向畸变系数。

径向畸变通常表现为两种典型形式：

- **桶形畸变（Barrel Distortion）**：图像边缘向外鼓出；
- **枕形畸变（Pincushion Distortion）**：图像边缘向内收缩。

![径向畸变](./images/PG2ibERDtot1aBx2JPIcclBEnif.png)

除了径向畸变，镜头光轴与图像传感器平面无法做到绝对理想对齐时，还会产生切向畸变，其表达式为：

$x_t = 2p_1 xy + p_2(r^2 + 2x^2)$

$y_t = p_1(r^2 + 2y^2) + 2p_2 xy$

其中 $(p_1, p_2)$ 为切向畸变系数。

![切向畸变](./images/WcB3bPgN3oEttJxpKj8cdb8ZnRf.png)

将径向畸变和切向畸变同时考虑后，可得到畸变后的归一化坐标：

$x_d = x(1 + k_1 r^2 + k_2 r^4 + k_3 r^6) + 2p_1 xy + p_2(r^2 + 2x^2)$

$y_d = y(1 + k_1 r^2 + k_2 r^4 + k_3 r^6) + p_1(r^2 + 2y^2) + 2p_2 xy$

随后，再通过相机内参将畸变后的归一化坐标转换为像素坐标：

$u = f_x x_d + c_x$

$v = f_y y_d + c_y$

因此，一台真实相机完整的成像过程可以概括为：

**世界坐标 → 相机坐标 → 归一化图像坐标 → 镜头畸变 → 像素坐标**

即：

$P_w \xrightarrow{R,t} P_c \xrightarrow{\div Z_c} (x,y) \xrightarrow{\text{Distortion}} (x_d,y_d) \xrightarrow{K} (u,v)$

![成像流水线](./images/GdNgbONSfoNdZYxxPyfcgvdCnHg.png)

---

### 相机标定究竟在求什么？

现在可以把“标定”理解成一次反向测量：标定板的角点在板上位置已知，软件在图像中找到这些角点，再调整模型参数，使“模型预测的像素位置”尽量贴近“实际检测到的位置”。求出的不是一组神秘数字，而是一份说明相机如何成像、如何纠正和如何与其他坐标系对齐的说明书。

通常需要求解的参数包括三类：

#### 相机内参

$K = \begin{bmatrix} f_x & 0 & c_x \\ 0 & f_y & c_y \\ 0 & 0 & 1 \end{bmatrix}$

描述相机自身的成像特性，包括焦距和主点位置。

#### 畸变参数

常见形式为：

$D = (k_1, k_2, p_1, p_2, k_3, \ldots)$

用于描述真实镜头相对于理想针孔模型产生的几何畸变。

#### 相机外参

$R, \quad t$

描述相机坐标系与世界坐标系、标定板坐标系或车辆坐标系之间的位置和姿态关系。

因此，可以将标定结果简单理解为：

$\{K, D, R, t\}$

**一句话记忆：**内参 $K$ 与畸变 $D$ 说明“这台相机怎样看”；外参 $(R,t)$ 说明“它相对某个参考坐标系在哪里、朝向哪里”。对于单目，参考物常是标定板；对于双目，参考物是另一台相机；对于车载或机器人系统，参考物通常是车体或机械臂基座。

对于 AVM、BEV、多摄像头拼接等应用，仅获得相机内参还不够。除了需要准确校正每个摄像头的镜头畸变，还必须获得各个摄像头相对于车辆坐标系或统一世界坐标系的外参，才能将不同摄像头观察到的内容正确映射到同一个鸟瞰平面中。

> **补充说明：**对于普通透视镜头，可以使用上述针孔模型配合 Brown-Conrady 畸变模型进行描述；对于视场角非常大的超广角或鱼眼镜头，普通针孔模型可能无法准确描述其投影特性，此时通常需要采用专门的 Fisheye、Kannala–Brandt 等鱼眼相机模型，而不是简单地不断增加径向畸变高阶参数。

### 手眼标定基础（M9 铺垫）

手眼标定解决的是相机坐标系与机械臂末端执行器（或基座）坐标系之间的变换问题，是视觉伺服、抓取规划的前置步骤。根据相机安装位置不同，分为两种典型构型：

| 维度 | Eye-in-hand（眼在手上） | Eye-to-hand（眼在手外） |
| --- | --- | --- |
| 安装方式 | 相机固定在机械臂末端，随臂运动 | 相机固定在工作空间外部，不随臂运动 |
| 标定目标 | 求解相机到末端法兰的变换 $T_{cam \to tool}$ | 求解相机到机械臂基座的变换 $T_{cam \to base}$ |
| 视野特点 | 随臂移动，可近距离观察目标，存在运动模糊风险 | 视野固定，可全程观察臂与目标，但分辨率受工作距离限制 |
| 典型方程 | $AX = XB$ | $AX = ZB$ |

![两种手眼构型](./images/EXzwbTRkMoVNR3xl8ubcoQKDndf.png)

本课程不展开手眼标定的具体求解，只要求理解两种构型的区别与适用场景。表中的 $AX=XB$、$AX=ZB$ 只是常见写法；不同资料对坐标系方向和变换命名的约定不同，不能脱离坐标系定义直接套用。M9 机械臂视觉模块将完整定义坐标系后再讲解求解与实操。

### 双目标定与极线校正

双目标定在单目标定的基础上，额外求解左右相机之间的外参关系 $(R, t)$——即左相机坐标系到右相机坐标系的旋转 $R$ 与平移 $t$。有了这组外参，就可以进一步对左右图像做**极线校正**。

先理解**极线几何**：空间中任意一点 $P$，在左图像上的投影为 $p_L$，则它在右图像上的对应点 $p_R$ 一定落在一条确定的直线上——这条直线就是**极线（epipolar line）**。未校正时，极线通常是倾斜的，立体匹配需要在二维区域内搜索对应点，计算量大且容易误匹配。

**极线校正（Stereo Rectification）**就是利用双目标定得到的 $(R, t)$，对左右两幅图像分别施加虚拟旋转，使两个相机的成像平面变为共面且行对齐。校正后，所有极线都变为水平直线，空间中同一点在左右图像上的投影严格位于同一水平行——立体匹配因此从二维搜索降为沿水平方向的一维搜索，效率和准确率均大幅提升。

极线校正（Rectification）的核心步骤：

1. 分别标定左右相机的内参与畸变系数（单目标定）。
2. 基于同步采集的标定板图像对，求解左右相机间的旋转 $R$ 与平移 $t$（双目标定核心）。
3. 利用 $(R, t)$ 构造左右相机的虚拟旋转矩阵 $R_1, R_2$，使两成像平面共面且行对齐。
4. 生成校正映射表（remap），对原始左右图像做重映射，得到校正后的图像对。

校正后，空间中同一点在左右图像上的投影严格位于同一水平行，立体匹配只需沿水平方向一维搜索。

![极线校正](./images/O2pFbJn65ouMDwxJyn6cTuUQnwg.png)

## 动手标定：从图像到可用参数

### 开始前：先确认这 4 件事

| 检查项 | 为什么重要 | 通过标准 |
| --- | --- | --- |
| 图像稳定 | 模糊、掉帧会让角点位置不可靠 | 图像话题持续发布，棋盘格边缘清晰 |
| 镜头状态固定 | 自动对焦会改变等效焦距，自动曝光可能影响角点检测 | 锁定焦距；尽量固定曝光和白平衡 |
| 标定板尺寸真实 | 格距写错会让距离尺度整体错误 | 实测一格边长，并换算为米 |
| 双目同步 | 同一时刻看到的必须是同一块板 | 优先硬件同步；否则记录并控制时间容差 |

> **注意：**本小节使用 ROS2 进行相机标定，如果还没安装 ROS2，请先参考 [1.4 机器人软件中间件：ROS2 Humble 快速上手](../../M01-Platform-and-Dev-Environment/1.4_Getting_Started_with_ROS2_Humble/README_zh_CN.md)。

### 环境准备与数据采集

**步骤 1：安装 ROS2 相机校准包**

```bash
sudo apt install ros-humble-camera-calibration
sudo apt install ros-${ROS_DISTRO}-v4l2-camera
```

**步骤 2：启动相机节点**

*终端 1*

```bash
ros2 run v4l2_camera v4l2_camera_node --ros-args \
    -r __node:=gmsl_cam0 \
    -r __ns:=/gmsl/cam0 \
    -p video_device:=/dev/video0 \
    -p image_size:="[1920,1536]" \
    -p pixel_format:=YUYV \
    -p output_encoding:=rgb8 \
    -p camera_frame_id:=gmsl_cam0_optical_frame \
    -p camera_info_url:=file:///home/seeed/.ros/camera_info/gmsl_cam0.yaml
```

**先确认驱动版本：**上述参数适用于常见的 `v4l2_camera` 用法；不同 GMSL / USB / MIPI 驱动的参数名可能不同。启动后运行 `ros2 param list /gmsl/cam0`，确认存在并实际生效的设备、分辨率、编码和 `camera_info_url` 参数，再进入标定。

打开另一个终端查看相机节点数据是否正常：

*终端 2*

```bash
#查看相机话题是否存在
ros2 topic list -t
#查看画面数据是否正常
ros2 topic hz /gmsl/cam0/image_raw
```

![话题列表](./images/CjIybHUa9our1nxs7ZFcCTJjn5g.png)

出现如上结果说明相机节点能够正常推流。

**步骤 3：准备标定板并启动标定程序**

本示例使用 [9×7 棋盘格](https://www.mrpt.org/downloads/camera-calibration-checker-board_9x7.pdf)：它有 9×7 个方格，因此内部可检测角点为 8×6。请以尺子实测一格边长；如果实测为 20 mm，则命令中的格距应写成 `0.020`（单位：米）。打印时务必关闭“适合页面”等缩放选项。

*启动单目标定*

```bash
ros2 run camera_calibration cameracalibrator \
  --no-service-check \
  --size 8x6 --square 0.020 \
  --ros-args -r image:=/gmsl/cam0/image_raw
```

> 参数与保存方式：
>
> - `--size 8x6` 是内角点数量：9×7 个方格对应 8×6 个内角点。
> - `--square 0.020` 是单个方格边长，单位为米；必须以实测值为准。
> - `--no-service-check` 允许没有 `SetCameraInfo` 服务的驱动启动标定工具。完成求解后点击 **SAVE** 保存结果；只有驱动确实提供该服务时才使用 **COMMIT**。

**步骤 4：采集标定图像**

将标定板置于相机视野内，改变标定板的位置和姿态，覆盖视野的不同区域（中心、四角、边缘）和不同深度。采集要点：

- 单目标定建议采集 **20–40 张**有效图像，双目标定建议采集 **30–50 对**同步图像。
- 标定板应占据视野的 **1/3 至 2/3** 面积，避免过小（角点检测精度低）或过大（超出视野）。

![标定板大小对比](./images/VJetbzVdfoyMkWxA3qhcZDyHnGe.jpg)

- 姿态应包含倾斜（绕 X/Y 轴旋转 30°–45°）和平面旋转（绕光轴旋转），避免所有图像姿态相似。

![采集覆盖策略](./images/ZJvkbcxIMoI2ucxjUfGcL3N3nnb.png)

- 确保标定板全程清晰无运动模糊，光照均匀无强反光。

![双目标定采集](./images/UOzBbirknoeGWyx3Zxcc6o3Dn7b.png)

> **采集技巧：**使用 $camera\_calibration$ 工具的实时预览窗口，当 X/Y/Size/Skew 四个进度条均达到绿色区域时再采集，可有效保证数据多样性。

采集足够数据后，点击 **CALIBRATE** 求解参数。计算完成后查看终端输出的内参、畸变系数和重投影误差，再点击 **SAVE** 保存结果，默认文件为 `/tmp/calibrationdata.tar.gz`。**COMMIT** 只用于驱动提供 `SetCameraInfo` 服务的情况；本教程采用保存 YAML 后由驱动启动时加载的方式，因此不要求 COMMIT 成功。

**结果转 YAML：**解压 `/tmp/calibrationdata.tar.gz`，单目包含 `ost.yaml`（即标准 camera_info YAML 格式），双目包含 `left.yaml` / `right.yaml`；按需重命名并复制到步骤 2 的 `camera_info_url` 路径（如 `/home/seeed/.ros/camera_info/gmsl_cam0.yaml`），重启相机节点即可让驱动加载新标定。

![矫正前后](./images/ZnfXb4jPvoZ78Xxkv9mcht1inRe.png)

### 双目标定

双目标定需要左右相机同步采集图像。使用 $camera\_calibration$ 时，启动命令需同时指定左右话题：

```bash
ros2 run camera_calibration cameracalibrator \
  --size 8x6 \
  --square 0.020 \
  --no-service-check \
  --ros-args -r left:=/stereo/left/image_raw \
  -r right:=/stereo/right/image_raw
```

**同步原则：**优先使用硬件同步，因此上面的标准命令不设置 `--approximate`。若设备只能软件同步，应先检查左右图像时间戳差，再从较小容差开始（例如 `--approximate=0.01`）；不要把 0.1 秒作为默认值，因为移动中的标定板在 100 ms 内可能已改变姿态。标定完成后，除各自的内参与畸变外，还会得到相机间 $R$、$T$，以及用于校正的 $R_1/R_2$ 和 $P_1/P_2$。

![极线校正前后](./images/ByPmbVDHRobNHfx0OhYcQdmAnfH.png)

使用 Kalibr 进行双目标定时，只需在 `--topics` 中列出左右两个话题，并在 `--models` 中对应指定两个相机模型即可。

### 精度评估与重投影误差

重投影误差（Reprojection Error）是标定是否可信的第一把尺子。把它想成“软件按标定参数画出的角点”和“图像里真实找到的角点”之间相差了多少像素：差得越小，说明模型越能解释这批图像；但它不能单独代表一切，还要结合图像清晰度、角点覆盖和双目的极线对齐一起判断。

$e_{rms} = \sqrt{\frac{1}{N} \sum_{i=1}^{N} \| \hat{p}_i - p_i \|^2}$

其中 $\hat{p}_i$ 为投影点，$p_i$ 为实际检测角点，$N$ 为总角点数。

| 重投影误差（像素） | 精度等级 | 说明 |
| --- | --- | --- |
| < 0.5 | 优秀 | 适合高精度测量、三维重建 |
| 0.5 – 1.0 | 良好 | 适合大多数 SLAM、检测应用 |
| 1.0 – 2.0 | 合格 | 可用于对精度要求不高的场景，建议优化采集数据 |
| > 2.0 | 不合格 | 需重新标定，检查镜头、标定板、采集质量 |

![重投影误差](./images/OAzGbzkpBoSutixCSFxcYUkLntc.png)

![逐帧误差](./images/RZLZbnjCioHYDSximPFc20vPnih.png)

表中的数值适合作为普通透视相机的起始参考，并非所有镜头和分辨率的统一门槛：超广角、鱼眼或低分辨率图像应结合具体模型判断。无论数值多少，都要逐张检查误差分布；少量图像明显高于平均值，往往意味着运动模糊、角点误检或标定板弯曲，应剔除后重新求解。

### 导出 camera_info YAML

标定完成后，需将参数保存为标准 YAML 格式，供后续模块调用。以下为单目 $camera\_info$ YAML 文件的标准格式：

```yaml
image_width: 1280
image_height: 720
camera_name: front_camera
camera_matrix:
  rows: 3
  cols: 3
  data: [640.5, 0.0, 640.0, 0.0, 640.5, 360.0, 0.0, 0.0, 1.0]
distortion_model: plumb_bob
distortion_coefficients:
  rows: 1
  cols: 5
  data: [0.05, -0.02, 0.001, -0.001, 0.0]
rectification_matrix:
  rows: 3
  cols: 3
  data: [1.0, 0.0, 0.0, 0.0, 1.0, 0.0, 0.0, 0.0, 1.0]
projection_matrix:
  rows: 3
  cols: 4
  data: [640.5, 0.0, 640.0, 0.0, 0.0, 640.5, 360.0, 0.0, 0.0, 0.0, 1.0, 0.0]
```

| 字段 | 把它理解为 | 主要使用者 |
| --- | --- | --- |
| `camera_matrix` | 内参 K：相机把归一化坐标换成像素的尺度和中心 | 去畸变、PnP、SLAM、测量 |
| `distortion_coefficients` | 畸变 D：镜头让边缘像素偏离理想位置的规律 | 图像校正 |
| `rectification_matrix` | 校正旋转 R：让双目图像行对齐的变换 | 立体匹配 |
| `projection_matrix` | 校正后的投影 P；右相机中还编码基线信息 | 双目深度计算 |

双目场景下，左右相机各保存一份 YAML，右相机的 $projection\_matrix$ 中包含基线信息（$P[0][3] = -f_x \cdot b$，其中 $b$ 为基线长度）。

在 ROS 系统中，推荐通过相机驱动的 $camera\_info\_url$ 参数加载该 YAML（与本课程步骤 2 的用法一致）：例如 $v4l2\_camera$ 指定 `-p camera_info_url:=file:///home/seeed/.ros/camera_info/gmsl_cam0.yaml`，驱动即按该标定发布 $sensor\_msgs/CameraInfo$ 话题，供图像校正、SLAM 等节点订阅。

### 进阶：四鱼眼标定（Web）与 2.4 产物衔接

**本节解决的问题：**主线已经用 ROS 把针孔相机的 `K/D` 和 `(R,t)` 标清楚。进阶把四台 198° 鱼眼标成 2.4 能直接消费的文件：JSON 给 AVM，`equidistant` YAML 给 ROS 驱动。实时拼接、`valid_mask`、metric/surround 验收留在 2.4，这里不启动 `avm_ros2`。

> **教学入口：**浏览器打开 `http://<jetson-ip>:8090`，按现有分页走：`/intrinsics` → `/extrinsics`（可选 `/seam`；网页 `/bev` 只是预览）。棋盘格用 **8×6 内角点、25 mm**，与主线 ROS 示例板分开，不要混用。主线 `plumb_bob` YAML 不喂 2.4。

#### 先理解：进阶外参是什么

把车体中心定义为 `base_link`。对每台相机，程序用地面棋盘格求出 `T_base_camera`（相机相对车体的位姿），以及“这套 `K/D/balance` 去畸变坐标 → 地面”的平面单应 `H`。`T_base_camera` 给 TF 看；`H` 才是后面 IPM 真正用的。这里的 BEV 是**地面平面**投影，不是任意高度的三维重建。

地面平面坐标系沿用配套包内部约定：`+X` 向右、`+Y` 向前、`+Z` 向上，和 ROS REP-103（`+X` 向前、`+Y` 向左）不同。阅读其他 ROS 资料或对接模块时，先换轴，不要直接抄数字。

#### 内参页为什么必须这样

198° 鱼眼在画面边缘已经不是“把 `plumb_bob` 的 `k` 再加几阶”能补回来的。针孔假设光线过光心后仍能用一张平面透视描述；视场过大后边缘残差会被后面的 `H` 和 IPM 放大，所以走 Kannala–Brandt / `equidistant` 4 参数，用 `cv2.fisheye.calibrate`。

**多样性，不是堆帧。**标定是反问题：姿态太像时 `K/D` 欠定。页面沿用 ROS `camera_calibration` 的 X/Y/Size/Skew 覆盖，而不是“多拍几张”。最少 15 张；四轴覆盖满，或张数到约 40，才建议 Calibrate。样本不够就点求解，边缘会看起来能去畸变，但外参接缝会漂。

**先算后存。**计算只出候选。先看去畸变预览和质量报告：fail 禁存，warn 需确认，再写盘。鱼眼边缘误差一旦写进正式内参，后面整条 `H` 都会跟着错。

**balance 缩放的是去畸变后的 `K_new`，不是改镜头。**`0` 裁掉无效区，`1` 留全视场，默认约 `0.8` 偏保留。内参预览和外参/BEV 必须用同一套，否则 `H` 对不上。

**双写产物：**YAML 给 ROS 驱动，JSON 给 AVM。新内参会把旧外参标 stale，因为 `H` 是在“这套 `K/D/balance` 的去畸变坐标”上量出来的。不要把主线 `plumb_bob` YAML 塞进这一路。

#### 步骤 1：启动标定台

标定台直接占 V4L2 设备，和 ROS 相机驱动互斥。先停掉任何 `camera_driver` / `avm_ros2`，再在 Jetson 上启动：

```bash
python3 tools/calib_web.py
```

外部电脑浏览器打开 `http://<jetson-ip>:8090`。不要用 `fisheye-avm-calib` 的 GPU hub `:8787`，那不是本课命令。

![四鱼眼界面](./images/Lc1sbKOiVonIhtxGzoMcOzglnFg.png)

#### 步骤 2：先走 /intrinsics

![内参标定页](./images/KoFMbMvwgokPQ7xyQpEctsgRnQf.png)

1. 每路相机单独采：棋盘格覆盖画面不同位置、远近和倾角，看 X/Y/Size/Skew，不要在同一姿态连拍凑数。
2. 样本够了再 Calibrate。先看去畸变预览：直线应被拉直，边缘不要花成波纹。
3. 质量报告 fail 不能存；warn 要看清原因再确认。保存后应出现该方向的 JSON 和 `distortion_model: equidistant` 的 YAML。

#### 外参页为什么必须这样

**直接测 `H`，不从 `K[r1 r2 t]` 推。**地面近似平面时，`H` 把“这台相机怎样看这块地”整包封住，避开模型残差。关键式：

$s\begin{bmatrix}u\\v\\1\end{bmatrix}=H\begin{bmatrix}X\\Y\\1\end{bmatrix}$

**卷尺摆位：**世界点是量的，像素点是检的。量错近边距，整张 BEV 的比例尺一起偏。

**连拍均值：**单帧角点有抖动和误检。稳定后连拍约 8 帧，对齐序、剔离群、对角点取平均再 `findHomography`。

**180° 歧义：**棋盘旋转 180° 看起来几乎一样。先用长短边去掉 90° 错解，再用“近边更大”的透视规律定 180°。解反了，这路画面会翻到车体对侧。

**网页 `/bev` 只预览。**实时 ROS 拼接、mask、metric/surround 验收留 2.4。可选 `/seam`：图割沿差异最小路径走缝，而不是重叠区大范围平均。

#### 步骤 3：再走 /extrinsics

![外参标定页](./images/ZpkCbeTqNowTyvxtqd1clvQvnog.png)

1. 每次只摆一个方向。棋盘格平铺地面，按页面给出的 `near_m` / `lateral_m` 用卷尺量到车心；默认近边约 0.35 m、长边横向。
2. 角点稳定后连拍锁定该路 `H`。四路都锁定并保存，才算外参完成。
3. 若某路画面翻到对侧，先查 180° 角点序，不要先回头改内参。
4. 可选进入 `/seam` 看相邻两路接缝；网页 `/bev` 只确认“大概能拼上”，不当 2.4 验收。

![接缝诊断页](./images/Ry6mbg0uDoTOYPx2UyUcCKtSnyv.png)

#### 步骤 4：只检查产物，不启动 avm_ros2

进阶过关看磁盘，不看 RViz。应同时有：

- `calib_results/{front,back,left,right}.json`
- `calib_results/extrinsics.json`
- `camera_info/{front,back,left,right}.yaml`（`distortion_model: equidistant`）

默认结果目录是 `/home/seeed/workspace/ros2_bev/calib_results/`。新内参会把旧外参标 stale：此时只重做外参，不必重走主线 ROS 标定。2.4 开头直接消费这些文件，不再重做标定。

#### 验收清单与排障方向

| 检查项 | 通过标准 | 不通过时优先检查 |
| --- | --- | --- |
| 内参模型 | 四路 YAML 都是 `equidistant`，不是主线 `plumb_bob` | 是否误把 ROS 单目标定文件喂给 AVM |
| 多样性 | 每路至少 15 张，X/Y/Size/Skew 有覆盖，而不是同一姿态堆帧 | 边缘去畸变发飘、外参接缝不稳 |
| 外参产物 | 四路 json + `extrinsics.json`，且未被标 stale | near_m、棋盘 8×6/25 mm、180° 角点序 |
| 网页预览 | `/bev` 能看见四路大致落在车体四周 | 只当预览；正式 metric 验收在 2.4 |

**边界提醒：**本节不控制底盘，也不把网页 BEV 当作激光雷达地图。它只交出 2.4 要用的标定文件。

#### 运行代码（Run the code）

> **说明**：下文命令中的 `<Jetson IP>` 请替换为你 Jetson 的实际 IP。可在 Jetson 终端运行 `hostname -I` 查询；请勿直接照抄固定地址。

本节代码在本仓库 `code/2.3_camera_calibration/`。`run_calib_web.sh` 与 `calib_web.py` 默认沿用 Jetson 上的以下路径，部署时按此布局摆放：

| 仓库文件 | Jetson 落点 |
| --- | --- |
| `j501_avm_calib/` | `~/ros2_ws/src/j501_avm_calib` |
| `calib_web.py`、`camera_probe_gui.py` | `~/workspace/ros2_bev/tools/` |
| `run_calib_web.sh` | `~/workspace/ros2_bev/scripts/` |

**clone / 拿到代码**（从本仓库 checkout 推到 Jetson）

```bash
scp -r code/2.3_camera_calibration/j501_avm_calib \
      seeed@<Jetson IP>:~/ros2_ws/src/
scp code/2.3_camera_calibration/calib_web.py \
    code/2.3_camera_calibration/camera_probe_gui.py \
    seeed@<Jetson IP>:~/workspace/ros2_bev/tools/
scp code/2.3_camera_calibration/run_calib_web.sh \
    seeed@<Jetson IP>:~/workspace/ros2_bev/scripts/
```

`j501_avm_calib` 是 `calib_web.py` 导入的算法包（`config`、`fisheye_math`、`homography`、`detect_board` 等）。Python 依赖 `numpy` / `opencv-python`（cv2）/ `PyYAML`，走系统 python3，不需要 venv。

**configure（构建）**

```bash
cd ~/ros2_ws
source /opt/ros/humble/setup.bash
colcon build --packages-select j501_avm_calib --symlink-install
source install/setup.bash
```

**run（启动标定台）**

```bash
# 先确认 camera_driver / avm_ros2 已停（共享 V4L2 设备，否则 device busy）
pkill -x camera_driver || true
cd ~/workspace/ros2_bev
bash scripts/run_calib_web.sh        # 等价 python3 tools/calib_web.py --port 8090
```

外部浏览器打开 `http://<Jetson IP>:8090`，按 `/intrinsics` → `/extrinsics`（可选 `/seam`；`/bev` 只作预览）执行。

**产物落盘**

- 内参 JSON + 外参：`~/workspace/ros2_bev/calib_results/{front,back,left,right}.json`、`~/workspace/ros2_bev/calib_results/extrinsics.json`
- camera_info YAML：`~/ros2_ws/src/j501_avm_calib/config/camera_info/{front,back,left,right}.yaml`（`distortion_model: equidistant`），并镜像写到 `~/ros2_ws/install/j501_avm_calib/share/j501_avm_calib/config/camera_info/`

## 产出物与验收标准

不要只以“工具显示成功”作为结束。完成标定后，请依次检查：参数文件能否被驱动加载、直线去畸变后是否更直、单目重投影误差是否合理、双目对应点是否落在同一水平行。四项都通过，才说明这份标定能被后续系统可靠使用。

### 交付清单

1. **标定参数 YAML 文件**：单目 1 份，双目 2 份（left.yaml / right.yaml），格式符合上述标准。
2. **标定精度报告**：包含重投影误差（总体 RMS + 逐帧分布）、内参矩阵、畸变系数、双目外参（如有）、采集图像数量与有效帧数。
3. **原始数据**（可选但建议保留）：标定用图像集或 ROS bag，便于后续复现与优化。
4. **进阶（可选，进入 2.4 前必交）**：`calib_results/{front,back,left,right}.json`、`extrinsics.json`，以及 4 份 `camera_info/<dir>.yaml`（`distortion_model: equidistant`）。此时不启动 `avm_ros2` 也能过关。主线 `plumb_bob` YAML 不在此列。

### 验收标准

- 普通透视镜头可将重投影 RMS ≤ 1.0 像素作为起始参考；超广角、鱼眼及高精度测量任务应结合镜头模型、分辨率、误差分布和实际应用验证，而非只看单一阈值。
- YAML 文件字段完整，可被 $image\_proc$ 或相机驱动（$camera\_info\_url$）正常加载。
- 双目校正后极线水平对齐，左右图像对应点行坐标差 ≤ 1 像素。
- 精度报告数据完整，误差分布无异常离群帧。
- 进阶：能口头回答为何不用 `plumb_bob` 标四鱼眼、为何要多样性而不是堆帧、为何新内参要重做外参、为何棋盘会有 180° 错解；磁盘产物齐全且 YAML 为 `equidistant`。

## 常见问题与排障

### 角点检测失败或不稳定

- **原因**：标定板过小、图像模糊、光照不足或过曝、棋盘格对比度不够。
- **解决**：增大标定板在视野中的占比，确保图像清晰，调整光照避免反光，使用高对比度标定板。

### 重投影误差偏高

- **原因**：采集数据姿态单一、存在运动模糊帧、镜头自动对焦导致焦距变化、标定板格距测量不准确。
- **解决**：增加采集图像的姿态多样性，逐帧检查并剔除高误差帧，锁定相机焦距与曝光，重新精确测量格距。

### 双目校正后极线不对齐

- **原因**：左右相机时间同步误差大、双目标定数据不足、相机间存在柔性振动。
- **解决**：使用硬件同步触发或减小 `--approximate` 容差，增加双目标定图像对数，确保相机刚性固定。

### Kalibr 运行报错或不收敛

- **原因**：bag 文件中图像话题不连续、标定板配置与实际不符、图像分辨率过高导致内存不足。
- **解决**：检查 bag 话题与帧率，核对 `aprilgrid.yaml` 参数，降低图像分辨率或使用 `--dont-show-extract` 选项。

### 进阶：四鱼眼标定失败或产物对不上 2.4

- **内参边缘发飘 / 外参接缝不稳**：不是再加几阶 `plumb_bob`。确认走了 `/intrinsics` 的 `equidistant`，X/Y/Size/Skew 有覆盖，而不是同一姿态堆帧。
- **新内参后旧外参变 stale**：`H` 绑定这套 `K/D/balance`。只重做 `/extrinsics`，不要回头改主线 ROS YAML。
- **某一路翻到车体对侧**：先查 180° 角点序（近边应更大），不要先重标内参。
- **2.4 读不到文件**：确认磁盘有 4 个 json、`extrinsics.json`、4 个 `equidistant` YAML；主线 `plumb_bob` 不能喂给 `avm_ros2`。

> **下一步：**完成本课程标定后，可进入 M3（视觉 SLAM）或 M4（目标检测）模块，将 $camera\_info$ YAML 配置到对应算法中。若涉及机械臂视觉，请继续学习 M9 手眼标定与视觉伺服。