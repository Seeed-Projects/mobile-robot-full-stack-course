# 2.3 相机标定：从看懂内外参到完成双目标定

## 课程概述

相机能拍到画面，却不天然知道画面里的一个像素在真实世界的哪里。标定就是为相机建立这把“尺子”和“方向盘”：让后续的图像去畸变、SLAM 定位、双目测距、三维重建和机械臂抓取，使用同一套可靠的几何关系。本课带你把一组看似抽象的参数，变成可以被 ROS2 节点直接加载和验证的 camera_info 配置。

### 先知道：这节课会带你完成什么

| 阶段 | 你会理解什么 | 最终能做什么 |
| --- | --- | --- |
| 看懂 | 像素如何对应到空间、内参和外参分别回答什么问题 | 读懂标定结果，不再把参数当作黑盒 |
| 采集 | 为什么棋盘格要覆盖不同位置、距离和倾角 | 采到足够多样、可用于求解的数据 |
| 验证 | 重投影误差和极线对齐分别说明什么 | 判断一份标定文件是否真的可用 |

![图：标定不是一次“点按钮”的操作，而是从数据采集、参数求解、误差验证到 YAML 部署的完整闭环。](./images/ZSmlbU0GCofrGJxMxaLc9xoTngd.png)

*图：标定不是一次“点按钮”的操作，而是从数据采集、参数求解、误差验证到 YAML 部署的完整闭环。*

### 学完后，你能做到什么

- 理解针孔相机模型与畸变模型的物理含义，掌握内参矩阵 $K$ 与畸变系数 $(k_1, k_2, p_1, p_2, k_3)$ 的求解方法。
- 理解外参（旋转 $R$ 与平移 $t$）的几何意义，能够建立相机坐标系与世界坐标系的变换关系。
- 完成单目与双目相机的标定流程，输出可用的 $camera\_info$ YAML 文件。
- 通过重投影误差定量评估标定精度，识别并排除低质量标定数据。
- 了解手眼标定的基本概念（Eye-in-hand / Eye-to-hand），为 M9 机械臂视觉模块做铺垫。

### 硬件与软件清单

| 类别 | 说明 |
| --- | --- |
| 计算平台 | J501 开发板（或等效 x86/ARM 主机，运行 Ubuntu 20.04/22.04） |
| 相机 | 1–2 路相机（GMSL / USB / MIPI，本课程以 GMSL 相机为例），单目标定用 1 路，双目标定用 2 路已同步相机 |
| 标定板 | 棋盘格（Chessboard）或 ChArUco 标定板，尺寸与工作距离匹配（本课程用 9×7 的 A4 板），格距已知且精确，打印时不缩放 |
| 标定工具 | ROS $camera\_calibration$ 包，或 Kalibr（支持多相机 + IMU 联合标定） |
| 辅助工具 | 卷尺（测量标定板格距）、三脚架或固定夹具、均匀照明环境 |

### 前置基础

- M2.1–M2.3：相机驱动正常工作，能够通过 ROS 话题或 SDK 稳定获取图像流。
- 线性代数基础：矩阵乘法、齐次坐标、旋转矩阵与平移向量的基本运算。
- ROS 基础：能够发布/订阅图像话题，理解 $sensor\_msgs/Image$ 与 $sensor\_msgs/CameraInfo$ 消息格式。
> 标定前检查：必须锁定焦距或关闭自动对焦；否则采集过程中内参会变化，结果不可复用。镜头应清洁、标定板应平整。曝光和白平衡以“角点清晰、无过曝和强反光”为目标；是否关闭自动曝光取决于相机和现场光照，不必机械地一刀切。

## 先读懂：相机怎样把世界变成像素

先把相机想成一台“把三维世界压到二维照片上”的测量仪器。标定不要求你先掌握复杂数学；你只需要依次回答三个问题：这台相机怎样把光线变成像素？镜头让直线偏了多少？相机与另一个坐标系之间相隔多远、朝向哪里？下面的公式都在回答这三个问题。

### 针孔相机模型与畸变

相机标定的核心，是用一组“已知在哪里”的点，反推出相机怎样把三维世界投到二维像素上。把它理解为校准一把看不见的尺子：同一个空间点投到哪里、看起来有多大、边缘会偏多少，都由这套模型决定。理想情况下，我们先用针孔相机模型（Pinhole Camera Model）描述这一过程。

设空间中的一点 $P$ 在世界坐标系中的坐标为：

$P_w = \begin{bmatrix} X_w \\ Y_w \\ Z_w \end{bmatrix}$

首先需要通过相机外参，将该点从世界坐标系转换到相机坐标系：

$P_c = R P_w + t$

其中，$R$ 表示两个坐标系的朝向差，$t$ 表示它们的原点位置差；二者统称外参（Extrinsic Parameters）。重要的是：外参一定要说清“相对于谁”。单目采集时，求出的通常是相机相对每一张标定板的临时位姿；双目或多相机系统中，我们真正关心的是相机之间固定不变的相对位姿。

![图：外参描述两个坐标系的关系。R 表示朝向差，t 表示两个原点的位置差；它们必须相对某个明确的参考坐标系理解。](./images/IxYkbwp8yo4GoxxBQTIcVkGPnog.png)

*图：外参描述两个坐标系的关系。R 表示朝向差，t 表示两个原点的位置差；它们必须相对某个明确的参考坐标系理解。*

在相机坐标系中，相机光心位于坐标原点：

$O_c = (0, 0, 0)$

对于 OpenCV 中常用的相机坐标系约定，$X_c$ 轴指向图像右侧，$Y_c$ 轴指向图像下方，$Z_c$ 轴沿相机光轴指向相机前方。

空间点 $P_c=(X_c, Y_c, Z_c)$ 发出的光线经过相机光心后，与成像平面相交。根据相似三角形关系，可以得到该点在归一化成像平面上的坐标：

$x = \frac{X_c}{Z_c}, \qquad y = \frac{Y_c}{Z_c}$

这里的 $(x, y)$ 称为归一化图像坐标（Normalized Image Coordinates）。它们还不是最终图像中的像素坐标，而是描述空间点相对于相机光轴的位置。

![图：针孔相机模型。空间点 P 经相机光心投影到成像平面，通过相似三角形关系得到归一化图像坐标 (x, y)](./images/PlkZbla1toR8cAx8jAjcBLfDnaf.png)

*图：针孔相机模型。空间点 P 经相机光心投影到成像平面，通过相似三角形关系得到归一化图像坐标 (x, y)*

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
![图：内参矩阵的作用。主点 (c_x, c_y) 给出像素坐标原点，f_x、f_y 把归一化坐标缩放为像素：u = f_x·x + c_x](./images/MKtqbrzpsoccf7xcMM5cOX0onOg.png)

*图：内参矩阵的作用。主点 (c_x, c_y) 给出像素坐标原点，f_x、f_y 把归一化坐标缩放为像素：u = f_x·x + c_x*

理想针孔模型下，归一化坐标与像素坐标之间满足：

$u = f_x x + c_x$

$v = f_y y + c_y$

因此：

$\begin{bmatrix} u \\ v \\ 1 \end{bmatrix} = K \begin{bmatrix} x \\ y \\ 1 \end{bmatrix}$

结合 $x = \frac{X_c}{Z_c}, \qquad y = \frac{Y_c}{Z_c}$，

可以得到常见的针孔相机投影公式：

$\begin{bmatrix} u \\ v \\ 1 \end{bmatrix} = \frac{1}{Z_c} \begin{bmatrix} f_x & 0 & c_x \\ 0 & f_y & c_y \\ 0 & 0 & 1 \end{bmatrix} \begin{bmatrix} X_c \\ Y_c \\ Z_c \end{bmatrix}$

需要注意的是，镜头上标注的物理焦距 $f$ 通常以 mm 为单位，而相机标定得到的 $(f_x, f_y)$ 以 pixel 为单位。两者并不是完全相同的物理量。

如果传感器在两个方向上的单像素尺寸分别为 $s_x$ 和 $s_y$，则可以近似表示为：

$f_x = \frac{f}{s_x}, \qquad f_y = \frac{f}{s_y}$

因此，在实际相机标定中，我们通常直接使用 $f_x$ 和 $f_y$，而不是直接使用镜头的毫米焦距 $f$。

---

### 镜头畸变

理想针孔模型假设光线经过一个理想的小孔完成投影，但真实镜头由多个光学镜片组成，因此实际图像通常会产生一定程度的几何畸变。

常见镜头畸变主要包括：

- 径向畸变（Radial Distortion）
- 切向畸变（Tangential Distortion）
需要特别注意：畸变模型作用的对象通常是归一化图像坐标 $(x, y)$，而不是最终的像素坐标 $(u, v)$。

定义：

$r^2 = x^2 + y^2$

对于常用的 Brown-Conrady 畸变模型，径向畸变可以表示为：

$x_r = x(1 + k_1 r^2 + k_2 r^4 + k_3 r^6)$

$y_r = y(1 + k_1 r^2 + k_2 r^4 + k_3 r^6)$

其中 $(k_1, k_2, k_3)$ 为径向畸变系数。

径向畸变通常表现为两种典型形式：

- 桶形畸变（Barrel Distortion）：图像边缘向外鼓出；
- 枕形畸变（Pincushion Distortion）：图像边缘向内收缩。
![图：径向畸变。左：理想成像；中：桶形畸变（k1<0，边缘点向中心收缩）；右：枕形畸变（k1>0，边缘点远离中心）。箭头为角点位移方向](./images/RhBdbjmWnogQv1xDV8FcpXdanAh.png)

*图：径向畸变。左：理想成像；中：桶形畸变（k1<0，边缘点向中心收缩）；右：枕形畸变（k1>0，边缘点远离中心）。箭头为角点位移方向*

除了径向畸变，镜头光轴与图像传感器平面无法做到绝对理想对齐时，还会产生切向畸变，其表达式为：

$x_t = 2p_1 xy + p_2(r^2 + 2x^2)$

$y_t = p_1(r^2 + 2y^2) + 2p_2 xy$

其中 $(p_1, p_2)$ 为切向畸变系数。

![图：切向畸变的物理成因。镜片组装时光轴与传感器平面无法绝对垂直，像点沿切向发生偏移](./images/XTzybe3AcoUuQ7xtRmdc7gWsnla.png)

*图：切向畸变的物理成因。镜片组装时光轴与传感器平面无法绝对垂直，像点沿切向发生偏移*

将径向畸变和切向畸变同时考虑后，可得到畸变后的归一化坐标：

$x_d = x(1 + k_1 r^2 + k_2 r^4 + k_3 r^6) + 2p_1 xy + p_2(r^2 + 2x^2)$

$y_d = y(1 + k_1 r^2 + k_2 r^4 + k_3 r^6) + p_1(r^2 + 2y^2) + 2p_2 xy$

随后，再通过相机内参将畸变后的归一化坐标转换为像素坐标：

$u = f_x x_d + c_x$

$v = f_y y_d + c_y$

因此，一台真实相机完整的成像过程可以概括为：

世界坐标 → 相机坐标 → 归一化图像坐标 → 镜头畸变 → 像素坐标

即：

$P_w \xrightarrow{R,t} P_c \xrightarrow{\div Z_c} (x,y) \xrightarrow{\text{Distortion}} (x_d,y_d) \xrightarrow{K} (u,v)$

![图：完整成像流水线。标定的任务就是求齐这条链上的所有参数：{K, D, R, t}](./images/W3CXbFcLVoPNpTxGInRcAqmHnjf.png)

*图：完整成像流水线。标定的任务就是求齐这条链上的所有参数：{K, D, R, t}*

---

### 相机标定究竟在求什么？

现在可以把“标定”理解成一次反向测量：标定板的角点在板上位置已知，软件在图像中找到这些角点，再调整模型参数，使“模型预测的像素位置”尽量贴近“实际检测到的位置”。求出的不是一个神秘数字，而是一份说明相机如何成像、如何纠正和如何与其他坐标系对齐的说明书。

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

一句话记忆：内参 $K$ 与畸变 $D$ 说明“这台相机怎样看”；外参 $(R,t)$ 说明“它相对某个参考坐标系在哪里、朝向哪里”。对于单目，参考物常是标定板；对于双目，参考物是另一台相机；对于车载或机器人系统，参考物通常是车体或机械臂基座。

对于 AVM、BEV、多摄像头拼接等应用，仅获得相机内参还不够。除了需要准确校正每个摄像头的镜头畸变，还必须获得各个摄像头相对于车辆坐标系或统一世界坐标系的外参，才能将不同摄像头观察到的内容正确映射到同一个鸟瞰平面中。

> 补充说明：对于普通透视镜头，可以使用上述针孔模型配合 Brown-Conrady 畸变模型进行描述；对于视场角非常大的超广角或鱼眼镜头，普通针孔模型可能无法准确描述其投影特性，此时通常需要采用专门的 Fisheye、Kannala–Brandt 等鱼眼相机模型，而不是简单地不断增加径向畸变高阶参数。

### 手眼标定基础（为 M9 铺垫）

手眼标定解决的是相机坐标系与机械臂末端执行器（或基座）坐标系之间的变换问题，是视觉伺服、抓取规划的前置步骤。根据相机安装位置不同，分为两种典型构型：

| 维度 | Eye-in-hand（眼在手上） | Eye-to-hand（眼在手外） |
| --- | --- | --- |
| 安装方式 | 相机固定在机械臂末端，随臂运动 | 相机固定在工作空间外部，不随臂运动 |
| 标定目标 | 求解相机到末端法兰的变换 $T_{cam \to tool}$ | 求解相机到机械臂基座的变换 $T_{cam \to base}$ |
| 视野特点 | 随臂移动，可近距离观察目标，存在运动模糊风险 | 视野固定，可全程观察臂与目标，但分辨率受工作距离限制 |
| 典型方程 | $AX = XB$ | $AX = ZB$ |

![image.png](./images/Vy1RbDqKDoEd87xClw5cbMJan4b.png)

![AX=XB 坐标系链路推导（四步流程）：1.坐标系链路 Base→End→Cam→Target 2.已知/未知变换 A已知·X未知·B已知 3.两姿态消元得 AX=XB 4.至少3组姿态构成2个方程](./images/ZZ7FbJyC4ooUpCxmp6ZcY6Gpnfb.png)

*AX=XB 坐标系链路推导（四步流程）：1.坐标系链路 Base→End→Cam→Target 2.已知/未知变换 A已知·X未知·B已知 3.两姿态消元得 AX=XB 4.至少3组姿态构成2个方程*

本课程不展开手眼标定的具体求解，只要求理解两种构型的区别与适用场景。表中的 $AX=XB$、$AX=ZB$ 只是常见写法；不同资料对坐标系方向和变换命名的约定不同，不能脱离坐标系定义直接套用。M9 机械臂视觉模块将完整定义坐标系后再讲解求解与实操。

### 双目标定与极线校正

双目标定在单目标定的基础上，额外求解左右相机之间的外参关系 $(R, t)$，使得两个相机的成像平面可以被重投影到同一平面上，从而使对应极线水平对齐，大幅降低立体匹配的搜索维度。

极线校正（Rectification）的核心步骤：

1. 分别标定左右相机的内参与畸变系数。
1. 基于同步采集的标定板图像对，求解左右相机间的旋转 $R$ 与平移 $t$。
1. 构造旋转矩阵 $R_1, R_2$，将左右图像各自旋转，使两成像平面共面且行对齐。
1. 生成校正映射表（remap），用于实时图像校正。
校正后，空间中同一点在左右图像上的投影位于同一水平行，立体匹配只需沿水平方向搜索。

![图：极线约束与极线校正。左：极线几何；右：校正后对应点位于同一水平行，立体匹配降为一维搜索](./images/QAKMbBXXHoX85nxOU7rcT5DWnQb.png)

*图：极线约束与极线校正。左：极线几何；右：校正后对应点位于同一水平行，立体匹配降为一维搜索*

## 动手标定：从图像到可用参数

### 开始前：先确认这 4 件事

| 检查项 | 为什么重要 | 通过标准 |
| --- | --- | --- |
| 图像稳定 | 模糊、掉帧会让角点位置不可靠 | 图像话题持续发布，棋盘格边缘清晰 |
| 镜头状态固定 | 自动对焦会改变等效焦距，自动曝光可能影响角点检测 | 锁定焦距；尽量固定曝光和白平衡 |
| 标定板尺寸真实 | 格距写错会让距离尺度整体错误 | 实测一格边长，并换算为米 |
| 双目同步 | 同一时刻看到的必须是同一块板 | 优先硬件同步；否则记录并控制时间容差 |

> 注意：本小节将使用 ROS2 来进行相机标定，如果还没安装 ROS2 请参考 1.4 机器人软件中间件：ROS2 Humble 快速上手

### 环境准备与数据采集

步骤 1：安装 ROS2 相机校准包

*安装 ROS2 相机标定与驱动包*

```bash
sudo apt install ros-humble-camera-calibration
sudo apt install ros-${ROS_DISTRO}-v4l2-camera
```

步骤 2：启动相机节点

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

先确认驱动版本：上述参数适用于常见的 v4l2_camera 用法；不同 GMSL / USB / MIPI 驱动的参数名可能不同。启动后运行 ros2 param list /gmsl/cam0，确认存在并实际生效的设备、分辨率、编码和 camera_info_url 参数，再进入标定。

打开另一个终端查看相机节点数据是否正常

*终端 2*

```bash
#查看相机话题是否存在
ros2 topic list -t
#查看画面数据是否正常
ros2 topic hz /gmsl/cam0/image_raw
```

![image.png](./images/CjIybHUa9our1nxs7ZFcCTJjn5g.png)

出现如上结果说明相机节点能够正常推流！

步骤 3：准备标定板并启动标定程序。

本示例使用 9×7 棋盘格：它有 9×7 个方格，因此内部可检测角点为 8×6。请以尺子实测一格边长；如果实测为 20 mm，则命令中的格距应写成 0.020（单位：米）。打印时务必关闭“适合页面”等缩放选项。

*启动单目标定（驱动不提供 SetCameraInfo 服务时也可采集并保存）*

```bash
ros2 run camera_calibration cameracalibrator \
  --no-service-check \
  --size 8x6 --square 0.020 \
  --ros-args -r image:=/gmsl/cam0/image_raw
```

> 参数与保存方式：
>
> - --size 8x6 是内角点数量：9×7 个方格对应 8×6 个内角点。
> ![9×7 棋盘格内角点示意：橙点为 8×6=48 个内角点，灰圈为边界点不计入 --size](./images/VB7ubnWghotkqXx3KCiclEpinPe.png)
>
> *9×7 棋盘格内角点示意：橙点为 8×6=48 个内角点，灰圈为边界点不计入 --size*
>
> - --square 0.020 是单个方格边长，单位为米；必须以实测值为准。
> - --no-service-check 允许没有 SetCameraInfo 服务的驱动启动标定工具。完成求解后点击 SAVE 保存结果；只有驱动确实提供该服务时才使用 COMMIT。

步骤 4：采集标定图像。

将标定板置于相机视野内，改变标定板的位置和姿态，覆盖视野的不同区域（中心、四角、边缘）和不同深度。采集要点：

- 单目标定建议采集 20–40 张有效图像，双目标定建议采集 30–50 对同步图像。
- 标定板应占据视野的 1/3 至 2/3 面积，避免过小（角点检测精度低）或过大（超出视野）。
![标定板大小对比：太小（<1/3视野）角点检测精度低，适中（1/3~2/3视野）推荐，太大（>2/3视野）角点可能超出视野](./images/UA1MbwF2FoS0D8x04pAcKVJon9c.png)

*标定板大小对比：太小（<1/3视野）角点检测精度低，适中（1/3~2/3视野）推荐，太大（>2/3视野）角点可能超出视野*

- 姿态应包含倾斜（绕 X/Y 轴旋转 30°–45°）和平面旋转（绕光轴旋转），避免所有图像姿态相似。
- 确保标定板全程清晰无运动模糊，光照均匀无强反光。
![Screenshot from 2026-08-21 15-39-55.png](./images/UOzBbirknoeGWyx3Zxcc6o3Dn7b.png)

![图：标定数据采集的覆盖策略。标定板需覆盖视野中心与四角、包含不同距离（大小）、倾斜与面内旋转，姿态越多样标定越稳](./images/CLO2bn7b0oWj7DxH3lYcqDbanAe.png)

*图：标定数据采集的覆盖策略。标定板需覆盖视野中心与四角、包含不同距离（大小）、倾斜与面内旋转，姿态越多样标定越稳*

> 采集技巧：使用 $camera\_calibration$ 工具的实时预览窗口，当 X/Y/Size/Skew 四个进度条均达到绿色区域时再采集，可有效保证数据多样性。

采集足够数据后，点击 CALIBRATE 求解参数。计算完成后查看终端输出的内参、畸变系数和重投影误差，再点击 SAVE 保存结果，默认文件为 /tmp/calibrationdata.tar.gz。COMMIT 只用于驱动提供 SetCameraInfo 服务的情况；本教程采用保存 YAML 后由驱动启动时加载的方式，因此不要求 COMMIT 成功。

结果转 YAML：解压 /tmp/calibrationdata.tar.gz，单目包含 ost.yaml（即标准 camera_info YAML 格式），双目包含 left.yaml / right.yaml；按需重命名并复制到步骤 2 的 camera_info_url 路径（如 /home/seeed/.ros/camera_info/gmsl_cam0.yaml），重启相机节点即可让驱动加载新标定。

![image.png](./images/ZnfXb4jPvoZ78Xxkv9mcht1inRe.png)

### 进阶：Kalibr（多相机 / IMU）

Kalibr 适用于多相机、相机—IMU 联合标定等进阶场景。它的优势不在于“自动更高精度”，而在于能把更完整的传感器模型、时间关系和观测数据放进同一次求解。若你当前只需为一台或一对普通相机生成 ROS2 的 camera_info，先完成前面的 camera_calibration 最小闭环即可。

> 注意：Kalibr 是 ROS1 工具，没有 apt 安装包（需源码编译或 docker），且只读取 ROS1 格式的 bag。本课程的 ROS2 bag 需先用 rosbags 库（pip install rosbags）转换为 ROS1 bag 才能交给 Kalibr，转换方法见其官方 wiki《ROS2 Calibration Using Kalibr》。若没有多相机 / 相机-IMU 联合标定需求，可跳过本节：本课程范围内 camera_calibration 已足够。

![AprilGrid 标定板参数示意：6×6 排列的 AprilTag，标注 tagCols/tagRows、tagSize、tagSpacing 三个关键参数，并对比棋盘格与 AprilGrid 的差异](./images/DK6Sbh18Vo4TuIxeSi8cQ5mQnQd.png)

*AprilGrid 标定板参数示意：6×6 排列的 AprilTag，标注 tagCols/tagRows、tagSize、tagSpacing 三个关键参数，并对比棋盘格与 AprilGrid 的差异*

步骤 1：准备标定板配置文件（如 $aprilgrid.yaml$）：

*Kalibr AprilGrid 配置示例*

```yaml
target_type: 'aprilgrid'
tagCols: 6
tagRows: 6
tagSize: 0.088
tagSpacing: 0.3
```

步骤 2：录制 ROS bag，包含图像话题（双目则包含左右两路）。

步骤 3：运行标定：

*Kalibr 单目标定命令*

```bash
kalibr_calibrate_cameras \
  --target aprilgrid.yaml \
  --bag calibration.bag \
  --models pinhole-radtan \
  --topics /camera/image_raw
```

输出格式提醒：Kalibr 通常生成 camchain-*.yaml、报告 PDF 和结果摘要；它不是可直接替换 camera_info 的同一种 YAML。若后续 ROS2 驱动只接受 camera_info，请按驱动所需格式转换或手动映射内参、畸变和双目参数。

其中 $--models$ 指定相机模型，$pinhole-radtan$ 表示针孔模型 + 径向切向畸变（对应 $k_1, k_2, p_1, p_2$）。鱼眼镜头可选用 $pinhole-equi$（等距畸变模型）。

### 双目标定

双目标定需要左右相机同步采集图像。使用 $camera\_calibration$ 时，启动命令需同时指定左右话题：

*启动双目标定（优先使用硬件同步）*

```bash
ros2 run camera_calibration cameracalibrator \
  --size 8x6 \
  --square 0.020 \
  --no-service-check \
  --ros-args -r left:=/stereo/left/image_raw \
  -r right:=/stereo/right/image_raw
```

同步原则：优先使用硬件同步，因此上面的标准命令不设置 --approximate。若设备只能软件同步，应先检查左右图像时间戳差，再从较小容差开始（例如 --approximate=0.01）；不要把 0.1 秒作为默认值，因为移动中的标定板在 100 ms 内可能已改变姿态。标定完成后，除各自的内参与畸变外，还会得到相机间 $R$、$T$，以及用于校正的 $R_1/R_2$ 和 $P_1/P_2$。

![双目标定极线校正前后对比：上排校正前极线不水平（对应点y坐标不同，匹配需2D搜索），下排校正后极线水平对齐（对应点在同一水平线，匹配只需1D搜索）](./images/XdOJb1j5IonIHoxDaPCcZcFWnwf.png)

*双目标定极线校正前后对比：上排校正前极线不水平（对应点y坐标不同，匹配需2D搜索），下排校正后极线水平对齐（对应点在同一水平线，匹配只需1D搜索）*

使用 Kalibr 进行双目标定时，只需在 $--topics$ 中列出左右两个话题，并在 $--models$ 中对应指定两个相机模型即可。

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

![图 A：重投影误差概念——绿色十字为检测角点 pᵢ，红色圆圈为重投影角点 p̂ᵢ，连线为像素误差 eᵢ](./images/HtNGbmprMocEb4x6GiYcgCCGnnc.png)

*图 A：重投影误差概念——绿色十字为检测角点 pᵢ，红色圆圈为重投影角点 p̂ᵢ，连线为像素误差 eᵢ*

![图 B：逐帧重投影误差散点图——蓝点为正常帧，红点为异常帧，绿色虚线为平均值，红色虚线为 1.0px 合格线](./images/TQMXb1NhXomDItxO3lDcJEAPn0f.png)

*图 B：逐帧重投影误差散点图——蓝点为正常帧，红点为异常帧，绿色虚线为平均值，红色虚线为 1.0px 合格线*

表中的数值适合作为普通透视相机的起始参考，并非所有镜头和分辨率的统一门槛：超广角、鱼眼或低分辨率图像应结合具体模型判断。无论数值多少，都要逐张检查误差分布；少量图像明显高于平均值，往往意味着运动模糊、角点误检或标定板弯曲，应剔除后重新求解。

### 导出 camera_info YAML

标定完成后，需将参数保存为标准 YAML 格式，供后续模块调用。以下为单目 $camera\_info$ YAML 文件的标准格式：

*camera_info.yaml 示例*

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
| camera_matrix | 内参 K：相机把归一化坐标换成像素的尺度和中心 | 去畸变、PnP、SLAM、测量 |
| distortion_coefficients | 畸变 D：镜头让边缘像素偏离理想位置的规律 | 图像校正 |
| rectification_matrix | 校正旋转 R：让双目图像行对齐的变换 | 立体匹配 |
| projection_matrix | 校正后的投影 P；右相机中还编码基线信息 | 双目深度计算 |

双目场景下，左右相机各保存一份 YAML，右相机的 $projection\_matrix$ 中包含基线信息（$P[0][3] = -f_x \cdot b$，其中 $b$ 为基线长度）。

在 ROS 系统中，推荐通过相机驱动的 $camera\_info\_url$ 参数加载该 YAML（与本课程步骤 2 的用法一致）：例如 $v4l2\_camera$ 指定 -p camera_info_url:=file:///home/seeed/.ros/camera_info/gmsl_cam0.yaml，驱动即按该标定发布 $sensor\_msgs/CameraInfo$ 话题，供图像校正、SLAM 等节点订阅。

### 进阶：四鱼眼外参标定与实时 BEV 拼接（ROS 2）

本节解决的问题：前文内参标定回答“每台相机怎样成像”；本节把前、后、左、右四台相机放到统一的 base_link 坐标系中，求出它们相对车体的位置与朝向，并把四路图像投影、融合为实时鸟瞰图（BEV）。显示与排障全部通过 ROS 2 topic 和 RViz2 完成。

> 适用对象：四台刚性安装的鱼眼/超广角相机，地面近似平坦。相机安装、焦距、分辨率和相机驱动配置一旦改变，必须重新做内参和外参。

#### 先理解：本节的外参是什么

把车体中心定义为 base_link：+X 向右、+Y 向前、+Z 向上。对每台相机，程序先利用已知地面棋盘格角点求出 T_base_camera（相机相对车体的位姿），再计算“去畸变图像 → BEV 画布”的平面单应矩阵 H。

因此，T_base_camera 用于 RViz 的 TF 和坐标关系；H 用于把地面像素落在同一鸟瞰图。这里的 BEV 是地面平面的投影，不能把它当作任意高度物体的完整三维重建。

#### 步骤 1：准备配套 ROS 2 包与配置

本节配套包名为 fisheye_avm_ros，应位于 ~/ros2_ws/src/fisheye_avm_ros。它只订阅 ROS 2 图像，不会直接打开 /dev/video*，因此不会与相机驱动争用设备。

*构建配套包*

```bash
cd ~/ros2_ws
source /opt/ros/$ROS_DISTRO/setup.bash
colcon build --packages-select fisheye_avm_ros
source install/setup.bash
```

编辑 ~/ros2_ws/src/fisheye_avm_ros/config/avm.yaml，逐项核对：四路 image_topic、方向映射、内参来源、棋盘格规格和棋盘格摆位。默认方向映射是 front=0、back=2、left=3、right=1；这只是示例，必须以实际相机安装方向为准。

*avm.yaml 中最需要核对的字段*

```yaml
cameras:
  front:
    image_topic: /cameras/front/image_raw
    camera_info_topic: /cameras/front/camera_info
    intrinsics_file: /home/seeed/fisheye-avm-calib/calib_results/front.json
calibration:
  board: {pattern_size: [8, 6], square_size_m: 0.025}
  placements:
    front: {near_m: 0.35, lateral_m: 0.0, orient: long-lateral}
bev:
  base_frame: base_link
  scale_px_per_meter: 200.0
  canvas_size: [800, 800]
```

> 内参模型检查：包优先支持 equidistant/fisheye 的 4 参数模型；也能读取普通 ROS plumb_bob，但超广角鱼眼在图像边缘的精度可能不足。无论内参来自 CameraInfo 还是 YAML/JSON，标定分辨率必须与实时图像完全一致。

#### 步骤 2：用地面棋盘格采集四路外参

每次只为一个方向摆放棋盘格：将棋盘格平铺在地面，按 placements.<方向>.near_m 和 lateral_m 用卷尺量到 base_link。四个方向都采集后再统一求解。测量的摆位误差会直接表现为拼接接缝误差。

*终端 1：启动外参标定节点*

```bash
source /opt/ros/$ROS_DISTRO/setup.bash
source ~/ros2_ws/install/setup.bash
ros2 launch fisheye_avm_ros avm_calibrate.launch.py \
  config:=~/ros2_ws/src/fisheye_avm_ros/config/avm.yaml
```

*终端 2：依次锁定四个方向的棋盘格*

```bash
ros2 topic pub --once /avm/calibration/capture std_msgs/msg/String "{data: front}"
ros2 topic pub --once /avm/calibration/capture std_msgs/msg/String "{data: back}"
ros2 topic pub --once /avm/calibration/capture std_msgs/msg/String "{data: left}"
ros2 topic pub --once /avm/calibration/capture std_msgs/msg/String "{data: right}"

# 四路都显示 capture locked 后才求解
ros2 service call /avm/calibration/solve std_srvs/srv/Trigger "{}"
```

求解成功会写入 ~/.ros/fisheye_avm/extrinsics.yaml。文件包含每路 T_base_camera、单应矩阵 H、位姿重投影 RMS 和 BEV 重投影 RMS；任一方向 RMS 超过配置阈值时不会覆盖正式结果。

#### 步骤 3：在 RViz2 中检查标定，而不是只看“求解成功”

*启动配套 RViz 布局*

```bash
source ~/ros2_ws/install/setup.bash
rviz2 -d $(ros2 pkg prefix fisheye_avm_ros)/share/fisheye_avm_ros/rviz/avm.rviz
```

- 查看 /avm/calibration/front|back|left|right/overlay：棋盘格内角点应完整覆盖且角点顺序稳定。
- Fixed Frame 设为 base_link：/avm/calibration/markers 显示车体中心和四块棋盘格的实测位置；TF 显示求出的相机位姿。
- 查看 /avm/diagnostics：必须没有“内参/图像分辨率不一致”“未检测到棋盘格”或 RMS 超阈值的 ERROR。

#### 步骤 4：启动实时 BEV 拼接

*终端 1：启动实时拼接*

```bash
# 仅在已安装 CUDA OpenCV 时使用实时主线
source /home/seeed/fisheye-avm-calib/scripts/env_opencv_cuda.sh
source /opt/ros/$ROS_DISTRO/setup.bash
source ~/ros2_ws/install/setup.bash
ros2 launch fisheye_avm_ros avm_bev.launch.py \
  config:=~/ros2_ws/src/fisheye_avm_ros/config/avm.yaml
```

在 RViz2 添加 Image 显示项并选择 /avm/bev/image_raw；还可查看单路 /avm/front|back|left|right/bev 来定位接缝问题。节点只融合时间差不超过 max_sync_delta_sec（默认 30 ms）的四帧，避免移动时把旧帧与新帧拼在一起。

> Jetson 当前验证状态：配套包已在 Jetson ROS 2 Humble 上构建并可启动；但当前设备检测到的 Python OpenCV 为 4.5.4、CUDA 设备数为 0，尚不具备 CUDA 实时拼接条件。默认实时节点会明确报错而不是静默降级。仅做 topic/几何调试时可加 allow_cpu:=true，但这不是实时性能验收。

#### 验收清单与排障方向

| 检查项 | 通过标准 | 不通过时优先检查 |
| --- | --- | --- |
| 外参结果 | extrinsics.yaml 含四路 H、T_base_camera 与 RMS | 棋盘格尺寸、near_m/lateral_m、方向映射 |
| 几何一致性 | RViz 中棋盘格位置、车体 Marker 与相机 TF 方向合理 | base_link 轴向、棋盘格朝向、相机前后左右是否填反 |
| 拼接效果 | 地面线条在接缝处连续，车辆静止时无明显错位 | 内参模型/分辨率、摆位测量、四路时间同步、曝光差异 |
| 实时性能 | diagnostics 显示 CUDA 后端且 BEV 连续发布 | CUDA OpenCV、环境脚本、图像分辨率与 Jetson 资源 |

边界提醒：本节不控制底盘或机械臂，也不把 BEV 当作激光雷达地图；它提供的是统一地面视角，为后续占据栅格、目标定位和导航/抓取感知提供可靠输入。

## 产出物与验收标准

不要只以“工具显示成功”作为结束。完成标定后，请依次检查：参数文件能否被驱动加载、直线去畸变后是否更直、单目重投影误差是否合理、双目对应点是否落在同一水平行。四项都通过，才说明这份标定能被后续系统可靠使用。

### 交付清单

1. 标定参数 YAML 文件：单目 1 份，双目 2 份（left.yaml / right.yaml），格式符合上述标准。
1. 标定精度报告：包含重投影误差（总体 RMS + 逐帧分布）、内参矩阵、畸变系数、双目外参（如有）、采集图像数量与有效帧数。
1. 原始数据（可选但建议保留）：标定用图像集或 ROS bag，便于后续复现与优化。

### 验收标准

- 普通透视镜头可将重投影 RMS ≤ 1.0 像素作为起始参考；超广角、鱼眼及高精度测量任务应结合镜头模型、分辨率、误差分布和实际应用验证，而非只看单一阈值。
- YAML 文件字段完整，可被 $image\_proc$ 或相机驱动（$camera\_info\_url$）正常加载。
- 双目校正后极线水平对齐，左右图像对应点行坐标差 ≤ 1 像素。
- 精度报告数据完整，误差分布无异常离群帧。

## 常见问题与排障

### 角点检测失败或不稳定

- 原因：标定板过小、图像模糊、光照不足或过曝、棋盘格对比度不够。
- 解决：增大标定板在视野中的占比，确保图像清晰，调整光照避免反光，使用高对比度标定板。

### 重投影误差偏高

- 原因：采集数据姿态单一、存在运动模糊帧、镜头自动对焦导致焦距变化、标定板格距测量不准确。
- 解决：增加采集图像的姿态多样性，逐帧检查并剔除高误差帧，锁定相机焦距与曝光，重新精确测量格距。

### 双目校正后极线不对齐

- 原因：左右相机时间同步误差大、双目标定数据不足、相机间存在柔性振动。
- 解决：使用硬件同步触发或减小 $--approximate$ 容差，增加双目标定图像对数，确保相机刚性固定。

### Kalibr 运行报错或不收敛

- 原因：bag 文件中图像话题不连续、标定板配置与实际不符、图像分辨率过高导致内存不足。
- 解决：检查 bag 话题与帧率，核对 $aprilgrid.yaml$ 参数，降低图像分辨率或使用 $--dont-show-extract$ 选项。
> 下一步：完成本课程标定后，可进入 M3（视觉 SLAM）或 M4（目标检测）模块，将 $camera\_info$ YAML 配置到对应算法中。若涉及机械臂视觉，请继续学习 M9 手眼标定与视觉伺服。

