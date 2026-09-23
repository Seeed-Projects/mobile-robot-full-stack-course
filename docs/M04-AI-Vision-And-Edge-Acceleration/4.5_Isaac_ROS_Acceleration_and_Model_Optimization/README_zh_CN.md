# 4.5 Isaac ROS 加速与模型优化实战

**状态：PLANNED。当前没有可运行实现。** 本章是集成与验收设计，不代表 Isaac ROS、NITROS、DLA 或 INT8 已经在 Jetson A 上落地。完整状态以 [`code/PROJECT_STATUS.md`](../code/PROJECT_STATUS.md) 为准。

## 课程概述

单模型跑得动，不等于多模块能一起跑。目前 M4.1、M4.2 为 `PASS`，M4.3 为 `VERIFIED`，M4.4 因运行时、权重和 RGB-D 输入不齐仍为 `BLOCKED`。规划中的「检测 → 分割 → 跟踪」管线需要重新测量拷贝、显存争抢与每帧 kernel 启动开销；现有 Hub 三模块切换不能算 Isaac ROS 多模型并行验收。

候选路线是先建立可复现的多模块基线，再评估 NITROS、固定输入形状、INT8、CUDA Graph 和 DLA 等选项。每一项都要有独立正确性与性能证据。本页给出实施顺序和验收设计；目前没有对应的 Isaac ROS 工作空间、可运行 launch 或优化报告。

基于版本：https://nvidia-isaac-ros.github.io/v/release-3.2/getting_started/index.html

ISAAC ROS version 3.2

## 先知道：这节课会带你完成什么

| 阶段  | 你会理解什么                                             | 最终能做什么                         |
| --- | -------------------------------------------------- | ------------------------------ |
| 读懂  | NITROS 零拷贝省掉的到底是哪一次拷贝，以及为什么必须跑在同一个进程里              | 能指着一条管线说出哪些话题还在走 D2H / H2D 拷贝  |
| 看透  | 同一个 Engine 在 FP16、INT8、CUDA Graph、DLA 四种选择下的收益边界   | 能说出某个优化在这条管线上为什么不生效，而不是照抄别人的参数 |
| 规划  | 检测与分割如何接入同一候选管线，跟踪如何消费检测输出 | 列出待开发组件及其真实 ROS 接口 |
| 定义验收 | trtexec、nsys、jtop、话题探针各自回答什么问题、各自的盲区在哪 | 写出未来优化报告的测量条件与通过标准 |

候选管线可以拆成四段：相机输入与预处理 → NITROS 传输 → 多模型推理 → 后处理与发布。实施时应先测基线，再逐项修改并用同一视频、同一预热帧数复测。本页所有性能门槛均为未来验收目标，没有本机实测通过记录。

## 规划完成后应能做到什么

- 搭出 Isaac ROS 开发容器，编译指定功能包，并跑通一个官方推理示例。

- 把 4.1–4.4 的模型统一导出成固定形状 ONNX，用 onnxsim 简化；检测与分割建成 FP16 与 INT8 两份 Engine，姿态模型按官方口径建 FP32 Engine。

- 写出多模型统一管线的 launch 与参数配置，让检测、分割、跟踪在同一进程内共享 GPU 内存。

- 用 CUDA Event 与话题时间戳测出端到端延迟的 p50 / p95 / p99，并分清预处理、推理、后处理各占多少。

- 完成 INT8 校准，用混合精度把掉点控制在 3% 以内，并判断某个模型该不该卸载到 DLA。

- 输出一份别人能照着重跑的优化报告：同一份视频、同一预热帧数、三次取中位数。

- 说清本页与 M11 部署优化与工程化 中「TensorRT / ONNX 模型优化与量化」部分的分工：本页调的是四个感知模型与管线本身，系统级部署与量产工程不在本页。

硬件与软件清单

- 平台：reComputer Robotics J5011（Jetson AGX Orin 32GB)
- JetPack 6.2.1
- ROS 2 Humble
- GMSL摄像头 / USB 摄像头



## 前置基础

- **当前基线**：M4.1 检测、M4.2 跟踪和 M4.3 分割可供未来集成；M4.4 尚未真实推理，不能列入已迁移模型。实施前先复核 [`code/PROJECT_STATUS.md`](../code/PROJECT_STATUS.md)。

- **系统与容器**：[1.2 JetPack 6.2 系统刷机与基础配置](https://seeedstudio.feishu.cn/docx/UweQdPUKYobmfMxYjZkcy1ZpnNh) 里完成过刷机与 TensorRT 可用性验证；[1.3 容器化开发环境与远程工具链](https://seeedstudio.feishu.cn/docx/Yab1dMx93oHkKRxzP59cV9KunDb) 里已有支持 GPU 直通的 Docker 环境；[1.4 机器人软件中间件：ROS2 Humble 快速上手](https://seeedstudio.feishu.cn/docx/QdL7dbITroR6btxqesrcNJE9nib) 里 ROS 2 节点、话题与跨机通信已经跑通。

- **通用基础**：能在命令行下改配置文件、能读 `colcon` 的构建日志（英文报错能定位到包名与文件），会用 `jtop` 或 `tegrastats` 看一眼 GPU 占用与温度。

> 监控工具先装好再往下做。本页几乎每个结论都要靠 `jtop` 或 `tegrastats` 读出的数字来判定：没有读数，你无法区分「管线真的变快了」和「这次测试恰好跑在更凉快的环境里」。

## 先读懂：一条 GPU 管线里，数据到底走了几趟

### Isaac ROS 把计算派给谁：GPU、DLA 与 PVA

Isaac ROS 是 NVIDIA 面向机器人应用的 ROS 2 节点集合，对外是标准 ROS 2 话题与服务，对内把计算放到 Jetson 的加速引擎上。要读懂后面的优化选项，先要分清这些引擎各自擅长什么。

除 GPU 之外，Jetson AGX Orin 还带 DLA（Deep Learning Accelerator，深度学习加速器）与 PVA（Programmable Vision Accelerator，可编程视觉加速器）。DLA 是面向推理的固定功能加速器，擅长卷积、ReLU、池化这类规整算子；PVA 面向视觉流水线里规则化的图像处理；GPU 什么算子都能跑，但在大规模卷积上能效比不如 DLA。所谓硬件加速不是把所有计算都推给 GPU，而是把不同形状的算子放到更合适的那一个引擎上。

| 引擎  | 适合什么                                        | 不适合什么                         |
| --- | ------------------------------------------- | ----------------------------- |
| GPU | 任意算子、动态形状、自定义 Plugin；多个模型用不同 CUDA Stream 并行 | 小算子的启动开销占比高；长时间满载时功耗与温度上升最快   |
| DLA | 规整的卷积、ReLU、池化；INT8 或 FP16 下的骨干网络            | 不支持的层会回退到 GPU；回退点两侧有格式转换与同步代价 |
| PVA | 规则化的图像处理，如固定模式的像素级运算                        | 通用张量计算与训练相关算子                 |

**一句话记忆：**GPU 回答「什么算子都能跑」；DLA 回答「规整卷积跑得更省电」；PVA 回答「规则化的图像处理不必占用 GPU」。

### NITROS：零拷贝省掉的到底是哪一次拷贝

裸 ROS 2 里图像从一个节点传到下一个节点，要走一条固定路径：GPU 显存 → CPU 内存（D2H 拷贝）→ 序列化 → 进程间或网络传输 → 反序列化 → 回到 GPU（H2D 拷贝）。这条路径每一步都有开销，先算一下量级。

一张 1080p、3 通道、8 位图像的单帧字节数：

$B_{frame} = 1920 \times 1080 \times 3 \approx 6.2\ \text{MB}$

如果每一跳都往返一次，30 FPS 下仅拷贝这一段就产生：

$B_{copy} = 6.2\ \text{MB} \times 30 \times 2 \approx 373\ \text{MB/s}$

373 MB/s 本身不是瓶颈，LPDDR5 的带宽远高于此。它真正吃掉的是 CPU 周期、内存带宽占用，以及每一帧固定的一次同步等待；在多模型管线里，这段等待会直接堆到端到端延迟的尾部。NITROS（NVIDIA Isaac Transport for ROS）基于 ROS 2 的类型适配（Type Adaptation）与进程内通信（Intra-Process Communication），让两个节点之间直接传 GPU 内存句柄，配合 CUDA Event 做同步，跳过 D2H / H2D 拷贝与序列化。

| 通信路径          | 数据怎么走                                   | 什么时候会走到这条路径                                           |
| ------------- | --------------------------------------- | ----------------------------------------------------- |
| 标准 ROS 2 跨进程  | D2H 拷贝 → 序列化 → 传输 → 反序列化 → H2D 拷贝       | 收发节点不在同一进程、消息类型未协商、QoS 不匹配                            |
| 共享内存（同机跨进程）   | GPU 数据先落到 CPU 共享内存，订阅端仍然要拷回 GPU         | 两个 NITROS 节点分别跑在不同进程或不同容器里                            |
| NITROS 进程内零拷贝 | 直接传 GPU 内存句柄，CUDA Event 同步，不做 D2H / H2D | 收发双方在同一进程（同一个 ComposableNodeContainer），且都支持 NITROS 类型 |

零拷贝要成立，三个条件必须同时满足：收发双方加载在同一个进程里；双方都支持 NITROS 类型（例如 `NitrosImage`、`NitrosTensorList`）；话题走的是支持协商的类型与 QoS 配置。任意一条不满足，管线就会静默退回上面两行里的某一条路径——功能照跑，延迟却回到优化前。

验证只在读话题时做，不看包名：拉出话题的类型信息，确认 negotiated type 落在 NITROS 类型上，而不是 `sensor_msgs/Image`。

### isaac_ros_dnn_inference：把推理封装成一个节点

这个包（3.x 线里的 `isaac_ros_dnn_inference`，apt 包名形如 `ros-humble-isaac-ros-dnn-inference`）把 TensorRT 推理包成一个标准 ROS 2 节点：输入是图像（NITROS 图像或 `sensor_msgs/Image`），输出是张量列表（`isaac_ros_tensor_list_interfaces/TensorList`），每个张量带 `name`、`shape`、`data_type`、`strides` 与数据指针。你不再写 TensorRT 的 context、binding 与显存分配代码，只填 Engine 路径和各张量的名字。

| 参数（3.x 线写法）[适用版本待核]（具体键名随发行版变化，未逐版比对） | 它决定什么                                           |
| ------------------------------------- | ----------------------------------------------- |
| Engine 文件路径                           | 加载哪一个 `.plan`；是否允许节点在 Engine 缺失时重建              |
| 输入张量名与绑定名                             | 把输入图像填到 Engine 的哪个输入上；名字必须与 ONNX 里的名字一致         |
| 输出张量名与绑定名                             | 把哪些输出暴露成 TensorList，后处理节点按名字取                   |
| 输入尺寸与预处理张量名                           | 预处理阶段的缩放目标，以及预处理结果写进哪个张量名；必须与 Engine 构建时的固定形状一致 |
| 预处理参数                                 | 归一化均值与标准差、通道顺序、是否交换 R 与 B；填错会让模型「能跑但结果全错」       |

这张表只给中文语义、不给具体键名，是因为键名本身是版本相关的：不确定时用 `ros2 param list <节点名>` 打印当前节点实际接受的参数，再照抄到 YAML 里，比照搬网络示例可靠。

节点只负责推理，不负责后处理。YOLO 的非极大值抑制（Non-Maximum Suppression, NMS）、分割的 Argmax 与颜色映射、姿态估计的解码都要你自己写一个订阅 TensorList 的节点。这正是本页与「直接用 Ultralytics 推理」的分界：推理交给标准节点，后处理留在你手里，4.1–4.4 写好的后处理逻辑可以复用，只需要换掉取数据的那一段。

复用 4.1 后处理时，只需要把 TensorList 转成字典；张量名与 4.1 导出 ONNX 时保持一致

```python
import numpy as np

def tensor_list_to_numpy(tensor_list):
    """把 isaac_ros_dnn_inference 的输出转成 {name: ndarray}，其余逻辑照用 4.1 的。"""
    out = {}
    for t in tensor_list:
        arr = np.asarray(t.data)                 # 接口给的是一维数据，按 shape 还原
        out[t.name] = arr.reshape(t.shape)
    return out

tensors = tensor_list_to_numpy(msg.tensors)
pred = tensors["output0"]                        # 形状由导出时的输入分辨率决定
boxes = m4_1_yolo_postprocess(pred)              # 直接复用 4.1 的实现
```

### ONNX 图与输入形状：什么时候固定，什么时候留动态轴

TensorRT 的优化都是在「已知形状」的前提下做的：知道形状才能融合层、才能为每层挑 kernel。所以输入形状不是导出时的一个随手选项，它决定了引擎能优化到什么程度。

| 选择          | 引擎能做什么                                   | 代价与适用场景                                   |
| ----------- | ---------------------------------------- | ----------------------------------------- |
| 固定所有维度      | 层融合与 kernel 选择最激进；可与 CUDA Graph 配合       | 换分辨率就要重新导出重建；本页默认这条路                      |
| 只留 batch 动态 | 推理时按实际 batch 选择，但每个 shape 首次都要重新选 kernel | 适合离线批量处理；实时场景 batch 通常为 1，收益有限            |
| 宽范围动态形状     | 只能按 min / opt / max 三个形状优化，中间形状靠插值配置     | 首次推理会多出一次 kernel 选择时间，且部分层无法用 INT8 kernel |

导出后先做图简化再建 Engine。onnxsim 会去掉常量折叠后残留的恒等算子与冗余节点，TensorRT 拿到更干净的图，后续的层融合才有空间；不简化的话，你会在 nsys 的时间线里看到一串没有实际计算的 kernel，白白占掉启动开销。

当模型里出现 TensorRT 不支持的算子时，只有两条路：把它改写成受支持的算子组合，或者写一个 TensorRT Plugin 把自定义算子的 CUDA kernel 接进引擎。Plugin 的门槛在于三件事必须同时写对——算子原型（怎么描述这个层、怎么校验输入）、序列化与反序列化（参数怎么存进 Engine）、输出形状推理。只有在无法改写时才走 Plugin；TensorRT 10 引入了 `IPluginV3`，并弃用 `IPluginV2` 系列接口（含 `PluginVersion` 与 `PluginCreatorVersion` 枚举），8.x 时代的示例需按官方 `sample_plugin_v2_to_v3_migration` 迁移到 V3，不能直接照抄。弃用不等于加载不了，但新写的 Plugin 应当直接落在 V3 上。

每次改模型后的标准动作：先简化 ONNX，再造 Engine，并复用计时缓存减少重复构建时间

```bash
onnxsim m4_det_best.onnx m4_det_best_sim.onnx
trtexec --onnx=m4_det_best_sim.onnx --saveEngine=m4_det_fp16.plan --fp16 --memPoolSize=workspace:4096 --timingCache=m4.timing
ls -lh m4_det_best_sim.onnx m4_det_fp16.plan
```

上面用的是 `--memPoolSize=workspace:4096`，不是 `--workspace=4096`：后者的解析在 TensorRT 8.6 里还在，到 10.x 已被移除，写成旧形式 trtexec 会直接报未知选项退出。不带单位时该值按 MiB 解释，4096 即 4 GiB 工作显存上限。4.1 用的是同一形式，两页保持一致。

### 量化：FP16 是默认档，INT8 要自己证明精度

FP16 是 Jetson 上代价最低的一档：权重与激活都用半精度，Tensor Core 原生支持，多数模型掉点可以忽略，相对 FP32 的速度提升立刻可见。它也有代价：某些逐元素运算在 FP16 下会累积误差，检测头最后一层的置信度分数容易抖动。本课程给 FP16 留的精度预算是相对 FP32 掉点不超过 1%。

「开 FP16 就一定更快、更省」只对一部分模型成立。官方 FoundationPose 文档页写明：由于 FP16 精度损失，FoundationPose 的 TensorRT Engine 在 TensorRT 10.3 及之后的版本里以 FP32 精度运行（官方原句："FoundationPose TensorRT engines are running with FP32 precision in TensorRT 10.3+ versions due to FP16 precision loss"）；3.2 版本化页同样载明该句，且 Isaac ROS 3.2 Update 14 把 isaac_ros_common 的 tensorrt 依赖 pin 到 10.3.0，所以这条结论在 3.x / Humble 线上同样成立。也就是说，4.4 的 6D 姿态模型在本页不参与 FP16 与 INT8 量化，只做形状固定；量化收益必须按模型逐个确认，不能给整条管线统一套一组参数。

INT8 完全不同：它不会自动变准，必须用校准（Post-Training Quantization, PTQ）从一批代表性输入里统计每一层激活的动态范围，再据此定 scale。量化后的精度损失这样定义：

$\Delta_{mAP} = \frac{mAP_{ref} - mAP_{int8}}{mAP_{ref}} \times 100\%$

本课程给 INT8 的预算是掉点不超过 3%。一旦超标，先怀疑校准集，再怀疑具体的层。校准集的硬要求是「与部署场景同分布」：100–500 张、覆盖光照变化与目标尺度变化、不要从训练集里随手挑几张凑数，更不要用测试视频的截图反哺自己。

- **先确认模型能不能量化**：以 4.4 的姿态模型为例，官方口径是 FP32（原因是 FP16 精度损失），这类模型不要硬上 INT8；量化前先查官方对该模型给出的精度说明。

- **混合精度**：把对精度最敏感的层（检测头输出的最后一层卷积、分割的 logits 层）保留 FP16，其余走 INT8。逐层放开比整模型回退更划算，代价是你需要一份逐层精度对比记录。

- **量化感知训练（Quantization-Aware Training, QAT）**：PTQ 怎么调都收不回来时才用，代价是要重训与额外的训练管线；本页默认走 PTQ，QAT 作为兜底。

- **先固定形状再量化**：动态形状会让部分层拿不到 INT8 kernel，TensorRT 会把它退回 FP16，结果就是「量化了但不是所有层都量化」，速度提升远低于预期。

**一句话记忆：**FP16 的矛盾是「几乎免费」与「不够快」，解法是 INT8 加混合精度，而不是把整个模型压到 INT8。

### CUDA Graph 与 DLA 卸载：收益靠什么撑起来

CUDA Graph 把一帧推理里成百上千次 kernel launch 记录成一张图，之后一次提交、一次同步。它的收益上限就是 kernel 启动开销在单帧总时间里的占比：模型小、batch 为 1、层数多而每层很薄时，启动开销占比高，收益明显；大 kernel 已经吃满算力时，收益只剩几个百分点。还有一条硬限制：Graph 一旦捕获，输入形状与显存地址就固定了，动态形状的 Engine 不能直接套用。

DLA 的收益同样有条件。DLA 支持的算子有限，不被支持的层会回退到 GPU 执行；回退本身不算慢，慢的是回退点两侧的格式转换与额外同步。回退比例可以这样算：

$r_{fallback} = \frac{n_{fallback}}{n_{total}}$

判断方法：回退层占比很低、且集中在网络首尾时，DLA 与 GPU 的分工才值得做；如果骨干网络中间就散落着回退层，整条执行路径会变得很碎，端到端反而变慢。不要只看 DLA 那部分 kernel 变快了，要看整帧时间有没有降。

**一句话记忆：**CUDA Graph 回答「启动开销占多少」，DLA 回答「回退点在哪里」；两个问题都没测清之前，卸载只是盲调。

### 性能观测：四个工具各自回答什么问题

同一台机器上四个人报出四个不同的 FPS，通常不是谁测错了，而是他们测的根本不是同一段。先分清楚每个工具测的是哪一段，再谈数字。

| 工具                | 能回答                                               | 不能回答                        | 典型用法                         |
| ----------------- | ------------------------------------------------- | --------------------------- | ---------------------------- |
| trtexec           | 单个 Engine 的纯推理延迟与吞吐，含 GPU Compute Time 与中位数       | 预处理、后处理、话题传输的开销；多模型争抢时的表现   | `--loadEngine=` 加载产物测纯推理基线   |
| nsys              | GPU 时间线上谁在跑、哪里有空档、是 compute-bound 还是 launch-bound | 看不到 CPU 侧后处理的实现问题，也看不到温度与功耗 | 对整条管线录一次 profile，找最大的一段时间空档  |
| jtop / tegrastats | GPU 利用率、显存占用、功耗、温度、时钟频率                           | 延迟分布与单帧耗时；也分不清哪一层慢          | 测试期间持续采样，落成 CSV 与延迟数据一起存档    |
| 话题探针              | 端到端延迟与发布频率，也就是用户真正感知到的那一段                         | 内部是哪一段慢；丢帧时读数会偏高            | `ros2 topic hz` 看频率，自写节点算分位数 |

延迟与吞吐的定义要先说清，否则四个工具的数字无法互相解释。端到端延迟按「同一帧」算，吞吐按「一段时间」算：

$L(n) = t_{out}(n) - t_{cap}(n)$

$\text{FPS} = \frac{N}{t_{out}(N) - t_{out}(1)}$

p95 这类分位数不是平均值：把 N 帧的延迟排序，取第 $k = \lceil 0.95N \rceil$ 个值。实时系统里决定卡不卡的是尾部延迟，所以验收只看 p95 与 p99，平均值仅作参考。

## 计划实验：把已验证模块接入候选 NITROS 管线

以下命令与配置仅为未来实施草案，当前快照没有 `m4_isaac_pipeline` 包或可运行的 4.5 launch。执行前需先完成环境兼容性、包实现和接口验收。

五个步骤按依赖顺序排列，每一步都产出能被下一步消费的东西：环境 → Engine → 管线 → 优化 → 报告。每一步的读数都要留档，最后一步的报告就是这些读数的汇总；中间跳过某一步的读数，后面就无法把收益归因到具体的改动。

### ENV SETUP

> 根据自己所在的区域选择不同的源配置，由于之前章节我们已经配置安装过了ROS2，现在我们只需要复制下面的一键配置命令进行额外安装即可！

China CDN

```Bash
bash <<'EOF'
set -e

echo "=== Isaac ROS 3.2 China Repository Setup ==="

CODENAME="$(lsb_release -cs)"
ARCH="$(dpkg --print-architecture)"

echo "Ubuntu codename: ${CODENAME}"
echo "Architecture: ${ARCH}"

if [ "${CODENAME}" != "jammy" ]; then
    echo "ERROR: Isaac ROS 3.2 on Jetson expects Ubuntu 22.04 (jammy)."
    exit 1
fi

if [ ! -f /opt/ros/humble/setup.bash ]; then
    echo "WARNING: /opt/ros/humble/setup.bash not found."
    echo "Isaac ROS 3.2 expects ROS 2 Humble."
else
    echo "ROS 2 Humble detected."
fi

sudo apt-get update
sudo apt-get install -y \
    gnupg \
    wget \
    curl \
    ca-certificates \
    lsb-release \
    software-properties-common

sudo add-apt-repository -y universe

# Isaac ROS GPG key - NVIDIA China CDN
wget -qO- https://isaac.download.nvidia.cn/isaac-ros/repos.key \
  | gpg --dearmor \
  | sudo tee /usr/share/keyrings/isaac-ros.gpg >/dev/null

# Isaac ROS 3.x repository
echo "deb [signed-by=/usr/share/keyrings/isaac-ros.gpg] https://isaac.download.nvidia.cn/isaac-ros/release-3 ${CODENAME} release-3.0" \
  | sudo tee /etc/apt/sources.list.d/isaac-ros.list >/dev/null

sudo apt-get update

echo
echo "=== Isaac ROS repository configured ==="
echo
echo "Repository:"
cat /etc/apt/sources.list.d/isaac-ros.list

echo
echo "Available Isaac ROS packages:"
apt-cache search '^ros-humble-isaac-ros-' | head -30 || true

echo
echo "DONE."
EOF
```

US CDN

```Bash
bash <<'EOF'
set -e

echo "=== Isaac ROS 3.2 International Repository Setup ==="

CODENAME="$(lsb_release -cs)"
ARCH="$(dpkg --print-architecture)"

echo "Ubuntu codename: ${CODENAME}"
echo "Architecture: ${ARCH}"

if [ "${CODENAME}" != "jammy" ]; then
    echo "ERROR: Isaac ROS 3.2 on Jetson expects Ubuntu 22.04 (jammy)."
    exit 1
fi

if [ ! -f /opt/ros/humble/setup.bash ]; then
    echo "WARNING: /opt/ros/humble/setup.bash not found."
    echo "Isaac ROS 3.2 expects ROS 2 Humble."
else
    echo "ROS 2 Humble detected."
fi

sudo apt-get update
sudo apt-get install -y \
    gnupg \
    wget \
    curl \
    ca-certificates \
    lsb-release \
    software-properties-common

sudo add-apt-repository -y universe

# Isaac ROS GPG key - NVIDIA International CDN
wget -qO- https://isaac.download.nvidia.com/isaac-ros/repos.key \
  | gpg --dearmor \
  | sudo tee /usr/share/keyrings/isaac-ros.gpg >/dev/null

# Isaac ROS 3.x repository
echo "deb [signed-by=/usr/share/keyrings/isaac-ros.gpg] https://isaac.download.nvidia.com/isaac-ros/release-3 ${CODENAME} release-3.0" \
  | sudo tee /etc/apt/sources.list.d/isaac-ros.list >/dev/null

sudo apt-get update

echo
echo "=== Isaac ROS repository configured ==="
echo
echo "Repository:"
cat /etc/apt/sources.list.d/isaac-ros.list

echo
echo "Available Isaac ROS packages:"
apt-cache search '^ros-humble-isaac-ros-' | head -30 || true

echo
echo "DONE."
EOF
```

### 步骤 1：搭 Isaac ROS 开发容器，跑通官方示例

先得到一个能编译相关功能包、能访问 GPU 的容器环境。官方示例没跑通之前不要动自己的模型，否则后面每个报错都要先花时间判断是环境问题还是模型问题。

在宿主机上准备开发容器；容器内的工作空间挂载到 /workspaces/isaac_ros-dev

```bash
sudo apt-get update
sudo apt-get install -y git git-lfs
git lfs install
mkdir -p /home/seeed/workspace
cd /home/seeed/workspace
git clone -b release-3.2 https://github.com/NVIDIA-ISAAC-ROS/isaac_ros_common.git
cd isaac_ros_common
mkdir /home/seeed/workspace/isaac_ros_ws
./scripts/run_dev.sh -d /home/seeed/workspace/isaac_ros_ws
```

- 工作空间路径走 `-d|--isaac_ros_dev_dir` 选项，不要写成裸位置参数：release-3.2 的 usage 为 `run_dev.sh {-d isaac_ros_dev directory path OPTIONAL}`，只在这个分支里给 `ISAAC_ROS_DEV_DIR` 赋值，裸路径会被 getopt 丢弃并回退到默认目录。可选参数还有 `-v/--verbose`、`-i/--image_key`、`-b/--skip_image_build`、`-a/--docker_arg`。

- 进容器后先确认三件事在 PATH 里：`which trtexec`、`ros2 pkg list | head`、`nvcc --version`。

- 首次构建只编译一个功能包，用 `colcon build --symlink-install --packages-select <包名>` 把编译时间压到可接受范围；一次全量编译容易在某个依赖上卡住。

- 示例跑通后立刻记一次 `ros2 topic hz` 的读数，作为「空管线」的基线；这份基线是后面每一步对照的起点。

### 步骤 2：把 M4.1–M4.4 的模型统一导出为固定形状 ONNX 并建 Engine

产物是检测与分割的 FP16 / INT8 Engine、姿态模型按官方口径的 FP32 Engine，以及一份计时缓存。四页模型的迁移方式并不相同：检测与分割从 Ultralytics 导出最省事；姿态模型的 ONNX 取自 NGC 镜像 `nvidia/isaac/foundationpose` 的 `1.0.0_onnx` 版本（Isaac ROS 3.2 文档的官方 wget 指向 `versions/1.0.0_onnx/files/refine_model.onnx`），4.x 线版本号本页不断言 [适用版本待核]（该版本号未取证），官方因 FP16 精度损失让它跑 FP32（理由见「量化」一节），所以不要给它建 FP16 / INT8 Engine；跟踪节点（4.2 的 ByteTrack / Bot-SORT）是 CPU 侧代码、不需要 Engine，只需和 GPU 节点装进同一个容器并约定话题。导出前先确认各自的输入尺寸与预处理参数。

导出并简化检测模型的 ONNX；分割与姿态模型用同一段脚本按各自的输入尺寸批量跑，产物是 *_sim.onnx

```python
import onnx
import onnxsim
from ultralytics import YOLO

# 固定形状导出：形状与部署时的预处理输出一致；不写 opset 走版本默认
YOLO("m4_det_best.pt").export(format="onnx", imgsz=(1080, 1920), dynamic=False, simplify=False)

model = onnx.load("m4_det_best.onnx")
simplified, ok = onnxsim.simplify(model)
onnx.save(simplified, "m4_det_best_sim.onnx")
print("simplify ok:", ok)
```

对每个 *_sim.onnx 建两份 Engine；INT8 需要先用校准图片生成校准缓存

```bash
trtexec --onnx=m4_det_best_sim.onnx --saveEngine=m4_det_fp16.plan --fp16 --memPoolSize=workspace:4096 --timingCache=m4.timing
trtexec --onnx=m4_det_best_sim.onnx --saveEngine=m4_det_int8.plan --int8 --calib=calib_det.cache --memPoolSize=workspace:4096 --timingCache=m4.timing
trtexec --loadEngine=m4_det_fp16.plan --memPoolSize=workspace:4096
ls -lh m4_det_fp16.plan m4_det_int8.plan
```

- 导出时不写死 `opset`：Ultralytics 的默认映射随 torch 版本变化（torch 2.x 映射到 17，与本页 PyTorch 2.6+ 的口径一致），钉在旧版本反而可能与新算子不匹配；确需固定时，把 opset 与 torch 版本一起写进报告，不要只写其中一个。

- 本页手动跑 `onnxsim`（onnx-simplifier 包），与 4.1 那条路由不同：4.1 走 Ultralytics 的 `simplify=True`，该开关在 ≥8.3 调 `onnxslim`、8.1–8.2 调 `onnxsim`，两者是不同包。两种做法都能合并冗余节点，本页需要显式控制简化时机（先看原图再决定），所以单独跑。

- 分割与姿态模型沿用同一流程，但要单独确认输出张量的名字与数量——后处理节点按名字取张量，名字对不上会直接报空。

- 校准缓存不是 trtexec 自动生成的文件，需要一段校准器脚本读取那 100–500 张图片后写出缓存；接口是 TensorRT 的 `IInt8EntropyCalibrator2`（Python 侧 `trt.IInt8EntropyCalibrator2`，TensorRT 10.x 仍在）。4.1 的 INT8 流程用的是同一套校准器脚本，两页可共用。

- 计时缓存只在首次构建时慢；换 JetPack 或 TensorRT 版本后要删掉重建，否则可能读到不兼容的旧记录。

- 每建好一份 Engine，先用 `trtexec --loadEngine=` 单独测一次纯推理延迟，这个数字是后面所有对照的基线。

### 步骤 3：搭多模型统一管线（检测 + 分割 + 跟踪）

候选设计是在同一进程内复用相机解码，并让检测结果进入跟踪节点。下表左侧是已实现的 M4 接口，右侧 `/m4/*` 只作未来适配器命名示例；尚无对应发布器或重映射 launch。

| 话题                 | 消息类型                                                                                                                                          | 谁发布 / 谁消费                                                                                                                                                                    |
| ------------------ | --------------------------------------------------------------------------------------------------------------------------------------------- | ---------------------------------------------------------------------------------------------------------------------------------------------------------------------------- |
| `/m4/image_raw`    | `sensor_msgs/Image`（协商成功后传输层是 NITROS 图像类型）                                                                                                    | 相机驱动 / 预处理节点与延迟探针                                                                                                                                                            |
| `/m4/detections`   | `vision_msgs/Detection2DArray`                                                                                                                | 检测后处理 / 跟踪节点与延迟探针                                                                                                                                                            |
| `/m4/segmentation` | `sensor_msgs/Image`（彩色掩膜）                                                                                                                     | 分割后处理 / 4.3 的可通行区域分析                                                                                                                                                         |
| `/m4/tracks`（规划命名） | `vision_msgs/Detection2DArray`；当前实际话题是 `/perception/tracks`，track ID 放在 `Detection2D.id`，没有速度或跟踪器状态字段 | `supervision.ByteTrack` 跟踪节点 / 后续消费者 |
| `/m4/pose`（未实现） | 不定义当前消息类型；M4.4 脚手架的主输出契约为 `/perception/object_pose`，`geometry_msgs/PoseStamped` | 等 M4.4 实际推理后再设计适配层 |

未来实现并验证 `m4_isaac_pipeline` 后，才可启动统一管线并测三路频率；以下命令目前不能作为实操步骤：

```bash
ros2 launch m4_isaac_pipeline m4_pipeline.launch.py
ros2 topic hz /m4/detections
ros2 topic hz /m4/segmentation
ros2 topic hz /m4/tracks
```

- 三类节点必须装进同一个 ComposableNodeContainer 才有进程内零拷贝；分成多个 `ros2 run` 启动，只会得到共享内存路径。

- 检测与分割共享同一次预处理输出（缩放、归一化、通道顺序），不要再各自跑一遍图像预处理节点，否则拷贝与算力都翻倍。

- 当前跟踪节点使用 `supervision.ByteTrack`，消费 `Detection2DArray` 检测框；未来适配方案必须保留 track ID 与空帧语义。

- 先量出「未优化的多模型管线」读数：三路话题同时 ≥30 Hz 是目标，达不到就把实际数字记下来，下一步逐项改。这个 30 Hz 只对检测 + 分割 + 跟踪成立。

- 若未来引入 `/m4/tracks`，它必须从现有 `/perception/tracks` 显式重映射，消息类型仍为 `vision_msgs/Detection2DArray`；当前没有这个重映射。

- 4.4 的姿态估计不能并进这条 30 Hz 回路：官方 benchmark 里姿态估计节点在 AGX Orin、720p 输入下，**Isaac ROS 3.2 线为 1.54 fps（约 780 ms 一帧）**；4.6 表里同一条是 0.502 fps（约 3800 ms/帧），但那套栈是 JetPack 7.x / ROS 2 Jazzy，不能与本页的 JetPack 6.2.1 / Humble 混用（数据取自 `isaac_ros_benchmark` 的 release-3.2 与 release-4.6 分支，同一台 AGX Orin）。跟踪阶段另算：官方 README 写明用 refine 模型做跟踪时在 Jetson Orin 平台上速度「exceeding 120 FPS」，慢的是姿态**估计**（首帧那一次）而不是跟踪（原句：exceeding 120 FPS at Jetson Orin，引自 isaac_ros_pose_estimation release-3.2/README.md L37。4.x/main 对应 Jetson Thor，3.2 对应 Jetson Orin，故本页只作量级对照）。所以姿态估计做成独立节点、按需触发（首帧估计是秒级），不进实时回路。

- 把姿态节点并进同一个容器前先算显存，两套官方页面给的不是同一个量，别混成一句：**3.2 版本化页**限定在模型转换阶段，写明 free GPU memory space 至少需要 **7.5 GB**；**4.x latest 页**说的是流水线峰值，约为 **7 GB**、建议预留 **≥8 GB**。本页钉在 3.x，转换阶段按 7.5 GB 留，运行阶段参考 7 GB 峰值。Jetson 是统一内存，这份占用要和相机缓冲、其他 Engine 一起算进预算。

- M4.4 当前脚手架的主输出契约是 `geometry_msgs/PoseStamped` 与 `camera_front` 父 TF；它仍为 `BLOCKED`，不能把 Isaac ROS 示例的消息类型当成现有节点输出。

- 容器内做大模型推理时把共享内存调大（`--shm-size=8g` 起步），否则 NITROS 的进程内路径可能因分配失败退回普通路径，而且退回是静默的——功能照跑，只会在延迟数字上体现。

### 步骤 4：按顺序做四层优化

四层优化各留一组可对照的 Engine，另外交出系统层配置。顺序不能颠倒：先固定形状，再量化，再上 Graph，最后才考虑 DLA；每一步都要重测一次，否则你无法把这四层各自的贡献拆开。

DLA 一种构建方式与系统层设置；CUDA Graph 不在这里构建，它是一次加载选项（见下）

```bash
trtexec --onnx=m4_det_best_sim.onnx --saveEngine=m4_det_dla.plan --int8 --useDLACore=0 --allowGPUFallback --memPoolSize=workspace:4096
sudo nvpmodel -m 0
sudo jetson_clocks
```

量 CUDA Graph 的收益：同一份 engine 开与不开，两次都在 trtexec 内跑推理循环

```bash
trtexec --loadEngine=m4_det_fp16.plan --memPoolSize=workspace:4096
trtexec --loadEngine=m4_det_fp16.plan --useCudaGraph --memPoolSize=workspace:4096
```

- 量化范围按模型区分：检测与分割走 INT8 校准，姿态模型保持官方口径的 FP32；给它做 INT8 校准既缺官方支持，也会把精度拉爆。

- INT8 校准：用与部署同分布的图片生成校准缓存后建 Engine，再在验证集上量 mAP。掉点超过 3% 就按混合精度思路逐层放开，而不是整模型退回 FP16。

- CUDA Graph 只对固定输入形状的 Engine 尝试，而且它**不是构建产物**：`--useCudaGraph` 与 `--duration`、`--warmUp`、`--avgRuns` 同属性能/推理选项，作用在该次 trtexec 自己的推理循环上，不会写进 `.plan`。所以不存在「graph 版 engine」这种文件，两个 `.plan` 的读数差会量到接近零。正确做法是上面那两行：同一份 engine 开与不开，差值就是被消掉的 kernel 启动开销。上生产时在推理节点里做捕获（`cudaStreamBeginCapture` / `cudaGraphLaunch`，或 PyTorch 的 `torch.cuda.CUDAGraph`），形状与显存地址固定是前提。收益低于 3% 时不值得为它增加代码复杂度。

- DLA：先读构建日志里被 GPU 接管的层列表，再决定是否上。只对比 DLA 侧 kernel 时间会得出错误结论，必须对比整帧时间。

- 系统层：MAXN 加时钟锁定的目的是让三次测试的读数可比；锁定频率后温度会上升，散热条件不足时反而触发降频，所以这一步要和温度采样一起做。

- 每改一项都要回落到步骤 3 的探针复测，并把「改动 → 指标变化」记成一行；没有对照的优化不能写进报告。

### 步骤 5：基准测试与优化报告

这一步产出一份可复现的对照表。测试口径固定为：同一段 1080p 测试视频（含机器人典型场景）、预热 100–500 帧、连续测 1000 帧、重复 3 次取中位数，全程保持 MAXN 与时钟锁定。

端到端延迟探针：把检测输出与最近一帧采集时刻配对，打印 p50 / p95 / p99

```python
import numpy as np
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from vision_msgs.msg import Detection2DArray

class LatencyProbe(Node):
    def __init__(self):
        super().__init__("m4_latency_probe")
        self.stamp = None
        self.lat = []
        self.create_subscription(Image, "/m4/image_raw", self.on_img, 1)
        self.create_subscription(Detection2DArray, "/m4/detections", self.on_det, 1)

    def on_img(self, msg):
        self.stamp = msg.header.stamp.sec + msg.header.stamp.nanosec * 1e-9

    def on_det(self, msg):
        if self.stamp is None:
            return
        now = self.get_clock().now().nanoseconds * 1e-9
        self.lat.append(now - self.stamp)
        if len(self.lat) >= 1000:
            a = np.array(self.lat)
            self.get_logger().info(
                "n=%d p50=%.2fms p95=%.2fms p99=%.2fms" % (
                    len(a), np.percentile(a, 50) * 1e3,
                    np.percentile(a, 95) * 1e3, np.percentile(a, 99) * 1e3))
            self.lat = []

rclpy.init()
rclpy.spin(LatencyProbe())
```

- 这个探针把「最近一帧采集时刻」与「本次检测输出」配对；管线跟得上时误差在一帧以内。若同时 `ros2 topic hz` 显示掉帧，读数会偏高，不能当成延迟测量结果。

- GPU 利用率、显存、功耗、温度用 `jtop` 采样，间隔全程保持一致；把采样结果与延迟数据一起存进报告目录，缺了条件的数据不能进报告。

- 报告里每个数字都带条件：分辨率、batch、精度模式、是否含预处理与后处理、用什么工具测的。

- 最后把「独立部署 / 多模型未优化 / 逐层优化后」三组放上同一张对照表，并逐行标出该层的收益与代价。

## 规划产出物与未来验收标准

以下产物均未完成，验收数值只是设计目标，不能记作 Jetson A 的实测成绩。

### 交付清单

1. Isaac ROS 工作空间与构建脚本：4.5 用到的功能包、launch 文件、一份可复用的容器启动命令。

2. Engine 与构建脚本：检测与分割各一份 FP16 与一份 INT8；姿态模型一份 FP32 Engine（官方口径，不做 FP16 / INT8 量化）；跟踪节点附其依赖与配置；另附 ONNX 简化脚本与校准缓存生成脚本。

3. 多模型统一管线配置：`m4_pipeline.launch.py`、各节点参数文件、话题与 QoS 对照表。

4. 基准测试脚本与报告：延迟探针、jtop 采样 CSV、三组对照表（独立部署 / 多模型未优化 / 逐层优化后）。

### 验收标准

| 检查项     | 通过标准                                                                                                                                    | 不通过时优先检查                                    |
| ------- | --------------------------------------------------------------------------------------------------------------------------------------- | ------------------------------------------- |
| 三模型并跑   | 模型集合限定为检测 + 分割 + 跟踪（不含 6D 姿态估计）：三路话题同时 ≥30 Hz（1080p、batch=1、FP16、含预处理与后处理，用 `ros2 topic hz` 测 30 s）                                     | 某一话题不发布时，先查节点是否在同一容器、类型协商是否成功               |
| 端到端延迟   | p50 ≤33 ms、p95 ≤40 ms、p99 ≤50 ms（1080p、batch=1、FP16、含预处理与后处理，延迟探针连续采样 1000 帧）                                                           | 先用 nsys 把预处理 / 推理 / 后处理三段拆开，看哪一段占比最大        |
| 吞吐      | 稳定段 ≥30 FPS（同上条件，`ros2 topic hz` 连续观测 30 s，不含启动前 100 帧）                                                                                 | 查后处理是否在 CPU 侧串行、检测与分割是否抢占同一条 CUDA Stream    |
| GPU 利用率 | 稳定运行 60 s 内 `jtop` 采样，GPU 利用率落在 60%–85% 区间且无持续饱和                                                                                        | 持续接近 100% 时降分辨率或上 INT8；长期低于 40% 时查管线是否串行等待  |
| 显存占用    | 检测 + 分割 + 跟踪并行时 `jtop` 读到的内存峰值不超过系统总内存的 70%；若把姿态节点一并加载，它按官方两套口径分别计入预算：模型转换阶段至少 7.5 GB 空闲（3.2 版本化页），流水线峰值约 7 GB、建议预留 ≥8 GB（4.x latest 页） | 接近上限时先降 batch 与输入分辨率，再把不并行的模型改为按需加载         |
| 功耗与温度   | MAXN 下 30 s 平均功耗不超过该档位上限（AGX Orin 的 MAXN 档位上限为 60 W，来源是 NVIDIA 官方规格，不是本站页面）；温度读数在 20 分钟内波动低于 5 °C 且不出现频率跌落                              | 先查散热风扇与散热片安装，再查 `nvpmodel` 档位是否被改动          |
| 零拷贝生效证据 | 话题协商类型为 NITROS 类型；节点在同一容器内；附关闭 NITROS 前后的延迟对照                                                                                           | 协商失败时查容器划分、QoS 与节点加载方式，而不是改模型               |
| 优化前后可复现 | 同一视频、同一预热帧数、三次取中位数，各次 p50 差异低于 10%；报告列出全部条件                                                                                             | 差异过大时先确认 `jetson_clocks` 是否锁定、是否有其他进程占用 GPU |

## 常见问题与排障

### 编译相关功能包时依赖缺失或 CUDA 架构报错

- **原因**：容器里的 ROS 2 发行版与拉到的包版本不匹配，或构建时环境变量 `CUDA_ARCHITECTURES` 为空，导致编译器不知道为哪个架构生成代码。

- **处理**：先确认容器内是 ROS 2 Humble，再重新 source `/opt/ros/humble/setup.bash` 后构建；架构问题按你的模组显式指定（AGX Orin 用 `87`），并检查镜像版本与主机的 JetPack 是否同一代。

- **处理**：构建失败信息里若指向某个未安装的依赖包，一次只装它再重试，不要一次装十个包——依赖顺序错乱会把真正的缺失项埋掉。

### 话题还是 sensor_msgs/Image，零拷贝没有生效

- **原因**：节点不在同一个进程（分别 `ros2 run` 启动）、有一端不支持 NITROS 类型，或 QoS 不匹配导致协商失败。

- **处理**：把收发节点写进同一个 ComposableNodeContainer 的 launch 文件，用 `ros2 node list` 与进程 PID 确认它们确实在同一个进程内；再拉一次话题信息确认协商类型是 NITROS 类型。

- **处理**：如果只是为了对比，临时把管线拆成两段进程，对照延迟差异；差异很小说明瓶颈本来就不在拷贝上，应该去查后处理或推理 kernel。

### 接了 DLA，端到端反而变慢

- **原因**：不支持的层回退到 GPU，回退点两侧出现格式转换与额外同步；层被切得太碎时，调度开销超过 DLA 省下的时间。

- **处理**：读构建日志确认回退层的比例与位置，回退层散落在骨干网络中间时直接放弃 DLA；只在回退集中在首尾时保留。

- **处理**：对比时用整帧延迟与功耗两个指标，而不是 DLA 侧 kernel 时间；如果整帧没有变快而功耗下降，那属于取舍问题，要在报告里写清。

### INT8 校准后掉点远超预期

- **原因**：校准集与部署场景不同分布（数量太少、场景单一、或直接用训练集图片）、输入形状不固定导致部分层退回 FP16、以及个别敏感层动态范围差异大。

- **处理**：换用 100–500 张部署场景实拍图重新校准，覆盖不同光照与目标尺度；确认 Engine 构建时用的是固定形状 ONNX。

- **处理**：仍超标时按逐层精度对比把最敏感的几层固定为 FP16，做成混合精度；最后才考虑 QAT 重训。

### 多模型并行时显存不足或节点被 OOM 杀掉

- **原因**：三个 Engine 同时加载且各自申请工作显存；容器共享内存过小；输入分辨率偏高导致中间张量放大。

- **处理**：统计三份 Engine 加 CUDA context 的实际占用，按 70% 上限做预算；把共享内存调到 `--shm-size=8g` 以上。

- **处理**：仍不够时先降输入分辨率或 batch，再把非必需模型改成按需加载（例如姿态估计仅在需要时启动），而不是把三模型拆成三条独立管线。

> **下一步：**把这条管线与对照表交给两处使用：[M6 决策层：定位导航与路径规划](https://seeedstudio.feishu.cn/docx/LTEkdYEgEo6xqwx0IfYcsfrSnYf) 与 [M8 执行层：云台与主动视觉](https://seeedstudio.feishu.cn/docx/FjJCdzbHFolniOxenGocRGMOnNP) 消费的是 `/m4/tracks` 的稳定 ID 与延迟指标，用来判断跟踪结果能不能直接喂给导航与云台；系统级部署与工程化（生产管线、量化策略选型、监控与升级）见 [M11 部署优化与工程化](https://seeedstudio.feishu.cn/docx/CIKodf7WXoNz1ExFP79cjcIpnQh)，本页只负责四个感知模型在这块板子上的加速落地与前后对照口径。结论有明确边界：所有性能数字都在 J501、MAXN、32GB、JetPack 6.2.1、TensorRT 10.x 与上述测试口径下成立，换平台、换分辨率或改用动态形状之后必须重测；跨 Isaac ROS 大版本引用的数字与接口，本页都标了依据版本——官方文档站渲染的是最新 4.x / Jazzy 线，照抄它的数字会与本页的 3.x + Humble 环境对不上。
