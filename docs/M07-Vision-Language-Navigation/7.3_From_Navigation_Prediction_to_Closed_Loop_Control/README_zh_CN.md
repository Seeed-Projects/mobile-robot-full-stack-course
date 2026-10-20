# 7.3 在Jetson上运行LightNav导航Demo

[English](README_en_US.md) | [中文](README_zh_CN.md)

前文介绍了LightNav如何根据图像和指令生成路径。本章在已经部署好的AGX Orin上运行底盘仿真：输入“找到冰箱并停在旁边”，观察模型预测、小车运动和新的相机画面如何连续更新。

![LightNav视觉语言导航运行演示](assets/demo.gif)

*图 7.3-1，底盘根据语言指令观察房间、预测路径并移动的运行过程。*

本章默认项目、模型权重、两个Python环境、场景和底盘资产已经准备好。首次安装请先完成[实验手册入口B](LAB_GUIDE_zh_CN.md#入口-b新设备准备)；使用自己的CAD时，参考[自定义底盘材料](../code/examples/custom-chassis/README_zh_CN.md)。下面使用已有项目的默认资产配置，不重新安装依赖。

## 7.3.1 在Jetson检查并启动服务

先通过SSH进入Jetson，在 **Jetson终端**检查项目：

```bash
cd /home/seeed/workspace/LightNav-0
bash scripts/jetson_lightnav.sh status
python3 scripts/jetson_sim_demo.py status
```

这里有两个服务：模型服务在8050端口负责GPU推理，Demo在8091端口负责仿真与网页。模型已经就绪时直接复用；Demo已经运行时，也先确认没有其他人正在操作，再使用现有实例。

**只启动尚未运行的服务。** 模型未运行时执行：

```bash
bash scripts/jetson_lightnav.sh start
bash scripts/jetson_lightnav.sh status
```

等待状态显示模型就绪，再启动尚未运行的Demo：

```bash
python3 scripts/jetson_sim_demo.py start --host 127.0.0.1 --port 8091
curl -fsS http://127.0.0.1:8091/api/health
```

健康检查应包含 `ok=true`、`model_ready=true`，仿真状态中的 `physics_alive`、`render_alive` 为true，`frame_age_s` 小于1。数秒后再次检查，模拟时间和画面应继续更新。启动命令返回进程信息后，仍要完成这一步，才能确认页面已经可用。

以上使用原部署目录中的默认资产。若你使用的是迁移后的课程资产，需要在启动时指定配置，具体参数见[实验手册入口A](LAB_GUIDE_zh_CN.md#入口-a已部署设备快速体验)。如果出现文件不存在，先核对资产路径，避免重复安装环境。

## 7.3.2 在电脑打开演示网页

下面的命令在 **自己的电脑终端**执行。把 `YOUR_JETSON_IP` 换成Jetson实际地址，用户名按设备修改。打开两个终端，分别建立两条连接。

终端一：

```bash
export JETSON_HOST=seeed@YOUR_JETSON_IP
ssh -N -o ControlMaster=no -o ControlPath=none \
  -o ExitOnForwardFailure=yes \
  -L 127.0.0.1:18091:127.0.0.1:8091 "$JETSON_HOST"
```

终端二：

```bash
export JETSON_HOST=seeed@YOUR_JETSON_IP
ssh -N -o ControlMaster=no -o ControlPath=none \
  -o ExitOnForwardFailure=yes \
  -L 127.0.0.1:18092:127.0.0.1:8091 "$JETSON_HOST"
```

保持两个终端开启，浏览器访问 **http://127.0.0.1:18091/**

当前页面分别用18091传输画面、18092传输控制，所以两条隧道都需要保留。命令运行后一直等待、没有继续输出是正常现象；如果提示端口被占用，先检查是否已有本机隧道。模型8050由Jetson后端连接，不需要另开浏览器通道。

## 7.3.3 输入一句指令，观察连续导航

确认网页中的第一、第三视角都在刷新，并且模型状态已就绪。输入：

> Locate the refrigerator in the room and stop beside it.

或者你也可以输入自己的任意指令！点击 **Start navigation**。一次启动后，系统会持续采集图像、请求新预测并跟踪可用路径，无需每预测一次就重新点击。

运行时重点看三处：

- **实时画面：** 小车当前看到什么，第三视角中的车身是否在移动。
- **最近预测：** 模型当时使用的观察，以及对应的目标指向和路径。
- **俯视轨迹：** 参考路径与小车实际移动路线的变化。

新预测需要等待计算；已有路径仍然有效时，小车可以继续跟踪。路径用尽、结果过期或触发检查时，页面会显示等待或停止原因。先读取状态，再决定是否重新运行，避免连续点击开始。

**STOP** 用于立即取消当前任务；**RESET** 用于在停车后回到当前配置的起点。重新尝试或修改指令前先STOP，确认已经停车。模型也可能自行输出停止；运行记录中要分别记下模型停止、用户取消或路径检查停车。

## 7.3.4 结束演示

先在网页点击STOP。需要关闭本组Demo时，在Jetson项目目录执行：

```bash
cd /home/seeed/workspace/LightNav-0
python3 scripts/jetson_sim_demo.py stop
```

随后在电脑的两个SSH终端分别按 `Ctrl+C` 关闭隧道。共享模型服务按教师或设备负责人的安排保留，使用启动器管理自己的Demo进程。

如需保留运行结果，可从启动输出或 `status` 返回的 `output` 字段找到本次日志目录。记录指令、终止原因与实际观察即可；详细分析步骤见[实验手册](LAB_GUIDE_zh_CN.md)。

## 7.3.5 遇到问题先检查这几项

| 现象 | 优先检查 |
| --- | --- |
| 网页打不开 | Jetson的Demo是否运行；电脑18091隧道是否建立 |
| 有画面，按钮无响应 | 18092控制隧道是否开启，是否有其他客户端控制任务 |
| 页面提示模型未就绪 | 先查8050模型服务状态；确认就绪后，由负责人重新启动空闲Demo |
| 小车等待或停车 | 读取页面终止原因，区分模型STOP、参考路径失效与碰撞检查 |

先把“服务是否就绪、网页是否连通、模型是否返回、车是否实际运动”逐项确认。首次部署、资产准备与更详细的问题定位，继续查阅[完整实验手册](LAB_GUIDE_zh_CN.md)。

[7.2 LightNav算法](../7.2_Semantic_Grounding_and_Learned_VLN/README_zh_CN.md) · [返回M7](../README_zh_CN.md)
