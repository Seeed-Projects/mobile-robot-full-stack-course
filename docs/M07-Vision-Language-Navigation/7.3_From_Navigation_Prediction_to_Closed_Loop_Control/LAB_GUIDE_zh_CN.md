# M7 实验手册：冰箱任务、Jetson部署与自定义底盘

[English](LAB_GUIDE_zh_CN.md) | [中文](LAB_GUIDE_zh_CN.md)

课堂目标是观察并解释“相机采集→真实模型生成 token→解码路径→控制器跟踪→运动后的新图像”，不要求每条语言指令都成功。模型STOP、危险路径、结果过期和观察失败都应如实保留。课程不连接真实底盘。

先读[7.2算法](../7.2_Semantic_Grounding_and_Learned_VLN/README_zh_CN.md)与[7.3实践](README_zh_CN.md)。本手册保留完整安装命令，课堂主案例为冰箱自由指令；原门框记录仅作可核验token和评分练习。

## 实验路线与准备

- **入口 A**：教师已经部署设备，先核对状态，再打开页面。约10～15分钟准备。
- **入口 B**：新设备，先准备源码、环境、模型和CAD。安装耗时取决于网络、缓存及设备，课前完成，不承诺固定时间。
- **无设备**：使用[真实历史样例](../code/evidence/README_zh_CN.md)完成实验二和五，明确标记离线分析，不声称自己运行了GPU或导航。

每项实验按“操作→预期→失败定位→提交物”完成。教师分配设备时只允许一组控制网页，其余旁观，避免互相取消任务。

### 先确定文件在哪台机器

本手册涉及三类位置： **Jetson 项目目录**运行模型和 Demo；**课程仓库目录**提供工具与离线样例；**自己的电脑**建立 SSH 隧道和打开浏览器。课程仓库可以与项目在同一台 Jetson 上，也可以位于自己的电脑；运行课程工具时，该机器必须能访问相应材料和日志。

进入需要课程工具的终端后，先将占位路径改成该机器上课程仓库的实际位置：

```bash
export COURSE_ROOT=/absolute/path/to/mobile-robot-full-stack-course
export MATERIALS="$COURSE_ROOT/docs/M07-Vision-Language-Navigation/code"
test -f "$MATERIALS/tools/analyze_run.py"
```

预期命令退出码为0；若找不到文件，先定位课程仓库。环境变量只在当前终端及其子进程有效，新开终端要重新设置。入口 A 的服务检查本身不依赖课程仓库，但后续实验二、五使用课程工具时需要完成本步。

## 入口 A：已部署设备快速体验

### A1. 在Jetson检查状态

下列命令在 **Jetson终端**运行，不是在电脑终端。默认部署目录和用户沿用实际验证路径：

```bash
cd /home/seeed/workspace/LightNav-0
bash scripts/jetson_lightnav.sh status
python3 scripts/jetson_sim_demo.py status
ss -ltnp | awk '$4 ~ /:(8050|8091)$/ {print}'
df -h .
free -h
```

`status` 只检查，不能把8050端口打开等同于一次GPU推理通过。模型使用身份记录和ready标记；Demo还需检查线程与新帧。

- 模型已就绪：复用，不重复加载第二份模型。
- 模型未运行：由该设备负责人执行 `bash scripts/jetson_lightnav.sh start`，等待 `status` 报ready。
- Demo已运行：检查任务是否active，不重启他人的任务。
- Demo未运行：按资产来源选择下面**一种**启动方式。

已有原项目资产时：

```bash
python3 scripts/jetson_sim_demo.py start --host 127.0.0.1 --port 8091
```

按入口B准备课程资产时：

```bash
python3 scripts/jetson_sim_demo.py start --host 127.0.0.1 --port 8091 \
  --chassis-config course-assets/dm-h65/config.json \
  --task-config course-assets/task.json
```

默认显示960×540；可显式 `--resolution 640x360` 降低显示负担，模型输入仍为480×270。不要在已运行实例上反复执行start。

```bash
curl -fsS http://127.0.0.1:8091/api/health
```

预期 `ok=true`、`model_ready=true`，`simulation.physics_alive`、`render_alive`均为true，`frame_age_s<1`；`telemetry_alive=true`、`telemetry_error=null`。间隔数秒检查 `sim_time` 和帧计数持续增加。若 `model_ready=false`，当前实现不会靠网页刷新自动重新探测模型；确认8050就绪后，仅由负责人重启本组空闲Demo。

2026-10-15现场只读检查时模型在运行、Demo未启动，不能用旧报告中“已保留运行”的文字替代上述检查。

### A2. 在电脑建立访问通道

下列命令在 **自己的电脑终端**运行。把SSH主机写为自己的设备；`j50-robotics`只是既有机器的SSH别名，不是课程要求。

```bash
export JETSON_HOST=seeed@YOUR_JETSON_IP
ssh -N -o ControlMaster=no -o ControlPath=none \
  -o ExitOnForwardFailure=yes \
  -L 127.0.0.1:18091:127.0.0.1:8091 "$JETSON_HOST"
```

保持第一个终端，在另一个电脑终端运行：

```bash
export JETSON_HOST=seeed@YOUR_JETSON_IP
ssh -N -o ControlMaster=no -o ControlPath=none \
  -o ExitOnForwardFailure=yes \
  -L 127.0.0.1:18092:127.0.0.1:8091 "$JETSON_HOST"
```

浏览器打开 `http://127.0.0.1:18091/`。当前页面在这个地址下用18092承载控制，18091承载画面，两条独立SSH连接避免视频拥塞拖住控制。**不能只开第一条，也不要随意换本机端口却不调整页面约定。** 8050由Jetson后端连接，不需要把模型端口转发给浏览器。

同一受控局域网也可由负责人显式让Demo监听 `0.0.0.0` 后访问 `http://<Jetson-IP>:8091/`；这是另一种连接方式。课程默认采用loopback加隧道，不把更改监听地址混进排障步骤。

## 入口 B：新设备准备

### B1. 源码与材料版本

先取得课程仓库、[最小复现材料](../code/README_zh_CN.md)以及教师单独提供的CAD包。仅克隆LightNav上游缺少课程启动器和底盘适配。

以下在 **新Jetson**运行。示例固定 `/home/seeed/workspace/LightNav-0`，该目录必须尚不存在；已有部署请使用入口A，不覆盖。

```bash
export COURSE_ROOT=/path/to/mobile-robot-full-stack-course
export MATERIALS="$COURSE_ROOT/docs/M07-Vision-Language-Navigation/code"
python3 "$MATERIALS/tools/check_materials.py"
mkdir -p /home/seeed/workspace
cd /home/seeed/workspace
git clone https://github.com/lightorigins/LightNav-0.git
cd LightNav-0
git checkout --detach 3015508b70fb6e30bbd66701e94c9b916dcb8df8
python3 "$MATERIALS/tools/install_overlay.py" --project "$PWD"
python3 "$MATERIALS/tools/install_overlay.py" --project "$PWD" --apply
```

第一次是预检查；只有全部文件与上游基线匹配才复制。已有不同内容会拒绝，不能使用强制覆盖绕过。覆盖层含共享模块，因此这套准备流程只用于新克隆，不对课堂正在运行的项目应用。

### B2. 硬件、空间和uv

目标平台为aarch64、JetPack6.2.1、CUDA12.6、AGX Orin32GB。新部署建议预留至少约30GiB磁盘，另考虑CAD、缓存与录像；模型本身约9.04GiB。这个规划值不是所有安装都够用的保证。新任务至少要有3GiB可用内存、3.5GiB磁盘余量；运行记录也会占空间。

```bash
uname -m
cat /etc/nv_tegra_release
nvcc --version
df -h /home/seeed/workspace
free -h
command -v uv
```

uv已安装时不重复安装。需要安装时使用官方用户态安装器：

```bash
curl -LsSf https://astral.sh/uv/install.sh -o /tmp/lightnav-uv-install.sh
sh /tmp/lightnav-uv-install.sh
export PATH="$HOME/.local/bin:$PATH"
uv python install 3.10.21 3.11.16
```

不修改系统Python/CUDA。冻结安装器的Seeed profile固定使用 `/home/seeed/.local/bin/uv`，模型锁中的两个wheel URL也包含原部署绝对路径。因此本手册直接支持上述用户名和目录；换用户名或目录时，在**新克隆副本**调整profile的uv路径与锁内两个`file://`前缀，保持文件名、版本和哈希不变，保存迁移diff。仅改环境变量不足以迁移该profile。

### B3. 模型环境与GPU验证

从[Seeed JetPack6.1/6.2 PyTorch参考](https://github.com/Seeed-Projects/reComputer-Jetson-for-Beginners/tree/main/3-Basic-Tools-and-Getting-Started/3.5-Pytorch)取得cp310、aarch64、CUDA12.6的两份wheel，放到下列目录。它们未随课程收录，不选x86_64、CPU或其他JetPack版本。

```bash
cd /home/seeed/workspace/LightNav-0
mkdir -p .jetson-lightnav/py310/wheels
cp "$MATERIALS/locks/requirements-lightnav-py310.lock" \
  .jetson-lightnav/py310/requirements.lock
sha256sum .jetson-lightnav/py310/wheels/torch-2.7.0-cp310-cp310-linux_aarch64.whl
sha256sum .jetson-lightnav/py310/wheels/torchvision-0.22.0-cp310-cp310-linux_aarch64.whl
bash scripts/jetson_lightnav.sh setup --profile seeed310
/home/seeed/.local/bin/uv pip check --python .venv-lightnav-py310/bin/python
```

预期哈希见[安装材料说明](../code/README_zh_CN.md#环境锁与外部二进制)。它们是本项目对下载物的固定实测值，不冒称Seeed发布的签名。安装器另外在项目内解压NVIDIA cuSPARSELt0.8.1.1，并以进程级 `LD_LIBRARY_PATH` 使用，不复制到系统库。

原设备普通用户的CUPTI权限受限，完整安装检查依赖一次独立profiling。下面只在教师完成系统核对后执行，sudo仅用于这个临时检查进程：

```bash
export P=/home/seeed/workspace/LightNav-0
sudo env LD_LIBRARY_PATH="$P/.jetson-lightnav/py310/native/libcusparse_lt-linux-aarch64-0.8.1.1_cuda12-archive/lib" \
  "$P/.venv-lightnav-py310/bin/python" "$P/scripts/jetson_lightnav_gpu_check.py" \
  --output "$P/.jetson-lightnav/py310/evidence/gate1-privileged" \
  --python-version 3.10.21 --torch-version 2.7.0 --vision-version 0.22.0
bash scripts/jetson_lightnav.sh check --profile seeed310
```

预期组合检查为PASS，数值有限且有真实CUDA kernel。课程附带的历史GPU证据只供阅读，**不能复制为新机的PASS**。CUPTI失败要区分权限、运行库与算子问题，不静默回退CPU或重装系统CUDA。

### B4. 固定模型与仿真环境

```bash
bash scripts/jetson_lightnav.sh download --profile seeed310
cat .jetson-lightnav/model-manifest.json
```

固定 `LightOriginsHQ/LightNav-0@826dc5fbfa37afa8293d2e336d329b6ffc0bfb64`，18个文件包含权重、tokenizer、processor、eval配置和RVQ bundle，不另下基础Qwen。预期manifest PASS，revision和逐文件校验一致。网络不通时可在联网电脑下载同一revision的完整文件与官方metadata后转入，再按安装脚本的verify-only入口检查；不能只拷贝一个权重文件或跳过哈希。

在仿真目录创建第二个环境：

```bash
cd /home/seeed/workspace/LightNav-0/mujoco_demo
uv venv --python 3.11.16 .venv-cpu
uv pip sync --python .venv-cpu/bin/python "$MATERIALS/locks/requirements-demo-cpu.lock"
uv pip check --python .venv-cpu/bin/python
MUJOCO_GL=egl .venv-cpu/bin/python - <<'CHECK'
import mujoco
model = mujoco.MjModel.from_xml_string('<mujoco><worldbody><geom type="plane" size="1 1 .1"/></worldbody></mujoco>')
data = mujoco.MjData(model)
mujoco.mj_forward(model, data)
with mujoco.Renderer(model, height=270, width=480) as renderer:
    renderer.update_scene(data)
    image = renderer.render()
    print(mujoco.__version__, image.shape, image.dtype)
CHECK
```

预期MuJoCo3.11.0、`(270,480,3)`、`uint8`。模型锁含包哈希，仿真锁为精确版本锁但没有每份wheel哈希，并含`-e .`，所以必须在 `mujoco_demo` 下解析，不能给它照搬 `--require-hashes`。这两种锁的保证不同。

### B5. CAD、场景与配置

课程保留上游val_2场景，不下载整个数据集。检查场景XML与引用资源存在；若是Git LFS指针或缺文件，先修复资源获取，不用空壳几何替代。

教师应提供 `dm-h65-course-assets-20261015.tar.gz`，其中45份真实简化CAD网格，约4.1MB压缩。其校验和在[资产包元数据](../code/configs/asset-bundle.json)。当前没有公共下载地址；**未拿到教师包的学生尚不能完成DM-H65从零复现**。资产发布许可未由本次课程重写建立，不自动上传网格。

```bash
cd /home/seeed/workspace/LightNav-0
export ASSET_BUNDLE=/path/to/dm-h65-course-assets-20261015.tar.gz
sha256sum "$ASSET_BUNDLE"
python3 "$MATERIALS/tools/prepare_assets.py" --project "$PWD" --bundle "$ASSET_BUNDLE"
python3 "$MATERIALS/tools/prepare_assets.py" --project "$PWD" --bundle "$ASSET_BUNDLE" --apply
python3 "$MATERIALS/tools/preflight.py" --project "$PWD" \
  --chassis-config course-assets/dm-h65/config.json --task-config course-assets/task.json
```

准备工具逐网格验证SHA256，仅把课程副本的几何JSON路径改到项目内部，不依赖真实SDK目录，不打开设备。冻结参数轮半径约0.085428m、轮距约0.370294m，不能混入旧SDK示例的0.0825/0.42等数值。任务配置另保留原始登记与迁移标记。

最后按入口A启动8050模型与8091 Demo。Demo没有兼容任意后台模型协议：课程覆盖层补充了请求关联与诊断字段，不能在未核对版本的第三方服务上直接宣称兼容。

## 实验一：现场检查与画面观察

**操作：** 只查看两次health，间隔至少两秒；查看第一人称、第三人称、预测原帧及俯视轨迹；尝试视角切换和小窗收起。

**预期：** 实时帧持续更新，首次预测前清楚显示“无预测”。第一视角来自虚拟约1m相机，第三视角展示真实CAD外观。模型观察不是每个显示帧更新，不能拿它的静止图判定渲染卡住。

**失败定位：** 8091拒绝连接查Demo；模型未就绪查8050；有画面无状态查控制通道和遥测；`RENDER_ERROR`查EGL/资源。先读日志，不重装环境。

**提交：** 现场版本、双环境版本、两次health中的sim_time/帧差值、一张底盘页面截图。截图需标注时间和当前是否存在任务。

## 实验二：解读真实预测

无设备时在课程仓库根目录执行：

```bash
python3 docs/M07-Vision-Language-Navigation/code/tools/analyze_run.py docs/M07-Vision-Language-Navigation/code/evidence \
  --state docs/M07-Vision-Language-Navigation/code/evidence/actual-state.jsonl --task-id navigation-0001
```

**操作：** 在课程材料的 `evidence/prediction-01/` 中打开 `input.jpg` 和 `raw.json`。核对 JPEG 哈希、seq、request_id、采集位姿、APOS/OPOS 像素、原始 token、顶层 stop 与全部十点。`seq` 是会话图像序号；预测编号是另一个计数。选择第一点，用采集时根位姿，将局部点旋转并平移到世界坐标。

**预期：** 五份历史输入均通过哈希；第一次为`seq=4`、动作码`[187,39,97]`、`stop=false`。第二点约0.300m是累计位置，不能再次累加到第一点。

**失败定位：** 图像hash不同先排查文件错配；缺字段写“未返回”，不补造；OPOS有点先看原图，不直接判正确。

**提交：** 原帧标注图、完整十点表、一个手算世界点及计时边界说明。

## 实验三：观察持续闭环

仅在教师允许本组操作的空闲Demo中执行。先记录当前起点；如需使用课程起点，在任务前RESET。冻结配置原本登记门框，因此冰箱指令属于未评分自由任务。请将任务结果记录为画面、实际轨迹和终止事件，独立评分栏填“未注册”。

输入：

```text
Locate the refrigerator in the room and stop beside it.
```

点击一次开始，等待整个过程或明确终止。任务上限300秒/40次正式预测，课堂可由教师事先规定更短的观察时长，到期主动STOP，并记录人为终止，不算严格成功。

**观察重点：** 模型计数是否增加；等待预测B时是否仍沿有效路径A运动；B的原始采集帧是否来自运动后的新位姿；是否因参考过期、耗尽或衔接失败停车。保持相同指令和起点完成本次观察，改变条件后应另建试验记录。

**预期：** 能形成真实运动与新观察，而不是一张旧图不断重放。当前模型并不保证该任务成功，路径危险、转圈或模型提前STOP均可能出现。收到MODEL_STOP_UNSCORED应记录“模型停止、未独立评分”；它不是token解码失败，也不是已证明严格成功。视频案例的17次预测不保证你本次得到相同结果。

**提交：** 至少两次预测及其新图像；如果实际产生运动，画出对应真实位姿；若未运动，给出准确终止事件。失败不通过改指令、起点和阈值来凑成功。

## 实验四：停止、取消与路径失效

本实验与任务成绩分开，标记“人工取消验证”。不使用故障注入程序控制任何实机。

1. 在一次正常任务推理等待时点击STOP。
2. 等待超过该次模型预计返回时间，确认小车没有被迟到结果重新启动。
3. 在空闲状态RESET，确认预测图与任务计数清理。
4. 经教师安排，在另一短任务中关闭控制页面，观察日志是否记录断连或浏览器租约失效；不要把后台标签页节流误记为模型失败。

**预期：** 运动学停车、参考清除、旧代次结果失效。停止不意味着有模型STOP，更不等于导航到达。

**提交：** 取消前后状态、取消原因及后续至少一次观察；若缺少运动或等待条件，写“该分支未覆盖”，不用一张空闲截图冒充测试通过。

## 实验五：从日志解释结果

先在 Jetson 项目目录运行 `python3 scripts/jetson_sim_demo.py status`，从 `process.output` 找到本次输出目录。每次任务在其 `navigation-XXXX` 子目录，实际位姿位于父目录 `state.jsonl`。

以下分析在**能同时访问课程工具与本次日志的终端**运行，先按“先确定文件在哪台机器”设置 `MATERIALS`。若在自己的电脑分析，先取得完整运行目录，保留原始 JPEG、请求响应和位姿文件；`RUN_ROOT` 填该机器上的路径。

```bash
export RUN_ROOT=/path/to/the/actual/sim-demo-run
export TASK_ID=navigation-0001  # 改为本次实际任务目录名
python3 "$MATERIALS/tools/analyze_run.py" "$RUN_ROOT/$TASK_ID" \
  --state "$RUN_ROOT/state.jsonl" --task-id "$TASK_ID"
```

工具核验原图与十点、统计请求响应时延、按任务ID提取实际轨迹。它只读取既有结果，**不独立重新认证严格成功**；`strict_success_reported`是原outcome字段。

运行结束后，记录以下内容：

- 原始预测、执行参考与实际轨迹三层；未修复时参考可以与原始路径相同，被拒绝时没有采用的参考。
- 模型STOP、用户STOP、碰撞风险、实际接触、超时和无进展的准确分类。
- 模型延迟、采集到采用年龄、运动占空比、模型Hz、模拟Hz、相机FPS、浏览器FPS各自口径。
- 样本量与版本。五次样本的p99不代表长期可靠性。

## 实验六：自己的CAD与独立预览

先取得有权使用的URDF和分部件网格，按[自定义底盘说明](../code/examples/custom-chassis/README_zh_CN.md)执行只读清单检查。填写部件映射、单位、根坐标、轮径、轮距、四脚轮与相机；没有脚轮的另一种底盘按真实结构记录，不强行补造。

独立预览从静态MJCF模板学习，再用自己的真实网格替换。模板primitive不等于完成CAD导入。当前DM-H65加载器固定43视觉link，新CAD需要在副本中编写对应后端，不能直接修改计数校验来假装适配。

**提交：** 清单JSON、资产SHA256、部件表、外观与碰撞对照、独立相机预览；如未完成房间后端，明确写“静态预览完成，闭环待接入”。本实验没有要求运行真实电机、CAN或实车标定。

## 常见问题

| 现象 | 先检查 | 不应采取的做法 |
| --- | --- | --- |
| 页面能开但不能启动 | model_ready、线程、帧新鲜度、资源、控制所有权 | 反复加载模型或关闭安全检查 |
| 推理后立刻停车 | outcome、原响应、路径检查、TTL和任务代次 | 一律称为GPU或token解码失败 |
| 小车追旧位置／来回转 | capture_pose、结果年龄、NEW_PATH_CAPTURE_MISMATCH、原始yaw | 把路径整体移到当前根位置 |
| 明明到了却未成功 | 是否有有效模型STOP、两秒条件、登记指令 | 只按距离强制成功 |
| 自由指令显示未评分 | task_context与登记指令 | 用门框坐标给冰箱任务评分 |
| OPOS看着一直不变 | 本次opos_id、原帧、历史点年龄、重投影 | 把上次点永久贴在实时图 |
| 黑屏／画面慢 | EGL、渲染线程、FFmpeg、H.264回退、两条通道 | 把模型Hz当浏览器FPS |
| wheel装上但GPU算子失败 | cp310/aarch64、sm_87和实际加载库 | 只凭cuda.is_available判通过 |
| 缺CAD或路径不匹配 | 45网格manifest、底盘几何JSON、课程资产迁移记录 | 用简化空壳替代后写“已复现” |

## 收尾

点击STOP、确认不再active。只停止由本组启动且身份匹配的Demo：

```bash
cd /home/seeed/workspace/LightNav-0
python3 scripts/jetson_sim_demo.py stop
```

两条SSH隧道在各自电脑终端用Ctrl-C退出。共享模型8050留给教师；不执行killall，不停止真实SDK服务。文档编写与离线核验没有自动启动上述实验，也没有重新完成新机安装。

[返回 M7](../README_zh_CN.md)
