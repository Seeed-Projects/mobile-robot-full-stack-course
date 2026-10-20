# LightNav-0教学动画：离线复算与维护

本目录提供7.2.2、7.2.4、7.2.5、7.2.6、7.2.7五段算法动画的源程序。只读取冻结的常量、一次离线解码结果和固定上游函数，生成插图与GIF；没有模型推理、仿真步进、CAN或网络操作，也不接入Demo热路径。

## 文件与来源

- `lightnav_reference.py`：固定上游commit `3015508b70fb6e30bbd66701e94c9b916dcb8df8` 的 `vln_utils.py` 指向编码函数与 `traj_vocab.py` 的 `compose_to_abs`，保留原文；两个上游文件的SHA256记录在配置中。
- `extract_rvq_example.py`：可选的一次性提取脚本。读取发布 `action_tokenizer` bundle，选出3个编号并写下解码全过程（逐层码字→运动增量→相加→复合位点），结果冻结在 `rvq-teaching-example.json`。没有checkpoint也能重建动画。
- `rvq-teaching-example.json`：冻结的三层解码算例，含每层码字贡献、10个位点、逐层修正量和256项统计。
- `example-config.json`：commit、checkpoint、网格与词表常量、bundle字段与npy哈希、场景裁切、图号、GRPO与交叉熵算例。
- `build_visuals.py`：确定性绘制五段GIF与五张静态图，写两个清单与 `teaching-numbers.json`。
- `verify_examples.py`：独立重算各项数字（网格往返、五个token、SE(2)复合、RVQ求和、GRPO优势、交叉熵），并验证导出文件。

场景底图直接复用7.2.3的[场景atlas](../../../7.2_Semantic_Grounding_and_Learned_VLN/assets/slowfast/scene-atlas.jpg)，不新生成图片；裁切框与7.2.3相同。运行源码与代码许可沿用[第三方说明](../../THIRD_PARTY.md)。

## 复算命令

在课程仓库根目录运行。需要Python、Pillow与NumPy；本轮验证版本为Pillow 12.2.0、NumPy 2.4.4，非运行时环境锁。字体使用Arial或DejaVu Sans，按脚本中的搜索路径读取。

```bash
export COURSE_ROOT="$PWD"
export TEACHING="$COURSE_ROOT/docs/M07-Vision-Language-Navigation/code/teaching/lightnav"
export ART="$COURSE_ROOT/docs/M07-Vision-Language-Navigation/7.2_Semantic_Grounding_and_Learned_VLN/assets/lightnav"
PYTHONDONTWRITEBYTECODE=1 python3 "$TEACHING/verify_examples.py" --assets "$ART"
```

修改素材后先在独立目录生成，检查后再更新教材资产，避免边生成边覆盖读者正在浏览的GIF：

```bash
export PREVIEW_DIR=/absolute/path/to/an/empty/lightnav-preview
mkdir -p "$PREVIEW_DIR"
PYTHONDONTWRITEBYTECODE=1 python3 "$TEACHING/build_visuals.py" --assets "$PREVIEW_DIR"
PYTHONDONTWRITEBYTECODE=1 python3 "$TEACHING/verify_examples.py" --assets "$PREVIEW_DIR"
```

场景底图默认从 `docs/M07-Vision-Language-Navigation/7.2_Semantic_Grounding_and_Learned_VLN/assets/slowfast/scene-atlas.jpg` 读取；放到别处时用 `--scene-atlas` 指定。脚本会先核对底图SHA256，不匹配直接报错，避免画风与7.2.3不一致。

若要从checkpoint重新导出RVQ算例（可选，不影响复现）：

```bash
PYTHONDONTWRITEBYTECODE=1 python3 "$TEACHING/extract_rvq_example.py" \
  --bundle /path/to/checkpoint/action_tokenizer \
  --out "$TEACHING/rvq-teaching-example.json"
```

重导后必须把新哈希写回 `example-config.json` 的 `rvq_example_sha256`，并重跑 `verify_examples.py`；`build_visuals.py` 会在哈希不符时拒绝渲染。

## 五段动画的分工

| 文件 | 对应小节 | 讲清的一件事 |
| --- | --- | --- |
| `01-decision-loop` | 7.2.2 | 一次决策：指令与历史进入，五个token出来，三个动作编号解码成十个位点，控制器跟踪后回到新观察 |
| `02-pointing-grid` | 7.2.4 | 像素→48×27网格→一个编号，再由编号复原格心；APOS与OPOS同网格两通道 |
| `03-rvq-residual` | 7.2.5 | 三层码字相加→除以权重→十个增量→复合；停止靠编号匹配而非路径长度 |
| `04-se2-compose` | 7.2.6 | 每步增量写在上一步自身坐标里，必须用上一步朝向旋转后再累加 |
| `05-training-stages` | 7.2.7 | 具身推理、SFT+DAgger、在线RL三个阶段各自的训练信号 |

## 数字与图像应怎样解释

网格为48列、27行，共1296格；480×270帧上每格10×10像素。`id=1+rW+c`，0保留；APOS的1297/1298/1299分别表示左转、右转、停止，OPOS没有这三个值，0表示本帧看不到目标。算例 `opos_id=505` 对应行10、列24，格心像素 $(245,105)$；`apos_id=1126` 落在更靠近地面的位置，两条通道因此不同。

RVQ算例使用的三个编号是 `[14, 42, 30]`，依次对应第0、1、2层；三层码字相加得到十组运动增量，再复合为十个位点。逐层修正量分别为约11.2cm、3.6cm、0cm，这是"该层解码路径与三层完整解码路径的平均逐点距离"，不是执行误差。三层各256项，组合数16,777,216是词表规模，不是可行路径数，也不是安全保证。停止码 `[6,122,174]` 的L0码字净位移约0.03cm，接近零但并非精确为零，所以源码用编号精确匹配判定停止。

`04` 的算例是手算：先0.5m到 $(0.5,0,0)$，原地左转90°到 $(0.5,0,\pi/2)$，再沿自身前向0.5m，落在 $(0.5,0.5,\pi/2)$。最后一步增加的是采集坐标的 $y$，因为车头已转向左方。十步各0.15m、不转向，第十点为1.50m；十个点只是空间顺序，没有声明这些动作必须在一秒内完成。

`05` 的GRPO算例回报取 $[0,1,2]$，组内标准化后优势约 $[-1.225,0,1.225]$；交叉熵算例为概率0.2→0.8时损失约1.609→0.223。两者都用正文同一组数字。

GIF按10FPS采样绘制，总时长16秒，循环播放；编码器合并连续相同画面形成停留帧。停留不是丢帧，也不是实际相机、模型或机器人的运行时序。

## 导出清单

- 五段 `.gif`：嵌入正文的动画，1200×675、16秒、10FPS采样。
- 五张同名 `.png`：暂停阅读用的静态分解图，取每段结论态。
- 生成脚本可另写 `animation-manifest.json`；当前教材副本不要求该可选清单，校验器直接读取GIF检查尺寸、时长和循环。
- `teaching-numbers.json`：正文引用的全部数字，便于与文字交叉核对。
- `asset-manifest.json`：新图与源文件的哈希和来源说明。

重新导出并检查通过后，再同步选定资产并更新资产及课程校验清单；原生成提示词保留作为来源记录。场景插画与教学网格不代表真实相机采集、模型激活或运行速度；动画不用来证明识别、导航或成功评分。旧05总览图和已有模型、控制、原始任务证据保持不变。
