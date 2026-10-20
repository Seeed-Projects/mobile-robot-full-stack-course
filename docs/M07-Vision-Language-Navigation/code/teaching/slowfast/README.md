# SlowFast教学动画：离线复算与维护

本目录提供7.2.3的时间选择、空间池化教学素材源。只读取冻结规则并生成插图，没有模型推理、仿真步进、CAN或网络操作，不接入Demo热路径。

## 文件与来源

- `slowfast_reference.py`：固定上游commit `3015508b70fb6e30bbd66701e94c9b916dcb8df8` 的纯Python选帧源码副本，保留原文；哈希在配置中。
- `timestamp_reference.py`：同版本 `processing.py` 的时间计算方法，提取为离线参考，文件内记录原文件SHA256。
- `example-config.json`：checkpoint `826dc5fbfa37afa8293d2e336d329b6ffc0bfb64` 的教学相关配置和场景裁切参数。
- `build_visuals.py`：精确绘制英文标注、选帧格子与特征网格，合成GIF及PNG。
- `verify_examples.py`：验证选择、边界、padding、平均池化、相对时间与实际导出的图片/GIF。

场景源由内置imagegen生成，见[提示词](../../../7.2_Semantic_Grounding_and_Learned_VLN/assets/slowfast/scene-prompt.json)与[场景底图](../../../7.2_Semantic_Grounding_and_Learned_VLN/assets/slowfast/scene-atlas.jpg)。裁切与排版仅为组成教学动画，不修改真实实验图片。运行源码与代码许可沿用[第三方说明](../../THIRD_PARTY.md)。

## 复算命令

在课程仓库根目录运行，使用自己的文档制作环境，不在Jetson模型或仿真环境里安装依赖。需要Python、Pillow与NumPy；本轮验证版本为Pillow12.2.0、NumPy2.4.4，非运行时环境锁。字体使用Arial或DejaVu Sans，按脚本中的搜索路径读取。

```bash
export COURSE_ROOT="$PWD"
export TEACHING="$COURSE_ROOT/docs/M07-Vision-Language-Navigation/code/teaching/slowfast"
export ART="$COURSE_ROOT/docs/M07-Vision-Language-Navigation/7.2_Semantic_Grounding_and_Learned_VLN/assets/slowfast"
PYTHONDONTWRITEBYTECODE=1 python3 "$TEACHING/verify_examples.py" --assets "$ART"
```

修改素材后先在独立目录生成，再检查后更新教材资产，避免边生成边覆盖读者正在浏览的GIF：

```bash
export PREVIEW_DIR=/absolute/path/to/an/empty/slowfast-preview
mkdir -p "$PREVIEW_DIR"
cp "$ART/scene-atlas.jpg" "$PREVIEW_DIR/scene-atlas.jpg"
PYTHONDONTWRITEBYTECODE=1 python3 "$TEACHING/build_visuals.py" --assets "$PREVIEW_DIR"
PYTHONDONTWRITEBYTECODE=1 python3 "$TEACHING/verify_examples.py" --assets "$PREVIEW_DIR"
```

## 数字与图像应怎样解释

240帧、最新239、4Hz；首末帧间隔为59.75秒，约60秒采集历史。选择结果为82帧，分层数量为current 2、fast 32、mid 20、long 26、anchor 2。本例没有padding，因此82个选择项也对应82份不同采集帧。

8×8、4×4、2×2是编码时间块的教学网格，分别64、16、4个空间位置，因子都相对原始8×8；没有测量或展示真实激活。一个通道的2×2算例为(1+3+5+7)/4=4。

GIF按10FPS采样绘制，总时长16秒，循环播放；编码器合并连续相同画面形成停留帧，文件帧数可以小于160。停留不是丢帧，也不是实际模型或机器人的运行时序。

## 导出清单

- `time-selection.gif`、`spatial-pooling.gif`：嵌入正文的两个动画。
- `time-selection.png`、`spatial-pooling.png`、`combined-example.png`：暂停阅读用的静态分解图。
- `selection-example.json`：完整原始选择下标及分层。
- 生成脚本可另写 `animation-manifest.json`；当前教材副本直接校验GIF尺寸、时长和循环。

重新导出并检查通过后，再同步选定资产并更新资产及课程校验清单；原生成提示词保留作为来源记录。

生成图有场景教学用途，不用来证明识别、导航或成功评分。旧05总览图和已有模型、控制、原始任务证据保持不变。
