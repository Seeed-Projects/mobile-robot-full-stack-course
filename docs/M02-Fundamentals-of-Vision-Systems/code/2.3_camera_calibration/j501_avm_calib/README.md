# j501_avm_calib — J501 四路鱼眼相机内外参标定 ROS2 Pipeline

在 AGX Orin（ROS 2 Humble）上对四路鱼眼相机（front/back/left/right）进行
**内参标定（ROS2 `camera_calibration` GUI / 无头自动）→ 解析 ROS2 CameraInfo
→ 外参标定（地面棋盘 H + 接缝精修）→ RViz 效果展示 → 量化效果评估**
的完整 pipeline。外参与评估算法移植自本地参考项目
`~/leucus/fisheye-avm-calib`（H 求解 / H-QC / 连拍均值 / 接缝精修 /
RMS 阈值全部一致）。

## 快速开始

```bash
cd ~/ros2_ws
colcon build --packages-select j501_avm_calib --symlink-install
source install/setup.bash

# 一条命令走完全部阶段（需要显示器跑 GUI；相机已接线）
ros2 run j501_avm_calib calib_pipeline --stage all

# 无显示器：内参用无头自动标定
ros2 run j501_avm_calib calib_pipeline --stage all --auto

# 已有内参（config/camera_info/*.yaml 已标定）时只做外参与展示
ros2 run j501_avm_calib calib_pipeline --stage all --skip-intrinsics
```

分阶段执行：

| 阶段 | 命令 |
|------|------|
| 相机探测 | `ros2 run j501_avm_calib camera_driver --probe` |
| 驱动 | `ros2 launch j501_avm_calib driver.launch.py` |
| 内参 GUI（单路） | `ros2 launch j501_avm_calib intrinsics.launch.py direction:=front` |
| 内参无头自动 | `ros2 launch j501_avm_calib intrinsics.launch.py direction:=front auto:=true` |
| 解析 CameraInfo | `ros2 run j501_avm_calib intrinsics_parse --all` |
| 内参评估 | `ros2 run j501_avm_calib evaluator --once --scope intrinsics` |
| 外参向导 | `ros2 run j501_avm_calib extrinsic_calibrator`（终端交互） |
| 外参评估 | `ros2 run j501_avm_calib evaluator --once --scope extrinsics` |
| RViz 展示 | `ros2 launch j501_avm_calib visualize.launch.py` |

## 内参标定（cameracalibrator GUI）操作

> 务必先启动相机驱动（pipeline 会自动做）。

1. GUI 窗口底部 trackbar「Camera type」**拖到 1（fisheye）**（默认是 pinhole！）
2. 手持打印棋盘（**8×6 内角，格宽 25 mm**，与 `config/calib_config.yaml` 一致）
   缓慢移动：覆盖画面左/右/上/下、大图/小图、各种倾斜，直到 CALIBRATE 可点
3. CALIBRATE → 看畸变修正效果 → **SAVE**（写 `/tmp/calibrationdata.tar.gz`，
   pipeline 会备份并用它回填逐视角 RMS）→ **COMMIT**（通过
   `/cameras/<dir>/set_camera_info` 服务把 CameraInfo 写回驱动 YAML）

对四路重复（pipeline 会逐路自动引导）。

## 解析与评估

- 解析：`config/camera_info/<dir>.yaml`（ROS2 标准 CameraInfo，equidistant 模型）
  → `calib_results/<dir>.json`（`K / D / D_inv / rms / image_size / model`，
  兼容参考项目格式）。
- RMS 回填：CameraInfo 不含 RMS —— 从 tar 里的采样图重检角点、重投影得到
  overall/逐视角 RMS。
- 评估（pipeline 闸门，fail 即停）：
  - 内参：总 RMS（<0.8px 合格 / >1.5px 不合格）、逐帧异常帧、fx/fy≤1.15、
    主点偏离中心≤12%、|D|≤0.35、D0 枕形异常、FOV 3×3 网格覆盖、≥12 帧；
  - 外参：H-QC（σ₂≥0.03 / 跨度≥25px / 中心落点≤350px / 翻转 / 板对边比≤1.35）、
    rms（<1px / 1~5px / ≥5px）、左右 H 对称比≤4、接缝精修 before/after、
    多点验证位（board 放新位置与实测 BEV 误差）。
- 输出：`calib_results/evaluation_report.json` + 终端摘要 + `/diagnostics` +
  `/calib/eval/report`。

## 外参标定

终端向导（中文命令提示），与参考项目一致：

1. **逐路顺序标定**（front→back→left→right）：把棋盘平贴地面放在该路摆位
   （默认板近边距车心 0.35 m，居中，长边横向）。连续稳定检出 10 帧后自动连拍
   8 帧求 H（H：去畸变图→BEV，100 px/m，画布 1000×1000），并做 H-QC；
   命令可用 `skip / relock / status / save`。
2. **接缝精修** `seam`：4 对（front+left、front+right、back+left、back+right），
   把板放到两路重叠区，两路同步达标自动精修从路 H。
3. **多点验证位** `verify <dir> <near> <lateral>`：把板放到手工测量过的新位置，
   得实测 vs 期望 BEV 误差（强回归评估）。

结果合并保存在 `calib_results/extrinsics.json`
（homographies / rms_errors / homography_qc / seam_refined / poses / verifications）。

## RViz 效果展示

`ros2 launch j501_avm_calib visualize.launch.py`（需显示器）打开预置配置：

- **BEV 拼接图** / **评估覆盖层**（期望板角=绿、实测=红、误差线=黄、
  0.25 m 世界网格、每路 rms 文字）
- 4 路去畸变预览（叠加像素网格，检查直线度）
- 3D 视图：base_link 地面网格、相机坐标系（PnP 6DoF TF）、标定板角点/误差
  向量 Marker

## 目录布局

```
config/calib_config.yaml       # 棋盘/摆位/BEV/相机设备/阈值（真源）
config/camera_info/<dir>.yaml  # ROS2 CameraInfo（GUI COMMIT 落盘处）
calib_results/<dir>.json       # 解析后的内参（兼容参考项目）
calib_results/extrinsics.json  # 外参 H + QC + 接缝 + poses
calib_results/evaluation_report.json
launch/  test/  j501_avm_calib/...
```

## 已知限制

- 本机 `camera_calibration_parsers`/`camera_info_manager` 只有 C++ 库无 python
  绑定 → 自实现 YAML 读写与 `set_camera_info` 服务（接口与官方一致）。
- BEV 拼接仅作**验证展示**（CPU 预览尺度 ≤5Hz，平面羽化融合）；生产级 GPU
  环视请用参考项目管线。
- 相机离线（无 `/dev/video*`）时：标定需先接线；全部算法可用
  `colcon test` 合成数据验证。