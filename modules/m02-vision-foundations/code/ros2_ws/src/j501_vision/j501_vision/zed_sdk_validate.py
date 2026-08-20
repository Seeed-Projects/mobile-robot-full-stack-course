#!/usr/bin/env python3
"""ZED 2i SDK 验证脚本（不依赖 cv2，使用 PIL 保存图像）

验证内容：
1. 相机识别（序列号/型号/固件）
2. 出厂标定参数（左右目内参、畸变、立体外参基线）
3. 深度估计 + 视差公式 Z = f*B/d 验证
4. 点云导出
"""
import numpy as np
from PIL import Image
import pyzed.sl as sl


def bgra_to_rgb(arr):
    return arr[:, :, 2::-1]  # BGRA -> RGB


def gray_to_turbo_like(v):
    """简单伪彩色：灰度三段渐变（蓝-绿-红）"""
    t = np.clip(v, 0, 1)
    r = np.clip(1.5 - np.abs(4 * t - 3), 0, 1)
    g = np.clip(1.5 - np.abs(4 * t - 2), 0, 1)
    b = np.clip(1.5 - np.abs(4 * t - 1), 0, 1)
    return (np.stack([r, g, b], -1) * 255).astype(np.uint8)


def main():
    init = sl.InitParameters()
    init.camera_resolution = sl.RESOLUTION.HD720
    init.camera_fps = 30
    init.depth_mode = sl.DEPTH_MODE.QUALITY
    init.coordinate_units = sl.UNIT.METER

    cam = sl.Camera()
    status = cam.open(init)
    if status != sl.ERROR_CODE.SUCCESS:
        raise SystemExit(f"相机打开失败: {status}")
    print("[OK] 相机打开成功")

    info = cam.get_camera_information()
    print(f"型号: {info.camera_model}")
    print(f"序列号: {info.serial_number}")
    print(f"相机固件版本: {info.camera_configuration.firmware_version}")

    calib = info.camera_configuration.calibration_parameters
    lc, rc = calib.left_cam, calib.right_cam
    print("\n===== 出厂标定参数（HD720）=====")
    print(f"左目内参: fx={lc.fx:.2f} fy={lc.fy:.2f} cx={lc.cx:.2f} cy={lc.cy:.2f}")
    print(f"左目畸变(校正后, 12 参数): {[round(float(v), 5) for v in lc.disto]}")
    print(f"右目内参: fx={rc.fx:.2f} fy={rc.fy:.2f} cx={rc.cx:.2f} cy={rc.cy:.2f}")
    print(f"右目畸变(校正后, 12 参数): {[round(float(v), 5) for v in rc.disto]}")
    T = calib.stereo_transform.get_translation().get()
    baseline = float(np.linalg.norm(T[:3]))
    print(f"立体外参 T = ({T[0]:.4f}, {T[1]:.4f}, {T[2]:.4f}) m, 基线 = {baseline*1000:.1f} mm")

    # 采集一帧
    rt = sl.RuntimeParameters()
    for _ in range(30):  # 预热
        cam.grab(rt)
    err = cam.grab(rt)
    if err != sl.ERROR_CODE.SUCCESS:
        raise SystemExit(f"grab 失败: {err}")

    left = sl.Mat()
    disp = sl.Mat()
    depth = sl.Mat()
    pc = sl.Mat()
    cam.retrieve_image(left, sl.VIEW.LEFT)
    cam.retrieve_measure(disp, sl.MEASURE.DISPARITY)
    cam.retrieve_measure(depth, sl.MEASURE.DEPTH)
    cam.retrieve_measure(pc, sl.MEASURE.XYZRGBA)

    left_np = left.get_data()
    disp_np = disp.get_data().astype(np.float32)
    depth_np = depth.get_data().astype(np.float32)
    pc_np = pc.get_data()

    Image.fromarray(bgra_to_rgb(left_np)).save("/tmp/zed2i_left_rect.png")

    # 深度伪彩色可视化（0-5m 截断）
    d = depth_np.copy()
    d[d <= 0] = np.nan
    d_norm = np.nan_to_num(d / 5.0, nan=0.0)
    Image.fromarray(gray_to_turbo_like(d_norm)).save("/tmp/zed2i_depth_color.png")
    np.save("/tmp/zed2i_depth_m.npy", depth_np)

    # 深度统计
    h, w = depth_np.shape
    center = depth_np[h//2-60:h//2+60, w//2-60:w//2+60]
    cv_ = center[center > 0]
    print(f"\n===== 深度统计（中心 120x120 区域）=====")
    print(f"有效像素: {len(cv_)}/{center.size}")
    med_z = float(np.median(cv_)) if len(cv_) else float('nan')
    print(f"中心深度: 均值={cv_.mean():.3f} m, 中位数={med_z:.3f} m, std={cv_.std():.4f} m")

    # 视差公式验证: Z = fx * B / d（视差图与深度图逐像素交叉验证）
    print(f"\n===== 视差公式验证 Z = fx*B/d =====")
    fin = np.isfinite(disp_np)
    print(f"视差图: finite={fin.sum()}, 正值={int((disp_np[fin] > 0).sum())}, 负值={int((disp_np[fin] < 0).sum())}")
    da = np.abs(disp_np)  # SDK 视差为负值约定，取绝对值即视差大小
    valid = fin & (da > 0) & np.isfinite(depth_np) & (depth_np > 0)
    print(f"视差/深度同时有效像素: {valid.sum()}/{disp_np.size}")
    if valid.sum() > 0:
        dv = da[valid].astype(np.float64)
        zv = depth_np[valid].astype(np.float64)
        z_calc = lc.fx * baseline / dv            # 由视差按公式推深度
        rel_err = (z_calc - zv) / zv              # 与 SDK 深度逐像素对比
        good = np.abs(rel_err) < 0.2
        print(f"视差中位数 d = {float(np.median(dv)):.2f} px, fx = {lc.fx:.2f} px, B = {baseline:.4f} m")
        print(f"公式深度中位数 = {float(np.median(z_calc)):.3f} m, SDK 深度中位数 = {float(np.median(zv)):.3f} m")
        print(f"逐像素相对误差: 中位数={float(np.median(rel_err))*100:.2f}%, |误差|<20% 的像素占 {good.mean()*100:.1f}%")

    # 点云统计与导出
    pts = pc_np.reshape(-1, 4)[:, :3].astype(np.float32)
    valid_pts = pts[np.all(np.isfinite(pts), axis=1)]
    print(f"\n===== 点云 =====")
    print(f"总点数: {pts.shape[0]}, 有效点: {len(valid_pts)}")

    sub = valid_pts[::4]
    with open("/tmp/zed2i_pointcloud.ply", "wb") as f:
        f.write(b"ply\nformat binary_little_endian 1.0\n")
        f.write(f"element vertex {len(sub)}\n".encode())
        f.write(b"property float x\nproperty float y\nproperty float z\n")
        f.write(b"end_header\n")
        f.write(sub.astype("<f4").tobytes())
    print(f"已导出 /tmp/zed2i_pointcloud.ply ({len(sub)} 点, 1/4 采样)")

    print("已保存: /tmp/zed2i_left_rect.png /tmp/zed2i_depth_color.png /tmp/zed2i_pointcloud.ply")

    cam.close()
    print("[OK] 相机已关闭")


if __name__ == "__main__":
    main()
