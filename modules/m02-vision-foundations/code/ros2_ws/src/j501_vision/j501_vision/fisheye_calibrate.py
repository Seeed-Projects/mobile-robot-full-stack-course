# ~/ros2_ws/src/j501_vision/j501_vision/fisheye_calibrate.py
"""
鱼眼相机标定（cv2.fisheye，Kannala-Brandt 等距模型）
适用于本教程实机 SG3S-ISX031C-GMSL2F + H190XA（190°）

用法：
  python3 fisheye_calibrate.py ~/calib_images --pattern 9x6 --square 0.025 \
      --out ~/ros2_ws/src/j501_vision/config/cam_front_fisheye.yaml
"""
import argparse
import glob
import os

import cv2
import numpy as np
import yaml


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument('imgdir', help='标定图像目录')
    ap.add_argument('--pattern', default='9x6', help='内角点数 列x行')
    ap.add_argument('--square', type=float, default=0.025, help='方格边长（米）')
    ap.add_argument('--out', required=True, help='输出 YAML 路径')
    args = ap.parse_args()

    cols, rows = map(int, args.pattern.split('x'))
    objp = np.zeros((1, cols * rows, 3), np.float32)
    objp[0, :, :2] = np.mgrid[0:cols, 0:rows].T.reshape(-1, 2) * args.square

    objpoints, imgpoints = [], []
    img_size = None
    for path in sorted(glob.glob(os.path.join(args.imgdir, '*.png'))):
        img = cv2.imread(path)
        if img is None:
            continue
        gray = cv2.cvtColor(img, cv2.COLOR_BGR2GRAY)
        img_size = gray.shape[::-1]
        ok, corners = cv2.findChessboardCorners(gray, (cols, rows))
        if ok:
            corners = cv2.cornerSubPix(
                gray, corners, (3, 3), (-1, -1),
                (cv2.TERM_CRITERIA_EPS + cv2.TERM_CRITERIA_MAX_ITER, 30, 1e-3))
            objpoints.append(objp)
            imgpoints.append(corners.reshape(1, -1, 2))
            print(f'[OK] {os.path.basename(path)}')
        else:
            print(f'[跳过] {os.path.basename(path)}：未检测到角点')

    if len(objpoints) < 5:
        raise SystemExit(f'有效图像仅 {len(objpoints)} 张，至少需要 5 张，请补采')

    K = np.zeros((3, 3))
    D = np.zeros((4, 1))
    flags = cv2.fisheye.CALIB_RECOMPUTE_EXTRINSIC | cv2.fisheye.CALIB_FIX_SKEW
    rms, _, rvecs, tvecs = cv2.fisheye.calibrate(
        objpoints, imgpoints, img_size, K, D,
        flags=flags,
        criteria=(cv2.TERM_CRITERIA_EPS + cv2.TERM_CRITERIA_MAX_ITER, 100, 1e-6))
    print(f'RMS 重投影误差: {rms:.4f} px（建议 < 0.5，否则重采重标）')

    # 逐图重投影误差，便于剔除坏图
    for i, (op, ip) in enumerate(zip(objpoints, imgpoints)):
        proj, _ = cv2.fisheye.projectPoints(
            op.reshape(-1, 3), rvecs[i], tvecs[i], K, D)
        err = cv2.norm(ip.reshape(-1, 2), proj.reshape(-1, 2), cv2.NORM_L2) / len(op)
        print(f'  图 {i:02d}: {err:.3f} px')

    # 保存为 camera_info 兼容 YAML（distortion_model: equidistant）
    calib = {
        'image_width': img_size[0],
        'image_height': img_size[1],
        'camera_name': 'isx031_h190xa',
        'camera_matrix': {
            'rows': 3, 'cols': 3, 'data': K.flatten().tolist()},
        'distortion_model': 'equidistant',
        'distortion_coefficients': {
            'rows': 1, 'cols': 4, 'data': D.flatten().tolist()},
        'rectification_matrix': {
            'rows': 3, 'cols': 3, 'data': np.eye(3).flatten().tolist()},
        'projection_matrix': {
            'rows': 3, 'cols': 4,
            'data': np.hstack([K, np.zeros((3, 1))]).flatten().tolist()},
    }
    os.makedirs(os.path.dirname(os.path.abspath(args.out)), exist_ok=True)
    with open(args.out, 'w') as f:
        yaml.safe_dump(calib, f)
    print(f'标定结果已保存: {args.out}')


if __name__ == '__main__':
    main()
