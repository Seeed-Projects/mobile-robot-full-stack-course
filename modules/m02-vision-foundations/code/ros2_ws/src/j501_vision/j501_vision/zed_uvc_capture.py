#!/usr/bin/env python3
"""ZED 2i UVC 采集验证脚本

从 /dev/video0 读取 Side-by-Side（左右目并排）帧，
分离左右目图像并保存，同时测量实际帧率。
"""
import time
import cv2

DEVICE = "/dev/video0"
W, H, FPS = 2560, 720, 30  # USB 3.0 模式：720p SBS（每眼 1280x720）

cap = cv2.VideoCapture(DEVICE, cv2.CAP_V4L2)
cap.set(cv2.CAP_PROP_FOURCC, cv2.VideoWriter_fourcc(*"YUYV"))
cap.set(cv2.CAP_PROP_FRAME_WIDTH, W)
cap.set(cv2.CAP_PROP_FRAME_HEIGHT, H)
cap.set(cv2.CAP_PROP_FPS, FPS)

if not cap.isOpened():
    raise SystemExit("无法打开相机")

print(f"实际协商格式: {int(cap.get(cv2.CAP_PROP_FRAME_WIDTH))}x"
      f"{int(cap.get(cv2.CAP_PROP_FRAME_HEIGHT))} @ "
      f"{cap.get(cv2.CAP_PROP_FPS)}fps")

# 预热（丢弃前几帧，让自动曝光稳定）
for _ in range(15):
    cap.read()

# 测帧率
n_frames = 30
t0 = time.time()
frame = None
for _ in range(n_frames):
    ok, frame = cap.read()
    if not ok:
        raise SystemExit("读帧失败")
elapsed = time.time() - t0
print(f"实测帧率: {n_frames / elapsed:.2f} fps ({n_frames} 帧 / {elapsed:.2f}s)")

h, w = frame.shape[:2]
left = frame[:, :w // 2]
right = frame[:, w // 2:]
print(f"SBS 帧尺寸: {w}x{h} -> 左目/右目: {left.shape[1]}x{left.shape[0]}")

cv2.imwrite("/tmp/zed2i_sbs.png", frame)
cv2.imwrite("/tmp/zed2i_left.png", left)
cv2.imwrite("/tmp/zed2i_right.png", right)
print("已保存: /tmp/zed2i_sbs.png /tmp/zed2i_left.png /tmp/zed2i_right.png")

cap.release()
