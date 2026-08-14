# yuyv2png.py：把 ISX031 的 YUYV 原始帧转为 PNG
import sys
import numpy as np
from PIL import Image

W, H = 1920, 1536
yuv = np.fromfile(sys.argv[1], dtype=np.uint8).reshape(H, W, 2)
Y = yuv[..., 0].astype(np.int32)
u = np.repeat(yuv[:, 0::2, 1].astype(np.int32), 2, axis=1)
v = np.repeat(yuv[:, 1::2, 1].astype(np.int32), 2, axis=1)
C, D, E = Y - 16, u - 128, v - 128
r = np.clip((298 * C + 409 * E + 128) >> 8, 0, 255)
g = np.clip((298 * C - 100 * D - 208 * E + 128) >> 8, 0, 255)
b = np.clip((298 * C + 516 * D + 128) >> 8, 0, 255)
Image.fromarray(np.stack([r, g, b], -1).astype(np.uint8)).save(sys.argv[2])
