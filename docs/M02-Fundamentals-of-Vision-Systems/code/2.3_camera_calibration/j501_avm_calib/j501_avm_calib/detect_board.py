# -*- coding: utf-8 -*-
"""棋盘检测(SB + 光度重试)，端口自参考项目 avm/detect_board_hires.py。

要点：
- 优先 findChessboardCornersSB：1920x1536 上比经典算法多撑一档距离（7px 格子
  仍可检出），"找不到"耗时恒定 ~300ms（经典 ~1300ms）。
- 光照 miss 只改亮度不改几何：原图 miss 后试伽马+CLAHE（连拍还试反色），
  命中后用原图 cornerSubPix 收回亚像素。
- 角点始终映射回原图分辨率。
"""
from __future__ import annotations

from typing import Optional, Sequence, Tuple

import numpy as np
import cv2

DETECT_FLAGS_FULL = (
    cv2.CALIB_CB_ADAPTIVE_THRESH | cv2.CALIB_CB_NORMALIZE_IMAGE
)
_SUBPIX_CRITERIA = (cv2.TERM_CRITERIA_EPS | cv2.TERM_CRITERIA_MAX_ITER, 30, 1e-3)

_HAS_SB = hasattr(cv2, "findChessboardCornersSB")
SB_FLAGS = cv2.CALIB_CB_NORMALIZE_IMAGE if _HAS_SB else 0

_PHOTO_EXTRA_SCAN = 1   # 扫描：原图 + 增强
_PHOTO_EXTRA_LOCK = 2   # 连拍：原图 + 增强 + 反色


def _as_gray(img: np.ndarray) -> np.ndarray:
    if img.ndim == 3:
        return cv2.cvtColor(img, cv2.COLOR_BGR2GRAY)
    return np.ascontiguousarray(img)


def _apply_gamma(gray: np.ndarray, gamma: float) -> np.ndarray:
    g = float(gamma)
    if abs(g - 1.0) < 0.05:
        return gray
    lut = np.array(
        [np.clip(pow(i / 255.0, g) * 255.0, 0, 255) for i in range(256)],
        dtype=np.uint8,
    )
    return cv2.LUT(gray, lut)


def enhance_gray_for_detect(gray: np.ndarray) -> np.ndarray:
    """同分辨率光度增强（压高光/提阴影）。不缩放、不改像素网格。"""
    g = np.ascontiguousarray(gray)
    mean = float(cv2.mean(g)[0])
    if mean < 65.0:
        gamma = 0.50
    elif mean < 95.0:
        gamma = 0.70
    elif mean > 200.0:
        gamma = 1.80
    elif mean > 165.0:
        gamma = 1.35
    else:
        gamma = 1.0
    out = _apply_gamma(g, gamma)
    clahe = cv2.createCLAHE(clipLimit=3.0, tileGridSize=(8, 8))
    return clahe.apply(out)


def photometric_variants(gray: np.ndarray, *, extra: int = 1):
    """返回 [(stage, image), ...]：raw 起步，extra>=1 加增强图，>=2 加反色。"""
    g = np.ascontiguousarray(gray)
    views = [("raw", g)]
    n = max(0, int(extra))
    if n >= 1:
        eq = enhance_gray_for_detect(g)
        views.append(("eq", eq))
        if n >= 2:
            views.append(("inv", cv2.bitwise_not(eq)))
    return views


def _refine_on_original(orig_gray: np.ndarray, corners: np.ndarray) -> np.ndarray:
    """光度图上检出的角点用原图收亚像素（保几何精度）。"""
    c = np.ascontiguousarray(corners, dtype=np.float32)
    return cv2.cornerSubPix(
        orig_gray, c, (5, 5), (-1, -1), _SUBPIX_CRITERIA
    ).astype(np.float32)


def _try_sb(gray: np.ndarray, pattern: Tuple[int, int]) -> Optional[np.ndarray]:
    if not _HAS_SB:
        return None
    found, corners = cv2.findChessboardCornersSB(gray, pattern, SB_FLAGS)
    if not found or corners is None:
        return None
    return np.ascontiguousarray(corners, dtype=np.float32)


def _try_classic(gray: np.ndarray, pattern: Tuple[int, int]) -> Optional[np.ndarray]:
    found, corners = cv2.findChessboardCorners(gray, pattern, DETECT_FLAGS_FULL)
    if not found or corners is None:
        return None
    corners = cv2.cornerSubPix(gray, corners, (5, 5), (-1, -1), _SUBPIX_CRITERIA)
    return np.ascontiguousarray(corners, dtype=np.float32)


def find_board_corners(gray: np.ndarray, pattern: Tuple[int, int], *,
                       use_sb: bool = True, photo_retry: bool = True,
                       allow_classic: bool = True) -> Optional[np.ndarray]:
    """灰度图找内角点，返回 Nx1x2 float32 / None。SB 优先 + 光度重试。"""
    gray = _as_gray(gray)
    extra = _PHOTO_EXTRA_LOCK if photo_retry else 0
    views = photometric_variants(gray, extra=extra)

    if use_sb and _HAS_SB:
        for name, view in views:
            corners = _try_sb(view, pattern)
            if corners is None:
                continue
            if name != "raw":
                corners = _refine_on_original(gray, corners)
            return corners

    if not allow_classic:
        return None

    for name, view in views:
        corners = _try_classic(view, pattern)
        if corners is None:
            continue
        if name != "raw":
            corners = _refine_on_original(gray, corners)
        return corners
    return None


def _gray_at(bgr: np.ndarray, width: int) -> tuple[np.ndarray, float]:
    """转灰度并按宽度缩放，返回 (gray, scale)。"""
    h, w = bgr.shape[:2]
    if width <= 0 or width >= w:
        gray = cv2.cvtColor(bgr, cv2.COLOR_BGR2GRAY) if bgr.ndim == 3 else bgr
        return gray, 1.0
    scale = width / float(w)
    nh = max(1, int(round(h * scale)))
    small = cv2.resize(bgr, (width, nh), interpolation=cv2.INTER_AREA)
    gray = cv2.cvtColor(small, cv2.COLOR_BGR2GRAY) if small.ndim == 3 else small
    return gray, scale


def scan_board(bgr: np.ndarray, pattern: Tuple[int, int], *,
               scan_width: int = 1920, use_sb: bool = True,
               photo_retry: bool = True) -> tuple[bool, Optional[np.ndarray],
                                                  float, str]:
    """实时扫描路径：SB only（故意不做"SB 失败退经典"，见参考项目 4.x 教训）。

    返回 (found, corners_full_res Nx1x2, scan_scale, stage)。stage 供 HUD 显示。
    """
    if bgr is None or bgr.size == 0:
        return False, None, 1.0, "miss"
    gray_s, scale_s = _gray_at(bgr, scan_width)
    extra = _PHOTO_EXTRA_SCAN if photo_retry else 0
    views = photometric_variants(gray_s, extra=extra)
    if use_sb and _HAS_SB:
        for name, view in views:
            corners = _try_sb(view, pattern)
            if corners is None:
                continue
            if name != "raw":
                corners = _refine_on_original(gray_s, corners)
            stage = "sb" if name == "raw" else f"sb-{name}"
            return True, corners / scale_s, scale_s, stage
        return False, None, scale_s, "miss"
    for name, view in views:
        corners = _try_classic(view, pattern)
        if corners is None:
            continue
        if name != "raw":
            corners = _refine_on_original(gray_s, corners)
        stage = "classic" if name == "raw" else f"classic-{name}"
        return True, corners / scale_s, scale_s, stage
    return False, None, scale_s, "miss"


def detect_board(bgr: np.ndarray, pattern: Tuple[int, int], *,
                 max_width: int = 1920,
                 try_scales: Optional[Sequence[float]] = None,
                 subpix: bool = True, use_sb: bool = True,
                 photo_retry: bool = True) -> tuple[bool, Optional[np.ndarray],
                                                    float]:
    """一次性高质量检测（连拍/求 H 用，慢但尽力）。

    返回 (found, corners_full_res Nx1x2, used_scale)。
    """
    if bgr is None or bgr.size == 0:
        return False, None, 1.0
    h, w = bgr.shape[:2]
    base = 1.0
    if max_width > 0 and w > max_width:
        base = float(max_width) / float(w)
    scales = list(try_scales) if try_scales else [1.0, 0.75, 0.5]
    scales = sorted({max(0.25, float(s)) for s in scales}, reverse=True)
    full_gray = None
    for rel in scales:
        scale = base * rel
        if scale >= 0.999:
            gray = cv2.cvtColor(bgr, cv2.COLOR_BGR2GRAY) if bgr.ndim == 3 else bgr
            scale = 1.0
        else:
            nw, nh = max(1, int(round(w * scale))), max(1, int(round(h * scale)))
            small = cv2.resize(bgr, (nw, nh), interpolation=cv2.INTER_AREA)
            gray = (cv2.cvtColor(small, cv2.COLOR_BGR2GRAY)
                    if small.ndim == 3 else small)
        corners = find_board_corners(gray, pattern, use_sb=use_sb,
                                     photo_retry=photo_retry, allow_classic=True)
        if corners is None:
            found, c = cv2.findChessboardCorners(gray, pattern,
                                                 DETECT_FLAGS_FULL)
            if found and c is not None:
                if subpix:
                    c = cv2.cornerSubPix(gray, c, (5, 5), (-1, -1),
                                         _SUBPIX_CRITERIA)
                corners = np.ascontiguousarray(c, dtype=np.float32)
        if corners is None:
            continue
        corners = np.ascontiguousarray(corners, dtype=np.float32)
        if scale != 1.0:
            corners = corners / float(scale)
            if subpix:
                if full_gray is None:
                    full_gray = (cv2.cvtColor(bgr, cv2.COLOR_BGR2GRAY)
                                 if bgr.ndim == 3 else bgr)
                corners = _refine_on_original(full_gray, corners)
        return True, corners, float(scale)
    return False, None, float(base)