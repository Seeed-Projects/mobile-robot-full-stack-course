# -*- coding: utf-8 -*-
"""标定效果评估节点/脚本：内参(K/D/RMS/FOV) + 外参(H-QC/接缝/验证位) +
pose 合理性，输出 JSON 报告 + 终端摘要 + ROS 诊断消息。

用法:
    ros2 run j501_avm_calib evaluator --once [--scope intrinsics|extrinsics|all]
                # 一次性评估并退出；exit 0=pass 1=warn 2=fail（pipeline 闸门）
    ros2 run j501_avm_calib evaluator            # 常驻节点，发布诊断与报告
    ros2 run j501_avm_calib evaluator --images-dir <root>   # 重算逐视角 RMS
"""
from __future__ import annotations

import argparse
import datetime
import json
import sys
from pathlib import Path

import numpy as np
import cv2
import rclpy
from rclpy.node import Node
from std_msgs.msg import String
from diagnostic_msgs.msg import DiagnosticArray, DiagnosticStatus, KeyValue

from j501_avm_calib.config import load_config, results_dir, DIRECTIONS
from j501_avm_calib.intrinsic_quality import evaluate_intrinsics
import j501_avm_calib.homography as HOM


def _load_intrinsics(path: Path):
    with path.open() as f:
        return json.load(f)


def _recompute_views(d: str, images: list[Path], K, D):
    """对采集图重检角点 -> 逐视角 RMS（无图像返回全 None）。"""
    from j501_avm_calib.detect_board import find_board_corners
    from j501_avm_calib.intrinsic_quality import compute_per_view_errors
    cfg = load_config()
    cols, rows = cfg["pattern_size"]
    square = float(cfg["chessboard"]["square_size_m"])
    objp = np.zeros((cols * rows, 1, 3), np.float64)
    objp[:, 0, :2] = np.mgrid[0:cols, 0:rows].T.reshape(-1, 2) * square
    obj_pts, img_pts, rvecs, tvecs = [], [], [], []
    for p in images:
        bgr = cv2.imread(str(p))
        if bgr is None:
            continue
        gray = cv2.cvtColor(bgr, cv2.COLOR_BGR2GRAY)
        c = find_board_corners(gray, (cols, rows), use_sb=True,
                               photo_retry=True, allow_classic=True)
        if c is None:
            continue
        cc = c.reshape(-1, 2)
        try:
            ok, rv, tv, _ = cv2.solvePnP(objp, cc, K, D,
                                         flags=cv2.SOLVEPNP_ITERATIVE)
            if not ok:
                continue
        except cv2.error:
            continue
        obj_pts.append(objp)
        img_pts.append(cc)
        rvecs.append(np.asarray(rv, dtype=np.float64).reshape(3, 1))
        tvecs.append(np.asarray(tv, dtype=np.float64).reshape(3, 1))
    if not obj_pts:
        return None, None, None, None, None, None
    per_view = compute_per_view_errors(obj_pts, img_pts, K, D, rvecs, tvecs)
    rms = float(np.sqrt(np.mean([e ** 2 for e in per_view])))
    return obj_pts, img_pts, rvecs, tvecs, per_view, rms


def _compute_intrinsic_report(d: str, data: dict, images: list[Path] | None,
                              qc_cfg: dict) -> dict:
    """单路内参评估（无图像时仅 K/D 合理性 + rms 数值判读）。"""
    K = np.asarray(data["K"], dtype=np.float64)
    D = np.asarray(data["D"], dtype=np.float64)
    size = data.get("image_size") or [1920, 1536]
    w, h = int(size[0]), int(size[1])
    rms = data.get("rms")
    obj_pts = img_pts = rvecs = tvecs = None
    per_view = data.get("per_view_rms")
    if images:
        obj_pts, img_pts, rvecs, tvecs, per_view, rms = _recompute_views(
            d, images, K, D)
    report = evaluate_intrinsics(
        K, D, float(rms) if rms is not None else 999.0, (w, h),
        obj_points=obj_pts, img_points=img_pts, rvecs=rvecs, tvecs=tvecs,
        per_view_errors=per_view, model=data.get("model", "equidistant"),
        qc=qc_cfg)
    report["rms_source"] = ("recomputed" if images
                            else "stored" if rms is not None else "missing")
    return report


def _extrinsic_report(extr_path: Path, cfg: dict) -> dict:
    """外参报告：H-QC / rms 阈值 / LR 对称 / 接缝 / 验证位 / pose。

    无 extrinsics.json 时 status='warn'（未完成状态，不算合格）。
    """
    out = {"available": False, "status": "warn", "directions": {},
           "warnings": ["尚未标定外参（无 extrinsics.json）"]}
    if not extr_path.is_file():
        return out
    with extr_path.open() as f:
        data = json.load(f)
    out["available"] = True
    out["warnings"] = []
    qc_cfg = cfg["qc"]
    n_bad = n_warn = 0
    for d in DIRECTIONS:
        if d not in data.get("homographies", {}):
            continue
        qc = (data.get("homography_qc") or {}).get(d) or {}
        rms = (data.get("rms_errors") or {}).get(d)
        warnings = list(qc.get("warnings") or [])
        if qc.get("status") == "bad":
            status = "fail"
        elif qc.get("status") == "warn":
            status = "warn"
        else:
            status = "pass"
        if rms is not None:
            r = float(rms)
            if r >= qc_cfg["rms_ok_px"]:
                status = "fail"
                warnings.append(
                    f"rms={r:.3f}px >= {qc_cfg['rms_ok_px']}px 不合格")
            elif r >= qc_cfg["rms_good_px"]:
                if status == "pass":
                    status = "warn"
                warnings.append(
                    f"rms={r:.3f}px >= {qc_cfg['rms_good_px']}px 偏低")
        entry = {"status": status, "rms": rms, "qc": qc.get("status"),
                 "warnings": warnings}
        pose = (data.get("poses") or {}).get(d)
        if pose:
            t = np.asarray(pose["t"], dtype=np.float64).ravel()
            pq = cfg["pose_qc"]
            hgt = float(t[2])
            entry["pose_height_m"] = hgt
            if not (pq["height_min_m"] <= hgt <= pq["height_max_m"]):
                entry["warnings"].append(
                    f"解算相机高度 {hgt:.3f}m 超范围"
                    f"[{pq['height_min_m']},{pq['height_max_m']}]（仅供参考）")
        out["directions"][d] = entry
        if status == "fail":
            n_bad += 1
        elif status == "warn":
            n_warn += 1

    lr = HOM.check_lr_symmetry(
        {d: np.asarray(m) for d, m in data.get("homographies", {}).items()},
        qc_cfg["lr_sigma_ratio_max"])
    if lr:
        out["warnings"].extend(lr)
        if n_bad == 0:
            n_warn += 1

    seams = data.get("seam_refined") or []
    if seams:
        out["seam"] = {
            "count": len(seams),
            "mean_improved_px": float(np.mean(
                [s.get("improved_px", 0) for s in seams])),
            "max_rms_after_px": float(max(
                s.get("rms_after_px", 0) for s in seams)),
        }
        if out["seam"]["max_rms_after_px"] > 2.0:
            out["warnings"].append(
                f"接缝精修后最大 rms {out['seam']['max_rms_after_px']:.2f}px"
                " > 2px：重叠区仍存明显错位")
            if n_bad == 0:
                n_warn += 1

    verifs = data.get("verifications") or []
    if verifs:
        out["verifications"] = verifs
        bad = [v for v in verifs if v.get("error_mean_px", 9e9) >= 5.0]
        if bad:
            out["warnings"].append(
                f"{len(bad)} 个验证位误差 >= 5px：外参在远离标定板处精度不足")
            n_bad += 1
        else:
            out["warnings"].append("✅ 验证位均 <5px，外参跨区域一致性良好")

    missing = [d for d in DIRECTIONS if d not in data.get("homographies", {})]
    if missing:
        out["status"] = "warn"
        out["warnings"].append(f"缺少 {missing} 路外参（未完成四路标定）")
    elif n_bad:
        out["status"] = "fail"
    elif n_warn:
        out["status"] = "warn"
    return out


_SCHEME = {"pass": 0, "warn": 1, "fail": 2}


def build_report(images_root: Path | None = None) -> dict:
    """完整评估 -> report dict。"""
    cfg = load_config()
    rd = results_dir()
    now = datetime.datetime.now().isoformat(timespec="seconds")
    report = {"generated_at": now, "verdict": "pass", "intrinsics": {},
              "extrinsics": {}, "summary": []}
    worst = 0
    for d in DIRECTIONS:
        p = rd / f"{d}.json"
        if not p.is_file():
            report["intrinsics"][d] = {"status": "fail",
                                       "error": f"缺少 {p}"}
            worst = max(worst, 2)
            report["summary"].append(f"[{d}] 无内参结果文件")
            continue
        data = _load_intrinsics(p)
        images = None
        if images_root:
            ip = images_root / d
            if ip.is_dir():
                images = sorted(ip.glob("*.png")) + sorted(ip.glob("*.jpg"))
        r = _compute_intrinsic_report(d, data, images, cfg["intrinsic_qc"])
        report["intrinsics"][d] = r
        worst = max(worst, _SCHEME[r["status"]])
        if r["status"] != "pass":
            report["summary"].extend(
                f"[内参 {d}] {w}" for w in r["warnings"])
    ex = _extrinsic_report(rd / "extrinsics.json", cfg)
    report["extrinsics"] = ex
    worst = max(worst, _SCHEME[ex["status"]])
    for w in ex["warnings"]:
        report["summary"].append(f"[外参] {w}")
    report["verdict"] = {0: "pass", 1: "warn", 2: "fail"}[min(worst, 2)]
    return report


def print_summary(report: dict, scope: str = "all"):
    print("\n" + "=" * 64)
    icon = {"pass": "✅", "warn": "⚠️", "fail": "❌"}
    print(f"  标定评估报告  {icon[report['verdict']]} "
          f"{report['verdict'].upper()}")
    print("=" * 64)
    if scope in ("all", "intrinsics"):
        print("  [内参]")
        for d, r in report["intrinsics"].items():
            if r.get("error"):
                print(f"    {d:6s} ❌ {r['error']}")
                continue
            rms_txt = ("N/A" if r["overall_rms"] > 900
                       else f"{r['overall_rms']:.4f}px")
            print(f"    {d:6s} {icon[r['status']]} rms={rms_txt} "
                  f"({r['rms_source']})")
            for w in r["warnings"]:
                print(f"           ⚠️  {w}")
    if scope in ("all", "extrinsics"):
        ex = report["extrinsics"]
        if not ex["available"]:
            print("  [外参] 尚未标定（无 extrinsics.json）")
        else:
            print("  [外参]")
            for d, r in ex["directions"].items():
                rms_txt = ("N/A" if r["rms"] is None
                           else f"{r['rms']:.3f}px")
                h_txt = (f"  高度={r['pose_height_m']:.3f}m"
                         if "pose_height_m" in r else "")
                print(f"    {d:6s} {icon[r['status']]} rms={rms_txt}"
                      f"  QC={r['qc']}{h_txt}")
            if ex.get("seam"):
                s = ex["seam"]
                print(f"    接缝精修 {s['count']} 次，平均改善 "
                      f"{s['mean_improved_px']:.2f}px，"
                      f"精修后最大 rms {s['max_rms_after_px']:.2f}px")
            for w in ex["warnings"]:
                print(f"           ⚠️  {w}")
    print("-" * 64)
    print("  结论: " + ("合格 ✅" if report["verdict"] == "pass" else
                       "可用但需关注 ⚠️" if report["verdict"] == "warn" else
                       "不合格 ❌，请按上方条目修正后重新评估"))
    print("=" * 64 + "\n")


class EvaluatorNode(Node):
    def __init__(self):
        super().__init__("calib_evaluator")
        self.report_pub = self.create_publisher(String,
                                                "/calib/eval/report", 1)
        self.diag_pub = self.create_publisher(DiagnosticArray,
                                              "/diagnostics", 1)
        self.timer = self.create_timer(5.0, self._tick)
        self.get_logger().info("标定评估节点启动，每 5s 发布评估报告/诊断")

    def _tick(self):
        report = build_report()
        msg = String()
        msg.data = json.dumps(report, ensure_ascii=False, indent=2)
        self.report_pub.publish(msg)
        self.diag_pub.publish(self._to_diagnostics(report))

    def _to_diagnostics(self, report: dict) -> DiagnosticArray:
        def status_of(s):
            if s == "pass":
                return DiagnosticStatus.OK
            if s == "warn":
                return DiagnosticStatus.WARN
            return DiagnosticStatus.ERROR
        arr = DiagnosticArray()
        arr.header.stamp = self.get_clock().now().to_msg()
        for d in DIRECTIONS:
            r = report["intrinsics"].get(d) or {}
            st = DiagnosticStatus()
            st.name = f"avm_calib/Intrinsic/{d}"
            st.hardware_id = "j501_avm_calib"
            st.level = status_of(r.get("status", "fail"))
            st.message = ("rms=N/A" if r.get("overall_rms", 999) > 900
                          else f"rms={r['overall_rms']:.3f}px")
            st.values = [KeyValue(key=k, value=str(v))
                         for k, v in r.items()
                         if not isinstance(v, (dict, list))]
            arr.status.append(st)
        ex = report["extrinsics"]
        st = DiagnosticStatus()
        st.name = "avm_calib/Extrinsic"
        st.hardware_id = "j501_avm_calib"
        st.level = status_of(ex["status"])
        st.message = ("未标定" if not ex["available"]
                      else f"{len(ex['directions'])} 路已标定")
        arr.status.append(st)
        st = DiagnosticStatus()
        st.name = "avm_calib/Overall"
        st.hardware_id = "j501_avm_calib"
        st.level = status_of(report["verdict"])
        st.message = report["verdict"]
        st.values = [KeyValue(key=f"summary[{i}]", value=w)
                     for i, w in enumerate(report["summary"][:20])]
        arr.status.append(st)
        return arr


def main(args=None):
    parser = argparse.ArgumentParser(description="标定效果评估")
    parser.add_argument("--once", action="store_true",
                        help="评估一次并退出（exit: 0 pass / 1 warn / 2 fail）")
    parser.add_argument("--scope", default="all",
                        choices=["intrinsics", "extrinsics", "all"])
    parser.add_argument("--images-dir", type=str, default=None,
                        help="每路 <dir>/<images> 重算逐视角 RMS")
    a, _ = parser.parse_known_args(args)
    if a.once:
        report = build_report(Path(a.images_dir) if a.images_dir else None)
        out = results_dir()
        out.mkdir(parents=True, exist_ok=True)
        with (out / "evaluation_report.json").open(
                "w", encoding="utf-8") as f:
            json.dump(report, f, ensure_ascii=False, indent=2)
        print_summary(report, a.scope)
        sys.exit(_SCHEME[report["verdict"]])
    rclpy.init(args=args)
    node = EvaluatorNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    except rclpy.executors.ExternalShutdownException:
        pass
    node.destroy_node()
    if rclpy.ok():
        rclpy.shutdown()


if __name__ == "__main__":
    main()