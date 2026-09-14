# -*- coding: utf-8 -*-
"""标定 Pipeline 编排 CLI：按阶段执行，评估闸门 fail 即停。

用法:
    ros2 run j501_avm_calib calib_pipeline --stage probe
    ros2 run j501_avm_calib calib_pipeline --stage intrinsics [--auto]
    ros2 run j501_avm_calib calib_pipeline --stage parse
    ros2 run j501_avm_calib calib_pipeline --stage eval_intrinsics
    ros2 run j501_avm_calib calib_pipeline --stage extrinsics
    ros2 run j501_avm_calib calib_pipeline --stage eval_extrinsics
    ros2 run j501_avm_calib calib_pipeline --stage visualize
    ros2 run j501_avm_calib calib_pipeline --stage all [--skip-intrinsics]
选项:
    --auto            内参无头自动标定（无需显示器）
    --skip-intrinsics 跳过内参阶段，复用已有 camera_info YAML
    --direction DIR   只标定某一路内参
    --keep-driver     结束后保持相机驱动运行
阶段闸门: evaluate 阶段 verdict=fail -> 停止（exit 2）；warn -> 继续并提示。
"""
from __future__ import annotations

import argparse
import json
import shutil
import subprocess
import sys
import time
from pathlib import Path

from j501_avm_calib.config import load_config, results_dir, DIRECTIONS

DRIVER_PID = "/tmp/j501_avm_calib_driver.pid"
TAR_BACKUP_DIR = "calib_tar"


def _run(cmd: list[str], check=True, **kw) -> subprocess.CompletedProcess:
    print(f"\n▶ {' '.join(str(c) for c in cmd)}\n", flush=True)
    return subprocess.run(cmd, **kw) if not check else \
        subprocess.run(cmd, check=True, **kw)


def _ros2_run(cmd: list[str], check=True, **kw):
    return _run(["ros2", "run", *cmd], check=check, **kw)


def _topic_exists(topic: str, timeout_s: float = 20.0) -> bool:
    t0 = time.time()
    while time.time() - t0 < timeout_s:
        r = subprocess.run(["ros2", "topic", "list"], capture_output=True,
                           text=True)
        if topic in r.stdout.splitlines():
            return True
        time.sleep(0.5)
    return False


def _start_driver(keep: bool = False) -> subprocess.Popen:
    proc = subprocess.Popen(
        ["ros2", "run", "j501_avm_calib", "camera_driver"],
        stdout=subprocess.PIPE, stderr=subprocess.STDOUT, text=True)
    with open(DRIVER_PID, "w") as f:
        f.write(str(proc.pid))
    print("▶ 相机驱动启动中，等待 /cameras/front/image_raw ...", flush=True)
    if not _topic_exists("/cameras/front/image_raw", 30.0):
        print("❌ 图像话题未出现。检查相机接线/设备号（calib_config.yaml）")
        proc.terminate()
        sys.exit(1)
    print("✅ 相机驱动就绪")
    if keep:
        return proc
    return proc


def _stop_driver(proc: subprocess.Popen | None):
    if proc is None:
        return
    proc.terminate()
    try:
        proc.wait(timeout=5)
    except subprocess.TimeoutExpired:
        proc.kill()
    try:
        Path(DRIVER_PID).unlink(missing_ok=True)
    except OSError:
        pass


def stage_probe():
    _ros2_run(["j501_avm_calib", "camera_driver", "--probe"])


def stage_intrinsics(cfg, direction=None, auto: bool = False) -> bool:
    """GUI(默认)/auto 内参标定。返回 True 表示完成。"""
    cols, rows = cfg["pattern_size"]
    square = cfg["chessboard"]["square_size_m"]
    dirs = [direction] if direction else list(DIRECTIONS)
    ok = True
    for d in dirs:
        print("=" * 60)
        print(f"  内参标定 [{d}]（{'无头自动' if auto else 'cameracalibrator GUI'}）")
        print("=" * 60)
        if auto:
            _ros2_run(["j501_avm_calib", "intrinsics_auto",
                       "--direction", d])
        else:
            print("  操作: 1) GUI 窗口内把 trackbar 切到 fisheye(=1)")
            print("        2) 缓慢移动棋盘覆盖画面各区域直到 CALIBRATE 可点")
            print("        3) CALIBRATE → SAVE → COMMIT（写回本机 YAML）")
            r = _run(["ros2", "run", "camera_calibration", "cameracalibrator",
                      "--size", f"{cols}x{rows}", "--square", f"{square}",
                      "--no-service-check",
                      "--fisheye-check-conditions", "--fisheye-fix-skew",
                      "--fisheye-recompute-extrinsicsts",
                      "--ros-args",
                      "-r", f"image:=/cameras/{d}/image_raw",
                      "-r", f"camera:=/cameras/{d}"], check=False)
            if r.returncode not in (0, 130):
                print(f"❌ [{d}] cameracalibrator 异常退出({r.returncode})")
                ok = False
        tar = Path("/tmp/calibrationdata.tar.gz")
        if tar.is_file():
            rd = results_dir()
            dest = rd / TAR_BACKUP_DIR / f"{d}_calibrationdata.tar.gz"
            dest.parent.mkdir(parents=True, exist_ok=True)
            shutil.copy(tar, dest)
            print(f"✅ [{d}] 采样图已备份: {dest}")
        else:
            print(f"⚠️  未发现 /tmp/calibrationdata.tar.gz（无 SAVE 步骤？"
                  f"rms 将缺失，可从 {d}_calibrationdata.tar.gz 备份回填）")
        # GUI 每路结束后立即解析
        _ros2_run(["j501_avm_calib", "intrinsics_parse", "--direction", d,
                   "--tar", str(tar) if tar.is_file() else ""], check=False)
    return ok


def stage_parse():
    _ros2_run(["j501_avm_calib", "intrinsics_parse", "--all"])


def _gate(scope: str) -> bool:
    """读 evaluation_report.json；verdict==fail -> False。"""
    p = results_dir() / "evaluation_report.json"
    if not p.is_file():
        print(f"⚠️  无评估报告 {p}（未运行评估阶段）")
        return True
    with p.open() as f:
        report = json.load(f)
    verdict = report.get("verdict", "pass")
    print(f"闸门检查 [{scope}]: verdict={verdict}")
    if verdict == "fail":
        print("❌ 评估不合格，pipeline 停止。请按报告修复后重跑对应阶段。")
        return False
    if verdict == "warn":
        print("⚠️  评估有警告，继续但建议关注。")
    return True


def main(args=None):
    parser = argparse.ArgumentParser(description="J501 AVM 标定 pipeline")
    parser.add_argument("--stage", required=True, choices=[
        "probe", "intrinsics", "parse", "eval_intrinsics", "extrinsics",
        "eval_extrinsics", "visualize", "all"])
    parser.add_argument("--auto", action="store_true")
    parser.add_argument("--skip-intrinsics", action="store_true")
    parser.add_argument("--direction", choices=list(DIRECTIONS), default=None)
    parser.add_argument("--keep-driver", action="store_true")
    a, _ = parser.parse_known_args(args)
    cfg = load_config()
    driver = None

    def need_driver():
        nonlocal driver
        if driver is None or driver.poll() is not None:
            driver = _start_driver(a.keep_driver)

    try:
        if a.stage == "probe":
            stage_probe()
            return
        if a.stage == "intrinsics":
            need_driver()
            stage_intrinsics(cfg, a.direction, a.auto)
            return
        if a.stage == "parse":
            stage_parse()
            _ros2_run(["j501_avm_calib", "evaluator", "--once",
                       "--scope", "intrinsics"], check=False)
            return
        if a.stage == "eval_intrinsics":
            _ros2_run(["j501_avm_calib", "evaluator", "--once",
                       "--scope", "intrinsics"])
            return
        if a.stage == "extrinsics":
            need_driver()
            _ros2_run(["j501_avm_calib", "extrinsic_calibrator"])
            return
        if a.stage == "eval_extrinsics":
            _ros2_run(["j501_avm_calib", "evaluator", "--once",
                       "--scope", "extrinsics"])
            return
        if a.stage == "visualize":
            viz = subprocess.Popen(["ros2", "run", "j501_avm_calib",
                                    "bev_publisher", "--rate", "5"])
            eval_node = subprocess.Popen(["ros2", "run", "j501_avm_calib",
                                          "evaluator"])
            rviz = subprocess.Popen([
                "ros2", "run", "rviz2", "rviz2",
                "-d", str(Path(__file__).resolve().parents[2]
                          / "config" / "rviz" / "avm_calib.rviz")])
            print("可视化已启动。Ctrl+C 退出。")
            try:
                viz.wait()
            except KeyboardInterrupt:
                pass
            for p in (viz, eval_node, rviz):
                p.terminate()
            return
        # ---- all ----
        stage_probe()
        need_driver()
        if not a.skip_intrinsics:
            if not stage_intrinsics(cfg, a.direction, a.auto):
                sys.exit(1)
            if not _gate("intrinsics"):
                sys.exit(2)
        else:
            print("⚠️  --skip-intrinsics：直接使用已有 camera_info YAML")
            stage_parse()
            if not _gate("intrinsics"):
                sys.exit(2)
        print("\n▶ 外参标定（向导）")
        _ros2_run(["j501_avm_calib", "extrinsic_calibrator"])
        if not _gate("extrinsics"):
            sys.exit(2)
        print("\n▶ 可视化（RViz）")
        viz = subprocess.Popen(["ros2", "run", "j501_avm_calib",
                                "bev_publisher", "--rate", "5"])
        rviz = subprocess.Popen([
            "ros2", "run", "rviz2", "rviz2",
            "-d", str(Path(__file__).resolve().parents[2]
                      / "config" / "rviz" / "avm_calib.rviz")])
        print("全部阶段完成 ✅  Ctrl+C 退出可视化。")
        try:
            viz.wait()
        except KeyboardInterrupt:
            pass
        for p in (viz, rviz):
            p.terminate()
    finally:
        if not a.keep_driver:
            _stop_driver(driver)


if __name__ == "__main__":
    main()