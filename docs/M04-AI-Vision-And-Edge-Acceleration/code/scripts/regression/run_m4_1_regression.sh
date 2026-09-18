#!/usr/bin/env bash
# M4.1 regression — YOLO11n TensorRT detector end-to-end gate.
#
# Verifies M4.1 deliverables:
#   1. ONNX + engine artefacts present
#   2. bev_detection package installed in ros2_ws
#   3. Unit tests pass (colcon test)
#   4. Smoke test (single image -> detections)
#   5. 30 s FPS benchmark (mean latency + Hz)
#
# Usage: scripts/regression/run_m4_1_regression.sh [--no-build] [--no-bench]
#   --no-build  do not build bev_detection if missing (fail instead)
#   --no-bench  skip the 30 s benchmark (smoke test only)
# Exit: 0 = PASS, 1 = FAIL. Artifacts: output/regression/m4_1/
set -uo pipefail

REPO="${REPO:-/home/seeed/mobile-robot-full-stack-course/modules/m04-ai-vision-and-edge-acceleration}"
ENGINE="$REPO/models/m4/detection/engines/yolo11n_fp16.engine"
ONNX="$REPO/models/m4/detection/onnx/yolo11n.onnx"
OUT="$REPO/output/regression/m4_1"
BENCH="$REPO/output/m4/4.1/benchmark.json"

NO_BUILD=0
NO_BENCH=0
for a in "$@"; do
  case "$a" in
    --no-build) NO_BUILD=1 ;;
    --no-bench) NO_BENCH=1 ;;
    *) echo "unknown option: $a" >&2; exit 2 ;;
  esac
done

FAIL=0
pass() { printf '  [PASS] %s\n' "$*"; }
fail() { printf '  [FAIL] %s\n' "$*"; FAIL=1; }
info() { printf '  [info] %s\n' "$*"; }

mkdir -p "$OUT"
echo "== M4.1 regression ($(date -Is)) =="

# --- 1. artefacts present -----------------------------------------------------
if [ -s "$ONNX" ]; then
  pass "onnx present: $(basename "$ONNX") ($(du -h "$ONNX" | cut -f1))"
else
  fail "missing ONNX: $ONNX"
fi
if [ -s "$ENGINE" ]; then
  pass "engine present: $(basename "$ENGINE") ($(du -h "$ENGINE" | cut -f1))"
else
  fail "missing engine: $ENGINE"
fi

# --- 2. package build --------------------------------------------------------
set +u
source /opt/ros/humble/setup.bash 2>/dev/null
cd "$REPO/ros2_ws"
source install/setup.bash 2>/dev/null
set -u

EXE="$REPO/ros2_ws/install/bev_detection/lib/bev_detection/yolo_trt_node"
if [ -x "$EXE" ]; then
  pass "bev_detection installed (yolo_trt_node)"
elif [ "$NO_BUILD" -eq 1 ]; then
  fail "bev_detection missing and --no-build given"
else
  info "building bev_detection…"
  if colcon build --packages-select bev_detection --cmake-args -DBUILD_TESTING=ON \
        >"$OUT/colcon_build.log" 2>&1; then
    pass "bev_detection built"
  else
    fail "bev_detection build (see $OUT/colcon_build.log)"
  fi
fi

# --- 3. unit tests ------------------------------------------------------------
if [ -f "$REPO/ros2_ws/build/bev_detection/CTestTestfile.cmake" ]; then
  if (cd "$REPO/ros2_ws" && colcon test --packages-select bev_detection \
        --event-handlers console_direct+ > "$OUT/colcon_test.log" 2>&1); then
    if (cd "$REPO/ros2_ws" && colcon test-result --all --verbose \
          > "$OUT/test_result.txt" 2>&1); then
      if grep -E "Summary:.*0 (errors|failures)" "$OUT/test_result.txt" >/dev/null; then
        pass "unit tests pass (colcon test-result: 0 errors, 0 failures)"
      else
        fail "unit tests reported failures (see $OUT/test_result.txt)"
      fi
    else
      info "colcon test-result unavailable; consult $OUT/colcon_test.log"
      pass "unit tests executed"
    fi
  else
    fail "colcon test failed (see $OUT/colcon_test.log)"
  fi
else
  info "no CTest build found; skipping unit-test gate"
fi

# --- 4. smoke test (single image -> detections) ------------------------------
if [ -x "$REPO/scripts/m4/test_yolo_node.sh" ]; then
  if bash "$REPO/scripts/m4/test_yolo_node.sh" > "$OUT/smoke.log" 2>&1; then
    pass "smoke test passed"
  else
    fail "smoke test failed (see $OUT/smoke.log)"
  fi
else
  info "scripts/m4/test_yolo_node.sh missing"
fi

# --- 4b. empty-frame contract smoke (M4.1 -> M4.2 contract) ----------------
if [ -x "$REPO/scripts/m4/test_empty_frame_contract.sh" ]; then
  if bash "$REPO/scripts/m4/test_empty_frame_contract.sh" > "$OUT/empty_frame.log" 2>&1; then
    pass "empty-frame contract test passed"
  else
    fail "empty-frame contract test failed (see $OUT/empty_frame.log)"
  fi
else
  info "scripts/m4/test_empty_frame_contract.sh missing"
fi

# --- 5. 30 s FPS benchmark ----------------------------------------------------
if [ "$NO_BENCH" -eq 1 ]; then
  info "benchmark skipped (--no-bench)"
elif [ -x "$REPO/scripts/m4/run_m4_1_benchmark.sh" ]; then
  info "running 30 s benchmark…"
  if bash "$REPO/scripts/m4/run_m4_1_benchmark.sh" 30 > "$OUT/bench.log" 2>&1 \
     && [ -s "$BENCH" ]; then
    RATE=$(python3 -c "import json;print(json.load(open('$BENCH'))['inference_rate_hz'])" 2>/dev/null || echo 0)
    MEAN_INF=$(python3 -c "import json;print(json.load(open('$BENCH'))['mean_inference_ms'])" 2>/dev/null || echo 0)
    MEAN_E2E=$(python3 -c "import json;print(json.load(open('$BENCH'))['mean_e2e_ms'])" 2>/dev/null || echo 0)
    pass "benchmark rate=${RATE} Hz, inf=${MEAN_INF} ms, e2e=${MEAN_E2E} ms"
  else
    fail "benchmark run (see $OUT/bench.log)"
  fi
else
  info "scripts/m4/run_m4_1_benchmark.sh missing"
fi

if [ "$FAIL" -eq 0 ]; then
  echo "== M4.1 regression: PASS =="
else
  echo "== M4.1 regression: FAIL =="
fi
exit "$FAIL"
