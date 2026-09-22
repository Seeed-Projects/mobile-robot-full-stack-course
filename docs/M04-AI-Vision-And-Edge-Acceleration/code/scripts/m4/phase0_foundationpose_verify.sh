#!/usr/bin/env bash
# scripts/m4/phase0_foundationpose_verify.sh
#
# Phase 0 — standalone NVlabs/FoundationPose deployment verification.
#
# PURPOSE
#   Verify that the NVlabs/FoundationPose backend can run on this Jetson
#   BEFORE we wrap it in ROS2. This isolates environment problems
#   (CUDA / torch / nvdiffrast / model weights) from ROS2 integration
#   problems, so we can fail fast and fix the right layer.
#
# STEPS (all must PASS for Phase 0 to pass)
#   1. PyTorch + CUDA smoke          (cuda.is_available(), version, device 0)
#   2. GPU detail probe             (nvidia-smi --query-gpu=*)
#   3. nvdiffrast.torch import      (renderer check)
#   4. FoundationPose model files   (weights/refine_model.pt, score_model.pt)
#   5. Official register example    (test/cup/ frame 1 → initial pose)
#   6. Official tracking example    (test/cup/ sequence → pose sequence)
#
# OUTPUT
#   output/m4/phase0/phase0_report.json
#   output/m4/phase0/phase0.log       (full transcript)
#
# If any step fails, phase0 returns non-zero; Phase 1 must not begin.

set -u

# FoundationPose needs torch, trimesh and nvdiffrast, all of which live in the
# conda env. The system python3 cannot import torch at all, so running this
# script under it fails at step 1 for a reason that looks like a code bug.
PYTHON="${PYTHON:-/home/seeed/miniconda3/envs/py310/bin/python}"
[[ -x "$PYTHON" ]] || PYTHON=python3

# ---- repo + paths ---------------------------------------------------
SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
# --- module anchors: derived from this script's own location, never hardcoded ---
# M4_ROOT is this M04 module; WS_ROOT is the repository root.
M4_ROOT="${M4_ROOT:-$(cd "$(dirname "${BASH_SOURCE[0]}")/../.." && pwd)}"
WS_ROOT="${WS_ROOT:-$(cd "$M4_ROOT/../.." && pwd)}"
FOUNDATIONPOSE_DIR="${FOUNDATIONPOSE_DIR:-${HOME}/FoundationPose}"
WEIGHTS_DIR="${FOUNDATIONPOSE_WEIGHTS:-$FOUNDATIONPOSE_DIR/weights}"
TESTDATA_DIR="${FOUNDATIONPOSE_TESTDATA:-$M4_ROOT/models/m4/pose/test/cup}"

OUT_DIR="$M4_ROOT/output/m4/phase0"
mkdir -p "$OUT_DIR"
LOG="$OUT_DIR/phase0.log"
REPORT="$OUT_DIR/phase0_report.json"
: > "$LOG"

report_json="{}"

# ---- colors --------------------------------------------------------
GREEN=$'\e[0;32m'; RED=$'\e[0;31m'; YELLOW=$'\e[1;33m'; NC=$'\e[0m'
ok()   { echo "${GREEN}  [OK]   $*${NC}" | tee -a "$LOG"; }
fail() { echo "${RED}  [FAIL] $*${NC}" | tee -a "$LOG"; }
warn() { echo "${YELLOW}  [WARN] $*${NC}" | tee -a "$LOG"; }
note() { echo "  [INFO] $*" | tee -a "$LOG"; }

# ---- helpers -------------------------------------------------------
upsert_json() {
    # upsert_json "<key>" "<value>"  (value is JSON-encoded string)
    local key="$1"
    local val="$2"
    report_json=$("$PYTHON" -c "
import json,sys
d = json.loads(r'''$report_json''')
d['$key'] = json.loads(r'''$val''')
print(json.dumps(d, indent=2))
")
}

# =================================================================
# Step 1: PyTorch + CUDA smoke
# =================================================================
echo "== Step 1: PyTorch + CUDA ==" | tee -a "$LOG"
STEP1_OUT=$("$PYTHON" - <<'PY' 2>&1 || echo "ERROR"
import json
try:
    import torch
    out = {
        "torch_version": torch.__version__,
        "cuda_available": bool(torch.cuda.is_available()),
        "cuda_version": torch.version.cuda,
        "device_count": torch.cuda.device_count(),
    }
    if out["cuda_available"]:
        out["device_name"] = torch.cuda.get_device_name(0)
        out["device_capability"] = list(torch.cuda.get_device_capability(0))
    print(json.dumps(out))
except Exception as e:
    print("ERROR", repr(e))
PY
)
echo "$STEP1_OUT" | tee -a "$LOG"

if echo "$STEP1_OUT" | grep -q '^ERROR'; then
    fail "PyTorch import failed"
    upsert_json "step1_torch" "false"
    upsert_json "phase0_pass" "false"
    echo "$report_json" > "$REPORT"
    exit 2
fi
upsert_json "step1_torch" "$STEP1_OUT"
if echo "$STEP1_OUT" | "$PYTHON" -c "import json,sys;d=json.loads(sys.stdin.read());sys.exit(0 if d['cuda_available'] else 1)"; then
    ok "PyTorch sees a CUDA device"
else
    fail "PyTorch cannot see a CUDA device; FoundationPose will not run"
    upsert_json "phase0_pass" "false"
    echo "$report_json" > "$REPORT"
    exit 2
fi

# =================================================================
# Step 2: nvidia-smi detail
# =================================================================
echo "== Step 2: nvidia-smi ==" | tee -a "$LOG"
SMILINE=$(nvidia-smi --query-gpu=name,memory.total,driver_version --format=csv,noheader 2>/dev/null | head -1 || echo "")
if [ -z "$SMILINE" ]; then
    fail "nvidia-smi returned nothing"
    upsert_json "phase0_pass" "false"
    echo "$report_json" > "$REPORT"
    exit 2
fi
ok "nvidia-smi: $SMILINE"
upsert_json "step2_smi" "\"$SMILINE\""

# =================================================================
# Step 3: nvdiffrast import
# =================================================================
echo "== Step 3: nvdiffrast.torch import ==" | tee -a "$LOG"
STEP3_OUT=$("$PYTHON" -c "
try:
    import nvdiffrast.torch as dr
    print('OK')
except Exception as e:
    print('ERROR', repr(e))
" 2>&1 || echo "ERROR")
echo "$STEP3_OUT" | tee -a "$LOG"
if [ "$STEP3_OUT" = "OK" ]; then
    ok "nvdiffrast.torch import OK"
    upsert_json "step3_renderer" "true"
else
    fail "nvdiffrast.torch import failed; install via foundationpose/setup.md"
    upsert_json "step3_renderer" "false"
    upsert_json "phase0_pass" "false"
    echo "$report_json" > "$REPORT"
    exit 2
fi

# =================================================================
# Step 4: model files
# =================================================================
echo "== Step 4: model weights ==" | tee -a "$LOG"
REFINE_OK=0; SCORE_OK=0
if [ -f "$WEIGHTS_DIR/refine_model.pt" ]; then
    ok "weights/refine_model.pt present"
    REFINE_OK=1
else
    fail "missing: $WEIGHTS_DIR/refine_model.pt"
    fail "  obtain via:"
    fail "    mkdir -p $WEIGHTS_DIR && cd $WEIGHTS_DIR"
    fail "    wget -q https://github.com/NVlabs/FoundationPose/releases/download/v1.1/refine_model.pt"
fi
if [ -f "$WEIGHTS_DIR/score_model.pt" ]; then
    ok "weights/score_model.pt present"
    SCORE_OK=1
else
    fail "missing: $WEIGHTS_DIR/score_model.pt"
    fail "  obtain via:"
    fail "    mkdir -p $WEIGHTS_DIR && cd $WEIGHTS_DIR"
    fail "    wget -q https://github.com/NVlabs/FoundationPose/releases/download/v1.1/score_model.pt"
fi
upsert_json "step4_models" "{\"refine_present\": $REFINE_OK, \"score_present\": $SCORE_OK}"
if [ "$REFINE_OK" = "0" ] || [ "$SCORE_OK" = "0" ]; then
    fail "weights missing; cannot proceed"
    upsert_json "phase0_pass" "false"
    echo "$report_json" > "$REPORT"
    exit 2
fi

# =================================================================
# Step 5: official register example
# =================================================================
echo "== Step 5: official register example ==" | tee -a "$LOG"
if [ ! -d "$TESTDATA_DIR/rgb" ]; then
    warn "test data missing under $TESTDATA_DIR"
    warn "  fetch via: bash scripts/m4/fetch_foundationpose_testdata.sh"
    upsert_json "step5_register" "false"
    upsert_json "step5_register_msg" "\"test data missing\""
else
    cd "$FOUNDATIONPOSE_DIR" || cd "$M4_ROOT"
    STEP5_OUT=$("$PYTHON" - <<PY 2>&1 || echo "ERROR"
import sys, os, json
import numpy as np
import torch
sys.path.insert(0, "$FOUNDATIONPOSE_DIR")
try:
    from estimater import FoundationPose
    from datareader import YcbineoatReader

    mesh_file = "$TESTDATA_DIR/textured.obj"
    reader = YcbineoatReader(
        rgb_dir="$TESTDATA_DIR/rgb",
        depth_dir="$TESTDATA_DIR/depth",
        mask_dir="$TESTDATA_DIR/mask",
        camera_K="$TESTDATA_DIR/cam_K.txt",
        start_frame=0,
        end_frame=1,
    )

    device = torch.device('cuda:0')
    import trimesh, nvdiffrast.torch as dr
    _mesh = trimesh.load(mesh_file)
    # Upstream: FoundationPose(model_pts, model_normals, ..., mesh, scorer,
    # refiner, glctx, debug, debug_dir). model_pts/normals are ARRAYS.
    # ScorePredictor(amp=True) and PoseRefinePredictor() take no weights path;
    # they resolve their own checkpoints under the checkout's weights/.
    est = FoundationPose(model_pts=_mesh.vertices,
                         model_normals=_mesh.vertex_normals,
                         mesh=_mesh,
                         scorer=ScorePredictor(),
                         refiner=PoseRefinePredictor(),
                         glctx=dr.RasterizeCudaContext(),
                         debug=0,
                         debug_dir="$WEIGHTS_DIR")

    rgb = reader.get_rgbs()[0]
    depth = reader.get_depths()[0]
    K = reader.K
    mask = reader.get_masks()[0]

    import time
    t0 = time.monotonic()
    pose = est.register(K=K, rgb=rgb, depth=depth, ob_mask=mask, iteration=5)
    dt = time.monotonic() - t0
    print("REGISTER_OK", "%.3fs" % dt, pose.shape, json.dumps(pose[0, :3, :3].tolist()))
except Exception as e:
    print("REGISTER_FAIL", repr(e))
PY
)
    echo "$STEP5_OUT" | tee -a "$LOG"
    if echo "$STEP5_OUT" | grep -q '^REGISTER_OK'; then
        ok "official register example succeeded"
        upsert_json "step5_register" "true"
        REGISTER_LATENCY=$(echo "$STEP5_OUT" | awk '/^REGISTER_OK/{print $2}')
        upsert_json "step5_register_latency" "\"$REGISTER_LATENCY\""
    else
        fail "official register example failed; see $LOG"
        upsert_json "step5_register" "false"
        upsert_json "step5_register_msg" "\"$STEP5_OUT\""
    fi
fi

# =================================================================
# Step 6: official tracking example
# =================================================================
echo "== Step 6: official tracking example ==" | tee -a "$LOG"
if [ ! -d "$TESTDATA_DIR/rgb" ]; then
    warn "test data missing under $TESTDATA_DIR"
    upsert_json "step6_tracking" "false"
else
    cd "$FOUNDATIONPOSE_DIR" || cd "$M4_ROOT"
    STEP6_OUT=$("$PYTHON" - <<PY 2>&1 || echo "ERROR"
import sys, os, json, time
import numpy as np
import torch
sys.path.insert(0, "$FOUNDATIONPOSE_DIR")
try:
    from estimater import FoundationPose, ScorePredictor, PoseRefinePredictor
    from datareader import YcbineoatReader

    mesh_file = "$TESTDATA_DIR/textured.obj"
    reader = YcbineoatReader(
        rgb_dir="$TESTDATA_DIR/rgb",
        depth_dir="$TESTDATA_DIR/depth",
        mask_dir="$TESTDATA_DIR/mask",
        camera_K="$TESTDATA_DIR/cam_K.txt",
        start_frame=0,
        end_frame=min(20, len(os.listdir("$TESTDATA_DIR/rgb"))) if os.path.isdir("$TESTDATA_DIR/rgb") else 1,
    )

    device = torch.device('cuda:0')
    import trimesh, nvdiffrast.torch as dr
    _mesh = trimesh.load(mesh_file)
    est = FoundationPose(model_pts=_mesh.vertices,
                         model_normals=_mesh.vertex_normals,
                         mesh=_mesh,
                         scorer=ScorePredictor(),
                         refiner=PoseRefinePredictor(),
                         glctx=dr.RasterizeCudaContext(),
                         debug=0,
                         debug_dir="$WEIGHTS_DIR")

    rgbs = reader.get_rgbs()
    depths = reader.get_depths()
    masks = reader.get_masks()
    K = reader.K
    n = min(len(rgbs), 20)

    pose = None
    t_per_frame = []
    for i in range(n):
        t0 = time.monotonic()
        if pose is None:
            pose = est.register(K=K, rgb=rgbs[i], depth=depths[i], ob_mask=masks[i], iteration=5)
        else:
            pose = est.track_one(rgb=rgbs[i], depth=depths[i], K=K, iteration=5)
        t_per_frame.append(time.monotonic() - t0)
    fps = 1.0 / (sum(t_per_frame) / len(t_per_frame))
    print("TRACK_OK", "n=%d mean=%.3fs fps=%.2f" % (n, sum(t_per_frame)/len(t_per_frame), fps))
except Exception as e:
    print("TRACK_FAIL", repr(e))
PY
)
    echo "$STEP6_OUT" | tee -a "$LOG"
    if echo "$STEP6_OUT" | grep -q '^TRACK_OK'; then
        ok "official tracking example succeeded"
        upsert_json "step6_tracking" "true"
        TRACK_FPS=$(echo "$STEP6_OUT" | awk '/^TRACK_OK/{for(i=1;i<=NF;i++)if($i~/^fps=/){print substr($i,5)}}')
        upsert_json "step6_track_fps" "$TRACK_FPS"
    else
        fail "official tracking example failed; see $LOG"
        upsert_json "step6_tracking" "false"
        upsert_json "step6_tracking_msg" "\"$STEP6_OUT\""
    fi
fi

# ---- final pass/fail ----------------------------------------------
PHASE0_PASS="true"
for k in step1_torch.cuda_available step3_renderer step4_models step5_register step6_tracking; do
    v=$("$PYTHON" -c "import json;d=json.loads(r'''$report_json''');print(d.get('$k', False))" 2>/dev/null || echo False)
    if [ "$v" != "True" ] && [ "$v" != "true" ]; then
        PHASE0_PASS="false"; break
    fi
done
upsert_json "phase0_pass" "$PHASE0_PASS"
echo "$report_json" > "$REPORT"

echo "" | tee -a "$LOG"
echo "=== Phase 0 summary ===" | tee -a "$LOG"
echo "  report:   $REPORT" | tee -a "$LOG"
echo "  log:      $LOG" | tee -a "$LOG"
if [ "$PHASE0_PASS" = "true" ]; then
    ok "Phase 0 PASS — ROS2 integration (Phase 1) may begin"
    exit 0
else
    fail "Phase 0 FAIL — resolve the failures above before starting Phase 1"
    exit 2
fi
