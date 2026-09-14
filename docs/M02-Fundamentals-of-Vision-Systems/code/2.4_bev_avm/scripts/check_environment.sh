#!/usr/bin/env bash
# Phase 0 environment gate: collect the full environment fingerprint.
# Read-only; never modifies the system. Output: diagnostics/environment.json
set -u
REPO_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
OUT_DIR="$REPO_DIR/diagnostics"
OUT_FILE="$OUT_DIR/environment.json"
mkdir -p "$OUT_DIR"

# --- helpers ---------------------------------------------------------------
jget() { # field fallback
  local v; v="$(eval "$1" 2>/dev/null | head -n1 | tr -d '\r')"; [ -n "$v" ] && echo "$v" || echo "$2";
}
yesno() { command -v "$1" >/dev/null 2>&1 && echo "yes" || echo "no"; }

# --- OS / board ------------------------------------------------------------
MODEL_RAW="$(cat /proc/device-tree/model 2>/dev/null | tr -d '\0')"
L4T="unknown"
if [ -f /etc/nv_tegra_release ]; then
  L4T="$(head -n1 /etc/nv_tegra_release | sed -n 's/.*R36.*/R36.x/p')"
  L4T="$(grep -m1 '# R' /etc/nv_tegra_release | sed 's/# R\([0-9]*\) (release), REVISION: \(.*\)/R\1.\2/')"
fi
JETPACK="$(dpkg-query -W -f='${Version}' nvidia-jetpack 2>/dev/null || echo unknown)"
UBUNTU="$(jget 'lsb_release -rs' unknown)"
KERNEL="$(uname -r)"

# --- accelerator stack -----------------------------------------------------
CUDA_VER="unknown"
[ -f /usr/local/cuda/version.json ] && CUDA_VER="$(grep -A2 '"cuda"' /usr/local/cuda/version.json | grep '"version"' | head -n1 | sed 's/.*: "\(.*\)".*/\1/')"
NVCC="$(command -v nvcc || echo /usr/local/cuda-12.6/bin/nvcc)"
NVCC_VER="$( [ -x "$NVCC" ] && "$NVCC" --version | grep release | head -n1 || echo not-installed )"
TRT_HDR=/usr/include/aarch64-linux-gnu/NvInferVersion.h
TRT_VER="unknown"
if [ -f "$TRT_HDR" ]; then
  TRT_MAJ="$(grep '#define NV_TENSORRT_MAJOR' "$TRT_HDR" | awk '{print $3}')"
  TRT_MIN="$(grep '#define NV_TENSORRT_MINOR' "$TRT_HDR" | awk '{print $3}')"
  TRT_PAT="$(grep '#define NV_TENSORRT_PATCH' "$TRT_HDR" | awk '{print $3}')"
  TRT_VER="$TRT_MAJ.$TRT_MIN.$TRT_PAT"
fi
TRT_LIBS="$(jget 'ls /usr/lib/aarch64-linux-gnu/libnvinfer.so.* /usr/lib/aarch64-linux-gnu/libnvinfer_plugin.so.* 2>/dev/null | xargs -n1 basename | tr "\n" " "' none)"
CUDNN_LIBS="$(jget 'ls /usr/lib/aarch64-linux-gnu/libcudnn.so.* 2>/dev/null | xargs -n1 basename | tr "\n" " "' none)"
CUDNN_HDR="$( [ -f /usr/include/cudnn_version.h ] && grep -m1 -E 'CUDNN_(MAJOR|MINOR|PATCH)' /usr/include/cudnn_version.h | tr '\n' ' ' || echo "header not found")"

# GPU architecture: derive from device model; verify with deviceQuery if present
GPU_ARCH="sm_87 (expected for AGX Orin 64GB/32GB)"
if command -v nvidia-smi >/dev/null 2>&1 && nvidia-smi -L >/dev/null 2>&1; then
  GPU_NAME="$(nvidia-smi -L | head -n1)"
else
  GPU_NAME="$(cat /proc/device-tree/model 2>/dev/null | tr -d '\0' | head -n1)"
fi
JETSON_MODEL="unknown"
JETSON_MODEL="$(grep -o 'agx-orin-[0-9]*g' /etc/nv_tegra_release 2>/dev/null | head -n1 || echo "AGX Orin (model string not in nv_tegra_release)")"
[ "$JETSON_MODEL" = "unknown" ] && JETSON_MODEL="$(echo "$MODEL_RAW" | sed 's/NVIDIA Jetson //')"

# --- toolchain -------------------------------------------------------------
GCC_VER="$(jget 'gcc --version | head -n1' none)"
CMAKE_VER="$(jget 'cmake --version | head -n1' none)"
ROS_DISTRO="$(ls /opt/ros 2>/dev/null | tr '\n' ' ' | sed 's/ //g')"
ROS_VER="$( [ -n "$ROS_DISTRO" ] && echo "2 ($ROS_DISTRO)" || echo none )"
OPENCV_VER="$(jget 'pkg-config --modversion opencv4 2>/dev/null || pkg-config --modversion opencv' none)"
EIGEN_VER="$(jget 'cat /usr/include/eigen3/Eigen/src/Core/util/Macros.h | grep "#define EIGEN_WORLD_VERSION" | head -n1' none)"
YAMLCPP="$(yesno yaml-cpp); $(ls /usr/include/yaml-cpp 2>/dev/null >/dev/null && echo header-ok || echo header-missing)"
PYTORCH="$(python3 -c 'import torch; print(torch.__version__)' 2>/dev/null || echo not-installed)"
PYTHON3="$(python3 --version 2>&1)"

# --- resources -------------------------------------------------------------
RAM_KB="$(awk '/MemTotal/{print $2}' /proc/meminfo)"
RAM_GB=$((RAM_KB/1024/1024))
DISK="$(df -BG "$REPO_DIR" | tail -n1 | awk '{print "total:"$2" free:"$4" mount:"$6}')"
POWER_MODE="$(nvpmodel -q 2>/dev/null | tr '\n' ' ' || echo unknown)"
CPUS="$(nproc)"
SENSOR_QOS="n/a"

# --- required builds known-good check --------------------------------------
# (checked again at each phase gate)
COMPUTE_SANITIZER="$( [ -x /usr/local/cuda/bin/compute-sanitizer ] && echo yes || echo no )"
TRTEXEC="$(yesno trtexec); /usr/src/tensorrt/bin/trtexec:$( [ -x /usr/src/tensorrt/bin/trtexec ] && echo yes || echo no )"
GIT_VER="$(git --version)"

cat > "$OUT_FILE" <<EOF
{
  "timestamp_utc": "$(date -u +%Y-%m-%dT%H:%M:%SZ)",
  "os": {
    "ubuntu": "$UBUNTU",
    "kernel": "$KERNEL",
    "arch": "$(uname -m)"
  },
  "jetson": {
    "model": "$JETSON_MODEL",
    "device_tree_model": "$MODEL_RAW",
    "l4t": "$L4T",
    "jetpack": "$JETPACK",
    "gpu_arch_expected": "$GPU_ARCH",
    "gpu_name": "$GPU_NAME"
  },
  "accelerators": {
    "cuda": "$CUDA_VER",
    "nvcc": "$NVCC_VER",
    "tensorrt": "$TRT_VER",
    "tensorrt_libs": "$TRT_LIBS",
    "cudnn_libs": "$CUDNN_LIBS",
    "cudnn_header": "$CUDNN_HDR"
  },
  "toolchain": {
    "gcc": "$GCC_VER",
    "cmake": "$CMAKE_VER",
    "git": "$GIT_VER",
    "python3": "$PYTHON3",
    "pytorch": "$PYTORCH",
    "opencv": "$OPENCV_VER",
    "eigen3": "$EIGEN_VER",
    "yaml_cpp": "$YAMLCPP",
    "ros2": "$ROS_VER"
  },
  "resources": {
    "ram_gb": $RAM_GB,
    "cpus": $CPUS,
    "power_mode": "$POWER_MODE",
    "disk_root": "$DISK"
  },
  "phase0_tools": {
    "compute_sanitizer": "$COMPUTE_SANITIZER",
    "trtexec_in_path": "$TRTEXEC"
  }
}
EOF

echo "wrote $OUT_FILE"
cat "$OUT_FILE"