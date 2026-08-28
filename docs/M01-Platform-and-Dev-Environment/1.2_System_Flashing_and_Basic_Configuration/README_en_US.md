# M1 Platform and Development Environment

> **Target Hardware**: reComputer Robotics J501 (AGX Orin 32GB), L4T 36.4.4 / JetPack 6.2.1
>
> **Host PC**: Ubuntu 22.04.5 LTS, x86_64

***

# 1.2 JetPack 6.2 System Flashing and Basic Configuration

**Learning objectives**: Complete the system initialization of the J501 and establish a stable development baseline.

**Hardware list**: J501, USB-C data cable, Ubuntu Host PC.

**Reference**: [Seeed Wiki — Robotics J501 Hardware and Quick Start](https://wiki.seeedstudio.com/ai_robotics_recomputer_j501_robotics_getting_started/) 

***

## 1.2.1 JetPack 6.2 Component Overview

| Component | Version   | Description                              |
| --------- | --------- | ---------------------------------------- |
| L4T       | 36.4.4    | Linux BSP (kernel + drivers)             |
| CUDA      | 12.6.11   | GPU parallel computing framework         |
| cuDNN     | 9.3.0.75  | Deep learning acceleration library       |
| TensorRT  | 10.3.0.30 | High-performance inference optimizer     |
| VPI       | 3.2.4     | Vision Programming Interface             |
| Vulkan    | 1.3.204   | Graphics/compute API                     |
| OpenCV    | 4.8.0     | Computer vision library (no CUDA)        |
| PyTorch   | 2.11.0    | Deep learning framework (conda environment) |
| FFmpeg    | 4.4.2     | Multimedia processing (with hardware decoding) |
| GStreamer | 1.20.3    | Multimedia pipeline framework            |

> **Device status**: JetPack 6.2.1 is preinstalled at the factory, so no reflashing is required. If you need to reflash, refer to Section 1.2.2 below.

***

## 1.2.2 Flashing the JetPack Operating System

> If the J501 already has a preinstalled system, you can skip to Section 1.2.3 "System Verification".

### (1) Prerequisites

- Ubuntu 22.04 Host PC (not a virtual machine)
- USB Type-C data cable

### (2) Downloading the Image

| JetPack | Module        | GMSL | Download Link                                                                                                                                          | SHA256      |
| ------- | ------------- | ---- | ------------------------------------------------------------------------------------------------------------------------------------------------------ | ----------- |
| 6.2.1   | AGX Orin 64GB | ✅    | [Download](https://seeedstudio88-my.sharepoint.com/:u:/g/personal/youjiang_yu_seeedstudio88_onmicrosoft_com/IQCiNRM83_Q1Qq2lodZbxxz7AQb046lJeZh4aTUo20T6ks4) | B858312B... |
| 6.2.1   | AGX Orin 32GB | ✅    | [Download](https://seeedstudio88-my.sharepoint.com/:u:/g/personal/youjiang_yu_seeedstudio88_onmicrosoft_com/IQAWnjh2iWzeRI1rKN5Bb_HxAX3P3GxPMHSb-60VmCPCgx4) | e868fd8c... |

> The image is large. Considering network and other factors during download, please verify integrity with `sha256sum <filename>` after the download completes.

### (3) Entering Forced Recovery Mode

**Step 1.** Connect the **USB2.0 DEVICE port** of the J501 to the Ubuntu Host PC with a USB-C cable.

![USB-C connection for recovery](images/1.2.2.3_usb_c_recovery.png)

**Step 2.** Press and hold the RECOVERY button with a pin.

**Step 3.** Connect the power.

**Step 4.** Release the RECOVERY button.

**Step 5.** Run `lsusb` on the Host PC. The following output indicates that recovery mode has been entered:

- AGX Orin 32GB: `0955:7223 NVidia Corp`
- AGX Orin 64GB: `0955:7023 NVidia Corp`

![lsusb recovery mode output](images/1.2.2.3_lsusb_recovery.png)

### (4) Flashing the System

```bash
# [Host PC terminal] Run the following commands on the Ubuntu host
# Extract the image
cd <path-to-image>
sudo tar xpf mfi_recomputer-robo-agx-orin-32g-j501-6.2.1-36.4.4-2025-11-04.tar.gz

# Run the flashing
cd mfi_recomputer-robo-agx-orin-j501x
sudo ./tools/kernel_flash/l4t_initrd_flash.sh --flash-only --massflash 1 --network usb0 --showlogs
```

Flashing success output:

![Flashing success output](images/1.2.2.4_flash_success.png)

> The flashing command takes about 2-10 minutes to run.

**Initial configuration:** Connect the J501 to a monitor with an HDMI cable and complete the system configuration:

![HDMI display setup](images/1.2.2.4_hdmi_setup.png)

> After flashing is complete, you can verify the system by connecting to the J501 via HDMI + keyboard/mouse, SSH, or the debug serial port (see 1.1.5 (1) Connection Methods).

***

## 1.2.3 System Verification

The following commands are executed on the J501 over SSH, and the outputs are the actual verification results. You can also run the same commands in a local terminal on the J501.

### (1) L4T Version

```bash
# [Host PC -> J501 SSH]
ssh J501 'cat /etc/nv_tegra-release'
```

**Actual output:**

```
# R36 (release), REVISION: 4.4, GCID: 41062509, BOARD: generic, EABI: aarch64, DATE: Mon Jun 16 16:07:13 UTC 2025
# KERNEL_VARIANT: oot
TARGET_USERSPACE_LIB_DIR=nvidia
TARGET_USERSPACE_LIB_DIR_PATH=usr/lib/aarch64-linux-gnu/nvidia
# Seeed Image Name mfi_recomputer-robo-agx-orin-32g-j501-6.2.1-36.4.4-2025-11-04.tar.gz
# branch R36.4.4
# commit ID 36a8a761b8a88785372afc7bd6d4e70643b76677
```

### (2) Operating System

```bash
# [Host PC -> J501 SSH]
ssh J501 'cat /etc/os-release'
```

**Actual output:**

```
PRETTY_NAME="Ubuntu 22.04.5 LTS"
NAME="Ubuntu"
VERSION_ID="22.04"
VERSION="22.04.5 LTS (Jammy Jellyfish)"
VERSION_CODENAME=jammy
ID=ubuntu
ID_LIKE=debian
UBUNTU_CODENAME=jammy
```

### (3) Kernel Version

```bash
# [Host PC -> J501 SSH]
ssh J501 'uname -a'
```

**Actual output:**

```
Linux seeed-desktop 5.15.148-tegra #1 SMP PREEMPT Tue Nov 4 08:05:33 UTC 2025 aarch64 aarch64 aarch64 GNU/Linux
```

**Interpretation:** Tegra-specific kernel 5.15.148, with support for SMP (Symmetric Multi-Processing) and PREEMPT (preemptive scheduling), aarch64 architecture.

### (4) CPU Information

```bash
# [Host PC -> J501 SSH]
ssh J501 'lscpu | head -20'
```

**Actual output:**

```
Architecture:                       aarch64
CPU op-mode(s):                     32-bit, 64-bit
Byte Order:                         Little Endian
CPU(s):                             8
On-line CPU(s) list:                0-7
Vendor ID:                          ARM
Model name:                         Cortex-A78AE
Model:                              1
Thread(s) per core:                 1
Core(s) per cluster:                4
Socket(s):                          -
Cluster(s):                         2
Stepping:                           r0p1
CPU max MHz:                        2188.8000
CPU min MHz:                        115.2000
BogoMIPS:                           62.50
Flags:                              fp asimd evtstrm aes pmull sha1 sha2 crc32 atomics fphp asimdhp ...
L1d cache:                          512 KiB (8 instances)
L1i cache:                          512 KiB (8 instances)
L2 cache:                           2 MiB (8 instances)
```

**Interpretation:** 8-core Cortex-A78AE (2 clusters × 4 cores), up to 2.2 GHz, with hardware acceleration for AES/SHA.

### (5) Memory and Storage

```bash
# [Host PC -> J501 SSH]
ssh J501 'free -h; echo "---"; df -h'
```

**Actual output:**

```
               total        used        free      shared  buff/cache   available
Mem:            29Gi       3.1Gi        23Gi        82Mi       3.8Gi        26Gi
Swap:           14Gi          0B        14Gi
---
Filesystem       Size  Used Avail Use% Mounted on
/dev/nvme0n1p1   116G   43G   68G  39% /
tmpfs             15G  172K   15G   1% /dev/shm
tmpfs            6.0G   28M  6.0G   1% /run
/dev/nvme0n1p10   63M  110K   63M   1% /boot/efi
tmpfs            3.0G  144K  3.0G   1% /run/user/1000
```

**Interpretation:** 32 GB LPDDR5 memory (29 GiB visible to the system), 116 GB NVMe SSD (39% used), 14 GB swap (zram).

### (6) GPU Status

```bash
# [Host PC -> J501 SSH]
ssh J501 'nvidia-smi'
```

**Actual output:**

```
Tue Jul 28 13:48:48 2026
+---------------------------------------------------------------------------------------+
| NVIDIA-SMI 540.4.0                Driver Version: 540.4.0      CUDA Version: 12.6     |
|-----------------------------------------+----------------------+----------------------+
| GPU  Name                 Persistence-M | Bus-Id        Disp.A | Volatile Uncorr. ECC |
| Fan  Temp   Perf          Pwr:Usage/Cap |         Memory-Usage | GPU-Util  Compute M. |
|=========================================+======================+======================|
|   0  Orin (nvgpu)                  N/A  | N/A              N/A |                  N/A |
| N/A   N/A  N/A               N/A /  N/A | Not Supported        |     N/A          N/A |
+-----------------------------------------+----------------------+----------------------+

+---------------------------------------------------------------------------------------+
| Processes:                                                                            |
|  No running processes found                                                           |
+---------------------------------------------------------------------------------------+
```

**Interpretation:** GPU name is Orin (nvgpu), driver 540.4.0, CUDA 12.6. On the Jetson platform, nvidia-smi does not display temperature/power/VRAM (unlike discrete GPUs, the Jetson GPU shares memory with the CPU).

***

## 1.2.4 CUDA Verification

### (1) CUDA SDK Version

```bash
# [Host PC -> J501 SSH]
ssh J501 'cat /usr/local/cuda-12.6/version.json | head -10'
```

**Actual output:**

```json
{
   "cuda" : {
      "name" : "CUDA SDK",
      "version" : "12.6.11"
   },
   "cuda_cccl" : {
      "name" : "CUDA C++ Core Compute Libraries",
      "version" : "12.6.37"
   },
   "cuda_compat" : {
      "name" : "CUDA Specific Libraries",
      "version" : "12.6.36890662"
   },
```

### (2) NVCC Compiler

```bash
# [Host PC -> J501 SSH]
ssh J501 '/usr/local/cuda/bin/nvcc --version'
```

**Actual output:**

```
nvcc: NVIDIA (R) Cuda compiler driver
Copyright (c) 2005-2024 NVIDIA Corporation
Built on Wed_Aug_14_10:14:07_PDT_2024
Cuda compilation tools, release 12.6, V12.6.68
Build cuda_12.6.r12.6/compiler.34714021_0
```

**Interpretation:** CUDA SDK 12.6.11, NVCC compiler 12.6.68, build date 2024-08-14.

> **Tip**: `nvcc` is not in the PATH by default. Add it to `~/.bashrc`: `export PATH=/usr/local/cuda/bin:$PATH`

***

## 1.2.5 cuDNN Verification

### (1) Package Version

```bash
# [Host PC -> J501 SSH]
ssh J501 'dpkg -l | grep -i cudnn'
```

**Actual output:**

```
ii  libcudnn9-cuda-12     9.3.0.75-1   arm64   cuDNN runtime libraries for CUDA 12.6
ii  libcudnn9-dev-cuda-12 9.3.0.75-1   arm64   cuDNN development headers for CUDA 12.6
ii  libcudnn9-samples     9.3.0.75-1   all     cuDNN samples
ii  nvidia-cudnn          6.2.1+b38    arm64   NVIDIA CUDNN Meta Package
ii  nvidia-cudnn-dev      6.2.1+b38    arm64   NVIDIA CUDNN dev Meta Package
```

### (2) Header File Version

```bash
# [Host PC -> J501 SSH]
ssh J501 'cat /usr/include/cudnn_version.h | grep CUDNN_MAJOR -A2'
```

**Actual output:**

```c
#define CUDNN_MAJOR 9
#define CUDNN_MINOR 3
#define CUDNN_PATCHLEVEL 0
```

**Interpretation:** cuDNN version 9.3.0

***

## 1.2.6 TensorRT Verification

### (1) Package Version

```bash
# [Host PC -> J501 SSH]
ssh J501 'dpkg -l | grep -i "tensorrt\|libnvinfer" | head -10'
```

**Actual output:**

```
ii  libnvinfer-bin          10.3.0.30-1+cuda12.5  arm64  TensorRT binaries
ii  libnvinfer-dev          10.3.0.30-1+cuda12.5  arm64  TensorRT development libraries
ii  libnvinfer-dispatch10   10.3.0.30-1+cuda12.5  arm64  TensorRT dispatch runtime
ii  libnvinfer-headers-dev  10.3.0.30-1+cuda12.5  arm64  TensorRT development headers
ii  libnvinfer-lean10       10.3.0.30-1+cuda12.5  arm64  TensorRT lean runtime
ii  libnvinfer-plugin10     10.3.0.30-1+cuda12.5  arm64  TensorRT plugin libraries
```

### (2) Python Bindings

```bash
# [Host PC -> J501 SSH]
ssh J501 'python3 -c "import tensorrt; print(tensorrt.__version__)"'
```

**Actual output:**

```
10.3.0
```

### (3) trtexec Tool Location

```bash
# [Host PC -> J501 SSH]
ssh J501 'ls /usr/src/tensorrt/bin/trtexec'
```

**Actual output:**

```
/usr/src/tensorrt/bin/trtexec
```

> **Tip**: `trtexec` is not in the default PATH. Either specify the full path when using it, or add `export PATH=/usr/src/tensorrt/bin:$PATH`.

***

## 1.2.7 Verification of Other Key Libraries

### (1) VPI

```bash
# [Host PC -> J501 SSH]
ssh J501 'dpkg -l | grep -i vpi | head -5'
```

**Actual output:**

```
ii  libnvvpi3           3.2.4       arm64  NVIDIA Vision Programming Interface library
ii  nvidia-vpi          6.2.1+b38   arm64  NVIDIA Vpi Meta Package
ii  python3.10-vpi3     3.2.4       arm64  NVIDIA VPI python 3.10 bindings
ii  vpi3-dev            3.2.4       arm64  NVIDIA VPI C/C++ development library
```

### (2) OpenCV

```bash
# [Host PC -> J501 SSH]
ssh J501 'python3 -c "import cv2; print(cv2.__version__)"'
```

**Actual output:**

```
4.8.0
```

> **Note**: The system OpenCV 4.8.0 **does not include CUDA acceleration**. If you need a CUDA-accelerated version, you have to compile it yourself or use it inside a container. The M4 module covers this in detail.

### (3) Vulkan

```bash
# [Host PC -> J501 SSH]
ssh J501 'dpkg -l | grep -i vulkan | head -5'
```

**Actual output:**

```
ii  libvulkan-dev:arm64        1.3.204.1-2              arm64  Vulkan loader dev files
ii  libvulkan1:arm64           1.3.204.1-2              arm64  Vulkan loader library
ii  nvidia-l4t-vulkan-sc     36.4.4-20250616085344    arm64  NVIDIA Vulkan SC runtime
```

### (4) PyTorch (conda py310 environment)

```bash
# [Host PC -> J501 SSH]
ssh J501 '/home/seeed/miniconda3/envs/py310/bin/python -c "import torch; print(torch.__version__); print(torch.cuda.is_available()); print(torch.cuda.get_device_name(0))"'
```

**Actual output:**

```
2.11.0
True
Orin
```

**Interpretation:** PyTorch 2.11.0, CUDA available, GPU device name is Orin.

***

## 1.2.8 jetson-stats Overview

```bash
# [Host PC -> J501 SSH]
ssh J501 'jetson_release'
```

**Actual output:**

```
Software part of jetson-stats 4.3.2 - (c) 2024, Raffaello Bonghi
Jetpack missing!
 - Model: NVIDIA Jetson AGX Orin Developer Kit
 - L4T: 36.4.4
NV Power Mode[3]: MODE_40W
Serial Number: [XXX Show with: jetson_release -s XXX]
Hardware:
 - P-Number: p3701-0004
 - Module: NVIDIA Jetson AGX Orin (32GB ram)
Platform:
 - Distribution: Ubuntu 22.04 Jammy Jellyfish
 - Release: 5.15.148-tegra
jtop:
 - Version: 4.3.2
 - Service: Active
Libraries:
 - CUDA: 12.6.68
 - cuDNN: 9.3.0.75
 - TensorRT: 10.3.0.30
 - VPI: 3.2.4
 - Vulkan: 1.3.204
 - OpenCV: 4.8.0 - with CUDA: NO
```

> **Note**: `Jetpack missing!` is a recognition/compatibility issue of jetson-stats 4.3.2 with JetPack 6.2 and does not affect actual use.

***

## 1.2.9 Power Mode Configuration

### (1) Checking the Current Mode

```bash
# [Host PC -> J501 SSH]
ssh J501 'nvpmodel -q'
```

**Actual output:**

```
NV Power Mode: MODE_40W
3
```

### (2) Switching to MAXN Mode

MAXN mode unlocks the full compute power (\~60W) and is suitable for AI inference and intensive computing:

```bash
# [J501 local terminal] sudo required; it is recommended to run this in the J501 display terminal or serial terminal
sudo nvpmodel -m 0
```

### (3) Locking Maximum Frequencies

```bash
# [J501 local terminal] sudo required
sudo jetson_clocks
```

Locks all CPU/GPU/EMC frequencies to their maximum values, ensuring stable performance without throttling.

> **Note**: Make sure the cooling fan is installed before switching to MAXN. Temperatures can reach 55-65°C under MAXN. If you run into password issues when using sudo over SSH, operate in the J501 display terminal instead.

### (4) Power Mode Reference Table

| Mode | Name      | Power | Use Cases                     |
| ---- | --------- | ----- | ----------------------------- |
| 0    | MAXN      | \~60W | AI inference, intensive computing |
| 1    | MODE\_30W | \~30W | Light tasks, power saving     |
| 2    | MODE\_50W | \~50W | Balanced mode                 |
| 3    | MODE\_40W | \~40W | Daily use (current)           |

***

## 1.2.10 Performance Benchmark

### (1) SSD Read/Write Benchmark

```bash
# [Host PC -> J501 SSH]
ssh J501 'dd if=/dev/zero of=/home/seeed/ssd_test bs=100M count=1 conv=fdatasync; rm -f /home/seeed/ssd_test'
```

**Actual output:**

```
1+0 records in
1+0 records out
104857600 bytes (105 MB, 100 MiB) copied, 0.312549 s, 335 MB/s
```

**Result: NVMe SSD write speed of 335 MB/s**

### (2) TensorRT Inference Benchmark

```bash
# [J501 local terminal] An ONNX model file is required
/usr/src/tensorrt/bin/trtexec --onnx=<model.onnx> --fp16 --iterations=100 --workspace=4096
```

> The M4 module will run a complete benchmark using an ONNX model exported from YOLO. Use `--help` to view the detailed parameters of `trtexec`.

***

## 1.2.11 Installing Missing Software After Flashing

The Seeed factory image comes with the core JetPack components preinstalled (CUDA/cuDNN/TensorRT/VPI/OpenCV/PyTorch/jetson-stats/can-utils, etc.), but the following software is **not preinstalled**. It will be needed in later modules, so the installation commands are provided here.

### (1) Docker Engine + NVIDIA Container Toolkit

Docker and NVIDIA Container Toolkit are the foundation of the containerized development environment in Section 1.3, and they are not installed after flashing.

```bash
# [J501 local terminal] Install Docker Engine
# 1. Add the official Docker GPG key
sudo apt-get update
sudo apt-get install -y ca-certificates curl gnupg
sudo install -m 0755 -d /etc/apt/keyrings
curl -fsSL https://download.docker.com/linux/ubuntu/gpg | sudo gpg --dearmor -o /etc/apt/keyrings/docker.gpg
sudo chmod a+r /etc/apt/keyrings/docker.gpg

# 2. Add the Docker apt repository
echo "deb [arch=arm64 signed-by=/etc/apt/keyrings/docker.gpg] https://download.docker.com/linux/ubuntu $(lsb_release -cs) stable" | sudo tee /etc/apt/sources.list.d/docker.list > /dev/null

# 3. Install Docker
sudo apt-get update
sudo apt-get install -y docker-ce docker-ce-cli containerd.io docker-buildx-plugin docker-compose-plugin

# 4. Add the current user to the docker group (run docker without sudo)
sudo usermod -aG docker $USER
# Log out and log back in, or run newgrp docker, for this to take effect

# 5. Verify
docker --version
# Expected: Docker version 27.x or higher
```

```bash
# [J501 local terminal] Install NVIDIA Container Toolkit (GPU passthrough)
# 1. Add the NVIDIA Container Toolkit repository
curl -fsSL https://nvidia.github.io/libnvidia-container/gpgkey | sudo gpg --dearmor -o /usr/share/keyrings/nvidia-container-toolkit-keyring.gpg
curl -s -L https://nvidia.github.io/libnvidia-container/stable/deb/nvidia-container-toolkit.list | \
  sed 's#deb https://#deb [signed-by=/usr/share/keyrings/nvidia-container-toolkit-keyring.gpg] https://#g' | \
  sudo tee /etc/apt/sources.list.d/nvidia-container-toolkit.list

# 2. Install
sudo apt-get update
sudo apt-get install -y nvidia-container-toolkit

# 3. Configure the Docker runtime
sudo nvidia-ctk runtime configure --runtime=docker
sudo systemctl restart docker

# 4. Verify GPU passthrough
docker run --rm --runtime=nvidia --gpus all ubuntu nvidia-smi
# Expected: Orin GPU information is displayed
```

### (2) ROS 2 Humble

ROS 2 Humble is the foundation of Section 1.4 and all subsequent robot modules, and it is not installed after flashing. It is recommended to use the **FishROS one-click installation** (a community one-click ROS installer), which automatically handles repositories and dependencies and is suitable for a quick start. The official manual installation is also provided as an alternative.

#### Option 1: FishROS One-Click Installation (Recommended)

```bash
# [J501 local terminal] One-click install ROS 2 Humble with FishROS
# 1. Run the one-click installation script
wget http://fishros.com/install -O fishros && . fishros

# 2. Select the following in the interactive menu:
#    [1] One-click install ROS
#    [2] humble
#    [1] Desktop version (includes rviz, rqt and other tools)
#    [Y] Configure Chinese mirror sources (faster downloads)

# 3. After installation completes, the script automatically writes environment variables to ~/.bashrc
#    Verify
source ~/.bashrc
ros2 --version 2>/dev/null || ros2 doctor --report | head -5
# Expected: ROS 2 Humble environment information is displayed
```

> **Note**: The FishROS script automatically completes all steps — adding the apt repository, importing the GPG key, installing ROS 2 Humble desktop, configuring environment variables, and so on — while using Chinese mirror sources to speed up downloads. It is suitable for beginners to quickly set up the environment.

#### Option 2: Official Manual Installation

```bash
# [J501 local terminal] Install ROS 2 Humble from the official sources
# 1. Add the ROS 2 apt repository
sudo apt update && sudo apt install -y software-properties-common
sudo add-apt-repository universe
sudo apt update && sudo apt install -y curl gnupg lsb-release
sudo curl -sSL https://raw.githubusercontent.com/ros/rosdistro/master/ros.key -o /usr/share/keyrings/ros-archive-keyring.gpg
echo "deb [arch=arm64 signed-by=/usr/share/keyrings/ros-archive-keyring.gpg] http://packages.ros.org/ros2/ubuntu $(. /etc/os-release && echo $VERSION_CODENAME) main" | sudo tee /etc/apt/sources.list.d/ros2.list > /dev/null

# 2. Install the ROS 2 Humble desktop full version (includes rviz, rqt and other tools)
sudo apt update
sudo apt install -y ros-humble-desktop

# 3. Install development tools
sudo apt install -y python3-colcon-common-extensions python3-rosdep python3-vcstool
sudo rosdep init
rosdep update

# 4. Configure environment variables (write to ~/.bashrc)
echo "source /opt/ros/humble/setup.bash" >> ~/.bashrc
source ~/.bashrc

# 5. Verify
ros2 --version 2>/dev/null || ros2 doctor --report | head -5
# Expected: ROS 2 Humble environment information is displayed
```

> **Note**: The official sources may be slow on networks in China. If you encounter timeouts, use Option 1 (FishROS) or replace the sources with Chinese mirrors yourself.

### (3) htop (System Monitoring)

```bash
# [J501 local terminal] Install htop
sudo apt install -y htop
# Verify
htop
```

### (4) Software Preinstallation Quick Reference Table

| Software     | Preinstalled         | Version   | Location                   | Notes                      |
| ------------ | -------------------- | --------- | -------------------------- | -------------------------- |
| CUDA         | ✅ Preinstalled       | 12.6.11   | /usr/local/cuda-12.6       | JetPack component          |
| cuDNN        | ✅ Preinstalled       | 9.3.0.75  | /usr/lib/aarch64-linux-gnu | JetPack component          |
| TensorRT     | ✅ Preinstalled       | 10.3.0.30 | /usr/src/tensorrt          | JetPack component          |
| VPI          | ✅ Preinstalled       | 3.2.4     | /usr/lib                   | JetPack component          |
| OpenCV       | ✅ Preinstalled       | 4.8.0     | System python3             | No CUDA acceleration       |
| PyTorch      | ✅ Preinstalled       | 2.11.0    | conda py310 environment    | Seeed preinstalled         |
| Miniconda3   | ✅ Preinstalled       | -         | \~/miniconda3              | Seeed preinstalled         |
| jetson-stats | ✅ Preinstalled       | 4.3.2     | /usr/local/bin             | jtop service running       |
| FFmpeg       | ✅ Preinstalled       | 4.4.2     | /usr/bin                   | With hardware decoding     |
| GStreamer    | ✅ Preinstalled       | 1.20.3    | /usr/bin                   | -                          |
| can-utils    | ✅ Preinstalled       | 2020.11.0 | /usr/bin                   | candump/cansend, etc.      |
| cmake        | ✅ Preinstalled       | 3.22.1    | /usr/bin                   | -                          |
| git          | ✅ Preinstalled       | 2.34.1    | /usr/bin                   | -                          |
| iperf3       | ✅ Preinstalled       | -         | /usr/bin                   | Network speed testing      |
| nvme CLI     | ✅ Preinstalled       | -         | /usr/sbin                  | NVMe management            |
| **Docker**   | ❌ **Not preinstalled** | -       | -                          | See installation commands in (1) |
| **ROS 2**    | ❌ **Not preinstalled** | -       | -                          | See installation commands in (2) |
| **htop**     | ❌ **Not preinstalled** | -       | -                          | See installation commands in (3) |

***

## 1.2.12 System Environment Overview Table

| Item       | Value                                | Verification Command                    |
| ---------- | ------------------------------------ | --------------------------------------- |
| Device Model | NVIDIA Jetson AGX Orin Developer Kit | `cat /proc/device-tree/model`       |
| Module     | AGX Orin 32GB (p3701-0004)           | `jetson_release`                        |
| SoC        | NVIDIA Tegra234                      | `cat /proc/device-tree/compatible`      |
| Serial Number | 1424225004690                     | `cat /proc/device-tree/serial-number`   |
| Operating System | Ubuntu 22.04.5 LTS             | `cat /etc/os-release`                   |
| Kernel     | 5.15.148-tegra                       | `uname -r`                              |
| L4T        | R36.4.4                              | `cat /etc/nv_tegra_release`             |
| JetPack    | 6.2.1                                | Image file name                         |
| CPU        | 8-core Cortex-A78AE @ 2.2GHz         | `lscpu`                                 |
| GPU        | Orin (nvgpu), Driver 540.4.0         | `nvidia-smi`                            |
| Memory     | 29GiB (32GB LPDDR5)                  | `free -h`                               |
| Storage    | NVMe 119GB (39% used)                | `df -h`                                 |
| CUDA       | 12.6.11 (NVCC 12.6.68)               | `nvcc --version`                        |
| cuDNN      | 9.3.0.75                             | `dpkg -l \| grep cudnn`                 |
| TensorRT   | 10.3.0.30                            | `python3 -c "import tensorrt"`          |
| VPI        | 3.2.4                                | `dpkg -l \| grep vpi`                   |
| OpenCV     | 4.8.0 (no CUDA)                      | `python3 -c "import cv2"`               |
| Vulkan     | 1.3.204                              | `dpkg -l \| grep vulkan`                |
| PyTorch    | 2.11.0 (conda py310)                 | `python -c "import torch"`              |
| Power Mode | MODE\_40W (Mode 3)                   | `nvpmodel -q`                           |
| SSD Write  | 335 MB/s                             | `dd if=/dev/zero ...`                   |

***

## 1.2.13 FAQ

| Issue                               | Cause                              | Solution                                                     |
| ----------------------------------- | ---------------------------------- | ------------------------------------------------------------ |
| `nvcc` command not found            | Not in the PATH                    | `export PATH=/usr/local/cuda/bin:$PATH`                      |
| `trtexec` command not found         | Not in the PATH                    | `export PATH=/usr/src/tensorrt/bin:$PATH`                    |
| `nvpmodel -m 0` requires a password | No passwordless sudo over SSH      | Run it in the J501 display terminal, or configure passwordless sudo |
| `jetson_clocks` errors out          | Not running as root                | `sudo jetson_clocks`                                         |
| SSH connection times out            | l4tbr0 routing conflict            | `sudo nmcli connection modify l4tbr0 ipv4.method disabled`   |
| OpenCV has no CUDA support          | The system package does not include CUDA | Compile it yourself or use a container (covered in M4) |
| `jetson_release` shows Jetpack missing | jetson-stats version compatibility | Does not affect use; simply ignore it                    |

***

## 1.1 & 1.2 Deliverables

After completing these two sections, you should have:

1. **Interface Function Reference Table** (1.1.7) — a reference for hardware integration in M2/M3
2. **Wiring Topology Diagram** (1.1.8) — a reference for system integration
3. **System Environment Overview Table** (1.2.12) — the verification baseline for subsequent modules
4. **Flashing capability** (1.2.2) — able to independently reflash the J501 system
5. **Performance configuration capability** (1.2.9) — able to switch power modes and lock frequencies according to the scenario
6. **Software installation** (1.2.11) — installation of Docker / ROS 2 / htop, etc.

***
