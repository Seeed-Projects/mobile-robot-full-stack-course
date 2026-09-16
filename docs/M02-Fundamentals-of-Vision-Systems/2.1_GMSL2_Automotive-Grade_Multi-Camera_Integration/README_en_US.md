# 2.1 GMSL2: Automotive-Grade Multi-Camera Integration

## What Is GMSL2?

GMSL2 (Gigabit Multimedia Serial Link 2) is a high-speed serializer/deserializer (SerDes) technology from Analog Devices (formerly Maxim Integrated), designed for transporting video, audio, and control data in automotive and other high-reliability scenarios. Over a single coaxial cable or shielded twisted pair (STP) it delivers up to 6 Gbps of forward data while simultaneously carrying a reverse control channel, combining HD video streams, I²C configuration, GPIO signaling, and even power delivery (PoC) on one cable. Compared with first-generation GMSL, GMSL2 offers significant improvements in bandwidth, transmission distance, EMC immunity, and functional safety, and it supports HDCP content protection. It is widely used in automotive surround-view cameras, ADAS sensors, domain-controller displays, and industrial robot vision — applications with strict real-time and reliability requirements.

![Mini-Fakra 4-in-1 cable](./images/DEq5bdwPZoVFk9x68Bpc55Hknlf.png)

## GMSL vs. CSI

GMSL and CSI (MIPI CSI-2) are not competing, mutually exclusive technologies; rather, they are two interface technologies at different layers that frequently work together. CSI-2 is a camera serial-interface protocol defined by the MIPI Alliance. It specifies the format for transferring image data from a sensor to a processor and is widely used for short, board-level connections in phones, embedded systems, and automotive SoCs. Its physical layer relies on D-PHY or C-PHY, and its typical transmission distance is on the centimeter scale. GMSL, by contrast, is a complete SerDes (serializer/deserializer) solution from Analog Devices designed for long-distance, high-interference environments such as automotive, and it can carry video, control signals, and even power over 15 meters on a single coaxial cable or STP. In a typical automotive camera link the two are used together: the image sensor outputs a CSI-2 signal into a GMSL serializer, which transmits it over the long cable; a GMSL deserializer then restores it to CSI-2 and feeds it into the domain-controller SoC. CSI-2 solves the protocol problem of "how to move image data between chips," while GMSL solves the physical-link problem of "how to transmit reliably over long distances."

![CSI vs GMSL](./images/DDeLbmDJqoh3IpxrMptcQEo4nNh.png)

## Hardware List

- reComputer Robotics J501 / reComputer mini J501 + GMSL expansion board
- [SG3S-ISX031C-GMSL2F](https://www.seeedstudio.com/SG3S-ISX031C-GMSL2F-p-6245.html) / SG8S-AR0820 GMSL2 camera
- [Fakra 4-in-1 cable](https://www.seeedstudio.com/Mini-fakra-Coaxial-Cable-4-in-1-0-5m-Female-to-Female-p-6484.html)

| ![J501 expansion board](./images/T5LFbUbVqoARCHxZR37cPey6nOe.png) | ![Sensing 3G camera](./images/XZtGbkTWqorbaNxgzfucztxlnWc.png) | ![Fakra 4-in-1 cable](./images/Hi4Pb2fbFo3SHWxPypecQuNonse.png) |
|---|---|---|
| reComputer mini J501 + GMSL expansion board | Sensing 3G GMSL camera | Fakra 4-in-1 cable |

## Usage Guide

### Hardware Connection

Follow the connection method shown in the diagram below to connect your own GMSL camera to the J501 carrier board.

![Hardware connection](./images/KvirbW6J4okHZixybbgcW0Z5nnh.png)

### Selecting the Device Tree

Follow the steps below to select the device tree.

> Note: The reference hardware used in this chapter is the Seeed reComputer J501 mini with a Sensing [SG3S-ISX031C-GMSL2F](https://www.seeedstudio.com/SG3S-ISX031C-GMSL2F-p-6245.html) camera. In practice, choose the device tree that matches your own hardware combination; for configuration details see [GMSL configuration selection](https://wiki.seeedstudio.com/cn/recomputer_j501_mini_getting_started/#%E6%89%A9%E5%B1%95%E7%AB%AF%E5%8F%A3---gmsl).

![Device tree configuration](./images/NK0DbLglwo77zjxFjCbcyr3vn3b.png)

After the device reboots, the connected cameras should be visible if the correct device-tree file was selected:

```bash
ls /dev/video*
```

![Video device list](./images/Sw07bMFXOoGLkBx5qmZc6aBYnBg.png)

### Multi-Camera Synchronization

This is the device-tree configuration on the J501 carrier board used to enable four GMSL cameras. At startup it creates the CSI/V4L2 links for the four cameras and configures the FSYNC input of the MAX96712 deserializer together with the MFP7 FSYNC outputs of the four MAX96717 serializers, so that the same synchronization clock is carried through the GMSL link to the FSIN/trigger pins of the four cameras, enabling the four sensors to expose on a common hardware-level cadence.

#### Inspecting the Media-Controller Topology

```bash
media-ctl -d /dev/media0 -p
```

![media-ctl topology](./images/P7A5bCip2ogMpjxrVWNcmw3onub.png)

This step verifies that the sensor, serializer, deserializer, CSI, and V4L2 nodes are connected.

#### Checking Whether the Cameras Are in Use

```bash
fuser /dev/video0 /dev/video1 /dev/video2 /dev/video3
```

No output means the cameras are not currently in use.

#### Setting the Serializer/Deserializer Formats

```bash
sudo media-ctl -d /dev/media0 --set-v4l2 '"ser_0_ch_0":1[fmt:YUYV8_1X16/1920x1536]'
sudo media-ctl -d /dev/media0 --set-v4l2 '"des_0_ch_0":0[fmt:YUYV8_1X16/1920x1536]'

sudo media-ctl -d /dev/media0 --set-v4l2 '"ser_1_ch_1":1[fmt:YUYV8_1X16/1920x1536]'
sudo media-ctl -d /dev/media0 --set-v4l2 '"des_0_ch_1":0[fmt:YUYV8_1X16/1920x1536]'

sudo media-ctl -d /dev/media0 --set-v4l2 '"ser_2_ch_2":1[fmt:YUYV8_1X16/1920x1536]'
sudo media-ctl -d /dev/media0 --set-v4l2 '"des_0_ch_2":0[fmt:YUYV8_1X16/1920x1536]'

sudo media-ctl -d /dev/media0 --set-v4l2 '"ser_3_ch_3":1[fmt:YUYV8_1X16/1920x1536]'
sudo media-ctl -d /dev/media0 --set-v4l2 '"des_0_ch_3":0[fmt:YUYV8_1X16/1920x1536]'
```

> Note: Set the resolution to match the cameras you actually have connected. The table below lists the GMSL camera models supported by the J501 carrier board and their corresponding resolutions.

| GMSL camera model | Resolution (YUYV8_1X16) | sensor_mode |
|---|---|---|
| SG3S-ISX031C-GMSL2F (Sensing 3G) | 1920×1536 | 0 |
| 2MP GMSL model supported by the J501 carrier board | 1920×1080 | 1 |
| SG8S-AR0820-GMSL2F | 3840×2160 | 2 |

#### Setting the V4L2 Node Format and Sensor Mode

```bash
for i in 0 1 2 3; do
  sudo v4l2-ctl -d /dev/video${i} \
    --set-fmt-video=width=1920,height=1536,pixelformat=YUYV \
    -c sensor_mode=0
done
```

> The `sensor_mode` parameter represents different GMSL camera models and resolutions:
>
> - sensor_mode=0 -------> YUYV8_1X16/1920x1536
> - sensor_mode=1 -------> YUYV8_1X16/1920x1080
> - sensor_mode=2 -------> YUYV8_1X16/3840x2160

Check whether the settings took effect:

```bash
for i in 0 1 2 3; do
  echo "===== /dev/video${i} ====="
  v4l2-ctl -d /dev/video${i} --get-fmt-video --get-parm
done
```

Expected output:

![Format parameter output](./images/YB3vbry1joFt6Ix1VfmchRzenzc.png)

#### Starting All Cameras Synchronously

Copy the following script to the J501 and run it:

```bash
#同时启动多个相机
bash gmsl4_start.sh preview
```

> Note: If your GMSL camera model is not the Sensing [SG3S-ISX031C-GMSL2F](https://www.seeedstudio.com/SG3S-ISX031C-GMSL2F-p-6245.html), change the script's resolution parameters `WIDTH, HEIGHT, FPS = 1920, 1536, 30` to the actual resolution.

![Four-channel synchronized preview](./images/BeIUbjExDoZ6VkxZLxxc933un9o.gif)

##### Run the Code

> **Note**: replace `<Jetson IP>` with your Jetson's actual IP. Find it by running `hostname -I` on the Jetson; do not reuse a fixed address.

The script is included in this repository at `code/2.1_gmsl2/gmsl4_start.sh` (from the Feishu 2.1 attachment, about 16 KB). It supports five modes: `preview / stream / verify / stop / status`.

1. Copy the script from this repository to the Jetson (remote address `<Jetson IP>`, user `seeed`):

   ```bash
   scp docs/M02-Fundamentals-of-Vision-Systems/code/2.1_gmsl2/gmsl4_start.sh seeed@<Jetson IP>:~/
   ```

2. Make it executable:

   ```bash
   ssh seeed@<Jetson IP> 'chmod +x ~/gmsl4_start.sh'
   ```

3. Start the four-camera preview:

   ```bash
   ssh seeed@<Jetson IP> 'bash ~/gmsl4_start.sh preview'
   ```

The script is configured by default for the SG3S-ISX031C-GMSL2F at 1920×1536@30 (the script sets `WIDTH, HEIGHT, FPS = 1920, 1536, 30`) and presets `sensor_mode=0`. When using another model, first change `WIDTH, HEIGHT, FPS` inside the script to that model's actual resolution (see the model table above), then run it.

The `preview` mode requires a display connected to the Jetson (local X11). Without a display, run `bash gmsl4_start.sh verify` for a headless self-check.