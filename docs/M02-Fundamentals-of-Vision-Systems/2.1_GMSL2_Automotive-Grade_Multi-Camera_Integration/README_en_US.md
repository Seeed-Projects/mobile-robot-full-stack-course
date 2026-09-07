# 2.1 GMSL2: Automotive-Grade Multi-Camera Integration

### What Is GMSL2?

GMSL2 (Gigabit Multimedia Serial Link 2) is a high-speed serializer/deserializer (SerDes) technology developed by Analog Devices (formerly Maxim Integrated). It is designed for video, audio, and control-data transmission in automotive and other high-reliability applications. A single coaxial cable or shielded twisted pair (STP) can carry up to 6 Gbps of forward data together with a reverse control channel, combining HD video, I²C configuration, GPIO signals, and even Power over Coax (PoC) on one cable. Compared with first-generation GMSL, GMSL2 significantly improves bandwidth, transmission distance, EMC immunity, and functional safety. It also supports HDCP content protection and is widely used in automotive surround-view cameras, ADAS sensors, domain-controller displays, and industrial robot vision systems that require low latency and high reliability.

![Mini-Fakra 4-in-1 GMSL Cable](./images/DEq5bdwPZoVFk9x68Bpc55Hknlf.png)

*Mini-Fakra 4-in-1 GMSL Cable*

### GMSL vs. CSI

GMSL and CSI (MIPI CSI-2) are not competing technologies. They operate at different layers and are commonly used together. CSI-2 is a camera serial-interface protocol defined by the MIPI Alliance. It specifies how image data is transferred from a sensor to a processor and is widely used for short, board-level connections in phones, embedded systems, and automotive SoCs. Its physical layer uses D-PHY or C-PHY, and its typical transmission distance is measured in centimeters. GMSL, by contrast, is a complete SerDes solution from Analog Devices for long-distance transmission in electrically noisy automotive environments. It carries video, control signals, and even power over a single coaxial or STP cable for distances beyond 15 meters. In a typical automotive camera link, the image sensor sends CSI-2 data to a GMSL serializer. After the long cable run, a GMSL deserializer converts it back to CSI-2 for the domain-controller SoC. In short, CSI-2 defines how image data moves between chips, while GMSL provides reliable long-distance physical transport.

![image.png](./images/DDeLbmDJqoh3IpxrMptcQEo4nNh.png)

### Hardware List

- reComputer Robotics J501 or reComputer Mini J501 with a GMSL expansion board
- SG3S-ISX031C-GMSL2F or SG8S-AR0820 GMSL2 camera
- Mini-Fakra 4-in-1 cable

| ![image.png](./images/T5LFbUbVqoARCHxZR37cPey6nOe.png)<br><br>reComputer Mini J501 with GMSL expansion board | ![image.png](./images/XZtGbkTWqorbaNxgzfucztxlnWc.png)<br><br>Sensing SG3S GMSL camera | ![image.png](./images/Hi4Pb2fbFo3SHWxPypecQuNonse.png)<br><br>Mini-Fakra 4-in-1 cable |
| --- | --- | --- |

### Usage Guide

#### Hardware Connection

Connect your GMSL cameras to the J501 carrier board as shown below.

![image.png](./images/KvirbW6J4okHZixybbgcW0Z5nnh.png)

#### Selecting the Device Tree

Follow the steps below to select the appropriate device tree.

> Note: This chapter uses a Seeed reComputer Mini J501 and Sensing SG3S-ISX031C-GMSL2F camera as its reference hardware. Select the device tree that matches your actual hardware combination. See the GMSL configuration-selection documentation for details.

![image.png](./images/NK0DbLglwo77zjxFjCbcyr3vn3b.png)

After restarting the device, the connected cameras should appear if the correct device-tree file was selected:

```bash
ls /dev/video*
```

![image.png](./images/Sw07bMFXOoGLkBx5qmZc6aBYnBg.png)

#### Multi-Camera Synchronization

The J501 carrier-board device-tree configuration enables four GMSL cameras. During startup, it creates four CSI/V4L2 camera links and configures the FSYNC input of the MAX96712 deserializer together with the MFP7 FSYNC outputs of the four MAX96717 serializers. This lets a common synchronization clock travel through the GMSL links to the FSIN/trigger pins of all four cameras, enabling the sensors to expose at the same hardware-controlled cadence.

##### Inspecting the Media-Controller Topology

```bash
media-ctl -d /dev/media0 -p
```

![image.png](./images/P7A5bCip2ogMpjxrVWNcmw3onub.png)

This verifies that the sensor, serializer, deserializer, CSI interface, and V4L2 node are connected.

##### Checking Whether the Cameras Are in Use

```bash
fuser /dev/video0 /dev/video1 /dev/video2 /dev/video3
```

No output means that the devices are not currently in use.

##### Setting the Serializer/Deserializer Formats

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

> Note: Set the resolution to match the cameras actually connected. The embedded Feishu table below lists the GMSL camera models supported by the J501 carrier board and their corresponding resolutions.

> [Embedded Feishu table (view in the source document)](https://seeedstudio.feishu.cn/wiki/YiGow5u7QiEifnkVGhycLgMonMg)

##### Setting the V4L2 Node Format and Sensor Mode

```bash
for i in 0 1 2 3; do
  sudo v4l2-ctl -d /dev/video${i} \
    --set-fmt-video=width=1920,height=1536,pixelformat=YUYV \
    -c sensor_mode=0
done
```

> The `sensor_mode` parameter selects the GMSL camera model and resolution:
>
> - `sensor_mode=0` → `YUYV8_1X16/1920x1536`
> - `sensor_mode=1` → `YUYV8_1X16/1920x1080`
> - `sensor_mode=2` → `YUYV8_1X16/3840x2160`

Verify that the settings took effect:

```bash
for i in 0 1 2 3; do
  echo "===== /dev/video${i} ====="
  v4l2-ctl -d /dev/video${i} --get-fmt-video --get-parm
done
```

Expected output:

![image.png](./images/YB3vbry1joFt6Ix1VfmchRzenzc.png)

##### Starting All Cameras Synchronously

Copy the following script to the J501 and run it:

```bash
# Start multiple cameras at the same time
bash gmsl4_start.sh preview
```

[Attachment: gmsl4_start.sh (Feishu source document)](https://seeedstudio.feishu.cn/wiki/YiGow5u7QiEifnkVGhycLgMonMg)

> Note: If your GMSL camera model is not the Sensing SG3S-ISX031C-GMSL2F, change the script's `WIDTH, HEIGHT, FPS = 1920, 1536, 30` values to the camera's actual resolution and frame rate.

![Demonstration GIF](./images/BeIUbjExDoZ6VkxZLxxc933un9o.gif)

