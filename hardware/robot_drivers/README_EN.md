# Simple AMR

[简体中文](README.md) | [English](README_EN.md)

Simple AMR is a two-wheel differential-drive stack for a Jetson, SocketCAN, and DM-H65 hub motors. It includes a Python SDK, a browser dashboard, and a ROS 2 Humble `/cmd_vel` bridge. The WebUI backend is the only process that opens CAN; browser and ROS 2 velocity control are mutually exclusive.

## Verified hardware configuration

| Item | Configuration |
| --- | --- |
| CAN | `can1`, Classic CAN, 1 Mbps |
| Left / right wheel | IDs `0x01` / `0x02`, installed directions `-1` / `+1` |
| Wheel radius / track width | `0.0825 m` / `0.42 m` |
| Project directory on Jetson | `/home/seeed/seeed_amr_sdk` |
| WebUI | `http://<Jetson-IP>:8765/` |

These values describe the current robot. Verify them before using the software on another chassis. Do not run a second motor-control process against `can1`.

## Manage the SDK on the configured Jetson

After a reboot, run the following from the project directory. **No argument defaults to WebUI control**; there is no interactive menu:

```bash
cd /home/seeed/seeed_amr_sdk
python3 scripts/start_simple_amr.py start         # default: WebUI
python3 scripts/start_simple_amr.py start ros2    # ROS 2 /cmd_vel
python3 scripts/start_simple_amr.py status        # inspect the stack
python3 scripts/start_simple_amr.py stop          # safely stop the whole SDK
```

The old no-argument, `webui`, and `ros2` forms still work. The script starts the manual CAN1 service, reuses or starts the WebUI, and selects the requested control mode. In `ros2` mode it also starts the bridge and waits for a zero-velocity ready state; in `webui` mode it stops the bridge. The WebUI remains available for status in either mode. `stop` first triggers emergency stop and verifies that motor output is disabled, then stops the bridge, gracefully exits the WebUI, and finally shuts down CAN1. If safe stopping cannot be confirmed, it reports an error and leaves CAN1 up. Install the CAN1 and ROS 2 units below before first use; older CAN0/J501 service templates are not suitable for this CAN1 device.

On this Jetson, you can optionally install a short command once (without enabling boot autostart):

```bash
chmod +x scripts/start_simple_amr.py
sudo ln -s /home/seeed/seeed_amr_sdk/scripts/start_simple_amr.py /usr/local/bin/simple-amr
simple-amr start
simple-amr start ros2
simple-amr status
simple-amr stop
```

Check the system:

```bash
ip -details -statistics link show can1
curl http://127.0.0.1:8765/api/state
```

CAN should report `UP`, `ERROR-ACTIVE`, and `bitrate 1000000`. Open `http://<Jetson-IP>:8765/` and confirm that the page shows `can1 · 实机模式` (real hardware), both motors are online, and emergency stop is not latched.

## Browser control

Select “网页控制” (WebUI control). Drive with the joystick, W/A/S/D, arrow keys, or the in-place rotation buttons. Releasing an input ramps down to zero velocity. There is no control-ownership or hold-to-enable step. Emergency stop disables the motors; reset it only after checking the physical area. Displayed target wheel speeds are **commanded values**, not measured motor speeds.

The dashboard's acceleration/deceleration time can be set while stopped from `0.3` to `1.5` seconds (default `0.8` seconds) and is saved on the Jetson in `~/.config/simple-amr/drive.json`. WebUI acceleration and release-to-stop use this ramp. A longer time feels gentler but increases stopping distance. This setting does not change ROS 2 command response or delay emergency stop, control-mode changes, or the communication watchdog.

## ROS 2 control

Run `python3 scripts/start_simple_amr.py ros2`, or select “ROS 2 控制” in the WebUI when the bridge is online. The bridge subscribes to `/cmd_vel` as `geometry_msgs/msg/Twist`: positive `linear.x` means forward, positive `angular.z` means left, and the other four components must be zero. Browser driving inputs are paused in ROS 2 mode.

```bash
source /opt/ros/humble/setup.bash
ros2 topic info /cmd_vel
sudo apt install ros-humble-teleop-twist-keyboard  # first installation only
ros2 run teleop_twist_keyboard teleop_twist_keyboard --ros-args -p speed:=0.1 -p turn:=0.4
```

`ros2 topic info` should show one subscriber. With the keyboard node's terminal focused, press `i` for forward, `,` for reverse, `j` for left, `l` for right, `k` to stop, and `Ctrl+C` to exit. The chassis stops if no velocity command arrives for more than 750 ms, so a single key press may produce only brief motion. See the [ROS 2 control guide](docs/ROS2.md) for detailed safety behavior.

## First installation or development setup

Prepare the Python environment on the Jetson. Built WebUI assets are included in the repository; Node.js is not required at runtime:

```bash
sudo apt update
sudo apt install -y can-utils gpiod python3-venv
cd /home/seeed/seeed_amr_sdk
python3 -m venv .venv
.venv/bin/python -m pip install -e .
.venv/bin/dm-h65-webui --help
```

On a new device, check the carrier-board documentation for termination, GPIO mapping, and CAN wiring first. The verified J401/CAN1 manual unit is `deploy/systemd/dm-h65-can1-manual.service` and uses `gpiochip2` line 4; **do not apply it unchanged to another carrier board**. After verifying the hardware, install the units without enabling boot autostart:

```bash
sudo cp deploy/systemd/dm-h65-can1-manual.service deploy/systemd/dm-h65-ros2-bridge.service /etc/systemd/system/
sudo systemctl daemon-reload
```

**Do not run an older `install_jetson.sh` unchanged**: it targets a CAN0/J501 configuration. `sudo bash scripts/setup_can.sh can1 1000000` configures CAN1 temporarily but does not install a systemd unit. See the [ROS 2 control guide](docs/ROS2.md) for bridge details.

Run read-only motor diagnostics only after stopping the WebUI chassis service; otherwise multiple processes may compete for CAN response frames:

```bash
.venv/bin/dm-h65 --interface can1 scan --first 0 --last 15
.venv/bin/dm-h65 --interface can1 status 0x01
.venv/bin/dm-h65 --interface can1 status 0x02
```

Run local tests and build the frontend:

```bash
PYTHONPATH=src python3 -m unittest discover -s tests -v
cd webui/frontend
npm install
npm run build
npm run test:sites
cd ../..
python3 scripts/sync_webui_assets.py
```

`sync_webui_assets.py` **replaces** `src/dm_h65/webui_static/` with the new build; run it only when updating the frontend. The Python package is in `src/dm_h65/`, the ROS 2 bridge in `scripts/ros2_cmd_vel_bridge.py`, the frontend in `webui/frontend/`, and tests in `tests/`.

## Safety

- The server limits absolute linear speed to `0.6 m/s` and angular speed to `2.0 rad/s`; both motors must be online before movement is allowed.
- Switching control modes stops the chassis. Loss of the ROS 2 bridge stops it; emergency stop disables the motors and returns to WebUI mode.
- Software emergency stop and command timeouts are **not substitutes for an independent hardware power-cut emergency stop**. Lift and secure the wheels and clear the area before the first motion test.
- The WebUI has no public-internet authentication. Do not expose port 8765 directly to the internet; use a trusted LAN, VPN, or SSH tunnel.
