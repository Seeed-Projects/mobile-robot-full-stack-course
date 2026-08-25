# DM-H65 Two-Wheel Mobile Chassis Python SDK

A Python SDK for controlling DM-H65 hub motors over Linux SocketCAN on NVIDIA Jetson. It includes single-motor control, differential-drive chassis control, command-line diagnostics, Jetson deployment services, and a remote debugging WebUI.

The SDK is integrated at `robot_drivers/` in the course repository root. It is a Python-only project and does not include or depend on the C++ implementation.

From the course repository root:

```bash
cd robot_drivers
```

The following configuration has been verified on a reComputer Robotics J501:

- CAN interface: `can0`, Classic CAN at `1 Mbps`
- Left wheel: motor ID `0x01`, installation direction `-1`
- Right wheel: motor ID `0x02`, installation direction `+1`
- Wheel radius: `0.0825 m`
- Track width: `0.42 m`
- Python: 3.10 or later

## Project Structure

```text
robot_drivers/
├── src/dm_h65/          # Installable Python package, CLI, and WebUI backend
├── examples/            # Single-motor, dual-motor, and bounded motion tests
├── tests/               # Unit tests that do not require CAN hardware
├── webui/frontend/      # React WebUI source
├── scripts/             # CAN, WebUI, test, and Jetson installation scripts
├── deploy/systemd/      # systemd service templates
├── docs/                # API, WebUI, deployment, and troubleshooting guides
└── pyproject.toml       # Python package configuration
```

The `robot_drivers/` directory is the only installation root for this SDK.

## Quick Installation

On the Jetson:

```bash
sudo apt update
sudo apt install -y can-utils python3-venv

cd /path/to/mobile-robotics-fullstack-course/robot_drivers
python3 -m venv .venv
source .venv/bin/activate
python -m pip install --upgrade pip
python -m pip install -e .
```

Verify the installed version, import location, and command-line entry points:

```bash
python -c "import dm_h65; print(dm_h65.__version__); print(dm_h65.__file__)"
dm-h65 --help
dm-h65-webui --help
```

Other Python applications can use the SDK after activating the same virtual environment:

```python
from dm_h65 import DMH65Motor, DifferentialDrive, PythonCanTransport
```

## Configure CAN0

The DM-H65 uses 1 Mbps by default. To configure `can0` temporarily:

```bash
sudo bash scripts/setup_can.sh can0 1000000
ip -details -statistics link show can0
```

A healthy interface should report values similar to:

```text
state ERROR-ACTIVE
bitrate 1000000
berr-counter tx 0 rx 0
```

Scan the bus and read both motors:

```bash
dm-h65 --interface can0 scan --first 0 --last 15
dm-h65 --interface can0 status 0x01
dm-h65 --interface can0 status 0x02
python examples/dual_motor_status.py
```

## Python API Examples

Read a motor without commanding motion:

```python
from dm_h65 import DMH65Motor, PythonCanTransport

with PythonCanTransport("can0") as can_bus:
    left = DMH65Motor(can_bus, 0x01, direction=-1)
    print(left.read_identity())
    print(left.read_telemetry())
```

Control the two-wheel differential-drive chassis:

```python
from dm_h65 import DMH65Motor, DifferentialDrive, PythonCanTransport

with PythonCanTransport("can0") as can_bus:
    left = DMH65Motor(can_bus, 0x01, direction=-1, max_speed_rad_s=8.0)
    right = DMH65Motor(can_bus, 0x02, direction=1, max_speed_rad_s=8.0)
    chassis = DifferentialDrive(
        left,
        right,
        wheel_radius_m=0.0825,
        track_width_m=0.42,
        max_wheel_speed_rad_s=8.0,
    )

    try:
        chassis.enable()
        chassis.hold_twist(
            linear_m_s=0.05,
            angular_rad_s=0.0,
            duration_s=1.0,
            command_hz=50.0,
        )
    finally:
        chassis.disable(restore_modes=True)
```

Before any motion test, lift and secure the wheels, clear the operating area, and prepare a physical power-cut emergency stop. The following examples enforce bounded speed and duration:

```bash
python examples/verified_rotation_test.py \
  --interface can0 --motor-id 0x01 \
  --velocity 0.5 --duration 1.0 --confirm-motion

python examples/differential_drive_test.py \
  --interface can0 --wheel-radius 0.0825 --track-width 0.42 \
  --linear 0.05 --duration 1.0 --confirm-motion
```

See the [Python API guide](docs/PYTHON_API.md) for class, method, and exception details.

## Start the WebUI

The built frontend is included in the Python package, so Node.js is not required at runtime:

```bash
source .venv/bin/activate
dm-h65-webui --host 0.0.0.0 --port 8765 --interface can0
```

Alternatively, start it from the source tree:

```bash
bash scripts/start_webui.sh
```

Open the following URL from a computer on the same network:

```text
http://<Jetson-IP>:8765/
```

For remote access through an SSH tunnel:

```bash
ssh -N -L 127.0.0.1:8765:127.0.0.1:8765 \
  -p 8084 seeed@18.136.89.187
```

Then open `http://127.0.0.1:8765/`. Confirm that the footer identifies `can0` and real-hardware mode; a server started with `--simulate` never sends CAN commands to the motors.

WebUI workflow:

1. Acquire control ownership.
2. Confirm that motors `0x01` and `0x02` are online.
3. Press and hold while dragging the joystick.
4. Release the joystick to stop and disable both motors.
5. Release control ownership when debugging is complete.

Control ownership remains valid until explicitly released, an emergency stop is triggered, or the service restarts. The independent 750 ms command watchdog stops and disables the motors if command updates are interrupted.

See the [WebUI guide](docs/WEBUI.md) for detailed operation and API information.

## Start Automatically on Jetson

The verified J501 setup uses `gpiochip2` line 9 to control the onboard CAN0 termination resistor. Install the Python environment and systemd services with:

```bash
sudo bash scripts/install_jetson.sh
```

Manage the WebUI service:

```bash
sudo systemctl start dm-h65-webui.service
sudo systemctl restart dm-h65-webui.service
sudo systemctl stop dm-h65-webui.service
systemctl status dm-h65-webui.service
journalctl -u dm-h65-webui.service -f
```

If the robot uses an external termination resistor or a different carrier board, read the [Jetson deployment guide](docs/DEPLOYMENT.md) and verify the GPIO mapping before installing the termination service.

## Development and Testing

```bash
python -m pip install -e ".[dev]"
bash scripts/test.sh
python -m build
```

For WebUI frontend development:

```bash
cd webui/frontend
npm install
npm run build
cd ../..
python scripts/sync_webui_assets.py
python -m build
```

The published Python package serves the built assets from `src/dm_h65/webui_static/`. Always synchronize the frontend build before creating a new wheel.

## Safety Constraints

- The SDK changes the control mode in RAM only. It does not send a parameter-save command or write the controller Flash.
- The WebUI limits linear speed to `0.6 m/s` and angular speed to `2.0 rad/s`.
- Both motors must be online before the chassis can be enabled.
- Emergency stop immediately commands zero speed, disables both motors, and releases control ownership.
- Resetting emergency stop does not re-enable the motors.
- A software emergency stop is not a substitute for an independent hardware power-cut emergency stop.
- Do not run multiple applications that consume response frames from the same CAN interface at the same time.

For startup timeouts, CAN bus-off, control ownership issues, or a WebUI that shows commands while the wheels do not move, see the [troubleshooting guide](docs/TROUBLESHOOTING.md).
