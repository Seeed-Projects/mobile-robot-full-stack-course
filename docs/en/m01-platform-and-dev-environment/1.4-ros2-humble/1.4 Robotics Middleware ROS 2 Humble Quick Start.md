# M1 Platform and Development Environment

> **Real hardware environment**: reComputer Robotics J501 (AGX Orin 32GB), L4T 36.4.4 / JetPack 6.2.1
>
> **Host PC**: Ubuntu 22.04.5 LTS, x86\_64

***

# 1.4 Robotics Software Middleware: ROS 2 Humble Quick Start

ROS 2 (Robot Operating System 2) is the standard software middleware in the robotics field. It provides inter-node communication, hardware abstraction, package management, and other capabilities, and serves as the foundation for all subsequent robotics modules (navigation, SLAM, robotic arm control). In this section, you will install ROS 2 Humble on both the J501 and the Host PC, write publisher/subscriber nodes, and implement cross-machine communication. This chapter follows the main thread of "installing ROS 2 → running the first demo → setting up the workspace" to prepare the ROS 2 development environment on both the J501 and the Host PC.

> **Why choose ROS 2 Humble?** Humble Hawksbill is the LTS (Long Term Support) release of ROS 2, supported until May 2027, and it matches Ubuntu 22.04 + JetPack 6.2 perfectly.
>
> **Environment notes**: This chapter takes the J501's built-in Ubuntu 22.04 system and a Host PC running Ubuntu 22.04 (x86) as examples. Both sides install ROS 2 using the FishROS one-click script (see 1.4.2 (2) for details); other installation methods such as building from source can be followed analogously.

### 1.4.1 ROS 2 Architecture Overview

This section introduces what ROS 2 and DDS are and how they work together; see the [official documentation](https://docs.ros.org/en/humble/index.html) for detailed background.

#### (1) Core Concepts

| Concept          | Description                                                             | Analogy                 |
| ---------------- | ----------------------------------------------------------------------- | ----------------------- |
| **Node**         | The smallest execution unit; each node is responsible for one functional module | A function in a program |
| **Topic**        | Asynchronous publish/subscribe communication, suitable for sensor data streams | A radio station         |
| **Service**      | Synchronous request/response communication, suitable for one-off commands | A function call         |
| **Action**       | Long-running tasks (with feedback and cancellation), suitable for navigation and more | An asynchronous task    |
| **Parameter**    | Runtime configuration of a node                                         | Command-line arguments  |
| **DDS**          | The underlying middleware, responsible for data transport               | TCP/IP                  |

#### (2) DDS Middleware

ROS 2 uses DDS (Data Distribution Service) as its underlying communication middleware by default. On the Jetson platform, CycloneDDS is used by default (it performs better than the default FastDDS).

![ROS 2 + DDS communication architecture diagram](images/1.4.1.2_dds_architecture.png)

#### (3) Communication Model Comparison

| Feature              | Topic           | Service      | Action                              |
| -------------------- | --------------- | ------------ | ----------------------------------- |
| Communication mode   | Publish/subscribe | Request/response | Goal/feedback/result            |
| Synchronicity        | Asynchronous    | Synchronous  | Asynchronous                        |
| Use cases            | Sensor data streams | One-off commands | Long tasks such as navigation and grasping |
| Example              | `/camera/image` | `/set_speed` | `/navigate_to_pose`                 |

### 1.4.2 Installing ROS 2 Humble

ROS 2 Humble is not preinstalled on the J501 and must be installed manually. This section provides two installation methods.

#### (1) Method 1: FishROS One-Click Installation (Recommended)

FishROS (a community one-click ROS installer) is a one-click installation tool maintained by the ROS community in China. It automatically configures domestic mirror sources and is well suited to network environments in China.

```bash
# [J501 local terminal] Install ROS 2 with one click using FishROS
wget http://fishros.com/install -O fishros && . fishros
```

Key interactive choices during installation:

1. Select `1` (one-click ROS/ROS2 installation)
2. Select `2` (ROS 2 Humble)
3. Select `1` (desktop-full, including visualization tools such as rviz and rqt)
4. Wait for the installation to finish (about 10-20 minutes, depending on the network)

![FishROS installation - Step 1](images/1.4.2.1_fishros_install_step1.png)

![FishROS installation - Step 2](images/1.4.2.1_fishros_install_step2.png)

![FishROS installation - Step 3](images/1.4.2.1_fishros_install_step3.png)

```bash
# [J501 local terminal] Verify the installation
source /opt/ros/humble/setup.bash
ros2 --version 2>/dev/null || ros2 doctor --report | grep "distribution"
```

![Installation complete and verified](images/1.4.2.1_fishros_install_done.png)

**Actual output:**

```
distribution: humble
```

#### (2) Method 2: Official apt Installation

If the FishROS installation fails, you can install manually from the official sources:

```bash
# [J501 local terminal] Add the official ROS 2 apt source
sudo apt update && sudo apt install -y software-properties-common
sudo add-apt-repository universe

# Add the ROS 2 GPG key
sudo apt update && sudo apt install -y curl gnupg lsb-release
sudo curl -sSL https://raw.githubusercontent.com/ros/rosdistro/master/ros.key \
    -o /usr/share/keyrings/ros-archive-keyring.gpg

# Add the ROS 2 source
echo "deb [arch=$(dpkg --print-architecture) signed-by=/usr/share/keyrings/ros-archive-keyring.gpg] \
http://packages.ros.org/ros2/ubuntu $(. /etc/os-release && echo $UBUNTU_CODENAME) main" | \
    sudo tee /etc/apt/sources.list.d/ros2.list > /dev/null

# Install ROS 2 Humble desktop-full
sudo apt update
sudo apt install -y ros-humble-desktop ros-dev-tools
```

> **Note**: The official sources are slow to access from China, so the FishROS method is recommended. `ros-dev-tools` includes development tools such as `colcon` and `rosdep`.

#### (3) Configuring Environment Variables

> **Note**: If you installed with FishROS (Method 1), the script has already written `source /opt/ros/humble/setup.bash` into `~/.bashrc` automatically. You can skip the adding step and verify directly.

First, check whether the environment variables are already configured:

```bash
# [J501 local terminal] Check whether the ROS environment variables are already in bashrc
grep "setup.bash" ~/.bashrc
```

- If the output contains `source /opt/ros/humble/setup.bash`, it is already configured; skip the next step.
- If there is no output, run the following commands to add it manually:

```bash
# [J501 local terminal] Add the ROS 2 environment variables to bashrc
echo 'source /opt/ros/humble/setup.bash' >> ~/.bashrc
source ~/.bashrc
```

```bash
# [J501 local terminal] Verify the environment variables
echo $ROS_DISTRO
echo $AMENT_PREFIX_PATH | tr ":" "\n" | head -3
```

![Verifying the ROS 2 environment variables](images/1.4.2.3_verify_env.png)

**Actual output:**

```
humble
/opt/ros/humble
```

```bash
# [Host PC -> J501 SSH] Verify the environment variables (non-interactive SSH requires manual sourcing)
ssh J501 'source /opt/ros/humble/setup.bash && echo $ROS_DISTRO && echo "$AMENT_PREFIX_PATH" | tr ":" "\n" | head -3'
```

**Actual output:**

```
humble
/opt/ros/humble
```

#### (4) Installing ROS 2 on the Host PC

Cross-machine communication requires ROS 2 Humble on the Host PC as well. The installation method is the same as on the J501:

```bash
# [Host PC terminal] FishROS one-click installation
wget http://fishros.com/install -O fishros && . fishros
```

![ROS 2 successfully installed on the Host PC](images/1.4.2.4_hostpc_ros2_installed.png)

```bash
# Configure the environment variables
echo 'source /opt/ros/humble/setup.bash' >> ~/.bashrc
source ~/.bashrc
```

```bash
# [Host PC terminal] Verify the installation
ros2 --version 2>/dev/null || echo $ROS_DISTRO
```

**Actual output:**

```
humble
```

### 1.4.3 The colcon Build System and Workspace Management

`colcon` is the official build system of ROS 2, used to build multiple packages in a workspace.

#### (1) Creating a Workspace

```bash
# [J501 local terminal] Create a ROS 2 workspace
mkdir -p ~/ros2_ws/src
cd ~/ros2_ws
colcon build --symlink-install
```

**Actual output:**

```
Summary: 0 packages finished
```

> **Note**: The `--symlink-install` argument lets changes to Python scripts take effect without rebuilding, and it is recommended during development. Having 0 packages built on the first build is normal, because the `src` directory is still empty.

```bash
# [J501 local terminal] Source the workspace environment variables
echo 'source ~/ros2_ws/install/setup.bash' >> ~/.bashrc
source ~/.bashrc
```

#### (2) Common colcon Commands

| Command                                  | Description                                  |
| ---------------------------------------- | -------------------------------------------- |
| `colcon build`                           | Build all packages in the workspace          |
| `colcon build --packages-select <pkg>`   | Build only the specified package             |
| `colcon build --symlink-install`         | Symlink installation (development mode)      |
| `colcon list`                            | List all packages in the workspace           |
| `colcon graph`                           | Show the package dependency graph            |

### 1.4.4 Writing Publisher/Subscriber Nodes

This section builds a **Robot Commander Demo**: the J501 acts as the robot side, periodically publishing system status (CPU temperature, memory, disk) while receiving control commands from the Host PC (turning the status LED on/off, sending greetings). This demo simulates the bidirectional communication pattern of "status reporting + command downlink" found in real robots.

#### (1) Creating a Python Package

```bash
# [J501 local terminal] Create a ROS 2 Python package
cd ~/ros2_ws/src
ros2 pkg create --build-type ament_python j501_robot \
    --license MIT \
    --description "J501 robot status publisher and command console" \
    --dependencies rclpy std_msgs
```

![Creating a ROS 2 Python package](images/1.4.4.1_create_pkg.png)

**Actual output:**

```
going to create a new package
package name: j501_robot
destination directory: /home/seeed/ros2_ws/src
package format: 3
version: 0.0.0
description: J501 robot status publisher and command console
maintainer: ['seeed <seeed@todo.todo>']
licenses: ['MIT']
build type: ament_python
dependencies: ['rclpy', 'std_msgs']
creating folder ./j501_robot
creating ./j501_robot/package.xml
creating source folder
creating folder ./j501_robot/j501_robot
creating ./j501_robot/setup.py
creating ./j501_robot/setup.cfg
creating folder ./j501_robot/resource
creating ./j501_robot/resource/j501_robot
creating ./j501_robot/j501_robot/__init__.py
creating folder ./j501_robot/test
creating ./j501_robot/test/test_copyright.py
creating ./j501_robot/test/test_flake8.py
creating ./j501_robot/test/test_pep257.py
```

> **Note**: With the `--license MIT` and `--description` arguments, the `WARNING: Unknown license` warning is avoided. Supported licenses include `Apache-2.0`, `MIT`, `BSD-3-Clause`, `GPL-3.0-only`, and so on.

#### (2) Robot Status Publisher (Runs on the J501)

This node simulates a robot periodically reporting its own status: CPU temperature, memory usage, disk usage, and a "heartbeat" counter. It also listens on the control command topic, replying when a greeting is received and simulating turning the LED on/off when an LED command is received.

```bash
# [J501 local terminal] Create the robot status publisher
cat > ~/ros2_ws/src/j501_robot/j501_robot/robot_status.py << 'PYEOF'
"""
J501 robot status publisher (runs on the J501)

Features:
  1. Periodically (every 2 seconds) publishes system status to the /robot/status topic
     - CPU temperature (read from thermal_zone0)
     - Memory usage (read from /proc/meminfo)
     - Disk usage (via df -h /)
     - Heartbeat counter (used to detect whether the node is alive)
  2. Listens on the /robot/cmd topic to receive control commands
     - LED:ON / LED:OFF  -> Simulates turning the LED on/off
     - SAY:<text>        -> Receives a greeting and replies
     - STATUS            -> Triggers an immediate status report

Communication model:
  /robot/status  <- publishes (String)  -> subscribed by the Host PC
  /robot/cmd     <- subscribes (String) <- published by the Host PC
"""

import os
import subprocess
import rclpy
from rclpy.node import Node
from std_msgs.msg import String


class RobotStatusNode(Node):
    """J501 robot status node - the robot side of the bidirectional communication"""

    def __init__(self):
        # Call the parent class constructor and set the node name to 'robot_status'
        super().__init__('robot_status')

        # Create a publisher: publishes String messages on the /robot/status topic
        # The argument 10 is the QoS (Quality of Service) depth, i.e. the buffer queue length
        self.status_pub = self.create_publisher(String, '/robot/status', 10)

        # Create a subscriber: listens for String messages on the /robot/cmd topic
        # The self.cmd_callback callback is invoked automatically when a message arrives
        self.cmd_sub = self.create_subscription(
            String, '/robot/cmd', self.cmd_callback, 10
        )

        # Heartbeat counter: incremented by 1 on every status publication,
        # used to detect whether the node is alive
        self.heartbeat = 0

        # Create a timer: calls self.publish_status every 2.0 seconds
        # This is the standard way to implement periodic tasks in ROS 2
        self.timer = self.create_timer(2.0, self.publish_status)

        # Print startup information so it is easy to confirm the node is running properly
        self.get_logger().info('🤖 J501 机器人状态节点已启动')
        self.get_logger().info('   发布话题: /robot/status')
        self.get_logger().info('   订阅话题: /robot/cmd')

    def publish_status(self):
        """Timer callback: collects system status and publishes it to /robot/status"""
        self.heartbeat += 1

        # Collect the three system metrics
        cpu_temp = self._get_cpu_temp()      # CPU temperature
        mem_usage = self._get_mem_usage()    # memory usage
        disk_usage = self._get_disk_usage()  # disk usage

        # Assemble the status string, example format: [#1] CPU=42.0°C | MEM=7898/32676MB(24%) | DISK=45G/116G(41%)
        msg = String()
        msg.data = (
            f'[#{self.heartbeat}] '
            f'CPU={cpu_temp}°C | '
            f'MEM={mem_usage} | '
            f'DISK={disk_usage}'
        )

        # Publish the message to the topic
        self.status_pub.publish(msg)
        self.get_logger().info(f'📤 上报: {msg.data}')

    def cmd_callback(self, msg):
        """
        Command callback: called when a /robot/cmd message is received

        Supported command formats:
          LED:ON     -> turn the LED on (simulated)
          LED:OFF    -> turn the LED off (simulated)
          SAY:<text> -> receive a greeting and reply
          STATUS     -> report the status immediately
        """
        cmd = msg.data.strip()  # strip leading/trailing whitespace

        # Command parsing: dispatch to different handling logic based on the prefix
        if cmd.startswith('LED:'):
            # LED command: format is LED:ON or LED:OFF
            led_state = cmd.split(':')[1]
            if led_state.upper() == 'ON':
                self.get_logger().info('💡 收到指令: 点亮 LED（模拟）')
                # In a real project you could call GPIO pin control functions here
            elif led_state.upper() == 'OFF':
                self.get_logger().info('💡 收到指令: 熄灭 LED（模拟）')
            else:
                self.get_logger().warn(f'⚠️ 未知 LED 指令: {led_state}')

        elif cmd.startswith('SAY:'):
            # Greeting command: format is SAY:<any text>
            # split(':', 1) splits only on the first colon,
            # avoiding issues when the text itself contains colons
            text = cmd.split(':', 1)[1]
            self.get_logger().info(f'💬 收到问候: "{text}" -> 回复: 你好！我是 J501 机器人')

        elif cmd == 'STATUS':
            # Query command: trigger an immediate status report (without waiting for the timer)
            self.get_logger().info('📋 收到查询指令，立即上报状态')
            self.publish_status()

        else:
            # Unknown command: print a warning for easier debugging
            self.get_logger().warn(f'⚠️ 未知指令: "{cmd}"')

    def _get_cpu_temp(self):
        """Read the CPU temperature (°C) via the sysfs thermal_zone0 interface"""
        try:
            # On Jetson, the CPU temperature is exposed at /sys/class/thermal/thermal_zone0/temp
            # The value is in milli-degrees Celsius; divide by 1000 to convert to °C
            with open('/sys/class/thermal/thermal_zone0/temp', 'r') as f:
                return f'{int(f.read().strip()) / 1000:.1f}'
        except Exception:
            return 'N/A'

    def _get_mem_usage(self):
        """Read memory usage via the /proc/meminfo interface"""
        try:
            with open('/proc/meminfo', 'r') as f:
                lines = f.readlines()
            # /proc/meminfo format:
            #   line 0: MemTotal:  33489000 kB
            #   line 2: MemAvailable: 25400000 kB
            total = int(lines[0].split()[1])       # total memory (kB)
            avail = int(lines[2].split()[1])        # available memory (kB)
            used = (total - avail) / 1024           # used (MB)
            total_mb = total / 1024                 # total (MB)
            pct = (total - avail) / total * 100     # usage (%)
            return f'{used:.0f}/{total_mb:.0f}MB({pct:.0f}%)'
        except Exception:
            return 'N/A'

    def _get_disk_usage(self):
        """Read the disk usage of the root partition via the df command"""
        try:
            # Run df -h / to get the root partition usage
            result = subprocess.run(
                ['df', '-h', '/'],
                capture_output=True, text=True, timeout=2
            )
            # Output format:
            #   Filesystem      Size  Used Avail Use% Mounted on
            #   /dev/nvme0n1p1  116G   45G   65G  41% /
            # parts[2]=used, parts[1]=total, parts[4]=usage
            parts = result.stdout.strip().split('\n')[1].split()
            return f'{parts[2]}/{parts[1]}({parts[4]})'
        except Exception:
            return 'N/A'


def main(args=None):
    """Node entry function"""
    # Initialize the ROS 2 Python client library
    rclpy.init(args=args)

    # Create the node instance
    node = RobotStatusNode()

    try:
        # spin() blocks the current thread and keeps processing callbacks
        # (timers, subscribers, etc.) until Ctrl+C is received or the node is shut down
        rclpy.spin(node)
    except KeyboardInterrupt:
        node.get_logger().info('🤖 机器人状态节点已停止')

    # Clean up resources: destroy the node and shut down rclpy
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
PYEOF
```

#### (3) Command Console (Runs on the Host PC)

This node runs on the Host PC. It subscribes to the J501's status while allowing the user to send control commands from the command line. The code is created on the J501 as well and synced to the Host PC via `scp` later.

```bash
# [J501 local terminal] Create the command console node (to be synced to the Host PC later)
cat > ~/ros2_ws/src/j501_robot/j501_robot/command_console.py << 'PYEOF'
"""
Host PC command console (runs on the Host PC)

Features:
  1. Subscribes to the /robot/status topic and displays the system status reported by the J501 in real time
  2. Provides an interactive command line; the user can send control commands to the J501 by entering a number
  3. Uses a multi-threaded executor to handle subscription callbacks and user input simultaneously

Numeric command mapping:
  1 -> LED:ON      turn the robot LED on
  2 -> LED:OFF     turn the robot LED off
  3 -> SAY:你好     send a greeting
  4 -> STATUS      request an immediate status report
  0 -> quit        exit the console

Communication model:
  /robot/status  <- subscribes (String) <- published by the J501
  /robot/cmd     <- publishes (String)  -> subscribed by the J501
"""

import sys
import threading
import rclpy
from rclpy.node import Node
from rclpy.executors import MultiThreadedExecutor
from std_msgs.msg import String

# Numeric command mapping: user enters a number -> the corresponding command string
# This way the user only needs to enter a number instead of memorizing the full command format
CMD_MAP = {
    '1': 'LED:ON',       # turn the LED on
    '2': 'LED:OFF',      # turn the LED off
    '3': 'SAY:你好J501，我是Host PC',  # send a greeting
    '4': 'STATUS',       # request an immediate status report
    '0': 'quit',         # exit the console
}


class CommandConsoleNode(Node):
    """Host PC command console node - the Host PC side of the bidirectional communication"""

    def __init__(self):
        # Call the parent class constructor and set the node name to 'command_console'
        super().__init__('command_console')

        # Create a subscriber: listens to the /robot/status topic published by the J501
        # The self.status_callback callback is invoked automatically when a message arrives
        self.status_sub = self.create_subscription(
            String, '/robot/status', self.status_callback, 10
        )

        # Create a publisher: sends control commands to the J501 on the /robot/cmd topic
        self.cmd_pub = self.create_publisher(String, '/robot/cmd', 10)

        # Print startup information and the command menu
        self.get_logger().info('🖥️  Host PC 指令控制台已启动')
        self.get_logger().info('   订阅话题: /robot/status')
        self.get_logger().info('   发布话题: /robot/cmd')
        self.get_logger().info('')
        self.get_logger().info('可用指令（输入数字即可）:')
        self.get_logger().info('  1 - 点亮机器人 LED')
        self.get_logger().info('  2 - 熄灭机器人 LED')
        self.get_logger().info('  3 - 发送问候语')
        self.get_logger().info('  4 - 请求立即上报状态')
        self.get_logger().info('  0 - 退出控制台')
        self.get_logger().info('')

    def status_callback(self, msg):
        """Status callback: called when a J501 status message is received"""
        self.get_logger().info(f'📥 收到机器人状态: {msg.data}')

    def send_cmd(self, cmd_text):
        """Send a control command to the /robot/cmd topic"""
        msg = String()
        msg.data = cmd_text
        self.cmd_pub.publish(msg)
        self.get_logger().info(f'📤 发送指令: {cmd_text}')


def main(args=None):
    """Node entry function"""
    # Initialize the ROS 2 Python client library
    rclpy.init(args=args)
    node = CommandConsoleNode()

    # Use a multi-threaded executor (MultiThreadedExecutor)
    # Reason: the default single-threaded executor cannot process subscription callbacks
    # while input() is blocking. The multi-threaded executor can process callbacks in one
    # thread and handle user input in another thread
    executor = MultiThreadedExecutor()
    executor.add_node(node)

    def input_loop():
        """
        User input loop (runs in a separate thread)

        Reads the number entered by the user, maps it to the full command via CMD_MAP,
        and sends it. Enter 0 or press Ctrl+C to exit
        """
        while rclpy.ok():
            try:
                # input() blocks the current thread waiting for user input
                user_input = input().strip()

                # Look up the command corresponding to the number
                cmd = CMD_MAP.get(user_input)
                if cmd is None:
                    # The user entered something not in the mapping table;
                    # send it as a raw command directly
                    cmd = user_input

                if cmd.lower() == 'quit':
                    break

                # Send the command to the J501
                node.send_cmd(cmd)

            except EOFError:
                # Non-interactive environments (e.g. piped input) trigger EOFError
                break

    # Start the input thread (daemon=True means it terminates automatically
    # when the main program exits)
    thread = threading.Thread(target=input_loop, daemon=True)
    thread.start()

    try:
        # spin() blocks the main thread and keeps processing subscription callbacks
        executor.spin()
    except KeyboardInterrupt:
        pass

    node.get_logger().info('🖥️  控制台已退出')
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
PYEOF
```

#### (4) Configuring entry\_points

```bash
# [J501 local terminal] Edit setup.py
cat > ~/ros2_ws/src/j501_robot/setup.py << 'EOF'
from setuptools import setup

package_name = 'j501_robot'

setup(
    name=package_name,
    version='0.1.0',
    packages=[package_name],
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='seeed',
    maintainer_email='seeed@todo.todo',
    description='J501 robot status publisher and command console',
    license='MIT',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'robot_status = j501_robot.robot_status:main',
            'console = j501_robot.command_console:main',
        ],
    },
)
EOF
```

#### (5) Building

```bash
# [J501 local terminal] Build the package
cd ~/ros2_ws
colcon build --packages-select j501_robot --symlink-install
```

**Actual output:**

```
--- stderr: j501_robot
---
Finished <<< j501_robot [2.45s]

Summary: 1 package finished [2.45s]
```

```bash
# [J501 local terminal - terminal 1] Run the robot status node
source ~/ros2_ws/install/setup.bash
ros2 run j501_robot robot_status
```

**Actual output (robot status):**

```
[INFO] [robot_status]: 🤖 J501 机器人状态节点已启动
[INFO] [robot_status]:    发布话题: /robot/status
[INFO] [robot_status]:    订阅话题: /robot/cmd
[INFO] [robot_status]: 📤 上报: [#1] CPU=42.0°C | MEM=7898/32676MB(24%) | DISK=45G/116G(41%)
[INFO] [robot_status]: 📤 上报: [#2] CPU=42.1°C | MEM=7900/32676MB(24%) | DISK=45G/116G(41%)
```

```bash
# [J501 local terminal - terminal 2] Send a command for testing
source /opt/ros/humble/setup.bash
ros2 topic pub --once /robot/cmd std_msgs/String "data: 'LED:ON'"
```

**Actual output (terminal 1 receives the command):**

```
[INFO] [robot_status]: 💡 收到指令: 点亮 LED（模拟）
```

```bash
# [J501 local terminal - terminal 2] Send a greeting (same as command 3 in the console's CMD_MAP)
ros2 topic pub --once /robot/cmd std_msgs/String "data: 'SAY:你好J501，我是Host PC'"
```

**Actual output (terminal 1 receives the command):**

```
[INFO] [robot_status]: 💡 收到问候: "你好J501，我是Host PC" -> 回复: 你好！我是 J501 机器人
```

> **Confirmation**: Bidirectional communication on a single machine works normally; both status publishing and command responses work.

### 1.4.5 Cross-Machine Communication Demo: Remotely Commanding the J501 Robot

ROS 2's distributed communication is based on DDS. Nodes on different hosts discover each other automatically via multicast and establish point-to-point communication (if there is no router between the two machines, `ROS_DOMAIN_ID` can be configured manually).

#### (1) Network Topology

![ROS 2 cross-machine communication topology diagram](images/1.4.5.1_network_topology.png)

#### (2) Syncing the Code to the Host PC

Running across machines requires the same package on the Host PC as well. Sync it from the J501 via `scp`:

```bash
# [Host PC terminal] Sync the j501_robot package from the J501
mkdir -p ~/ros2_ws/src
scp -r J501:~/ros2_ws/src/j501_robot ~/ros2_ws/src/

# Build on the Host PC
cd ~/ros2_ws
colcon build --packages-select j501_robot --symlink-install
```

#### (3) Starting the Cross-Machine Demo

**Step 1: Run the robot status node on the J501**

```bash
# [J501 local terminal] Start the robot status publisher
source ~/ros2_ws/install/setup.bash
ros2 run j501_robot robot_status
```

**Actual output (J501):**

```
[INFO] [robot_status]: 🤖 J501 机器人状态节点已启动
[INFO] [robot_status]:    发布话题: /robot/status
[INFO] [robot_status]:    订阅话题: /robot/cmd
[INFO] [robot_status]: 📤 上报: [#1] CPU=42.0°C | MEM=7898/32676MB(24%) | DISK=45G/116G(41%)
[INFO] [robot_status]: 📤 上报: [#2] CPU=42.1°C | MEM=7900/32676MB(24%) | DISK=45G/116G(41%)
```

**Step 2: Run the command console on the Host PC**

```bash
# [Host PC terminal] Start the command console
source ~/ros2_ws/install/setup.bash
ros2 run j501_robot console
```

![Host PC command console started](images/1.4.5.3_hostpc_console.png)

**Actual output (the Host PC automatically receives the J501 status):**

```
[INFO] [command_console]: 🖥️  Host PC 指令控制台已启动
[INFO] [command_console]:    订阅话题: /robot/status
[INFO] [command_console]:    发布话题: /robot/cmd
[INFO] [command_console]:
[INFO] [command_console]: 可用指令（输入数字即可）:
[INFO] [command_console]:   1 - 点亮机器人 LED
[INFO] [command_console]:   2 - 熄灭机器人 LED
[INFO] [command_console]:   3 - 发送问候语
[INFO] [command_console]:   4 - 请求立即上报状态
[INFO] [command_console]:   0 - 退出控制台
[INFO] [command_console]:
[INFO] [command_console]: 📥 收到机器人状态: [#1] CPU=42.0°C | MEM=7898/32676MB(24%) | DISK=45G/116G(41%)
[INFO] [command_console]: 📥 收到机器人状态: [#2] CPU=42.1°C | MEM=7900/32676MB(24%) | DISK=45G/116G(41%)
```

**Step 3: Send control commands from the Host PC**

Enter a number in the console to send the corresponding command:

[🎬 Cross-machine communication demo video (click to play)](https://files.seeedstudio.com/1.4.5.3_cross_machine_demo.mp4)

Enter `1` (turn the LED on):

```
1
```

**Actual output (Host PC):**

```
[INFO] [command_console]: 📤 发送指令: LED:ON
```

**Actual output (received by the J501 at the same time):**

```
[INFO] [robot_status]: 💡 收到指令: 点亮 LED（模拟）
```

Enter `3` (send a greeting):

```
3
```

**Actual output (J501):**

```
[INFO] [robot_status]: 💡 收到问候: "你好J501，我是Host PC" -> 回复: 你好！我是 J501 机器人
```

Enter `4` (request an immediate status report):

```
4
```

**Actual output (immediate report from the J501):**

```
[INFO] [robot_status]: 📋 收到查询指令，立即上报状态
[INFO] [robot_status]: 📤 上报: [#15] CPU=42.3°C | MEM=7905/32676MB(24%) | DISK=45G/116G(41%)
```

**Actual output (received by the Host PC):**

```
[INFO] [command_console]: 📥 收到机器人状态: [#15] CPU=42.3°C | MEM=7905/32676MB(24%) | DISK=45G/116G(41%)
```

Enter `0` to exit the console:

```
0
```

> **Confirmation**: The Host PC successfully subscribes to the status reports from the J501, and the J501 successfully receives the control commands from the Host PC. Bidirectional cross-machine communication works perfectly!

#### (4) Inspecting Remote Topics

```bash
# [Host PC terminal] List the topics published on the J501
ros2 topic list
```

**Actual output:**

```
/parameter_events
/robot/cmd
/robot/status
/rosout
```

```bash
# [Host PC terminal] Inspect the /robot/status topic
ros2 topic info /robot/status
ros2 topic hz /robot/status
ros2 topic echo /robot/status --once
```

**Actual output:**

```
Type: std_msgs/msg/String
Publisher count: 1
Subscriber count: 1
---
average rate: 0.5 Hz
---
data: '[#3] CPU=42.1°C | MEM=7900/32676MB(24%) | DISK=45G/116G(41%)'
---
```

```bash
# [Host PC terminal] List the remote nodes
ros2 node list
```

**Actual output:**

```
/command_console
/robot_status
```

> **Confirmation**: The `/robot_status` node runs on the J501 and the `/command_console` node runs on the Host PC; DDS discovered the nodes across machines automatically.

#### (5) Configuring CycloneDDS (Optional Optimization)

If the default DDS discovery is unstable, you can switch to CycloneDDS:

```bash
# [J501 local terminal] Install CycloneDDS
sudo apt install -y ros-humble-rmw-cyclonedds-cpp

# Configure the default DDS implementation
echo 'export RMW_IMPLEMENTATION=rmw_cyclonedds_cpp' >> ~/.bashrc
source ~/.bashrc
```

```bash
# [Host PC terminal] Configure CycloneDDS the same way
sudo apt install -y ros-humble-rmw-cyclonedds-cpp
echo 'export RMW_IMPLEMENTATION=rmw_cyclonedds_cpp' >> ~/.bashrc
source ~/.bashrc
```

```bash
# [Host PC -> J501 SSH] Verify the DDS implementation
ssh J501 'echo $RMW_IMPLEMENTATION'
```

**Actual output:**

```
rmw_cyclonedds_cpp
```

#### (6) ROS\_DOMAIN\_ID Isolation (Optional)

If there are multiple groups of ROS 2 devices on the network, you can isolate the communication domains with `ROS_DOMAIN_ID`:

```bash
# [J501 local terminal] Set DOMAIN_ID to 42
echo 'export ROS_DOMAIN_ID=42' >> ~/.bashrc
source ~/.bashrc
```

```bash
# [Host PC terminal] Set the same DOMAIN_ID
echo 'export ROS_DOMAIN_ID=42' >> ~/.bashrc
source ~/.bashrc
```

> **Note**: Only nodes with the same `ROS_DOMAIN_ID` can discover each other. Valid values are 0-101 (it is recommended to avoid 0 and 1 to prevent conflicts with the default group).

### 1.4.6 Introduction to Isaac ROS and image\_pipeline Verification

Isaac ROS is a collection of hardware-accelerated ROS 2 packages provided by NVIDIA. It leverages the Jetson's GPU, DLA, PVA, and other accelerators to achieve high-performance perception. This section first introduces the Isaac ROS ecosystem, then runs the `isaac_ros_image_pipeline` example to verify GPU-accelerated image processing.

> **References**:
>
> - Isaac ROS official documentation: <https://nvidia-isaac-ros.github.io/v/release-3.2/>
> - Image Pipeline repository: <https://github.com/NVIDIA-ISAAC-ROS/isaac_ros_image_pipeline>

#### (1) Isaac ROS Core Packages

| Package                       | Function                                                        | Accelerator |
| ----------------------------- | --------------------------------------------------------------- | ----------- |
| `isaac_ros_image_pipeline`    | Image format conversion, undistortion, cropping, and scaling    | GPU + PVA   |
| `isaac_ros_apriltag`          | AprilTag visual tag detection                                   | GPU + DLA   |
| `isaac_ros_stereo_image_proc` | Stereo vision depth estimation                                  | GPU         |
| `isaac_ros_nitros`            | NITROS accelerated inference pipeline                           | GPU + DLA   |
| `isaac_ros_dnn_inference`     | Deep learning inference (TensorRT)                              | GPU + DLA   |

> **Isaac ROS vs. the traditional image\_pipeline**: The traditional `image_pipeline` processes each pixel about 3 times on the CPU (undistortion, scaling, color conversion), whereas `isaac_ros_image_pipeline` performs the same operations using the GPU and dedicated hardware engines, with near-zero CPU usage.

#### (2) System Requirements and Prerequisites

| Item                     | Requirement                                      | Status on the J501   |
| ------------------------ | ------------------------------------------------ | -------------------- |
| Hardware                 | Jetson Orin (Nano 4GB not recommended)           | AGX Orin 32GB        |
| JetPack                  | 6.1 or 6.2                                       | 6.2.1 (L4T 36.4.4)   |
| ROS 2                    | Humble                                           | Installed            |
| Docker                   | 27.2.0+ (required by Isaac ROS 3.2)              | 29.6.2               |
| NVIDIA Container Toolkit | GPU passthrough configured                       | Configured           |
| VPI                      | 3.x (required for PVA acceleration)              | 3.2.4                |
| Disk space               | ≥30GB (container images + data)                  | 65GB available       |

```bash
# [Host PC -> J501 SSH] Verify the Isaac ROS prerequisites
ssh J501 'echo "=== JetPack ==="; cat /etc/nv_tegra_release 2>/dev/null | head -1; echo "=== Docker ==="; docker --version; echo "=== ROS2 ==="; source /opt/ros/humble/setup.bash && echo "ROS_DISTRO=$ROS_DISTRO"; echo "=== VPI ==="; dpkg -l | grep -i "nvidia-vpi " | awk "{print \$2, \$3}"; echo "=== 磁盘 ==="; df -h / | tail -1'
```

**Actual output:**

```
=== JetPack ===
# R36 (release), REVISION: 4.4, GCID: 34842085, BOARD: t186ref, EABI: aarch64
=== Docker ===
Docker version 29.6.2, build dfc4efb
=== ROS2 ===
ROS_DISTRO=humble
=== VPI ===
nvidia-vpi 6.2.1+b38
=== 磁盘 ===
/dev/nvme0n1p1  116G   46G   65G  42% /
```

> **Key confirmation**: JetPack R36.4.4 corresponds to Isaac ROS 3.2, Docker 29.6.2 satisfies the ≥27.2.0 requirement, VPI 3.2.4 supports PVA acceleration, and 65GB of disk space is available.

#### (3) Setting Up the Isaac ROS Development Environment (Docker Approach)

NVIDIA strongly recommends using the Isaac ROS Dev Docker image for development; this image automatically configures the correct dependency versions and the Isaac Apt repository.

**Step 1: Clone the isaac\_ros\_common repository**

```bash
# [J501 local terminal] Clone the Isaac ROS common utilities repository
mkdir -p ~/isaac_ros_ws/src
cd ~/isaac_ros_ws
git clone https://github.com/NVIDIA-ISAAC-ROS/isaac_ros_common.git ~/isaac_ros_ws/src/isaac_ros_common
```

**Step 2: Start the Isaac ROS Dev container**

```bash
# [J501 local terminal] Start the development container (the image is pulled automatically on first run, about 15-20 minutes)
cd ~/isaac_ros_ws/src/isaac_ros_common/scripts
./run_dev.sh
```

> **Note**: `run_dev.sh` automatically pulls the Isaac ROS Dev image matching the JetPack version, mounts the workspace and GPU devices, and then enters an interactive terminal inside the container. The first image pull is about 5-8GB, so make sure the network connection is good.

**Step 3: Verify the environment inside the container**

```bash
# [Inside the Isaac ROS Dev container] Verify the environment inside the container
ros2 topic list
echo "ROS_DISTRO=$ROS_DISTRO"
nvidia-smi
```

> **Expected output**: Inside the container, `ROS_DISTRO=humble`, and `nvidia-smi` shows the Orin GPU, which indicates that GPU passthrough is working.

#### (4) Running the isaac\_ros\_image\_pipeline Example

**Step 1: Clone the image\_pipeline repository**

```bash
# [Inside the Isaac ROS Dev container] Clone the image_pipeline repository
cd /workspaces/isaac_ros_ws/src
git clone https://github.com/NVIDIA-ISAAC-ROS/isaac_ros_image_pipeline.git
```

**Step 2: Install dependencies and build**

```bash
# [Inside the Isaac ROS Dev container] Install ROS 2 dependencies
cd /workspaces/isaac_ros_ws
sudo apt-get update
rosdep install --from-paths src --ignore-src -y --rosdistro humble

# Build the image_pipeline packages
colcon build --packages-select isaac_ros_image_proc isaac_ros_image_pipeline
```

**Step 3: Run the image processing example**

```bash
# [Inside the Isaac ROS Dev container - terminal 1] Start the isaac_ros_image_proc undistortion node
source /workspaces/isaac_ros_ws/install/setup.bash
ros2 launch isaac_ros_image_proc isaac_ros_image_proc.launch.py
```

```bash
# [Inside the Isaac ROS Dev container - terminal 2] List the topics
source /workspaces/isaac_ros_ws/install/setup.bash
ros2 topic list
ros2 topic info /image_rect_color
```

> **Expected output**:
>
> ```
> /image_rect_color
> /image_raw
> /parameter_events
> /rosout
> ```
>
> `/image_raw` is the input image, and `/image_rect_color` is the output image after GPU undistortion.

**Step 4: Compare CPU vs GPU performance (optional)**

```bash
# [Inside the Isaac ROS Dev container] Watch GPU utilization
watch -n 1 nvidia-smi
```

> **Verification points**: While running `isaac_ros_image_proc`, GPU utilization should rise noticeably while CPU utilization stays low. Compared with the traditional `image_proc` (the CPU version), CPU usage drops significantly.

#### (5) Reserved Interfaces

> **Follow-up course connections**:
>
> - **M2.2** will use `isaac_ros_image_pipeline` to implement GPU-accelerated undistortion and format conversion of camera images
> - **M2.4** will use `isaac_ros_apriltag` to implement visual tag localization
> - **M4.2** will use `isaac_ros_dnn_inference` to implement object detection inference

> **Note**: The complete installation and in-depth practice of Isaac ROS will be covered in the M2 perception layer course. This section only verifies the availability of the base environment, ensuring that Docker + GPU passthrough + ROS 2 Humble work together.

### 1.4.7 FAQ

| Problem                          | Cause                                  | Solution                                                  |
| -------------------------------- | -------------------------------------- | --------------------------------------------------------- |
| `ros2: command not found`        | Environment variables not sourced      | `source /opt/ros/humble/setup.bash`                       |
| `colcon: command not found`      | colcon not installed                   | `sudo apt install python3-colcon-common-extensions`       |
| Cannot see the other machine's topics | Not on the same subnet, or blocked by a firewall | Confirm `ping` connectivity; open ports with `sudo ufw allow` |
| `RMW_IMPLEMENTATION` not set     | FastDDS is used by default             | `export RMW_IMPLEMENTATION=rmw_cyclonedds_cpp`            |
| `rosdep` command not found       | rosdep not installed                   | `sudo apt install python3-rosdep`                         |
| Changes to a Python node do not take effect | `--symlink-install` was not used | `colcon build --symlink-install --packages-select <pkg>`  |
| `entry_points` error             | console\_scripts not configured in setup.py | Check the node names in `entry_points`               |
| talker runs but the listener receives nothing | DDS discovery failed       | Try switching to CycloneDDS or configuring `ROS_DOMAIN_ID` |
| `Could not find package`         | apt sources not updated                | `sudo apt update && sudo apt install ros-humble-<pkg>`    |
| Chinese topic names are garbled  | locale not set to UTF-8                | `sudo locale-gen en_US en_US.UTF-8 && sudo update-locale` |

<br />

***

> **M1 Module Summary**
>
> Through the four sections 1.1-1.4, you have completed:
>
> - **1.1** Hardware Platform Analysis — learned the interfaces, specifications, and hardware verification methods of the J501
> - **1.2** System Flashing and Configuration — flashing JetPack 6.2, power mode switching, installing missing software
> - **1.3** Containerized Development Environment — Docker + NVIDIA Runtime + VS Code Remote + Jupyter Lab
> - **1.4** ROS 2 Humble — installation, the colcon workspace, writing nodes, cross-machine communication
>
> **Next step**: M2 Perception Layer — starting with "seeing", connecting GMSL2 multi-cameras and depth cameras, and leveraging the GPU to accelerate image processing.
