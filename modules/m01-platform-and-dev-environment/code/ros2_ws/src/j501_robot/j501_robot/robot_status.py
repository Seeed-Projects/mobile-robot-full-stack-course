"""
J501 机器人状态发布者（运行在 J501 上）

功能：
  1. 定时（每 2 秒）发布系统状态到 /robot/status 话题
     - CPU 温度（读取 thermal_zone0）
     - 内存使用率（读取 /proc/meminfo）
     - 磁盘使用率（执行 df -h /）
     - 心跳计数器（用于检测节点是否存活）
  2. 监听 /robot/cmd 话题，接收控制指令
     - LED:ON / LED:OFF  → 模拟点亮/熄灭 LED
     - SAY:<文本>        → 接收问候语并回复
     - STATUS            → 立即触发一次状态上报

通信模型：
  /robot/status  ← 发布（String）  → Host PC 订阅
  /robot/cmd     ← 订阅（String）  ← Host PC 发布
"""

import os
import subprocess
import rclpy
from rclpy.node import Node
from std_msgs.msg import String


class RobotStatusNode(Node):
    """J501 机器人状态节点 — 双向通信的机器人端"""

    def __init__(self):
        # 调用父类构造函数，设置节点名称为 'robot_status'
        super().__init__('robot_status')

        # 创建发布者：在 /robot/status 话题上发布 String 消息
        # 参数 10 是 QoS（Quality of Service）的 depth，表示缓冲队列长度
        self.status_pub = self.create_publisher(String, '/robot/status', 10)

        # 创建订阅者：监听 /robot/cmd 话题上的 String 消息
        # 收到消息时自动调用 self.cmd_callback 回调函数
        self.cmd_sub = self.create_subscription(
            String, '/robot/cmd', self.cmd_callback, 10
        )

        # 心跳计数器：每次发布状态时 +1，用于检测节点是否存活
        self.heartbeat = 0

        # 创建定时器：每 2.0 秒调用一次 self.publish_status
        # 这是 ROS 2 中实现周期性任务的标准方式
        self.timer = self.create_timer(2.0, self.publish_status)

        # 打印启动信息，方便确认节点已正常运行
        self.get_logger().info('🤖 J501 机器人状态节点已启动')
        self.get_logger().info('   发布话题: /robot/status')
        self.get_logger().info('   订阅话题: /robot/cmd')

    def publish_status(self):
        """定时回调函数：采集系统状态并发布到 /robot/status"""
        self.heartbeat += 1

        # 采集三项系统指标
        cpu_temp = self._get_cpu_temp()      # CPU 温度
        mem_usage = self._get_mem_usage()    # 内存使用率
        disk_usage = self._get_disk_usage()  # 磁盘使用率

        # 组装状态字符串，格式示例：[#1] CPU=42.0°C | MEM=7898/32676MB(24%) | DISK=45G/116G(41%)
        msg = String()
        msg.data = (
            f'[#{self.heartbeat}] '
            f'CPU={cpu_temp}°C | '
            f'MEM={mem_usage} | '
            f'DISK={disk_usage}'
        )

        # 发布消息到话题
        self.status_pub.publish(msg)
        self.get_logger().info(f'📤 上报: {msg.data}')

    def cmd_callback(self, msg):
        """
        指令回调函数：收到 /robot/cmd 消息时被调用

        支持的指令格式：
          LED:ON     → 点亮 LED（模拟）
          LED:OFF    → 熄灭 LED（模拟）
          SAY:<文本>  → 接收问候语并回复
          STATUS     → 立即上报一次状态
        """
        cmd = msg.data.strip()  # 去除首尾空白字符

        # 指令解析：根据前缀分发到不同的处理逻辑
        if cmd.startswith('LED:'):
            # LED 指令：格式为 LED:ON 或 LED:OFF
            led_state = cmd.split(':')[1]
            if led_state.upper() == 'ON':
                self.get_logger().info('💡 收到指令: 点亮 LED（模拟）')
                # 实际项目中这里可以调用 GPIO 引脚控制函数
            elif led_state.upper() == 'OFF':
                self.get_logger().info('💡 收到指令: 熄灭 LED（模拟）')
            else:
                self.get_logger().warn(f'⚠️ 未知 LED 指令: {led_state}')

        elif cmd.startswith('SAY:'):
            # 问候指令：格式为 SAY:<任意文本>
            # split(':', 1) 表示只分割第一个冒号，避免文本中包含冒号时出错
            text = cmd.split(':', 1)[1]
            self.get_logger().info(f'💬 收到问候: "{text}" -> 回复: 你好！我是 J501 机器人')

        elif cmd == 'STATUS':
            # 查询指令：立即触发一次状态上报（不等定时器）
            self.get_logger().info('📋 收到查询指令，立即上报状态')
            self.publish_status()

        else:
            # 未知指令：打印警告，方便调试
            self.get_logger().warn(f'⚠️ 未知指令: "{cmd}"')

    def _get_cpu_temp(self):
        """读取 CPU 温度（°C），通过 sysfs thermal_zone0 接口"""
        try:
            # Jetson 的 CPU 温度暴露在 /sys/class/thermal/thermal_zone0/temp
            # 返回值为毫摄氏度，需要除以 1000 转换为 °C
            with open('/sys/class/thermal/thermal_zone0/temp', 'r') as f:
                return f'{int(f.read().strip()) / 1000:.1f}'
        except Exception:
            return 'N/A'

    def _get_mem_usage(self):
        """读取内存使用率，通过 /proc/meminfo 接口"""
        try:
            with open('/proc/meminfo', 'r') as f:
                lines = f.readlines()
            # /proc/meminfo 格式：
            #   line 0: MemTotal:  33489000 kB
            #   line 2: MemAvailable: 25400000 kB
            total = int(lines[0].split()[1])       # 总内存（kB）
            avail = int(lines[2].split()[1])        # 可用内存（kB）
            used = (total - avail) / 1024           # 已用（MB）
            total_mb = total / 1024                 # 总量（MB）
            pct = (total - avail) / total * 100     # 使用率（%）
            return f'{used:.0f}/{total_mb:.0f}MB({pct:.0f}%)'
        except Exception:
            return 'N/A'

    def _get_disk_usage(self):
        """读取根分区磁盘使用率，通过 df 命令"""
        try:
            # 执行 df -h / 获取根分区使用情况
            result = subprocess.run(
                ['df', '-h', '/'],
                capture_output=True, text=True, timeout=2
            )
            # 输出格式：
            #   Filesystem      Size  Used Avail Use% Mounted on
            #   /dev/nvme0n1p1  116G   45G   65G  41% /
            # parts[2]=已用, parts[1]=总量, parts[4]=使用率
            parts = result.stdout.strip().split('\n')[1].split()
            return f'{parts[2]}/{parts[1]}({parts[4]})'
        except Exception:
            return 'N/A'


def main(args=None):
    """节点入口函数"""
    # 初始化 ROS 2 Python 客户端库
    rclpy.init(args=args)

    # 创建节点实例
    node = RobotStatusNode()

    try:
        # spin() 阻塞当前线程，持续处理回调（定时器、订阅者等）
        # 直到收到 Ctrl+C 或节点被关闭
        rclpy.spin(node)
    except KeyboardInterrupt:
        node.get_logger().info('🤖 机器人状态节点已停止')

    # 清理资源：销毁节点并关闭 rclpy
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
