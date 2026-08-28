"""
Host PC 指令控制台（运行在 Host PC 上）

功能：
  1. 订阅 /robot/status 话题，实时显示 J501 上报的系统状态
  2. 提供交互式命令行，用户输入数字即可发送控制指令到 J501
  3. 使用多线程执行器，同时处理订阅回调和用户输入

数字指令对照表：
  1 → LED:ON      点亮机器人 LED
  2 → LED:OFF     熄灭机器人 LED
  3 → SAY:你好     发送问候语
  4 → STATUS      请求立即上报状态
  0 → quit        退出控制台

通信模型：
  /robot/status  ← 订阅（String）  ← J501 发布
  /robot/cmd     ← 发布（String）  → J501 订阅
"""

import sys
import threading
import rclpy
from rclpy.node import Node
from rclpy.executors import MultiThreadedExecutor
from std_msgs.msg import String

# 数字指令映射表：用户输入数字 → 对应的指令字符串
# 这样用户只需输入数字，无需记忆完整指令格式
CMD_MAP = {
    '1': 'LED:ON',       # 点亮 LED
    '2': 'LED:OFF',      # 熄灭 LED
    '3': 'SAY:你好J501，我是Host PC',  # 发送问候语
    '4': 'STATUS',       # 请求立即上报状态
    '0': 'quit',         # 退出控制台
}


class CommandConsoleNode(Node):
    """Host PC 指令控制台节点 — 双向通信的 Host PC 端"""

    def __init__(self):
        # 调用父类构造函数，设置节点名称为 'command_console'
        super().__init__('command_console')

        # 创建订阅者：监听 J501 发布的 /robot/status 话题
        # 收到消息时自动调用 self.status_callback 回调函数
        self.status_sub = self.create_subscription(
            String, '/robot/status', self.status_callback, 10
        )

        # 创建发布者：向 J501 发送控制指令到 /robot/cmd 话题
        self.cmd_pub = self.create_publisher(String, '/robot/cmd', 10)

        # 打印启动信息和指令菜单
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
        """状态回调函数：收到 J501 状态消息时被调用"""
        self.get_logger().info(f'📥 收到机器人状态: {msg.data}')

    def send_cmd(self, cmd_text):
        """发送控制指令到 /robot/cmd 话题"""
        msg = String()
        msg.data = cmd_text
        self.cmd_pub.publish(msg)
        self.get_logger().info(f'📤 发送指令: {cmd_text}')


def main(args=None):
    """节点入口函数"""
    # 初始化 ROS 2 Python 客户端库
    rclpy.init(args=args)
    node = CommandConsoleNode()

    # 使用多线程执行器（MultiThreadedExecutor）
    # 原因：默认的单线程执行器在 input() 阻塞时无法处理订阅回调
    # 多线程执行器可以在一个线程处理回调，另一个线程处理用户输入
    executor = MultiThreadedExecutor()
    executor.add_node(node)

    def input_loop():
        """
        用户输入循环（在独立线程中运行）

        读取用户输入的数字，通过 CMD_MAP 映射为完整指令后发送
        输入 0 或 Ctrl+C 退出
        """
        while rclpy.ok():
            try:
                # input() 会阻塞当前线程，等待用户输入
                user_input = input().strip()

                # 查找数字对应的指令
                cmd = CMD_MAP.get(user_input)
                if cmd is None:
                    # 用户输入了不在映射表中的内容，直接作为原始指令发送
                    cmd = user_input

                if cmd.lower() == 'quit':
                    break

                # 发送指令到 J501
                node.send_cmd(cmd)

            except EOFError:
                # 非交互环境（如管道输入）会触发 EOFError
                break

    # 启动输入线程（daemon=True 表示主程序退出时自动结束）
    thread = threading.Thread(target=input_loop, daemon=True)
    thread.start()

    try:
        # spin() 阻塞主线程，持续处理订阅回调
        executor.spin()
    except KeyboardInterrupt:
        pass

    node.get_logger().info('🖥️  控制台已退出')
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
