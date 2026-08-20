#!/usr/bin/env python3
"""MAX96712 链路诊断脚本

通过 I2C 读取 MAX96712 的关键寄存器，诊断 GMSL2 链路状态。
"""

import subprocess
import sys

# MAX96712 关键寄存器定义
REGISTERS = {
    0x0001: ("REV_ID", "芯片版本 ID"),
    0x0003: ("LINK_LOCK", "链路锁定状态"),
    0x0006: ("LINK_EN", "链路使能"),
    0x000D: ("CSI_OUT_EN", "CSI 输出使能"),
    0x0010: ("CSI1_LANE", "CSI-1 通道数"),
    0x0011: ("CSI2_LANE", "CSI-2 通道数"),
    0x0030: ("PIPE_EN", "数据管道使能"),
    0x0100: ("ERR_CNT_L0", "Link 0 错误计数"),
    0x0102: ("ERR_CNT_L1", "Link 1 错误计数"),
    0x0104: ("ERR_CNT_L2", "Link 2 错误计数"),
    0x0106: ("ERR_CNT_L3", "Link 3 错误计数"),
}

# I2C 总线号和设备地址
# 实机实测（J501 + Seeed GMSL 扩展板）：MAX96712 解串器 7-bit 地址为 0x29，
# 两片分别位于 i2c-3（3-0029）与 i2c-7（7-0029）
I2C_BUS = 7
I2C_ADDR = 0x29

def read_register(bus, addr, reg):
    """通过 i2cget 读取 16-bit 寄存器值

    Args:
        bus: I2C 总线号
        addr: I2C 设备地址（7-bit）
        reg: 寄存器地址（16-bit）

    Returns:
        int: 寄存器值，失败返回 None
    """
    cmd = [
        'sudo', 'i2cget', '-y', str(bus),
        hex(addr), hex(reg), 'w'
    ]
    try:
        result = subprocess.run(
            cmd, capture_output=True, text=True, timeout=5
        )
        if result.returncode == 0:
            return int(result.stdout.strip(), 16)
    except (subprocess.TimeoutExpired, ValueError):
        pass
    return None

def diagnose_link():
    """执行完整链路诊断"""
    print("=" * 60)
    print("MAX96712 GMSL2 链路诊断报告")
    print("=" * 60)
    print(f"I2C 总线: /dev/i2c-{I2C_BUS}")
    print(f"设备地址: 0x{I2C_ADDR:02x}")
    print("-" * 60)

    for reg_addr, (name, desc) in REGISTERS.items():
        value = read_register(I2C_BUS, I2C_ADDR, reg_addr)
        if value is not None:
            print(f"[0x{reg_addr:04x}] {name:15s} = 0x{value:04x}  "
                  f"# {desc}")
        else:
            print(f"[0x{reg_addr:04x}] {name:15s} = 读取失败  "
                  f"# {desc}")

    print("-" * 60)

    # 解读链路锁定状态
    lock_val = read_register(I2C_BUS, I2C_ADDR, 0x0003)
    if lock_val is not None:
        print("\n链路锁定状态解读：")
        for i in range(4):
            status = "锁定" if (lock_val >> i) & 1 else "未锁定"
            print(f"  Link {i}: {status}")

    # 解读错误计数
    print("\n错误计数解读：")
    for i in range(4):
        err_reg = 0x0100 + i * 2
        err_val = read_register(I2C_BUS, I2C_ADDR, err_reg)
        if err_val is not None:
            if err_val == 0:
                print(f"  Link {i}: 无错误")
            elif err_val < 10:
                print(f"  Link {i}: {err_val} 个错误（可接受）")
            else:
                print(f"  Link {i}: {err_val} 个错误（需检查线缆）")

if __name__ == '__main__':
    diagnose_link()
