# Python API

## 导入

```python
from dm_h65 import (
    ControlMode,
    DifferentialDrive,
    DMH65Motor,
    PythonCanTransport,
    Register,
    ResponseTimeout,
)
```

`PythonCanTransport` 是基于 `python-can` 的 SocketCAN 适配器。它支持上下文管理器，离开 `with` 时自动关闭 CAN 句柄。

## 单电机 `DMH65Motor`

```python
with PythonCanTransport("can0") as bus:
    motor = DMH65Motor(
        bus,
        motor_id=0x01,
        direction=-1,
        max_speed_rad_s=3.0,
    )
```

主要参数：

- `motor_id`：驱动器 CAN ID，当前底盘左轮为 `0x01`、右轮为 `0x02`。
- `direction`：安装方向修正，只能是 `1` 或 `-1`。
- `max_speed_rad_s`：软件速度上限，超过时抛出 `ValueError`。

常用只读方法：

- `is_online()`：读取 ESC ID 判断是否在线。
- `read_identity()`：读取 ID、控制模式、软件版本、CAN 波特率等。
- `read_telemetry()`：读取母线电压、PCB 温度、电机温度和位置。
- `read_register(register)`：读取一个寄存器。
- `receive_feedback()`：接收并解析普通反馈帧。

控制方法：

- `select_velocity_mode()`：将 RAM 控制模式切换为速度模式 3，不保存到 Flash。
- `enable()` / `disable()`：使能或失能驱动器。
- `set_velocity(rad_s)`：发送速度命令。
- `set_speed_rpm(rpm)`：以 rpm 发送速度命令。
- `stop()`：发送零速度。
- `stop_and_disable()`：停止并失能。
- `clear_error()`：发送清错命令。

所有运动代码都应使用 `try/finally`：

```python
motor.enable()
try:
    motor.set_velocity(0.5)
finally:
    motor.stop_and_disable()
```

## 两轮底盘 `DifferentialDrive`

```python
left = DMH65Motor(bus, 0x01, direction=-1, max_speed_rad_s=8.0)
right = DMH65Motor(bus, 0x02, direction=1, max_speed_rad_s=8.0)
drive = DifferentialDrive(
    left,
    right,
    wheel_radius_m=0.0825,
    track_width_m=0.42,
    max_wheel_speed_rad_s=8.0,
    max_wheel_acceleration_rad_s2=3.0,
)
```

主要方法：

- `read_state()`：读取左右电机遥测。
- `enable()`：检查两台电机在线、临时切换速度模式并成对使能。
- `wheel_speeds_from_twist(linear, angular)`：把底盘速度换算为左右轮速度。
- `set_twist(linear, angular)`：发送一次底盘速度命令。
- `hold_twist(..., duration_s, command_hz)`：周期发送命令，到时减速停止。
- `set_wheel_speeds(left, right)`：直接设置左右轮角速度。
- `stop()` / `disable()`：停止或成对失能。

长时间控制循环应以 20–100 Hz 持续调用 `set_twist()`，并由上层实现命令超时看门狗。WebUI 已内置 750 ms 看门狗。

## 异常

- `ResponseTimeout`：电机在指定时间内未回复寄存器请求。
- `ProtocolError`：收到的帧不符合协议。
- `CommunicationError`：CAN 发送失败。
- `ValueError`：ID、方向、速度或几何参数无效。

捕获超时示例：

```python
try:
    telemetry = motor.read_telemetry(timeout_s=0.2)
except ResponseTimeout as exc:
    print(f"电机无响应: {exc}")
```

## 线程与总线所有权

SDK 是同步 API。一个 `PythonCanTransport` 实例应由一个控制流程串行使用，因为寄存器请求与响应必须配对。不要让 WebUI、状态脚本和另一个控制程序同时消费同一个 CAN 接口的响应帧。

