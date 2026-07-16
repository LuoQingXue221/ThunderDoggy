# ThunderDoggy

ThunderDoggy 是一套运行在 ESP32 MicroPython 上的月球小车控制程序，集成了 4 个 CAN 驱动电机、6 个底盘转向舵机、4 自由度机械臂、相机舵机、PS2 手柄遥控及相机 UART 通信。

项目按照“硬件协议层 → 设备功能层 → 组合控制层 → 应用入口”组织，便于学习、调试和扩展。

## 运行环境

本项目不是普通桌面 Python 程序，必须运行在支持以下模块的 ESP32 MicroPython 固件中：

- machine.Pin
- machine.UART
- esp32.CAN
- _thread
- ustruct

电脑端建议使用 mpremote 上传和调试：

    python -m pip install mpremote
    python -m mpremote connect list

machine、esp32 等模块由开发板固件提供，不能通过 pip 安装。编辑器在桌面 Python 环境中提示“无法解析导入”，不等同于开发板运行失败。

> esp32.CAN 可能属于特定 ESP32 MicroPython 固件扩展。烧录固件前请确认固件提供该接口。

## 项目结构

    ThunderDoggy/
    ├── README.md
    └── CarControlCode/
        ├── boot.py              # MicroPython 启动阶段，保持硬件空闲
        ├── main.py              # 硬件初始化、对象组装和程序入口
        ├── robot_config.py      # 引脚、设备 ID、速度和角度限位
        │
        ├── motor_lib.py         # 电机 CAN 底层协议
        ├── servo_lib.py         # Fashion Star 舵机 UART 底层协议
        ├── ps2_lib.py           # PS2 手柄 GPIO 模拟 SPI 协议
        │
        ├── servo_control.py     # 舵机功能映射与角度限制
        ├── chassis_control.py   # 底盘组合运动控制
        ├── arm_control.py       # 机械臂和相机组合控制
        └── ps2_control.py       # PS2 按键、摇杆业务映射

## 软件架构

    main.py
    ├── 相机 UART 接收线程
    └── ps2_control.py
        ├── chassis_control.LunarRover
        │   ├── motor_lib.MotorBus ───────> esp32.CAN
        │   └── servo_control.ServoControl
        │       └── servo_lib.ServoBus ───> machine.UART
        └── arm_control.RobotArm
            └── servo_control.ServoControl

    ps2_lib.PS2Controller ────────────────> machine.Pin
    robot_config.py ──────────────────────> 所有控制模块

### 模块职责

| 模块 | 职责 |
| --- | --- |
| boot.py | ESP32 上电首先执行；当前不初始化外设，避免提前占用 UART/CAN |
| main.py | 初始化 UART、CAN、舵机、电机、机械臂和 PS2，并选择运行模式 |
| robot_config.py | 管理引脚、设备 ID、速度上限、角度限位和初始姿态 |
| motor_lib.py | 构造 CAN 扩展帧，配置电机速度模式、PI、加速度、使能和速度 |
| servo_lib.py | 实现 Fashion Star 舵机同步控制、角度读取、锁力及重置圈数 |
| servo_control.py | 将舵机 ID 映射为底盘、相机、机械臂和预留舵机功能 |
| chassis_control.py | 将 4 个驱动电机与 6 个转向舵机组合为整车运动接口 |
| arm_control.py | 管理机械臂内部角度状态，提供复位、点动和角度同步接口 |
| ps2_lib.py | 读取 PS2 手柄，保存最近一次有效按键和摇杆快照 |
| ps2_control.py | 将手柄输入映射为底盘、机械臂和相机动作 |

## 当前硬件配置

硬件参数集中定义在 CarControlCode/robot_config.py。

| 外设 | 当前配置 |
| --- | --- |
| 舵机 UART | UART2，115200 baud，TX=16，RX=17 |
| 相机 UART | UART1，115200 baud，TX=5，RX=6 |
| 电机 CAN | CAN0，1 Mbps，TX=8，RX=18 |
| PS2 手柄 | DI=9，DO=10，CS=11，CLK=12 |
| 运行模式 | RUN_MODE = ps2 |

### 设备 ID

| 设备 | ID |
| --- | --- |
| 左前、右前、左后、右后驱动电机 | 1、2、3、4（CAN 总线） |
| 左前、右前、左后、右后转向舵机 | 1、2、3、4（UART 总线） |
| 左中、右中转向舵机 | 5、6 |
| 机械臂 Pitch1 | 7 |
| 相机舵机 | 8 |
| 机械臂 Roll | 9 |
| 机械臂 Pitch2、Pitch3 | 10、11 |
| 预留舵机 | 14，默认未启用 |

电机和舵机使用不同总线，因此相同 ID 不会互相冲突。

## 上传到开发板

先确认开发板串口：

    python -m mpremote connect list

假设开发板端口为 COM6，在项目根目录依次执行：

    python -m mpremote connect COM6 fs cp CarControlCode/boot.py :boot.py
    python -m mpremote connect COM6 fs cp CarControlCode/main.py :main.py
    python -m mpremote connect COM6 fs cp CarControlCode/robot_config.py :robot_config.py
    python -m mpremote connect COM6 fs cp CarControlCode/motor_lib.py :motor_lib.py
    python -m mpremote connect COM6 fs cp CarControlCode/servo_lib.py :servo_lib.py
    python -m mpremote connect COM6 fs cp CarControlCode/servo_control.py :servo_control.py
    python -m mpremote connect COM6 fs cp CarControlCode/chassis_control.py :chassis_control.py
    python -m mpremote connect COM6 fs cp CarControlCode/arm_control.py :arm_control.py
    python -m mpremote connect COM6 fs cp CarControlCode/ps2_lib.py :ps2_lib.py
    python -m mpremote connect COM6 fs cp CarControlCode/ps2_control.py :ps2_control.py
    python -m mpremote connect COM6 reset

请将 COM6 替换为实际开发板端口。上传前应架空车轮或断开电机动力，防止程序启动后车辆意外运动。

## 启动流程

1. ESP32 执行 boot.py。
2. 加载 main.py 并等待 3 秒。
3. 初始化舵机 UART、相机 UART 和 CAN。
4. 重置舵机多圈计数并锁住舵机。
5. 创建电机、舵机、机械臂和底盘控制对象。
6. 启动相机 UART 后台接收线程。
7. 根据 RUN_MODE 进入 PS2 遥控或 idle 示例模式。

如果 CAN 初始化失败，程序会调用 machine.reset()。此时应检查 CAN 引脚、固件接口和外设占用情况。

## 运行模式

在 robot_config.py 中修改：

    RUN_MODE = ps2  # idle | ps2

- ps2：启动 PS2 手柄遥控，是当前默认模式。
- idle：执行 main.py 中的学生控制示例，然后进入相机数据处理循环。

修改或启用 idle 示例前，建议先架空车轮，并且每次只启用一段示例代码。

## PS2 操作说明

### 底盘模式

| 操作 | 功能 |
| --- | --- |
| 右摇杆上下 | 前进、后退 |
| 左摇杆左右 | 普通转向 |
| R2 + 右摇杆左右 | 原地旋转 |
| R1 | 停车 |
| × | 失能驱动电机 |
| △ | 重新配置并使能驱动电机 |
| SELECT | 退出 PS2 控制 |

### 机械臂模式

按住 L2 进入机械臂控制模式：

| 操作 | 功能 |
| --- | --- |
| 右摇杆左右 | Roll |
| 右摇杆上下 | Pitch1 |
| 左摇杆上下 | Pitch2 |
| 方向键上下 | Pitch3 |
| 方向键左右 | 相机舵机 |
| ○ | 机械臂回初始姿态，相机回零 |

进入机械臂模式时，程序会先停止底盘，并读取真实舵机角度以同步内部控制状态。

## 主要控制接口

### 底盘

    rover.prepare()
    rover.drive(speed_rad_s=2.0, steer_angle_deg=20.0)
    rover.pivot_turn(speed_rad_s=1.0)
    rover.stop()
    rover.disable()

### 机械臂与相机

    rover.arm.apply_initial_pose()
    rover.arm.sync_from_servos()
    rover.arm.jog_joints(roll_delta_deg=2.0)

    rover.arm.sync_camera_from_servo()
    rover.arm.jog_camera(delta_deg=8.0)

### 舵机功能层

    rover.servo_control.set_steering_angles(
        20.0, 0.0, -20.0,
        -20.0, 0.0, 20.0,
    )

    rover.servo_control.set_camera_angle(-30.0)
    rover.servo_control.set_arm_joint_angles(0.0, 50.0, -140.0, 0.0)

所有舵机入口都会根据 robot_config.py 中的限位裁剪目标角度。

## 相机串口数据

相机 UART 接收线程把数据保存到共享字典中。当前业务逻辑期望每帧为 6 个以空格分隔的字段：

    颜色1 颜色2 颜色3 数量1 数量2 数量3

示例：

    red green blue 2 1 3

颜色出现顺序代表目标在视野中的位置，数量字段必须能转换为整数。

## 安全提示

- 第一次调试或修改底盘代码时，应架空车轮。
- 修改电机方向、舵机方向或设备 ID 前，应确认实际接线和机械结构。
- 电机速度环 PI 参数需要在电机失能状态下写入。
- PS2 数据失效或超时时，控制循环会调用 rover.stop()。
- 程序启动时会重置舵机圈数并锁住舵机，机械臂周围应保持无障碍。

## 当前注意事项

- 项目暂未提供自动化测试。
- 桌面 CPython 无法直接运行 main.py，因为它不提供 ESP32 硬件模块。
- 编辑器需要额外配置 MicroPython 类型存根，才能消除 machine、esp32 等导入警告。
- 相机数据解析分别存在于 main.py 和 ps2_control.py，后续可考虑提取为独立模块。

## 作者

王笑，2026-05-28。
