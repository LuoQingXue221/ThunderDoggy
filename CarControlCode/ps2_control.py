"""
PS2 遥控业务控制逻辑。

本文件负责把 PS2 按键和摇杆映射到底盘、相机和机械臂动作。
ps2_lib.py 只负责手柄底层读取和安全接收。

键位布局：
  左摇杆     → 底盘二维平移（前后/左右）
  右摇杆 Y   → 底盘原地旋转
  右摇杆 X   → 相机旋转
  十字键左右 → Roll 舵机
  十字键上下 → Pitch1（45 kg 舵机）
  L1          → Pitch2 上
  L2          → Pitch2 下
  R1 / R2    → Pitch3 上下
  □          → 收紧夹爪
  ○          → 放松夹爪
  ×          → 急停（失能电机）
  △          → 使能电机
  START      → 使用已识别二维码任务，启动/取消九宫格自动任务
  L3         → 启动/取消手动巡线后的自动对位夹取
  R3         → 打印九宫格标定姿态
  L3 + R3    → 全部舵机复位
  SELECT     → 退出 PS2 控制

作者 王笑
日期 20260528
更新 20260717 — 重写键位映射
更新 20260719 — 合入视觉自动任务，保留原键位逻辑
"""

import math
import time

from arm_control import ArmKinematicsError
from autonomous_control import AutonomousController, describe_reason
from robot_config import (
    CAMERA_MANUAL_ANGLE_DEG,
    MAX_MOTOR_RPM,
    PIVOT_SPEED_SCALE,
    RESERVE_SERVO_ENABLED,
    clamp,
)

_MAX_MOTOR_RAD_S = MAX_MOTOR_RPM * 2.0 * math.pi / 60.0
_MAX_PIVOT_RAD_S = _MAX_MOTOR_RAD_S * PIVOT_SPEED_SCALE
_ARM_JOG_STEP_DEG = 2

_CAMERA_JOG_STEP_DEG = 8

# 底盘摇杆死区（百分比值 0–100，低于此值视为无操作）
_CHASSIS_DEADZONE_PCT = 15

# 夹爪状态
_gripper_angle_deg = 0.0
_GRIPPER_STEP_DEG = 5.0
_GRIPPER_MIN_DEG = -90.0
_GRIPPER_MAX_DEG = 90.0
_GRIPPER_SERVO_ID = 14
_gripper_warned = False

_last_arm_error_key = None
_last_arm_error_ms = 0


# ==============================================================================
# 核心摇杆数据处理函数
# ==============================================================================
def map_joystick(raw_val, center=128, deadzone=12):
    """
    【摇杆数据映射核心】
    将摇杆的原始 ADC 数据 (通常为 0-255) 转换为 -100 到 100 的百分比数值。

    参数说明:
    - raw_val: 手柄底层读取到的原始摇杆数据 (0~255)
    - center: 摇杆的物理中位值 (默认128)
    - deadzone: 死区范围，摇杆在这个范围内的微小偏移会被忽略，防止摇杆回中不良导致漂移
    """
    offset = int(raw_val) - center

    if abs(offset) <= deadzone:
        return 0

    # 3. 确定方向：正向推为 1，反向拉为 -1
    sign = 1 if offset > 0 else -1
    active_range = 127.0 - deadzone
    mapped = int(((abs(offset) - deadzone) / active_range) * 100.0) * sign

    return clamp(mapped, -100, 100)


def small_motion(value, threshold):
    """过滤微小动作，如果计算出的运动增量小于设定阈值，则直接归零。"""
    return 0.0 if abs(value) < threshold else value


def button_pressed(data, btn):
    """通过位与运算判断底层传来的复合数据中，某个特定按键是否被按下。"""
    return (data & btn) == btn


def ticks_ms():
    if hasattr(time, "ticks_ms"):
        return time.ticks_ms()
    return int(time.time() * 1000)


def ticks_diff(a, b):
    if hasattr(time, "ticks_diff"):
        return time.ticks_diff(a, b)
    return a - b


def print_arm_error(err):
    """打印机械臂错误，并进行防刷屏处理（1秒内相同的错误只报一次）。"""
    global _last_arm_error_key, _last_arm_error_ms
    now_ms = ticks_ms()
    key = (err.reason, err.message)
    if key == _last_arm_error_key and ticks_diff(now_ms, _last_arm_error_ms) < 1000:
        return
    _last_arm_error_key = key
    _last_arm_error_ms = now_ms
    print("机械臂目标无效：%s，%s" % (err.reason, err.message))


# ==============================================================================
# 夹爪控制
# ==============================================================================
def _set_gripper_angle(rover, delta):
    """按步进增量调整夹爪角度。"""
    global _gripper_angle_deg, _gripper_warned

    _gripper_angle_deg = clamp(
        _gripper_angle_deg + delta,
        _GRIPPER_MIN_DEG,
        _GRIPPER_MAX_DEG,
    )

    if RESERVE_SERVO_ENABLED:
        rover.servo_control.set_reserve_servo_angle(
            _GRIPPER_SERVO_ID, _gripper_angle_deg,
        )
    elif not _gripper_warned:
        print("夹爪未启用，请在 robot_config.py 中设置 RESERVE_SERVO_ENABLED = True")
        _gripper_warned = True


def _print_calibration_snapshot(rover, data, serial=None):
    """打印机械臂姿态和当前白纸高度，供九宫格覆盖范围标定。"""
    if rover.arm is not None:
        try:
            pose = rover.arm.sync_from_servos()
            print(
                "GRID_POSE=(%.1f, %.1f, %.1f, %.1f)"
                % (
                    pose["roll_deg"],
                    pose["pitch1_deg"],
                    pose["pitch2_deg"],
                    pose["pitch3_deg"],
                )
            )
        except ArmKinematicsError as err:
            print_arm_error(err)

    if serial is not None:
        serial.write(b"@CALIBRATE_GRID\n")
        print("GRID_CAL：已通知相机采集30个稳定帧，请保持车辆和九宫格静止。")

    # 九宫格标定结果由相机完成 30 帧采样后经 UART0 返回；不再输出旧白纸标定状态。

# ==============================================================================
# 主循环控制
# ==============================================================================
def ps2_loop(rover, ps2, data, serial):
    global _gripper_angle_deg

    print("PS2 控制：")
    print("  左摇杆=平移  右摇杆Y=底盘旋转  右摇杆X=相机")
    print("  十字键左右=Roll  十字键上下=Pitch1(45kg)")
    print("  L1=Pitch2上  L2=Pitch2下  L1+L2=保持  R1/R2=Pitch3")
    print("  □收紧夹爪  ○放松夹爪  ×急停  △使能")
    print("  START=完整自动  L3=对正/任务夹取  R3=打印标定姿态")
    print("  SELECT=退出  L3+R3=复位")

    auto = AutonomousController(rover, serial)
    start_button_latched = False
    dock_button_latched = False
    pose_print_latched = False

    if rover.arm is not None:
        try:
            rover.arm.sync_from_servos()
            print("机械臂真实角度同步成功。")
        except ArmKinematicsError as err:
            print_arm_error(err)

    while True:
        # 【第一步：触发底层更新】
        ps2.update()
        auto.observe_qr(data)

        # 【第二步：获取手柄快照】
        # fresh: 数据是否有效（布尔值）
        # buttons: 按键状态码
        # lx / ly: 左摇杆 X / Y 轴原始值 (0-255)
        # rx / ry: 右摇杆 X / Y 轴原始值 (0-255)
        fresh, buttons, lx_raw, ly_raw, rx_raw, ry_raw, _ = ps2.snapshot()

        # 数据无效 → 停车并取消自动任务
        if not fresh:
            if auto.active:
                auto.cancel("ps2_lost")
            else:
                rover.stop()
            continue

        # ======================================================================
        # 系统级按键（最高优先级）
        # ======================================================================

        # SELECT：退出控制
        if button_pressed(buttons, ps2.PS2_BTN_SELECT):
            auto.cancel("select_exit")
            rover.stop()
            print("SELECT：退出 PS2 控制。")
            break

        # L3 + R3：全部复位
        if (button_pressed(buttons, ps2.PS2_BTN_L3) and
                button_pressed(buttons, ps2.PS2_BTN_R3)):
            # 组合键复位后即使先松开 R3，也不能误触发单按 L3。
            dock_button_latched = True
            auto.cancel("servo_reset")
            rover.stop()
            if rover.arm is not None:
                try:
                    rover.arm.apply_initial_pose()
                except ArmKinematicsError as err:
                    print_arm_error(err)
                rover.servo_control.set_camera_angle(CAMERA_MANUAL_ANGLE_DEG)
                rover.arm.camera_angle_deg = CAMERA_MANUAL_ANGLE_DEG
            rover.center_chassis_servos()
            _gripper_angle_deg = 0.0
            _set_gripper_angle(rover, 0.0)
            print("L3+R3：全部舵机已复位。")
            time.sleep_ms(500)
            continue

        # ×：急停（取消自动任务并失能电机）
        if button_pressed(buttons, ps2.PS2_BTN_CROSS):
            auto.cancel("motor_disable")
            rover.disable()
            time.sleep_ms(200)
            continue

        # △：使能电机
        if button_pressed(buttons, ps2.PS2_BTN_TRIANGLE):
            if auto.active:
                print("自动任务未退出，不允许重新使能底盘。")
            else:
                rover.enable_motors()
                print("底盘电机已使能。")
            time.sleep_ms(200)
            continue

        # START：使用已缓存二维码任务，启动/取消九宫格自动任务。
        start_pressed = button_pressed(buttons, ps2.PS2_BTN_START)
        if not start_pressed:
            start_button_latched = False
        elif not start_button_latched:
            start_button_latched = True
            if auto.active:
                if auto.session == "full":
                    auto.cancel("ps2_start_toggle")
                else:
                    print("局部自动进行中，请按 L3 取消。")
            else:
                ok, reason = auto.start_full(data)
                if not ok and reason == "motors_disabled":
                    print("START完整自动拒绝：底盘电机已失能，请先按 △ 使能。")
                elif not ok and reason == "qr_task_missing":
                    print("START拒绝：请先让相机识别二维码并等待任务确认。")
                elif not ok:
                    print("START完整自动拒绝：", describe_reason(reason))
            time.sleep_ms(120)
            continue

        # L3：手动巡线后启动/取消对位夹取。L3+R3 复位逻辑优先。
        dock_pressed = button_pressed(buttons, ps2.PS2_BTN_L3)
        if not dock_pressed:
            dock_button_latched = False
        elif not dock_button_latched:
            dock_button_latched = True
            if auto.active:
                if auto.session == "dock":
                    auto.cancel("ps2_l3_toggle")
                else:
                    print("完整自动进行中，请按 START 取消。")
            else:
                ok, reason = auto.start_dock(data)
                if not ok and reason == "motors_disabled":
                    print("L3局部自动拒绝：底盘电机已失能，请先按 △ 使能。")
                elif not ok:
                    print("L3局部自动拒绝：", describe_reason(reason))
            time.sleep_ms(120)
            continue

        # 自动任务接管期间不下发人工摇杆和关节命令。
        if auto.active:
            auto.update(data)
            time.sleep_ms(20)
            continue

        # R3：打印当前九宫格标定姿态；L3+R3 的复位逻辑优先。
        pose_print_pressed = button_pressed(buttons, ps2.PS2_BTN_R3)
        if not pose_print_pressed:
            pose_print_latched = False
        elif not pose_print_latched:
            pose_print_latched = True
            _print_calibration_snapshot(rover, data, serial)

        # ======================================================================
        # 夹爪控制
        # ======================================================================
        if button_pressed(buttons, ps2.PS2_BTN_SQUARE):
            _set_gripper_angle(rover, _GRIPPER_STEP_DEG)
        if button_pressed(buttons, ps2.PS2_BTN_CIRCLE):
            _set_gripper_angle(rover, -_GRIPPER_STEP_DEG)

        # ======================================================================
        # 机械臂关节控制（各按键可同时生效）
        # ======================================================================
        roll_delta = 0.0
        pitch1_delta = 0.0
        pitch2_delta = 0.0
        pitch3_delta = 0.0

        # 十字键左右 → Roll
        if button_pressed(buttons, ps2.PS2_BTN_LEFT):
            roll_delta -= _ARM_JOG_STEP_DEG
        if button_pressed(buttons, ps2.PS2_BTN_RIGHT):
            roll_delta += _ARM_JOG_STEP_DEG

        # 十字键上下 → Pitch1（45kg 舵机）
        if button_pressed(buttons, ps2.PS2_BTN_UP):
            pitch1_delta += _ARM_JOG_STEP_DEG
        if button_pressed(buttons, ps2.PS2_BTN_DOWN):
            pitch1_delta -= _ARM_JOG_STEP_DEG

        # L1 / L2 分别上下点动；同时按下时增量相互抵消。
        if button_pressed(buttons, ps2.PS2_BTN_L1):
            pitch2_delta += _ARM_JOG_STEP_DEG
        if button_pressed(buttons, ps2.PS2_BTN_L2):
            pitch2_delta -= _ARM_JOG_STEP_DEG

        # R1 / R2 → Pitch3
        if button_pressed(buttons, ps2.PS2_BTN_R1):
            pitch3_delta += _ARM_JOG_STEP_DEG
        if button_pressed(buttons, ps2.PS2_BTN_R2):
            pitch3_delta -= _ARM_JOG_STEP_DEG

        # 过滤噪声后统一下发一次 jog_joints
        roll_delta = small_motion(roll_delta, 0.5)
        pitch1_delta = small_motion(pitch1_delta, 0.5)
        pitch2_delta = small_motion(pitch2_delta, 0.5)
        pitch3_delta = small_motion(pitch3_delta, 0.5)

        if (roll_delta != 0.0 or pitch1_delta != 0.0 or
                pitch2_delta != 0.0 or pitch3_delta != 0.0):
            if rover.arm is not None:
                try:
                    rover.arm.jog_joints(
                        roll_delta,
                        pitch1_delta,
                        pitch2_delta,
                        pitch3_delta,
                    )
                except ArmKinematicsError as err:
                    print_arm_error(err)

        # ======================================================================
        # 相机控制（右摇杆 X 轴）
        # ======================================================================
        rx_mapped = map_joystick(rx_raw)
        if abs(rx_mapped) > _CHASSIS_DEADZONE_PCT:
            # 右摇杆向左 → 相机逆时针 → 正 delta
            # 右摇杆向右 → 相机顺时针 → 负 delta
            camera_delta = -rx_mapped / 100.0 * _CAMERA_JOG_STEP_DEG
            camera_delta = small_motion(camera_delta, 0.5)
            if camera_delta != 0.0 and rover.arm is not None:
                try:
                    rover.arm.jog_camera(camera_delta)
                except ArmKinematicsError as err:
                    print_arm_error(err)

        # ======================================================================
        # 底盘控制
        # ======================================================================
        # 右摇杆 Y 轴 → 底盘原地旋转（优先于左摇杆平移）
        ry_mapped = -map_joystick(ry_raw)

        if abs(ry_mapped) > _CHASSIS_DEADZONE_PCT:
            turn_speed = ry_mapped / 100.0 * _MAX_PIVOT_RAD_S
            rover.pivot_turn(turn_speed)
        else:
            # 左摇杆 → 底盘二维平移
            dx = map_joystick(lx_raw)       # -100~100，右正左负
            dy = -map_joystick(ly_raw)      # -100~100，前正后负

            if abs(dx) < _CHASSIS_DEADZONE_PCT and abs(dy) < _CHASSIS_DEADZONE_PCT:
                rover.stop()
            else:
                # 方向角 → 转向角；模长 → 速度
                angle_rad = math.atan2(dx, dy)
                angle_deg = math.degrees(angle_rad)
                magnitude = min(100.0, math.sqrt(dx * dx + dy * dy))

                # 将 -180°~180° 映射到 ±90° 转向范围：
                # 后半平面（|angle| > 90°）→ 翻转角度 180° 并反转电机
                if angle_deg > 90.0:
                    angle_deg -= 180.0
                    speed_sign = -1.0
                elif angle_deg < -90.0:
                    angle_deg += 180.0
                    speed_sign = -1.0
                else:
                    speed_sign = 1.0

                speed_rad_s = speed_sign * (magnitude / 100.0) * _MAX_MOTOR_RAD_S
                rover.drive(speed_rad_s, angle_deg)

        time.sleep_ms(50)
