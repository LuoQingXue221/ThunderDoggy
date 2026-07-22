"""test 模式：自动对正后的无扭矩机械臂角度测量。"""

import time

from arm_control import ArmKinematicsError
from autonomous_control import AutonomousController, describe_reason


_TELEMETRY_INTERVAL_MS = 200


def _pressed(buttons, button):
    return (buttons & button) == button


def _ticks_add(value, delta):
    return time.ticks_add(value, delta) if hasattr(time, "ticks_add") else value + delta


def _ticks_diff(first, second):
    return time.ticks_diff(first, second) if hasattr(time, "ticks_diff") else first - second


def _ticks_ms():
    return time.ticks_ms() if hasattr(time, "ticks_ms") else int(time.time() * 1000)


def _sleep_ms(value):
    if hasattr(time, "sleep_ms"):
        time.sleep_ms(value)
    else:
        time.sleep(float(value) / 1000.0)


def _print_pose(rover):
    if rover.arm is None:
        print("TEST_ARM_POSE_ERROR,arm_missing")
        return False
    try:
        pose = rover.arm.sync_from_servos()
    except ArmKinematicsError as error:
        print("TEST_ARM_POSE_ERROR,%s" % error.reason)
        return False
    print("TEST_ARM_POSE,roll=%.1f,pitch1=%.1f,pitch2=%.1f,pitch3=%.1f" % (
        pose["roll_deg"], pose["pitch1_deg"],
        pose["pitch2_deg"], pose["pitch3_deg"],
    ))
    return True


def test_loop(rover, ps2, data, serial):
    """仅允许 L3 对正和 R3 四关节读角的安全测试循环。"""
    print("TEST 模式：L3=仅自动对正/取消，R3=开启或关闭四关节读角，SELECT=退出")
    print("TEST 安全：机械臂与夹爪已释放扭矩，请用支架托住后再手动调整。")

    auto = AutonomousController(rover, serial)
    telemetry_enabled = False
    telemetry_due = 0
    r3_latched = False
    l3_latched = False

    while True:
        ps2.update()
        fresh, buttons, _, _, _, _, _ = ps2.snapshot()
        if not fresh:
            if auto.active:
                auto.cancel("ps2_lost")
            continue

        if _pressed(buttons, ps2.PS2_BTN_SELECT):
            if auto.active:
                auto.cancel("select_exit")
            rover.stop()
            rover.disable()
            print("SELECT：退出 TEST 模式，机械臂保持当前扭矩状态。")
            break

        combo_pressed = (_pressed(buttons, ps2.PS2_BTN_L3) and
                         _pressed(buttons, ps2.PS2_BTN_R3))
        if combo_pressed:
            # test 模式明确禁用 L3+R3 复位，也避免本次组合键触发单键逻辑。
            l3_latched = True
            r3_latched = True
        else:
            l3_pressed = _pressed(buttons, ps2.PS2_BTN_L3)
            if not l3_pressed:
                l3_latched = False
            elif not l3_latched:
                l3_latched = True
                if auto.active:
                    auto.cancel("ps2_l3_toggle")
                    rover.disable()
                else:
                    # test 上电时电机保持失能；仅在自动对正期间临时使能。
                    if not getattr(rover, "motors_enabled", False):
                        rover.enable_motors()
                    ok, reason = auto.start_alignment_only(data)
                    if not ok:
                        rover.disable()
                        print("TEST L3对正拒绝：", describe_reason(reason))

            r3_pressed = _pressed(buttons, ps2.PS2_BTN_R3)
            if not r3_pressed:
                r3_latched = False
            elif not r3_latched:
                r3_latched = True
                telemetry_enabled = not telemetry_enabled
                if telemetry_enabled:
                    telemetry_due = _ticks_ms()
                    print("TEST 四关节读角已开启：每200ms输出一次。")
                else:
                    print("TEST 四关节读角已关闭。")

        if auto.active:
            auto.update(data)
            if auto.mode == "fault":
                rover.disable()

        if telemetry_enabled:
            now = _ticks_ms()
            if _ticks_diff(now, telemetry_due) >= 0:
                _print_pose(rover)
                telemetry_due = _ticks_add(now, _TELEMETRY_INTERVAL_MS)

        _sleep_ms(20)
