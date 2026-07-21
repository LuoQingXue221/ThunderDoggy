"""手动巡线后自动对位流程的桌面回归测试。"""

import os
import sys
import unittest
from contextlib import redirect_stdout
from io import StringIO


ROOT = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
CONTROL_DIR = os.path.join(ROOT, "CarControlCode")
if CONTROL_DIR not in sys.path:
    sys.path.insert(0, CONTROL_DIR)

import autonomous_control as auto_module
import chassis_control as chassis_module
import ps2_control as ps2_module
from arm_control import ArmKinematicsError, RobotArm
from vision_protocol import new_camera_data


class FakeSerial:
    def __init__(self):
        self.writes = []

    def write(self, value):
        self.writes.append(value)


class FakeServoControl:
    def __init__(self):
        self.camera_angles = []
        self.camera_readback = 0.0

    def set_camera_angle(self, angle, speed_deg_s=None):
        self.camera_angles.append((angle, speed_deg_s))

    def read_camera_angle(self):
        return self.camera_readback

    def set_reserve_servo_angle(self, servo_id, angle):
        pass


class FakeRobotArm:
    def __init__(self):
        self.camera_angle_deg = 0.0
        self.jogs = []
        self.initial_pose_count = 0

    def apply_initial_pose(self):
        self.initial_pose_count += 1

    def jog_camera(self, delta):
        self.jogs.append(("camera", delta))

    def jog_joints(self, roll, pitch1, pitch2, pitch3):
        self.jogs.append(("joints", roll, pitch1, pitch2, pitch3))

    def sync_from_servos(self):
        return {"roll_deg": 0.0, "pitch1_deg": 0.0,
                "pitch2_deg": 0.0, "pitch3_deg": 0.0}


class FakeRover:
    def __init__(self):
        self.servo_control = FakeServoControl()
        self.arm = FakeRobotArm()
        self.motors_enabled = True
        self.calls = []

    def stop(self):
        self.calls.append(("stop",))

    def center_chassis_servos(self):
        self.calls.append(("center",))

    def pivot_turn(self, speed, acc_rad_s2=None):
        self.calls.append(("pivot", speed, acc_rad_s2))

    def drive(self, speed, angle, acc_rad_s2=None):
        self.calls.append(("drive", speed, angle, acc_rad_s2))

    def disable(self):
        self.motors_enabled = False
        self.calls.append(("disable",))

    def enable_motors(self):
        self.motors_enabled = True
        self.calls.append(("enable",))


def set_qr(data, version, payload):
    data["qr_version"] = version
    data["qr"] = {"sequence": version, "payload": payload, "rx_ms": 0}


def board_packet(sequence, now, x=190, y=40, width=260, height=260):
    return {"sequence": sequence, "frame_w": 640, "frame_h": 480,
            "value": {"x": x, "y": y, "w": width, "h": height,
                      "pixels": width * height, "cx": x + width // 2,
                      "cy": y + height // 2},
            "rx_ms": now}


def blocks_packet(sequence, now):
    return {"sequence": sequence, "frame_w": 640, "frame_h": 480,
            "items": [{"color": "blue", "x": 235, "y": 190,
                       "w": 30, "h": 30, "pixels": 800,
                       "cx": 250, "cy": 205}],
            "rx_ms": now}


class AutonomousDockTests(unittest.TestCase):
    def setUp(self):
        self.now = 1000
        self.original_ticks_ms = auto_module.ticks_ms
        auto_module.ticks_ms = lambda: self.now
        self.rover = FakeRover()
        self.serial = FakeSerial()
        self.controller = auto_module.AutonomousController(self.rover, self.serial)
        self.data = new_camera_data()

    def tearDown(self):
        auto_module.ticks_ms = self.original_ticks_ms

    def finish_camera_settle(self):
        for _ in range(auto_module.AUTO_CAMERA_SETTLE_OBSERVATIONS):
            self.controller.update(self.data)
            self.now += auto_module.AUTO_CAMERA_VERIFY_INTERVAL_MS

    def test_latest_valid_cache_and_local_task_freeze(self):
        set_qr(self.data, 1, "blue yellow 1 2")
        self.assertTrue(self.controller.observe_qr(self.data))
        self.assertEqual(("blue", "yellow", "yellow"), self.controller.cached_task)

        set_qr(self.data, 2, "not-a-task")
        self.assertFalse(self.controller.observe_qr(self.data))
        self.assertEqual(("blue", "yellow", "yellow"), self.controller.cached_task)

        ok, reason = self.controller.start_dock(self.data)
        self.assertTrue(ok)
        self.assertIsNone(reason)
        frozen = self.controller.task

        set_qr(self.data, 3, "red")
        self.controller.observe_qr(self.data)
        self.assertEqual(("red",), self.controller.cached_task)
        self.assertEqual(frozen, self.controller.task)

    def test_local_start_without_task_enters_alignment_only_mode(self):
        ok, reason = self.controller.start_dock(self.data)
        self.assertTrue(ok)
        self.assertIsNone(reason)
        self.assertTrue(self.controller.active)
        self.assertEqual("dock", self.controller.session)
        self.assertEqual("camera_settle", self.controller.mode)
        self.finish_camera_settle()
        self.assertEqual("acquire_board", self.controller.mode)
        self.assertEqual((), self.controller.task)

    def test_alignment_only_stops_after_board_is_stable(self):
        self.controller.start_dock(self.data)
        self.finish_camera_settle()
        self.data["board"] = board_packet(1, self.now)
        self.controller.update(self.data)
        self.assertEqual("dock", self.controller.mode)

        for sequence in range(2, 2 + auto_module.AUTO_GRID_DOCK_STABLE_OBSERVATIONS):
            self.now += 10
            self.data["board"] = board_packet(sequence, self.now)
            self.controller.update(self.data)
        self.assertEqual("aligned", self.controller.mode)
        self.assertTrue(self.controller.active)
        self.assertFalse(self.rover.motors_enabled)
        self.assertEqual("disable", self.rover.calls[-1][0])

    def test_automatic_modes_reject_disabled_drive_motors(self):
        self.rover.motors_enabled = False
        ok, reason = self.controller.start_dock(self.data)
        self.assertFalse(ok)
        self.assertEqual("motors_disabled", reason)
        ok, reason = self.controller.start_full(self.data)
        self.assertFalse(ok)
        self.assertEqual("motors_disabled", reason)

    def test_camera_must_reach_calibrated_angle_before_board_acquire(self):
        self.rover.servo_control.camera_readback = 30.0
        self.data["board"] = board_packet(7, self.now)
        self.controller.start_dock(self.data)
        self.controller.update(self.data)
        self.assertEqual("camera_settle", self.controller.mode)
        self.assertEqual((0.0, 30), self.rover.servo_control.camera_angles[-1])

        self.rover.servo_control.camera_readback = 0.0
        self.now += auto_module.AUTO_CAMERA_VERIFY_INTERVAL_MS
        self.controller.update(self.data)
        self.now += auto_module.AUTO_CAMERA_VERIFY_INTERVAL_MS
        self.controller.update(self.data)
        self.assertEqual("acquire_board", self.controller.mode)
        self.assertIsNone(self.data["board"])

    def test_camera_readback_timeout_stays_stopped(self):
        self.rover.servo_control.camera_readback = None
        self.controller.start_dock(self.data)
        self.now += auto_module.AUTO_CAMERA_SETTLE_TIMEOUT_MS
        self.controller.update(self.data)
        self.assertEqual("fault", self.controller.mode)
        self.assertEqual("camera_settle_timeout", self.controller.fault_reason)

    def test_invalid_board_frames_distinguish_detection_from_uart_failure(self):
        self.controller.start_dock(self.data)
        self.finish_camera_settle()
        self.data["board"] = {
            "sequence": 1, "frame_w": 640, "frame_h": 480,
            "value": None, "rx_ms": self.now,
        }
        self.controller.update(self.data)
        self.now += auto_module.AUTO_BOARD_ACQUIRE_TIMEOUT_MS
        self.data["board"]["rx_ms"] = self.now
        self.controller.update(self.data)
        self.assertEqual("fault", self.controller.mode)
        self.assertEqual("board_not_detected_timeout", self.controller.fault_reason)

    def test_board_acquire_timeout_stays_stopped(self):
        set_qr(self.data, 1, "blue")
        self.controller.observe_qr(self.data)
        self.controller.start_dock(self.data)
        self.finish_camera_settle()
        self.now += auto_module.AUTO_BOARD_ACQUIRE_TIMEOUT_MS
        self.controller.update(self.data)
        self.assertEqual("fault", self.controller.mode)
        self.assertEqual("board_link_timeout", self.controller.fault_reason)
        self.assertTrue(self.controller.active)
        self.assertEqual("dock", self.controller.session)
        self.assertEqual("stop", self.rover.calls[-1][0])

    def test_board_dock_and_uncalibrated_arm_safety_lock(self):
        set_qr(self.data, 1, "blue")
        self.controller.observe_qr(self.data)
        self.controller.start_dock(self.data)
        self.finish_camera_settle()

        self.data["board"] = board_packet(1, self.now)
        self.controller.update(self.data)
        self.assertEqual("dock", self.controller.mode)

        for sequence in range(2, 2 + auto_module.AUTO_GRID_DOCK_STABLE_OBSERVATIONS):
            self.now += 10
            self.data["board"] = board_packet(sequence, self.now)
            self.controller.update(self.data)
        self.assertEqual("settle", self.controller.mode)

        self.now += auto_module.AUTO_GRID_SETTLE_MS
        self.controller.update(self.data)
        self.assertEqual("grid", self.controller.mode)

        for sequence in range(20, 20 + auto_module.AUTO_GRID_LAYOUT_STABLE_OBSERVATIONS):
            self.now += 10
            self.data["board"] = board_packet(sequence, self.now)
            self.data["blocks"] = blocks_packet(sequence, self.now)
            self.controller.update(self.data)
        self.assertEqual("fault", self.controller.mode)
        self.assertTrue(self.controller.active)

    def test_full_mode_waits_for_new_qr_and_freezes_first_task(self):
        set_qr(self.data, 1, "blue")
        self.controller.observe_qr(self.data)
        self.controller.start_full(self.data)
        self.assertEqual((), self.controller.task)
        self.assertEqual(("blue",), self.controller.cached_task)

        set_qr(self.data, 2, "red")
        self.controller.observe_qr(self.data)
        self.assertEqual(("red",), self.controller.task)

        set_qr(self.data, 3, "yellow")
        self.controller.observe_qr(self.data)
        self.assertEqual(("yellow",), self.controller.cached_task)
        self.assertEqual(("red",), self.controller.task)

    def test_complete_holds_until_cancel_without_stopping_stream(self):
        class FinishedArm:
            def update(self):
                return True

            def cancel(self):
                pass

        self.controller.active = True
        self.controller.session = "dock"
        self.controller.mode = "arm"
        self.controller.task = ("blue",)
        self.controller.pending_cell = (1, 1)
        self.controller.arm = FinishedArm()
        self.controller.update(self.data)
        self.assertEqual("complete", self.controller.mode)
        self.assertTrue(self.controller.active)
        self.assertFalse(any(b"@STREAM_STOP" in value for value in self.serial.writes))
        self.controller.cancel("test_ack")
        self.assertFalse(self.controller.active)
        self.assertEqual("idle", self.controller.mode)


class FakePS2:
    PS2_BTN_SELECT = 0x0001
    PS2_BTN_L3 = 0x0002
    PS2_BTN_R3 = 0x0004
    PS2_BTN_START = 0x0008
    PS2_BTN_UP = 0x0010
    PS2_BTN_RIGHT = 0x0020
    PS2_BTN_DOWN = 0x0040
    PS2_BTN_LEFT = 0x0080
    PS2_BTN_L2 = 0x0100
    PS2_BTN_R2 = 0x0200
    PS2_BTN_L1 = 0x0400
    PS2_BTN_R1 = 0x0800
    PS2_BTN_TRIANGLE = 0x1000
    PS2_BTN_CIRCLE = 0x2000
    PS2_BTN_CROSS = 0x4000
    PS2_BTN_SQUARE = 0x8000

    def __init__(self, frames):
        self.frames = iter(frames)
        self.current = 0

    def update(self):
        self.current = next(self.frames)

    def snapshot(self):
        return True, self.current, 128, 128, 128, 128, 0


class PS2FlowTests(unittest.TestCase):
    def setUp(self):
        self.original_sleep_ms = getattr(ps2_module.time, "sleep_ms", None)
        ps2_module.time.sleep_ms = lambda _value: None

    def tearDown(self):
        if self.original_sleep_ms is None:
            delattr(ps2_module.time, "sleep_ms")
        else:
            ps2_module.time.sleep_ms = self.original_sleep_ms

    def test_reset_combo_does_not_fall_through_to_l3_and_up_jogs_pitch1(self):
        ps2 = FakePS2([
            FakePS2.PS2_BTN_L3 | FakePS2.PS2_BTN_R3,
            FakePS2.PS2_BTN_L3,
            0,
            FakePS2.PS2_BTN_UP,
            FakePS2.PS2_BTN_SELECT,
        ])
        rover, serial, data = FakeRover(), FakeSerial(), new_camera_data()
        set_qr(data, 1, "blue")
        ps2_module.ps2_loop(rover, ps2, data, serial)

        self.assertFalse(any(b"@STREAM_START" in value for value in serial.writes))
        joint_jogs = [value for value in rover.arm.jogs if value[0] == "joints"]
        self.assertIn(("joints", 0.0, 2.0, 0.0, 0.0), joint_jogs)

    def test_l3_starts_and_cancels_local_mode(self):
        ps2 = FakePS2([
            FakePS2.PS2_BTN_L3,
            0,
            FakePS2.PS2_BTN_L3,
            FakePS2.PS2_BTN_SELECT,
        ])
        rover, serial, data = FakeRover(), FakeSerial(), new_camera_data()
        set_qr(data, 1, "blue")
        ps2_module.ps2_loop(rover, ps2, data, serial)

        starts = [value for value in serial.writes if b"@STREAM_START" in value]
        self.assertEqual(1, len(starts))
        self.assertFalse(any(b"@STREAM_STOP" in value for value in serial.writes))

    def test_start_does_not_cancel_local_mode(self):
        ps2 = FakePS2([
            FakePS2.PS2_BTN_L3,
            0,
            FakePS2.PS2_BTN_START,
            0,
            FakePS2.PS2_BTN_L3,
            FakePS2.PS2_BTN_SELECT,
        ])
        rover, serial, data = FakeRover(), FakeSerial(), new_camera_data()
        set_qr(data, 1, "blue")
        ps2_module.ps2_loop(rover, ps2, data, serial)

        starts = [value for value in serial.writes if b"@STREAM_START" in value]
        self.assertEqual(1, len(starts))

    def test_l3_does_not_cancel_full_mode(self):
        ps2 = FakePS2([
            FakePS2.PS2_BTN_START,
            0,
            FakePS2.PS2_BTN_L3,
            0,
            FakePS2.PS2_BTN_START,
            FakePS2.PS2_BTN_SELECT,
        ])
        rover, serial, data = FakeRover(), FakeSerial(), new_camera_data()
        set_qr(data, 1, "blue")
        ps2_module.ps2_loop(rover, ps2, data, serial)

        starts = [value for value in serial.writes if b"@STREAM_START" in value]
        self.assertEqual(1, len(starts))

    def test_r3_snapshot_prints_normalized_board_height(self):
        rover, data = FakeRover(), new_camera_data()
        now = ps2_module.ticks_ms()
        data["board"] = board_packet(1, now, height=243)
        output = StringIO()
        with redirect_stdout(output):
            ps2_module._print_calibration_snapshot(rover, data)
        text = output.getvalue()
        self.assertIn("GRID_POSE=", text)
        self.assertIn("height_480=243", text)
        self.assertIn("suggested_target=243", text)


class FakeArmServoControl:
    def __init__(self, values):
        self.values = values
        self.commands = []

    def read_arm_joint_angles(self):
        return self.values.copy()

    def set_arm_joint_angles(self, roll, pitch1, pitch2, pitch3,
                             speed_deg_s=None):
        self.commands.append((roll, pitch1, pitch2, pitch3, speed_deg_s))


class RobotArmSafetyTests(unittest.TestCase):
    def test_incremental_jog_requires_real_angle_sync(self):
        arm = RobotArm(FakeArmServoControl({
            "roll": 0.0, "pitch1": 10.0, "pitch2": -20.0, "pitch3": 5.0,
        }))
        with self.assertRaises(ArmKinematicsError) as context:
            arm.jog_joints(pitch1_delta_deg=2.0)
        self.assertEqual("state_unsynced", context.exception.reason)

    def test_sync_then_jog_uses_measured_pose(self):
        servo = FakeArmServoControl({
            "roll": 1.0, "pitch1": 10.0, "pitch2": -20.0, "pitch3": 5.0,
        })
        arm = RobotArm(servo)
        arm.sync_from_servos()
        arm.jog_joints(pitch1_delta_deg=2.0)
        self.assertEqual((1.0, 12.0, -20.0, 5.0), servo.commands[-1][:4])

    def test_sync_rejects_out_of_range_measured_pose(self):
        arm = RobotArm(FakeArmServoControl({
            "roll": 0.0, "pitch1": 999.0, "pitch2": 0.0, "pitch3": 0.0,
        }))
        with self.assertRaises(ArmKinematicsError) as context:
            arm.sync_from_servos()
        self.assertEqual("joint_limit", context.exception.reason)
        self.assertFalse(arm.joints_synced)


class GridArmPoseTests(unittest.TestCase):
    class MovingArm:
        def __init__(self):
            self.commands = []

        def move_to_pose(self, *pose, speed_deg_s=None):
            self.commands.append((pose, speed_deg_s))

    class ArmRover:
        def __init__(self):
            self.arm = GridArmPoseTests.MovingArm()

    def setUp(self):
        self.original_calibrated = auto_module.ARM_GRID_CALIBRATED
        self.original_poses = auto_module.ARM_GRID_POSES

    def tearDown(self):
        auto_module.ARM_GRID_CALIBRATED = self.original_calibrated
        auto_module.ARM_GRID_POSES = self.original_poses

    def test_cell_uses_one_direct_grab_pose(self):
        grab = (10.0, 20.0, -30.0, 40.0)
        auto_module.ARM_GRID_CALIBRATED = True
        auto_module.ARM_GRID_POSES = {(1, 2): grab}
        executor = auto_module.GridArmExecutor(self.ArmRover())

        ok, reason = executor.start(1, 2)

        self.assertTrue(ok)
        self.assertIsNone(reason)
        self.assertEqual(3, len(executor.steps))
        self.assertEqual(grab, executor.steps[1][0])
        self.assertEqual(auto_module.ARM_GRID_GRAB_WAIT_MS, executor.steps[1][1])

    def test_old_hover_touch_shape_is_rejected(self):
        auto_module.ARM_GRID_CALIBRATED = True
        auto_module.ARM_GRID_POSES = {
            (0, 0): {"hover": (0, 0, 0, 0), "touch": (1, 1, 1, 1)},
        }
        executor = auto_module.GridArmExecutor(self.ArmRover())
        self.assertEqual((False, "pose_missing"), executor.start(0, 0))


class FakeMotorBus:
    def __init__(self):
        self.disabled = []

    def disable_all(self, motor_ids):
        self.disabled.append(tuple(motor_ids))


class FakeChassisServoControl:
    def __init__(self):
        self.servo_bus = object()

    def set_steering_angles(self, *args, **kwargs):
        pass


class ChassisStartupSafetyTests(unittest.TestCase):
    def test_prepare_keeps_drive_motors_disabled(self):
        original_sleep_ms = getattr(chassis_module.time, "sleep_ms", None)
        chassis_module.time.sleep_ms = lambda _value: None
        try:
            motor_bus = FakeMotorBus()
            rover = chassis_module.LunarRover(
                motor_bus, FakeChassisServoControl(), arm=None,
            )
            rover.prepare()
        finally:
            if original_sleep_ms is None:
                delattr(chassis_module.time, "sleep_ms")
            else:
                chassis_module.time.sleep_ms = original_sleep_ms
        self.assertFalse(rover.motors_enabled)
        self.assertEqual([rover.motor_ids], motor_bus.disabled)


if __name__ == "__main__":
    unittest.main()
