"""TEST 模式的无扭矩机械臂标定行为。"""

import os
import sys
import unittest
from contextlib import redirect_stdout
from io import StringIO


ROOT = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
CONTROL_DIR = os.path.join(ROOT, "CarControlCode")
if CONTROL_DIR not in sys.path:
    sys.path.insert(0, CONTROL_DIR)

import test_control
from autonomous_control import AutonomousController
from servo_control import get_test_locked_servo_ids, get_test_released_servo_ids
from vision_protocol import new_camera_data


class FakeSerial:
    def __init__(self):
        self.writes = []

    def write(self, value):
        self.writes.append(value)


class FakeArm:
    def __init__(self):
        self.sync_count = 0
        self.camera_angle_deg = 0.0

    def sync_from_servos(self):
        self.sync_count += 1
        return {"roll_deg": 1.0, "pitch1_deg": 2.0,
                "pitch2_deg": 3.0, "pitch3_deg": 4.0}

    def cancel(self):
        pass


class FakeServoControl:
    def __init__(self):
        self.camera_readback = 0.0

    def read_camera_angle(self):
        return self.camera_readback

    def set_camera_angle(self, angle, speed_deg_s=None):
        pass


class FakeRover:
    def __init__(self):
        self.arm = FakeArm()
        self.servo_control = FakeServoControl()
        self.motors_enabled = True
        self.calls = []

    def stop(self):
        self.calls.append("stop")

    def disable(self):
        self.motors_enabled = False
        self.calls.append("disable")

    def enable_motors(self):
        self.motors_enabled = True
        self.calls.append("enable")

    def center_chassis_servos(self):
        self.calls.append("center")


class FakePS2:
    PS2_BTN_SELECT = 0x0001
    PS2_BTN_L3 = 0x0002
    PS2_BTN_R3 = 0x0004

    def __init__(self, frames):
        self.frames = iter(frames)
        self.current = 0

    def update(self):
        self.current = next(self.frames)

    def snapshot(self):
        return True, self.current, 128, 128, 128, 128, 0


class TestModeTests(unittest.TestCase):
    def test_servo_groups_keep_arm_and_gripper_released(self):
        self.assertEqual((1, 2, 3, 4, 5, 6, 8), get_test_locked_servo_ids())
        released = get_test_released_servo_ids()
        self.assertEqual((7, 9, 10, 11), released[:4])
        self.assertNotIn(8, released)

    def test_alignment_only_ignores_cached_qr_task(self):
        rover, serial, data = FakeRover(), FakeSerial(), new_camera_data()
        controller = AutonomousController(rover, serial)
        controller.cached_task = ("blue", "blue")
        ok, reason = controller.start_alignment_only(data)
        self.assertTrue(ok)
        self.assertIsNone(reason)
        self.assertEqual((), controller.task)
        self.assertEqual("dock", controller.session)
        self.assertIn(b"@STREAM_START\n", serial.writes)

    def test_r3_telemetry_reads_without_joint_position_command(self):
        rover, serial, data = FakeRover(), FakeSerial(), new_camera_data()
        ps2 = FakePS2([
            FakePS2.PS2_BTN_R3,
            0,
            FakePS2.PS2_BTN_R3,
            0,
            FakePS2.PS2_BTN_SELECT,
        ])
        original_sleep = test_control._sleep_ms
        test_control._sleep_ms = lambda _value: None
        output = StringIO()
        try:
            with redirect_stdout(output):
                test_control.test_loop(rover, ps2, data, serial)
        finally:
            test_control._sleep_ms = original_sleep
        self.assertIn("TEST_ARM_POSE,roll=1.0,pitch1=2.0,pitch2=3.0,pitch3=4.0",
                      output.getvalue())
        self.assertGreaterEqual(rover.arm.sync_count, 1)
        self.assertFalse(any(value == "enable" for value in rover.calls))


if __name__ == "__main__":
    unittest.main()
