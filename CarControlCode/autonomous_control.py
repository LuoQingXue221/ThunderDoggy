"""ESP32自动控制：把相机原始观测换算为巡线、对位、格子和机械臂动作。"""

import math
import time

from arm_control import ArmKinematicsError
from robot_config import (
    ARM_GRID_CALIBRATED, ARM_GRID_HOME_WAIT_MS, ARM_GRID_HOVER_WAIT_MS,
    ARM_GRID_MOVE_SPEED_DEG_S, ARM_GRID_POSES, ARM_GRID_TOUCH_WAIT_MS,
    ARM_INIT_PITCH1_DEG, ARM_INIT_PITCH2_DEG, ARM_INIT_PITCH3_DEG,
    ARM_INIT_ROLL_DEG, AUTO_BOARD_TIMEOUT_MS, AUTO_CAMERA_FORWARD_ANGLE_DEG,
    AUTO_CORNER_CONFIRM_OBSERVATIONS, AUTO_CORNER_SIDE_MIN_PIXELS,
    AUTO_CORNER_SIDE_RATIO_X100, AUTO_DOCK_RANGE_DEADZONE_PX,
    AUTO_DOCK_SPEED_RAD_S, AUTO_DOCK_X_DEADZONE_PX, AUTO_DRIVE_ACC_RAD_S2,
    AUTO_GRID_DOCK_STABLE_OBSERVATIONS, AUTO_GRID_DOCK_TARGET_HEIGHT_PX_480,
    AUTO_GRID_ENTRY_HEIGHT_PX_480, AUTO_GRID_LAYOUT_STABLE_OBSERVATIONS,
    AUTO_GRID_SETTLE_MS, AUTO_GRID_TARGET_TIMEOUT_MS,
    AUTO_LINE_ALIGN_DEADZONE_PX, AUTO_LINE_COMMAND_TIMEOUT_MS,
    AUTO_LINE_KP_DEG_PER_PX, AUTO_LINE_MAX_STEER_DEG,
    AUTO_LINE_MIN_CONFIDENCE, AUTO_LINE_SPEED_RAD_S, AUTO_PIVOT_SPEED_RAD_S,
    AUTO_TURN_ALIGNED_OBSERVATIONS, AUTO_TURN_MIN_MS, AUTO_TURN_TIMEOUT_MS,
    clamp,
)
from vision_protocol import VALID_COLORS


def ticks_ms():
    return time.ticks_ms() if hasattr(time, "ticks_ms") else int(time.time() * 1000)


def ticks_diff(a, b):
    return time.ticks_diff(a, b) if hasattr(time, "ticks_diff") else a - b


def ticks_add(a, b):
    return time.ticks_add(a, int(b)) if hasattr(time, "ticks_add") else a + int(b)


def parse_task(payload):
    tokens = str(payload).strip().lower().replace(",", " ").split()
    if not tokens:
        return None
    colors, counts = (), ()
    if len(tokens) % 2 == 0:
        half = len(tokens) // 2
        if all(v in VALID_COLORS for v in tokens[:half]):
            try:
                values = tuple(int(v) for v in tokens[half:])
            except ValueError:
                values = ()
            if values and all(v >= 0 for v in values):
                colors, counts = tuple(tokens[:half]), values
    if not colors and all(v in VALID_COLORS for v in tokens):
        colors, counts = tuple(tokens), tuple(1 for _ in tokens)
    sequence = []
    for color, count in zip(colors, counts):
        sequence.extend([color] * count)
    return tuple(sequence) if sequence else None


class GridArmExecutor:
    def __init__(self, rover):
        self.rover, self.active, self.steps = rover, False, []
        self.index, self.deadline = -1, 0

    @staticmethod
    def _pose(value):
        return isinstance(value, (tuple, list)) and len(value) == 4 and all(
            isinstance(v, (int, float)) for v in value)

    def start(self, row, column):
        if self.rover.arm is None:
            return False, "arm_missing"
        if not ARM_GRID_CALIBRATED:
            return False, "uncalibrated"
        poses = ARM_GRID_POSES.get((row, column))
        if not poses or not self._pose(poses.get("hover")) or not self._pose(poses.get("touch")):
            return False, "pose_missing"
        home = (ARM_INIT_ROLL_DEG, ARM_INIT_PITCH1_DEG,
                ARM_INIT_PITCH2_DEG, ARM_INIT_PITCH3_DEG)
        self.steps = [(home, ARM_GRID_HOME_WAIT_MS),
                      (poses["hover"], ARM_GRID_HOVER_WAIT_MS),
                      (poses["touch"], ARM_GRID_TOUCH_WAIT_MS),
                      (poses["hover"], ARM_GRID_HOVER_WAIT_MS),
                      (home, ARM_GRID_HOME_WAIT_MS)]
        self.index, self.active = -1, True
        try:
            self._advance()
        except ArmKinematicsError as error:
            self.active = False
            return False, "arm_%s" % error.reason
        return True, None

    def _advance(self):
        self.index += 1
        if self.index >= len(self.steps):
            self.active = False
            return
        pose, wait = self.steps[self.index]
        self.rover.arm.move_to_pose(*pose, speed_deg_s=ARM_GRID_MOVE_SPEED_DEG_S)
        self.deadline = ticks_add(ticks_ms(), wait)

    def update(self):
        if self.active and ticks_diff(ticks_ms(), self.deadline) >= 0:
            self._advance()
        return not self.active

    def cancel(self):
        self.active, self.steps = False, []


class VisionMath:
    @staticmethod
    def line(raw):
        fw, fh, points = raw["frame_w"], raw["frame_h"], raw["points"]
        weighted, weights, total_pixels, valid = 0, 0, 0, 0
        wide_far = False
        for index, point in enumerate(points):
            if point is None:
                continue
            weight = (5, 3, 1)[index]
            weighted += (point["cx"] - fw // 2) * weight
            weights += weight
            total_pixels += point["pixels"]
            valid += 1
            if index > 0 and point["w"] > point["h"] * 2:
                wide_far = True
        error = 0 if not weights else weighted * 640 // (weights * fw)
        scale_num, scale_den = 640 * 480, fw * fh
        confidence = min(100, total_pixels * scale_num // max(1, scale_den * 30))
        left = raw["left_pixels"] * scale_num // max(1, scale_den)
        right = raw["right_pixels"] * scale_num // max(1, scale_den)
        corner = "none"
        if wide_far or valid <= 2:
            larger, smaller = max(left, right), min(left, right)
            if (larger >= AUTO_CORNER_SIDE_MIN_PIXELS and
                    (smaller <= 0 or larger * 100 >= smaller * AUTO_CORNER_SIDE_RATIO_X100)):
                corner = "left" if left > right else "right"
        lost = confidence < AUTO_LINE_MIN_CONFIDENCE
        aligned = (not lost and abs(error) <= AUTO_LINE_ALIGN_DEADZONE_PX
                   and corner == "none" and valid >= 2)
        return {"error": error, "confidence": confidence, "lost": lost,
                "corner": corner, "aligned": aligned}

    @staticmethod
    def board(packet):
        board = packet["value"]
        if board is None:
            return None
        fw, fh = packet["frame_w"], packet["frame_h"]
        dx = (board["cx"] - fw // 2) * 640 // fw
        target = fh * AUTO_GRID_DOCK_TARGET_HEIGHT_PX_480 // 480
        range_error = (target - board["h"]) * 480 // fh
        return {"dx": dx, "range_error": range_error,
                "height_480": board["h"] * 480 // fh}

    @staticmethod
    def grid(block_packet, board_packet):
        board = board_packet["value"]
        if board is None:
            return []
        x, y, w, h = board["x"], board["y"], board["w"], board["h"]
        ix, iy = w * 8 // 100, h * 8 // 100
        left, top, width, height = x + ix, y + iy, max(1, w - 2 * ix), max(1, h - 2 * iy)
        result = []
        for source in block_packet["items"]:
            if not (x <= source["cx"] <= x + w and y <= source["cy"] <= y + h):
                continue
            item = source.copy()
            dx = item["cx"] - block_packet["frame_w"] // 2
            dy = item["cy"] - block_packet["frame_h"] // 2
            item["dx"], item["dy"] = dx, dy
            item["distance"] = int(math.sqrt(dx * dx + dy * dy) + 0.5)
            rx = max(0, min(width - 1, item["cx"] - left))
            ry = max(0, min(height - 1, item["cy"] - top))
            item["cell"] = (min(2, ry * 3 // height), min(2, rx * 3 // width))
            result.append(item)
        result.sort(key=lambda v: (-v["cy"], -v["pixels"]))
        return result


class AutonomousController:
    def __init__(self, rover, serial):
        self.rover, self.serial = rover, serial
        self.arm = GridArmExecutor(rover)
        self.active, self.mode = False, "idle"
        self.task, self.task_index, self.completed = (), 0, []
        self.last_qr_version = self.last_line_seq = self.last_grid_seq = -1
        self.corner, self.corner_count, self.aligned_count = "none", 0, 0
        self.turn_started = self.board_seen = self.deadline = 0
        self.dock_stable = self.layout_stable = 0
        self.layout_key, self.target_missing_since, self.pending_cell = None, 0, None

    def _write(self, text):
        self.serial.write(str(text).encode("utf-8"))

    def start(self, data):
        self.rover.stop()
        self.rover.center_chassis_servos()
        self.rover.servo_control.set_camera_angle(AUTO_CAMERA_FORWARD_ANGLE_DEG, speed_deg_s=30)
        self.active, self.mode = True, "line"
        self.task, self.task_index, self.completed = (), 0, []
        self.last_qr_version = data.get("qr_version", 0)
        self.last_line_seq = self.last_grid_seq = -1
        self.corner, self.corner_count, self.aligned_count = "none", 0, 0
        self.dock_stable = self.layout_stable = 0
        self.layout_key, self.pending_cell = None, None
        data["qr"] = data["line"] = data["blocks"] = data["board"] = None
        self.arm.cancel()
        self._write("@STREAM_START\n")
        print("AUTO启动：ESP32负责全部计算，等待原始视觉量")

    def cancel(self, reason="manual_stop"):
        self.rover.stop()
        self.arm.cancel()
        was_active = self.active
        self.active, self.mode = False, "idle"
        if was_active:
            self._write("@STREAM_STOP\n")
            print("AUTO停止:", reason)

    def _fail(self, reason):
        self.rover.stop()
        self.arm.cancel()
        self.active, self.mode = True, "fault"
        print("AUTO故障:", reason)

    @staticmethod
    def _fresh(value, timeout=AUTO_LINE_COMMAND_TIMEOUT_MS):
        return value is not None and ticks_diff(ticks_ms(), value.get("rx_ms", 0)) <= timeout

    def _read_qr(self, data):
        version = data.get("qr_version", 0)
        if version == self.last_qr_version:
            return
        self.last_qr_version = version
        raw = data.get("qr")
        task = parse_task(raw.get("payload", "")) if raw else None
        if task:
            self.task, self.task_index, self.completed = task, 0, []
            print("二维码任务:", self.task)
        else:
            print("二维码任务格式无效")

    def _begin_turn(self, direction):
        self.rover.stop()
        speed = -AUTO_PIVOT_SPEED_RAD_S if direction == "left" else AUTO_PIVOT_SPEED_RAD_S
        self.rover.pivot_turn(speed, acc_rad_s2=AUTO_DRIVE_ACC_RAD_S2)
        self.turn_started, self.aligned_count, self.mode = ticks_ms(), 0, "turn"

    def _grid_packets(self, data):
        blocks, board = data.get("blocks"), data.get("board")
        if not blocks or not board or blocks["sequence"] != board["sequence"]:
            return None, None
        if not self._fresh(blocks, 800) or not self._fresh(board, 800):
            return None, None
        return blocks, board

    def _update_line(self, data):
        blocks, board = self._grid_packets(data)
        if self.task and board and board["value"] is not None:
            items = VisionMath.grid(blocks, board)
            geometry = VisionMath.board(board)
            if items and geometry["height_480"] >= AUTO_GRID_ENTRY_HEIGHT_PX_480:
                self.rover.stop()
                self.board_seen, self.dock_stable, self.mode = ticks_ms(), 0, "dock"
                return
        raw = data.get("line")
        if not self._fresh(raw):
            self.rover.stop()
            return
        result = VisionMath.line(raw)
        if raw["sequence"] != self.last_line_seq:
            self.last_line_seq = raw["sequence"]
            if result["corner"] != "none" and result["corner"] == self.corner:
                self.corner_count += 1
            elif result["corner"] != "none":
                self.corner, self.corner_count = result["corner"], 1
            else:
                self.corner, self.corner_count = "none", 0
            if self.corner_count >= AUTO_CORNER_CONFIRM_OBSERVATIONS:
                self._begin_turn(self.corner)
                return
        if result["lost"]:
            self.rover.stop()
            return
        steer = clamp(result["error"] * AUTO_LINE_KP_DEG_PER_PX,
                      -AUTO_LINE_MAX_STEER_DEG, AUTO_LINE_MAX_STEER_DEG)
        self.rover.drive(AUTO_LINE_SPEED_RAD_S, steer,
                         acc_rad_s2=AUTO_DRIVE_ACC_RAD_S2)

    def _update_turn(self, data):
        elapsed = ticks_diff(ticks_ms(), self.turn_started)
        if elapsed > AUTO_TURN_TIMEOUT_MS:
            self._fail("turn_timeout")
            return
        raw = data.get("line")
        if not self._fresh(raw) or raw["sequence"] == self.last_line_seq:
            return
        self.last_line_seq = raw["sequence"]
        result = VisionMath.line(raw)
        self.aligned_count = self.aligned_count + 1 if (
            elapsed >= AUTO_TURN_MIN_MS and result["aligned"]) else 0
        if self.aligned_count >= AUTO_TURN_ALIGNED_OBSERVATIONS:
            self.rover.stop()
            self.rover.center_chassis_servos()
            self.corner, self.corner_count, self.mode = "none", 0, "line"

    def _update_dock(self, data):
        board = data.get("board")
        if not self._fresh(board) or board["value"] is None:
            self.rover.stop()
            if ticks_diff(ticks_ms(), self.board_seen) > AUTO_BOARD_TIMEOUT_MS:
                self._fail("board_lost")
            return
        self.board_seen = ticks_ms()
        geometry = VisionMath.board(board)
        new = board["sequence"] != self.last_grid_seq
        if new:
            self.last_grid_seq = board["sequence"]
        if abs(geometry["dx"]) > AUTO_DOCK_X_DEADZONE_PX:
            speed = AUTO_PIVOT_SPEED_RAD_S * 0.65
            self.rover.pivot_turn(-speed if geometry["dx"] < 0 else speed,
                                  acc_rad_s2=AUTO_DRIVE_ACC_RAD_S2)
            self.dock_stable = 0
        elif abs(geometry["range_error"]) > AUTO_DOCK_RANGE_DEADZONE_PX:
            speed = AUTO_DOCK_SPEED_RAD_S
            self.rover.drive(-speed if geometry["range_error"] < 0 else speed, 0,
                             acc_rad_s2=AUTO_DRIVE_ACC_RAD_S2)
            self.dock_stable = 0
        else:
            self.rover.stop()
            if new:
                self.dock_stable += 1
        if self.dock_stable >= AUTO_GRID_DOCK_STABLE_OBSERVATIONS:
            self.rover.stop()
            self.deadline, self.mode = ticks_add(ticks_ms(), AUTO_GRID_SETTLE_MS), "settle"

    def _update_grid(self, data):
        blocks, board = self._grid_packets(data)
        if not board or board["value"] is None:
            if ticks_diff(ticks_ms(), self.board_seen) > AUTO_BOARD_TIMEOUT_MS:
                self._fail("grid_board_lost")
            return
        self.board_seen = ticks_ms()
        if board["sequence"] == self.last_grid_seq:
            return
        self.last_grid_seq = board["sequence"]
        items = VisionMath.grid(blocks, board)
        key = tuple(sorted((v["color"], v["cell"][0], v["cell"][1]) for v in items))
        if key and key == self.layout_key:
            self.layout_stable += 1
        else:
            self.layout_key, self.layout_stable = key, 1 if key else 0
        color = self.task[self.task_index] if self.task_index < len(self.task) else None
        candidates = [v for v in items if v["color"] == color and v["cell"] not in self.completed]
        if not candidates:
            if not self.target_missing_since:
                self.target_missing_since = ticks_ms()
            elif ticks_diff(ticks_ms(), self.target_missing_since) > AUTO_GRID_TARGET_TIMEOUT_MS:
                self._fail("target_missing_%s" % color)
            return
        self.target_missing_since = 0
        if self.layout_stable < AUTO_GRID_LAYOUT_STABLE_OBSERVATIONS:
            return
        target = candidates[0]
        print("目标:", target["color"], "cell=", target["cell"],
              "dx/dy/distance=", target["dx"], target["dy"], target["distance"])
        ok, reason = self.arm.start(target["cell"][0], target["cell"][1])
        if not ok:
            self._fail(reason)
            return
        self.pending_cell, self.mode = target["cell"], "arm"

    def _update_arm(self):
        try:
            finished = self.arm.update()
        except ArmKinematicsError as error:
            self._fail("arm_%s" % error.reason)
            return
        if not finished:
            return
        if self.pending_cell not in self.completed:
            self.completed.append(self.pending_cell)
        self.pending_cell = None
        self.task_index += 1
        if self.task_index >= len(self.task):
            self.rover.stop()
            self.mode = "complete"
            self._write("@STREAM_STOP\n")
            print("二维码任务完成")
        else:
            self.layout_key, self.layout_stable = None, 0
            self.deadline, self.mode = ticks_add(ticks_ms(), AUTO_GRID_SETTLE_MS), "settle"

    def update(self, data):
        if not self.active:
            return
        self._read_qr(data)
        if self.mode == "line":
            self._update_line(data)
        elif self.mode == "turn":
            self._update_turn(data)
        elif self.mode == "dock":
            self._update_dock(data)
        elif self.mode == "settle":
            self.rover.stop()
            if ticks_diff(ticks_ms(), self.deadline) >= 0:
                self.last_grid_seq, self.layout_key, self.layout_stable = -1, None, 0
                self.mode = "grid"
        elif self.mode == "grid":
            self._update_grid(data)
        elif self.mode == "arm":
            self._update_arm()
        elif self.mode in ("fault", "complete"):
            self.rover.stop()
