"""ESP32自动控制：把相机原始观测换算为巡线、对位、格子和机械臂动作。"""

import math
import time

from arm_control import ArmKinematicsError
from robot_config import (
    ARM_GRID_CALIBRATED, ARM_GRID_HOME_WAIT_MS, ARM_GRID_HOVER_WAIT_MS,
    ARM_GRID_MOVE_SPEED_DEG_S, ARM_GRID_POSES, ARM_GRID_TOUCH_WAIT_MS,
    ARM_INIT_PITCH1_DEG, ARM_INIT_PITCH2_DEG, ARM_INIT_PITCH3_DEG,
    ARM_INIT_ROLL_DEG, AUTO_BOARD_ACQUIRE_TIMEOUT_MS,
    AUTO_BOARD_COMMAND_TIMEOUT_MS, AUTO_BOARD_TIMEOUT_MS,
    AUTO_CAMERA_FORWARD_ANGLE_DEG, AUTO_CAMERA_SETTLE_OBSERVATIONS,
    AUTO_CAMERA_SETTLE_TIMEOUT_MS, AUTO_CAMERA_SETTLE_TOLERANCE_DEG,
    AUTO_CAMERA_VERIFY_INTERVAL_MS,
    AUTO_CORNER_CONFIRM_OBSERVATIONS, AUTO_CORNER_SIDE_MIN_PIXELS,
    AUTO_CORNER_SIDE_RATIO_X100, AUTO_DOCK_RANGE_DEADZONE_PX,
    AUTO_DOCK_SPEED_RAD_S, AUTO_DOCK_X_DEADZONE_PX, AUTO_DRIVE_ACC_RAD_S2,
    AUTO_GRID_DOCK_STABLE_OBSERVATIONS, AUTO_GRID_DOCK_TARGET_HEIGHT_PX_480,
    AUTO_GRID_ENTRY_HEIGHT_PX_480, AUTO_GRID_LAYOUT_STABLE_OBSERVATIONS,
    AUTO_GRID_SETTLE_MS, AUTO_GRID_TARGET_TIMEOUT_MS,
    AUTO_GROUND_ALIGN_ANGLE_DEADZONE_X10, AUTO_GROUND_ALIGN_BOTTOM_DEADZONE_PX_480,
    AUTO_GROUND_ALIGN_CENTER_Y_DEADZONE_PX_480,
    AUTO_GROUND_ALIGN_BOTTOM_RATIO_X1000, AUTO_GROUND_ALIGN_MARGIN_X_PX_640,
    AUTO_GROUND_ALIGN_MARGIN_Y_PX_480, AUTO_GROUND_ALIGN_PIVOT_SPEED_RAD_S,
    AUTO_GROUND_ALIGN_PIVOT_SIGN,
    AUTO_GROUND_ALIGN_STABLE_OBSERVATIONS, AUTO_GROUND_ALIGN_TIMEOUT_MS,
    AUTO_GROUND_ALIGN_SEARCH_SPEED_RAD_S, AUTO_GROUND_ALIGN_TRANSLATE_SPEED_RAD_S,
    AUTO_GROUND_ALIGN_X_DEADZONE_PX_640,
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
    def ground(packet):
        """把部分/完整九宫格斜框转换成 L2 搜索与最终对正误差。"""
        board = packet["value"]
        if board is None:
            return None
        fw, fh = packet["frame_w"], packet["frame_h"]
        margin_x = max(1, fw * AUTO_GROUND_ALIGN_MARGIN_X_PX_640 // 640)
        margin_y = max(1, fh * AUTO_GROUND_ALIGN_MARGIN_Y_PX_480 // 480)
        corners = board.get("corners") or [[board["x"], board["y"]],
                                           [board["x"] + board["w"], board["y"]],
                                           [board["x"] + board["w"], board["y"] + board["h"]],
                                           [board["x"], board["y"] + board["h"]]]
        bottom = max(point[1] for point in corners)
        target_bottom = fh * AUTO_GROUND_ALIGN_BOTTOM_RATIO_X1000 // 1000
        complete = bool(board.get("complete", True))
        target_x = board.get("center_x", -1) if complete else -1
        target_y = board.get("center_y", -1) if complete else -1
        if target_x < 0 or target_y < 0:
            target_x, target_y = board["cx"], board["cy"]
        return {
            "dx": (target_x - fw // 2) * 640 // fw,
            "center_dy": (fh // 2 - target_y) * 480 // fh,
            "bottom_error": (target_bottom - bottom) * 480 // fh,
            "angle_x10": board.get("angle_x10", 0),
            "angle_valid": bool(board.get("angle_valid", False)),
            "observed": board.get("observed", 9), "complete": complete,
            "contained": all(margin_x <= p[0] <= fw - margin_x and
                             margin_y <= p[1] <= fh - margin_y for p in corners),
            "too_large": (board["w"] > fw - 2 * margin_x or
                          board["h"] > fh - 2 * margin_y),
        }

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
        self.active, self.mode, self.session = False, "idle", "idle"
        self.task, self.task_index, self.completed = (), 0, []
        self.cached_task, self.cached_qr_payload = (), ""
        self.last_qr_version = self.last_line_seq = self.last_grid_seq = -1
        self.corner, self.corner_count, self.aligned_count = "none", 0, 0
        self.turn_started = self.board_seen = self.deadline = 0
        self.dock_stable = self.layout_stable = 0
        self.ground_stable = 0
        self.layout_key, self.target_missing_since, self.pending_cell = None, 0, None
        self.after_camera_mode, self.camera_check_at = "idle", 0
        self.camera_stable = 0
        self.board_message_seen = self.board_invalid_reported = False
        self.fault_reason = None

    def _write(self, text):
        self.serial.write(str(text).encode("utf-8"))

    def observe_qr(self, data):
        """持续缓存最新有效二维码；运行中的任务一旦锁定便不再被覆盖。"""
        version = data.get("qr_version", 0)
        if version == self.last_qr_version:
            return False
        self.last_qr_version = version
        raw = data.get("qr")
        if not raw:
            return False
        task = parse_task(raw.get("payload", ""))
        if not task:
            print("二维码任务格式无效，已保留上一个有效任务")
            return False
        self.cached_task = task
        self.cached_qr_payload = raw.get("payload", "")
        print("二维码任务已缓存:", self.cached_task)
        if self.active and self.session == "full" and not self.task:
            self.task, self.task_index, self.completed = tuple(task), 0, []
            print("完整自动任务已锁定:", self.task)
        return True

    def _prepare_start(self, session, mode, task=()):
        self.rover.stop()
        self.rover.center_chassis_servos()
        self.rover.servo_control.set_camera_angle(AUTO_CAMERA_FORWARD_ANGLE_DEG, speed_deg_s=30)
        self.active, self.mode, self.session = True, "camera_settle", session
        self.after_camera_mode = mode
        self.task, self.task_index, self.completed = tuple(task), 0, []
        self.last_line_seq = self.last_grid_seq = -1
        self.corner, self.corner_count, self.aligned_count = "none", 0, 0
        self.dock_stable = self.layout_stable = 0
        self.ground_stable = 0
        self.layout_key, self.pending_cell = None, None
        self.target_missing_since = 0
        self.camera_stable = 0
        self.camera_check_at = ticks_ms()
        self.deadline = ticks_add(ticks_ms(), AUTO_CAMERA_SETTLE_TIMEOUT_MS)
        self.board_message_seen = self.board_invalid_reported = False
        self.fault_reason = None
        self.arm.cancel()
        self._write("@STREAM_START\n")
        print("相机回自动标定角 %.1f deg，读回确认前车辆保持停车"
              % AUTO_CAMERA_FORWARD_ANGLE_DEG)

    def start_full(self, data):
        """使用已缓存二维码任务，从稳定九宫格获取阶段启动。"""
        if self.active:
            return False, "already_active"
        if not getattr(self.rover, "motors_enabled", True):
            return False, "motors_disabled"
        if not self.cached_task:
            return False, "qr_task_missing"
        data["blocks"] = data["board"] = None
        self._prepare_start("full", "acquire_board", self.cached_task)
        print("AUTO完整模式启动：二维码任务已锁定，等待稳定九宫格（不执行巡线）")
        return True, None

    def start_dock(self, data):
        """
        从白纸获取/对位阶段启动。

        有缓存任务时在对正后继续选块和轻触；无任务时只对正并停车。
        """
        if self.active:
            return False, "already_active"
        if not getattr(self.rover, "motors_enabled", True):
            return False, "motors_disabled"
        self._prepare_start("dock", "acquire_board", self.cached_task)
        if self.task:
            print("局部自动启动：任务已锁定，先等待相机到位")
        else:
            print("手动标定对正启动：无二维码任务，先等待相机到位")
        return True, None

    def start_ground_align(self, data):
        """L2：只把白色地块摆正到取放固定姿态，不执行二维码或机械臂。"""
        if self.active:
            return False, "already_active"
        if not getattr(self.rover, "motors_enabled", True):
            return False, "motors_disabled"
        self._prepare_start("ground_align", "acquire_board")
        print("L2 九色块外框自动对正启动：等待完整九宫格后低速平移、旋转")
        return True, None

    def cancel(self, reason="manual_stop"):
        self.rover.stop()
        self.arm.cancel()
        was_active = self.active
        self.active, self.mode, self.session = False, "idle", "idle"
        self.task, self.task_index, self.completed = (), 0, []
        self.pending_cell = None
        self.after_camera_mode = "idle"
        if was_active:
            print("AUTO停止:", reason)

    def _fail(self, reason):
        self.rover.stop()
        self.arm.cancel()
        self.active, self.mode = True, "fault"
        self.fault_reason = reason
        print("AUTO故障:", reason)

    @staticmethod
    def _fresh(value, timeout=AUTO_LINE_COMMAND_TIMEOUT_MS):
        return value is not None and ticks_diff(ticks_ms(), value.get("rx_ms", 0)) <= timeout

    def _begin_turn(self, direction):
        self.rover.stop()
        speed = -AUTO_PIVOT_SPEED_RAD_S if direction == "left" else AUTO_PIVOT_SPEED_RAD_S
        self.rover.pivot_turn(speed, acc_rad_s2=AUTO_DRIVE_ACC_RAD_S2)
        self.turn_started, self.aligned_count, self.mode = ticks_ms(), 0, "turn"

    def _grid_packets(self, data):
        blocks, board = data.get("blocks"), data.get("board")
        if not blocks or not board or blocks["sequence"] != board["sequence"]:
            return None, None
        if (not self._fresh(blocks, AUTO_BOARD_COMMAND_TIMEOUT_MS) or
                not self._fresh(board, AUTO_BOARD_COMMAND_TIMEOUT_MS)):
            return None, None
        return blocks, board

    def _update_camera_settle(self, data):
        """停车等待相机到标定角；只接受舵机到位后的新视觉帧。"""
        self.rover.stop()
        now = ticks_ms()
        if ticks_diff(now, self.camera_check_at) < 0:
            return
        self.camera_check_at = ticks_add(now, AUTO_CAMERA_VERIFY_INTERVAL_MS)
        angle = self.rover.servo_control.read_camera_angle()
        if angle is not None:
            angle = float(angle)
            if self.rover.arm is not None:
                self.rover.arm.camera_angle_deg = angle
            if abs(angle - AUTO_CAMERA_FORWARD_ANGLE_DEG) <= AUTO_CAMERA_SETTLE_TOLERANCE_DEG:
                self.camera_stable += 1
            else:
                self.camera_stable = 0
        else:
            self.camera_stable = 0

        if self.camera_stable >= AUTO_CAMERA_SETTLE_OBSERVATIONS:
            next_mode = self.after_camera_mode
            self.mode, self.after_camera_mode = next_mode, "idle"
            # 清除相机运动期间采集的几何量，确保控制只使用到位后的新帧。
            data["line"] = data["blocks"] = data["board"] = None
            self.last_line_seq = self.last_grid_seq = -1
            if next_mode == "acquire_board":
                self.deadline = ticks_add(now, AUTO_BOARD_ACQUIRE_TIMEOUT_MS)
                self.board_message_seen = self.board_invalid_reported = False
                print("相机已到自动标定角，开始等待新的白纸识别结果（最长 %d ms）"
                      % AUTO_BOARD_ACQUIRE_TIMEOUT_MS)
            else:
                print("相机已到自动标定角，开始完整自动流程")
            return

        if ticks_diff(now, self.deadline) >= 0:
            if angle is None:
                print("相机舵机无角度读回，请检查 ID 8、舵机 UART 和供电")
            else:
                print("相机舵机未到位：readback=%.1f, target=%.1f"
                      % (angle, AUTO_CAMERA_FORWARD_ANGLE_DEG))
            self._fail("camera_settle_timeout")

    def _update_acquire_board(self, data):
        self.rover.stop()
        board = data.get("board")
        if self._fresh(board, AUTO_BOARD_COMMAND_TIMEOUT_MS):
            self.board_message_seen = True
            if (board["value"] is not None and
                    board["value"].get("complete", False)):
                self.board_seen, self.dock_stable = ticks_ms(), 0
                self.ground_stable = 0
                self.last_grid_seq = -1
                self.deadline = ticks_add(ticks_ms(), AUTO_GROUND_ALIGN_TIMEOUT_MS)
                self.mode = "ground_align"
                print("已识别稳定九宫格：开始角度归零和中心重合对正")
                return
            if not self.board_invalid_reported:
                self.board_invalid_reported = True
                print("已收到 BOARD_RAW，但白纸未通过视觉检测；请查看 MaixCAM 黄色白纸框")
        if ticks_diff(ticks_ms(), self.deadline) >= 0:
            if self.board_message_seen:
                self._fail("board_not_detected_timeout")
            else:
                print("未收到新的 BOARD_RAW，请检查 MaixCAM 应用、UART交叉接线和共地")
                self._fail("board_link_timeout")

    def _update_ground_align(self, data):
        """角度 -> 横移 -> 前后逐项收敛，避免同时动作造成低速底盘抖动。"""
        board = data.get("board")
        now = ticks_ms()
        if (not self._fresh(board, AUTO_BOARD_COMMAND_TIMEOUT_MS) or
                board["value"] is None or
                not board["value"].get("complete", False)):
            self.rover.stop()
            if ticks_diff(now, self.board_seen) > AUTO_BOARD_TIMEOUT_MS:
                self._fail("ground_lost")
            return
        if ticks_diff(now, self.deadline) >= 0:
            self._fail("ground_align_timeout")
            return
        self.board_seen = now
        value = VisionMath.ground(board)
        if value["too_large"]:
            self._fail("ground_too_large")
            return
        if value["complete"] and not value["angle_valid"]:
            self._fail("ground_angle_unavailable")
            return
        new = board["sequence"] != self.last_grid_seq
        if new:
            self.last_grid_seq = board["sequence"]
        if (value["angle_valid"] and
                abs(value["angle_x10"]) > AUTO_GROUND_ALIGN_ANGLE_DEADZONE_X10):
            action = "pivot_right" if value["angle_x10"] > 0 else "pivot_left"
            speed = AUTO_GROUND_ALIGN_PIVOT_SPEED_RAD_S
            self.rover.pivot_turn((speed if value["angle_x10"] > 0 else -speed) *
                                  AUTO_GROUND_ALIGN_PIVOT_SIGN,
                                  acc_rad_s2=AUTO_DRIVE_ACC_RAD_S2)
            self.ground_stable = 0
        elif abs(value["dx"]) > AUTO_GROUND_ALIGN_X_DEADZONE_PX_640:
            action = "right" if value["dx"] > 0 else "left"
            speed = AUTO_GROUND_ALIGN_TRANSLATE_SPEED_RAD_S
            self.rover.drive(speed, 90 if value["dx"] > 0 else -90,
                             acc_rad_s2=AUTO_DRIVE_ACC_RAD_S2)
            self.ground_stable = 0
        elif not value["complete"]:
            action = "search_backward_%d" % value["observed"]
            self.rover.drive(-AUTO_GROUND_ALIGN_SEARCH_SPEED_RAD_S, 0,
                             acc_rad_s2=AUTO_DRIVE_ACC_RAD_S2)
            self.ground_stable = 0
        elif abs(value["center_dy"]) > AUTO_GROUND_ALIGN_CENTER_Y_DEADZONE_PX_480:
            forward = value["center_dy"] > 0
            action = "center_forward" if forward else "center_backward"
            speed = AUTO_GROUND_ALIGN_TRANSLATE_SPEED_RAD_S
            self.rover.drive(speed if forward else -speed, 0,
                             acc_rad_s2=AUTO_DRIVE_ACC_RAD_S2)
            self.ground_stable = 0
        elif not value["contained"]:
            # 中心已经对齐但外框仍贴边时只后退缩小画面；不再把九宫格
            # 下边缘固定到 90% 高度，避免与“九宫格中心居中”目标互相冲突。
            action = "backward_for_margin"
            speed = AUTO_GROUND_ALIGN_TRANSLATE_SPEED_RAD_S
            self.rover.drive(-speed, 0,
                             acc_rad_s2=AUTO_DRIVE_ACC_RAD_S2)
            self.ground_stable = 0
        else:
            action = "stable"
            self.rover.stop()
            if new:
                self.ground_stable += 1
        if new:
            print("GROUND_ALIGN: seen=%d complete=%d dx=%d dy=%d bottom=%d angle_x10=%d contained=%d action=%s"
                  % (value["observed"], 1 if value["complete"] else 0, value["dx"],
                     value["center_dy"], value["bottom_error"], value["angle_x10"],
                     1 if value["contained"] else 0, action))
        if self.ground_stable >= AUTO_GROUND_ALIGN_STABLE_OBSERVATIONS:
            self.rover.stop()
            self.rover.center_chassis_servos()
            if self.session == "ground_align":
                self.active, self.mode = False, "aligned"
                print("L2 九宫格中心自动对正完成；电机保持使能，可继续手动操作")
            elif self.session == "dock" and not self.task:
                self.rover.disable()
                self.mode = "aligned"
                print("九宫格中心对正完成，底盘电机已失能；按 L3 返回手动")
            else:
                self.deadline = ticks_add(ticks_ms(), AUTO_GRID_SETTLE_MS)
                self.mode = "settle"
                print("九宫格中心对正完成，继续执行任务")

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
        if (not self._fresh(board, AUTO_BOARD_COMMAND_TIMEOUT_MS) or
                board["value"] is None or
                not board["value"].get("complete", False)):
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
            action = "pivot_left" if geometry["dx"] < 0 else "pivot_right"
            speed = AUTO_PIVOT_SPEED_RAD_S * 0.65
            self.rover.pivot_turn(-speed if geometry["dx"] < 0 else speed,
                                  acc_rad_s2=AUTO_DRIVE_ACC_RAD_S2)
            self.dock_stable = 0
        elif abs(geometry["range_error"]) > AUTO_DOCK_RANGE_DEADZONE_PX:
            action = "backward" if geometry["range_error"] < 0 else "forward"
            speed = AUTO_DOCK_SPEED_RAD_S
            self.rover.drive(-speed if geometry["range_error"] < 0 else speed, 0,
                             acc_rad_s2=AUTO_DRIVE_ACC_RAD_S2)
            self.dock_stable = 0
        else:
            action = "stable"
            self.rover.stop()
            if new:
                self.dock_stable += 1
        if new:
            print(
                "AUTO_DOCK: dx=%d px, height_480=%d, target=%d, "
                "range_error=%d, action=%s"
                % (
                    geometry["dx"], geometry["height_480"],
                    AUTO_GRID_DOCK_TARGET_HEIGHT_PX_480,
                    geometry["range_error"], action,
                )
            )
        if self.dock_stable >= AUTO_GRID_DOCK_STABLE_OBSERVATIONS:
            self.rover.stop()
            if self.session == "dock" and not self.task:
                self.rover.disable()
                self.mode = "aligned"
                print("白纸对正完成，底盘电机已失能；按 L3 返回手动标定")
            else:
                self.deadline, self.mode = ticks_add(ticks_ms(), AUTO_GRID_SETTLE_MS), "settle"

    def _update_grid(self, data):
        blocks, board = self._grid_packets(data)
        if (not board or board["value"] is None or
                not board["value"].get("complete", False)):
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
            print("二维码任务完成，车辆保持停车")
        else:
            self.layout_key, self.layout_stable = None, 0
            self.deadline, self.mode = ticks_add(ticks_ms(), AUTO_GRID_SETTLE_MS), "settle"

    def update(self, data):
        self.observe_qr(data)
        if not self.active:
            return
        if self.mode == "camera_settle":
            self._update_camera_settle(data)
        elif self.mode == "acquire_board":
            self._update_acquire_board(data)
        elif self.mode == "dock":
            self._update_dock(data)
        elif self.mode == "ground_align":
            self._update_ground_align(data)
        elif self.mode == "settle":
            self.rover.stop()
            if ticks_diff(ticks_ms(), self.deadline) >= 0:
                self.last_grid_seq, self.layout_key, self.layout_stable = -1, None, 0
                self.mode = "grid"
        elif self.mode == "grid":
            self._update_grid(data)
        elif self.mode == "arm":
            self._update_arm()
        elif self.mode in ("fault", "complete", "aligned"):
            self.rover.stop()
