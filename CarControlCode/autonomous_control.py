"""ESP32自动控制：把相机原始观测换算为巡线、对位、格子和机械臂动作。"""

import math
import time

from arm_control import ArmKinematicsError
from robot_config import (
    ARM_GRID_CALIBRATED, ARM_GRID_MOVE_SETTLE_MS, ARM_GRID_MOVE_SPEED_DEG_S,
    ARM_GRID_POSES, ARM_GRIPPER_CLOSED_DEG, ARM_GRIPPER_OPEN_DEG,
    ARM_GRIPPER_SERVO_ID, ARM_GRIPPER_SETTLE_MS, ARM_GRIPPER_SPEED_DEG_S,
    ARM_HOPPER_POSE, AUTO_BOARD_ACQUIRE_TIMEOUT_MS,
    AUTO_BOARD_COMMAND_TIMEOUT_MS, AUTO_BOARD_TIMEOUT_MS,
    AUTO_CAMERA_SETTLE_OBSERVATIONS,
    AUTO_CAMERA_SETTLE_TIMEOUT_MS, AUTO_CAMERA_SETTLE_TOLERANCE_DEG,
    AUTO_CAMERA_VERIFY_INTERVAL_MS,
    AUTO_CORNER_CONFIRM_OBSERVATIONS, AUTO_CORNER_SIDE_MIN_PIXELS,
    AUTO_CORNER_SIDE_RATIO_X100, AUTO_DOCK_RANGE_DEADZONE_PX,
    AUTO_DOCK_SPEED_RAD_S, AUTO_DOCK_X_DEADZONE_PX, AUTO_DRIVE_ACC_RAD_S2,
    AUTO_GRID_DOCK_STABLE_OBSERVATIONS, AUTO_GRID_DOCK_TARGET_HEIGHT_PX_480,
    AUTO_GRID_ENTRY_HEIGHT_PX_480, AUTO_GRID_LAYOUT_STABLE_OBSERVATIONS,
    AUTO_GRID_REFERENCE_ANGLE_X10, AUTO_GRID_REFERENCE_CX_PX_640,
    AUTO_GRID_REFERENCE_CY_PX_480, AUTO_GRID_REFERENCE_HEIGHT_PX_480,
    AUTO_GRID_REFERENCE_LEFT_RIGHT_X1000,
    AUTO_GRID_REFERENCE_PERSPECTIVE_TOLERANCE_X1000,
    AUTO_GRID_REFERENCE_SCALE_TOLERANCE_PERCENT,
    AUTO_GRID_REFERENCE_TOP_BOTTOM_X1000, AUTO_GRID_REFERENCE_WIDTH_PX_640,
    AUTO_GRID_SETTLE_MS, AUTO_GRID_SNAPSHOT_RETRY_MS, AUTO_GRID_SNAPSHOT_TIMEOUT_MS,
    AUTO_GROUND_ALIGN_ANGLE_DEADZONE_X10, AUTO_GROUND_ALIGN_BOTTOM_DEADZONE_PX_480,
    AUTO_GROUND_ALIGN_CENTER_Y_DEADZONE_PX_480,
    AUTO_GROUND_ALIGN_BOTTOM_RATIO_X1000, AUTO_GROUND_ALIGN_MARGIN_X_PX_640,
    AUTO_GROUND_ALIGN_MARGIN_Y_PX_480, AUTO_GROUND_ALIGN_PIVOT_SPEED_RAD_S,
    AUTO_GROUND_ALIGN_PIVOT_SIGN,
    AUTO_GROUND_REACQUIRE_TIMEOUT_MS,
    AUTO_GROUND_SEARCH_INITIAL_WAIT_MS, AUTO_GROUND_SEARCH_MAX_PULSES,
    AUTO_GROUND_SEARCH_MIN_BLOCKS, AUTO_GROUND_SEARCH_PULSE_MS,
    AUTO_GROUND_SEARCH_SETTLE_MS, AUTO_GROUND_SEARCH_SPEED_RAD_S,
    AUTO_GROUND_ALIGN_RELAXED_CENTER_Y_DEADZONE_PX_480,
    AUTO_GROUND_ALIGN_RELAXED_STABLE_OBSERVATIONS,
    AUTO_GROUND_ALIGN_RELAXED_X_DEADZONE_PX_640,
    AUTO_GROUND_ALIGN_STABLE_OBSERVATIONS, AUTO_GROUND_ALIGN_TIMEOUT_MS,
    AUTO_GROUND_ALIGN_TRANSLATE_SPEED_RAD_S,
    AUTO_GROUND_ALIGN_X_DEADZONE_PX_640,
    AUTO_LINE_ALIGN_DEADZONE_PX, AUTO_LINE_COMMAND_TIMEOUT_MS,
    AUTO_LINE_KP_DEG_PER_PX, AUTO_LINE_MAX_STEER_DEG,
    AUTO_LINE_MIN_CONFIDENCE, AUTO_LINE_SPEED_RAD_S, AUTO_PIVOT_SPEED_RAD_S,
    AUTO_TURN_ALIGNED_OBSERVATIONS, AUTO_TURN_MIN_MS, AUTO_TURN_TIMEOUT_MS,
    CAMERA_MANUAL_ANGLE_DEG, CAMERA_VISION_ANGLE_DEG,
    clamp,
)
from vision_protocol import VALID_COLORS


def ticks_ms():
    return time.ticks_ms() if hasattr(time, "ticks_ms") else int(time.time() * 1000)


def ticks_diff(a, b):
    return time.ticks_diff(a, b) if hasattr(time, "ticks_diff") else a - b


def ticks_add(a, b):
    return time.ticks_add(a, int(b)) if hasattr(time, "ticks_add") else a + int(b)


_REASON_TEXT = {
    "ps2_lost": "手柄信号丢失", "select_exit": "操作员退出",
    "servo_reset": "执行全部舵机复位", "motor_disable": "执行急停失能",
    "ps2_start_toggle": "再次按下START", "ps2_l3_toggle": "再次按下L3",
    "camera_settle_timeout": "相机舵机到位超时",
    "board_not_detected_timeout": "九宫格检测超时",
    "board_link_timeout": "九宫格数据通信超时", "ground_lost": "对正时九宫格丢失",
    "ground_align_timeout": "九宫格自动对正超时", "ground_too_large": "九宫格距离过近",
    "ground_angle_unavailable": "九宫格角度不可用", "turn_timeout": "原地转向超时",
    "board_lost": "九宫格丢失", "grid_snapshot_missing": "冻结九宫格快照缺失",
    "already_active": "已有自动流程正在运行", "motors_disabled": "底盘电机未使能",
    "qr_task_missing": "尚未取得二维码任务", "arm_missing": "未连接机械臂",
    "uncalibrated": "机械臂九宫格动作尚未标定", "pose_missing": "目标格位动作缺失",
    "grid_snapshot_timeout": "对正后九宫格颜色快照超时",
    "grid_snapshot_invalid": "对正后九宫格颜色快照无效",
    "task_capacity_exceeded": "二维码请求超过当前两行可抓取容量",
}
_ARM_REASON_TEXT = {
    "read_failed": "舵机角度读取失败", "state_unsynced": "机械臂角度尚未同步",
    "joint_limit": "机械臂目标超出限位",
    "gripper_read_failed": "夹爪角度读取失败", "gripper_unavailable": "夹爪舵机未启用",
}


def describe_reason(reason):
    if reason in _REASON_TEXT:
        return _REASON_TEXT[reason]
    if str(reason).startswith("target_missing_"):
        return "冻结布局中缺少目标颜色：%s" % str(reason)[15:]
    if str(reason).startswith("arm_"):
        detail = str(reason)[4:]
        return "机械臂执行失败：%s" % _ARM_REASON_TEXT.get(detail, detail)
    return str(reason)


def _action_text(action):
    values = {
        "pivot_right": "向右原地旋转", "pivot_left": "向左原地旋转",
        "right": "向右平移", "left": "向左平移",
        "center_forward": "向前对齐中心", "center_backward": "向后对齐中心",
        "backward_for_margin": "后退以保留画面边距",
        "stable_strict": "严格对正稳定中", "stable_relaxed": "实用对正稳定中",
        "backward": "后退", "forward": "前进",
    }
    return values.get(action, str(action))


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
    if any(sequence.count(color) > 2 for color in set(sequence)):
        return None
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
        if (not isinstance(poses, dict) or not self._pose(poses.get("hover")) or
                not self._pose(poses.get("grab"))):
            return False, "pose_missing"
        self.steps = [
            ("move", poses["hover"]),
            ("move", poses["grab"]),
            ("gripper", ARM_GRIPPER_CLOSED_DEG),
            ("move", poses["hover"]),
            ("move", ARM_HOPPER_POSE),
            ("gripper", ARM_GRIPPER_OPEN_DEG),
        ]
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
        action, value = self.steps[self.index]
        if action == "move":
            current = (
                getattr(self.rover.arm, "roll_deg", value[0]),
                getattr(self.rover.arm, "pitch1_deg", value[1]),
                getattr(self.rover.arm, "pitch2_deg", value[2]),
                getattr(self.rover.arm, "pitch3_deg", value[3]),
            )
            delta = max(abs(float(target) - float(old))
                        for target, old in zip(value, current))
            wait = int(delta * 1000.0 / ARM_GRID_MOVE_SPEED_DEG_S) + ARM_GRID_MOVE_SETTLE_MS
            self.rover.arm.move_to_pose(*value, speed_deg_s=ARM_GRID_MOVE_SPEED_DEG_S)
        elif action == "gripper":
            current = self.rover.servo_control.read_reserve_servo_angle(
                ARM_GRIPPER_SERVO_ID)
            if current is None:
                raise ArmKinematicsError(
                    "gripper_read_failed", "读取夹爪舵机角度失败，自动抓取已停止。")
            sent = self.rover.servo_control.set_reserve_servo_angle(
                ARM_GRIPPER_SERVO_ID, value,
                speed_deg_s=ARM_GRIPPER_SPEED_DEG_S)
            if not sent:
                raise ArmKinematicsError(
                    "gripper_unavailable", "夹爪舵机未启用，自动抓取已停止。")
            wait = (int(abs(float(value) - float(current)) * 1000.0 /
                        ARM_GRIPPER_SPEED_DEG_S) + ARM_GRIPPER_SETTLE_MS)
        else:
            raise ArmKinematicsError("invalid_action", "未知机械臂动作。")
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
        def edge_length(a, b):
            return int(math.sqrt((a[0] - b[0]) ** 2 +
                                 (a[1] - b[1]) ** 2) + 0.5)

        top = edge_length(corners[0], corners[1])
        right = edge_length(corners[1], corners[2])
        bottom_edge = edge_length(corners[3], corners[2])
        left = edge_length(corners[0], corners[3])
        width_640 = ((top + bottom_edge) * 320 // max(1, fw))
        height_480 = ((left + right) * 240 // max(1, fh))
        width_error = AUTO_GRID_REFERENCE_WIDTH_PX_640 - width_640
        height_error = AUTO_GRID_REFERENCE_HEIGHT_PX_480 - height_480
        width_as_height = width_error * AUTO_GRID_REFERENCE_HEIGHT_PX_480 // max(
            1, AUTO_GRID_REFERENCE_WIDTH_PX_640)
        range_error = (height_error + width_as_height) // 2
        width_tolerance = max(1, AUTO_GRID_REFERENCE_WIDTH_PX_640 *
                              AUTO_GRID_REFERENCE_SCALE_TOLERANCE_PERCENT // 100)
        height_tolerance = max(1, AUTO_GRID_REFERENCE_HEIGHT_PX_480 *
                               AUTO_GRID_REFERENCE_SCALE_TOLERANCE_PERCENT // 100)
        top_bottom_x1000 = top * 1000 // max(1, bottom_edge)
        left_right_x1000 = left * 1000 // max(1, right)
        perspective_ok = (
            abs(top_bottom_x1000 - AUTO_GRID_REFERENCE_TOP_BOTTOM_X1000) <=
            AUTO_GRID_REFERENCE_PERSPECTIVE_TOLERANCE_X1000 and
            abs(left_right_x1000 - AUTO_GRID_REFERENCE_LEFT_RIGHT_X1000) <=
            AUTO_GRID_REFERENCE_PERSPECTIVE_TOLERANCE_X1000)
        return {
            "dx": (target_x * 640 // fw) - AUTO_GRID_REFERENCE_CX_PX_640,
            "center_dy": AUTO_GRID_REFERENCE_CY_PX_480 - (target_y * 480 // fh),
            "bottom_error": (target_bottom - bottom) * 480 // fh,
            "angle_x10": board.get("angle_x10", 0),
            "angle_error_x10": (board.get("angle_x10", 0) -
                                  AUTO_GRID_REFERENCE_ANGLE_X10),
            "angle_valid": bool(board.get("angle_valid", False)),
            "width_640": width_640, "height_480": height_480,
            "width_error": width_error, "height_error": height_error,
            "range_error": range_error,
            "scale_ok": (abs(width_error) <= width_tolerance and
                         abs(height_error) <= height_tolerance),
            "top_bottom_x1000": top_bottom_x1000,
            "left_right_x1000": left_right_x1000,
            "perspective_ok": perspective_ok,
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
        self.frozen_grid = ()
        self.pick_queue, self.pick_index = (), 0
        self.snapshot_request_id = None
        self.snapshot_request_counter = 0
        self.snapshot_min_version = 0
        self.snapshot_last_request_ms = 0
        self.last_qr_version = self.last_line_seq = self.last_grid_seq = -1
        self.corner, self.corner_count, self.aligned_count = "none", 0, 0
        self.turn_started = self.board_seen = self.deadline = 0
        self.dock_stable = self.layout_stable = 0
        self.ground_stable = 0
        self.ground_stable_required = AUTO_GROUND_ALIGN_STABLE_OBSERVATIONS
        self.ground_missing_since = None
        self.ground_search_phase, self.ground_search_deadline = "idle", 0
        self.ground_search_pulses = 0
        self.last_ground_value = None
        self.layout_key, self.target_missing_since, self.pending_cell = None, 0, None
        self.after_camera_mode, self.camera_check_at = "idle", 0
        self.camera_target_angle = CAMERA_VISION_ANGLE_DEG
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
        self.rover.servo_control.set_camera_angle(CAMERA_VISION_ANGLE_DEG,
                                                  speed_deg_s=30)
        if self.rover.arm is not None:
            self.rover.arm.camera_angle_deg = CAMERA_VISION_ANGLE_DEG
        print("二维码扫描成功：任务已发送，视觉相机回正到 %.1f°"
              % CAMERA_VISION_ANGLE_DEG)
        if self.active and self.session == "full" and not self.task:
            self.task, self.task_index, self.completed = tuple(task), 0, []
            print("完整自动任务已锁定:", self.task)
        return True

    def _start_camera_transition(self, target_angle, next_mode):
        """下发相机绝对角度，并进入带读回确认的停车等待状态。"""
        self.camera_target_angle = float(target_angle)
        self.after_camera_mode = next_mode
        self.camera_stable = 0
        self.camera_check_at = ticks_ms()
        self.deadline = ticks_add(ticks_ms(), AUTO_CAMERA_SETTLE_TIMEOUT_MS)
        self.rover.servo_control.set_camera_angle(self.camera_target_angle,
                                                  speed_deg_s=30)
        if self.rover.arm is not None:
            self.rover.arm.camera_angle_deg = self.camera_target_angle
        self.mode = "camera_settle"

    def _prepare_start(self, session, mode, task=()):
        self.rover.stop()
        self.rover.center_chassis_servos()
        self.active, self.mode, self.session = True, "idle", session
        self.task, self.task_index, self.completed = tuple(task), 0, []
        self.last_line_seq = self.last_grid_seq = -1
        self.corner, self.corner_count, self.aligned_count = "none", 0, 0
        self.dock_stable = self.layout_stable = 0
        self.ground_stable = 0
        self.ground_stable_required = AUTO_GROUND_ALIGN_STABLE_OBSERVATIONS
        self.ground_missing_since = None
        self.ground_search_phase, self.ground_search_deadline = "idle", 0
        self.ground_search_pulses = 0
        self.last_ground_value = None
        self.frozen_grid = ()
        self.pick_queue, self.pick_index = (), 0
        self.snapshot_request_id = None
        self.snapshot_min_version = 0
        self.snapshot_last_request_ms = 0
        self.layout_key, self.pending_cell = None, None
        self.target_missing_since = 0
        self.board_message_seen = self.board_invalid_reported = False
        self.fault_reason = None
        self.arm.cancel()
        self._write("@STREAM_START\n")
        self._start_camera_transition(CAMERA_VISION_ANGLE_DEG, mode)
        print("相机回到视觉角 %.1f°，读回确认前车辆保持停车"
              % CAMERA_VISION_ANGLE_DEG)

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
        print("完整自动模式启动：二维码任务已锁定，等待稳定九宫格（不执行巡线）")
        return True, None

    def start_dock(self, data):
        """
        从白纸获取/对位阶段启动。

        有缓存任务时在对正后获取颜色快照并夹取；无任务时只对正并停车。
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

    def cancel(self, reason="manual_stop"):
        self.rover.stop()
        self.arm.cancel()
        was_active = self.active
        self.active, self.mode, self.session = False, "idle", "idle"
        self.task, self.task_index, self.completed = (), 0, []
        self.frozen_grid = ()
        self.pick_queue, self.pick_index = (), 0
        self.snapshot_request_id = None
        self.snapshot_last_request_ms = 0
        self.pending_cell = None
        self.after_camera_mode = "idle"
        if was_active:
            print("自动流程停止：", describe_reason(reason))

    def _fail(self, reason):
        self.rover.stop()
        self.arm.cancel()
        self.active, self.mode = True, "fault"
        self.fault_reason = reason
        print("自动流程故障：", describe_reason(reason))

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

    @staticmethod
    def _configured_pose(value):
        return (isinstance(value, dict) and GridArmExecutor._pose(value.get("hover")) and
                GridArmExecutor._pose(value.get("grab")))

    def _build_pick_queue(self):
        remaining = {}
        for color in self.task:
            remaining[color] = remaining.get(color, 0) + 1
        if any(count > 2 for count in remaining.values()):
            return False
        colors = {item["cell"]: item["color"] for item in self.frozen_grid}
        queue = []
        for cell in sorted(ARM_GRID_POSES):
            if not self._configured_pose(ARM_GRID_POSES.get(cell)):
                continue
            color = colors.get(cell)
            if color is not None and remaining.get(color, 0) > 0:
                queue.append({"cell": cell, "color": color})
                remaining[color] -= 1
        if any(count > 0 for count in remaining.values()):
            return False
        self.pick_queue, self.pick_index = tuple(queue), 0
        return bool(self.pick_queue)

    def _freeze_grid_colors(self, snapshot):
        """冻结相机在对正后重新采集的显式九格颜色快照。"""
        items = tuple({"cell": (item["row"], item["column"]),
                       "color": item["color"]} for item in snapshot.get("items", ()))
        cells = set(item["cell"] for item in items)
        expected = set((row, column) for row in range(3) for column in range(3))
        if len(items) != 9 or cells != expected:
            return False
        # 比赛规则：每列只放一种颜色；不满足时拒绝冻结错误布局。
        for column in range(3):
            if len(set(item["color"] for item in items
                       if item["cell"][1] == column)) != 1:
                return False
        self.frozen_grid = items
        self.layout_key = tuple(sorted((item["color"], item["cell"][0], item["cell"][1])
                                       for item in self.frozen_grid))
        self.layout_stable = AUTO_GRID_LAYOUT_STABLE_OBSERVATIONS
        return True

    def _begin_grid_snapshot(self, data):
        self.rover.stop()
        self.snapshot_request_counter = (self.snapshot_request_counter + 1) & 0x7FFFFFFF
        if self.snapshot_request_counter == 0:
            self.snapshot_request_counter = 1
        self.snapshot_request_id = self.snapshot_request_counter
        self.snapshot_min_version = data.get("grid_colors_version", 0)
        data["grid_colors"] = None
        self.deadline = ticks_add(ticks_ms(), AUTO_GRID_SNAPSHOT_TIMEOUT_MS)
        self.snapshot_last_request_ms = ticks_ms()
        self.mode = "grid_snapshot_wait"
        self._write("@GRID_SNAPSHOT_REQ,%d\n" % self.snapshot_request_id)
        print("九宫格对正成功：保持停车和视觉角，重新采集颜色快照 request=%d" %
              self.snapshot_request_id)

    def _update_grid_snapshot_wait(self, data):
        self.rover.stop()
        now = ticks_ms()
        snapshot = data.get("grid_colors")
        is_new = data.get("grid_colors_version", 0) > self.snapshot_min_version
        if (is_new and snapshot is not None and
                snapshot.get("request_id") == self.snapshot_request_id):
            if not self._fresh(snapshot, AUTO_GRID_SNAPSHOT_TIMEOUT_MS):
                self._fail("grid_snapshot_invalid")
                return
            if not self._freeze_grid_colors(snapshot):
                self._fail("grid_snapshot_invalid")
                return
            if not self._build_pick_queue():
                self._fail("task_capacity_exceeded")
                return
            print("九宫格颜色快照已冻结，行优先抓取队列:", self.pick_queue)
            self._start_camera_transition(CAMERA_MANUAL_ANGLE_DEG, "grid")
            print("相机开始偏转到 +90°；到位后自动抓取")
            return
        if ticks_diff(now, self.deadline) >= 0:
            self._fail("grid_snapshot_timeout")
            return
        if ticks_diff(now, self.snapshot_last_request_ms) >= AUTO_GRID_SNAPSHOT_RETRY_MS:
            self._write("@GRID_SNAPSHOT_REQ,%d\n" % self.snapshot_request_id)
            self.snapshot_last_request_ms = now

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
            if abs(angle - self.camera_target_angle) <= AUTO_CAMERA_SETTLE_TOLERANCE_DEG:
                self.camera_stable += 1
            else:
                self.camera_stable = 0
        else:
            self.camera_stable = 0

        if self.camera_stable >= AUTO_CAMERA_SETTLE_OBSERVATIONS:
            next_mode = self.after_camera_mode
            self.mode, self.after_camera_mode = next_mode, "idle"
            if next_mode == "acquire_board":
                # 回到视觉角后，只接受相机到位以后产生的新九宫格帧。
                data["line"] = data["blocks"] = data["board"] = None
                self.last_line_seq = self.last_grid_seq = -1
                self.deadline = ticks_add(now, AUTO_BOARD_ACQUIRE_TIMEOUT_MS)
                self.board_message_seen = self.board_invalid_reported = False
                print("相机已回正，开始等待新的九宫格识别结果（最长 %d ms）"
                      % AUTO_BOARD_ACQUIRE_TIMEOUT_MS)
            elif next_mode == "grid":
                self.layout_stable = AUTO_GRID_LAYOUT_STABLE_OBSERVATIONS
                print("相机已偏转到 +90°，开始按冻结九宫格执行抓取")
            elif next_mode == "aligned":
                self.active = False
                print("相机已偏转到 +90°，自动对正流程完成")
            else:
                print("相机舵机已到目标角 %.1f°" % self.camera_target_angle)
            return

        if ticks_diff(now, self.deadline) >= 0:
            if angle is None:
                print("相机舵机无角度读回，请检查 ID 8、舵机 UART 和供电")
            else:
                print("相机舵机未到位：读回=%.1f°，目标=%.1f°"
                      % (angle, self.camera_target_angle))
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
                self.ground_stable_required = AUTO_GROUND_ALIGN_STABLE_OBSERVATIONS
                self.ground_missing_since = None
                self.ground_search_phase, self.ground_search_deadline = "idle", 0
                self.ground_search_pulses = 0
                self.last_ground_value = None
                self.last_grid_seq = -1
                self.deadline = ticks_add(ticks_ms(), AUTO_GROUND_ALIGN_TIMEOUT_MS)
                self.mode = "ground_align"
                print("已识别稳定九宫格：开始角度归零和中心重合对正")
                return
            if not self.board_invalid_reported:
                self.board_invalid_reported = True
            print("已收到九宫格数据，但视觉稳定条件尚未通过")
        if ticks_diff(ticks_ms(), self.deadline) >= 0:
            if self.board_message_seen:
                self._fail("board_not_detected_timeout")
            else:
                print("未收到新的 BOARD_RAW，请检查 MaixCAM 应用、UART交叉接线和共地")
                self._fail("board_link_timeout")

    def _update_ground_align(self, data):
        """角度 -> 横移 -> 中心纵向逐项收敛；尺寸只用于安全保护。"""
        board = data.get("board")
        now = ticks_ms()
        deadline_expired = ticks_diff(now, self.deadline) >= 0
        if (not self._fresh(board, AUTO_BOARD_COMMAND_TIMEOUT_MS) or
                board["value"] is None or
                not board["value"].get("complete", False)):
            self.rover.stop()
            self.ground_stable = 0
            self.ground_stable_required = AUTO_GROUND_ALIGN_STABLE_OBSERVATIONS
            if self.ground_missing_since is None:
                self.ground_missing_since = now
                print("视觉暂时不完整，车辆已停车等待重捕获")
            elif ticks_diff(now, self.ground_missing_since) > AUTO_GROUND_REACQUIRE_TIMEOUT_MS:
                self._fail("ground_lost")
                return
            if deadline_expired:
                self._ground_align_timeout()
                return
            self._update_ground_search(data, now)
            return
        if self.ground_missing_since is not None:
            print("九宫格已重新稳定，继续自动对正")
            self.ground_missing_since = None
        self.ground_search_phase, self.ground_search_deadline = "idle", 0
        self.board_seen = now
        value = VisionMath.ground(board)
        self.last_ground_value = value
        if value["too_large"]:
            self._fail("ground_too_large")
            return
        if value["complete"] and not value["angle_valid"]:
            self._fail("ground_angle_unavailable")
            return
        new = board["sequence"] != self.last_grid_seq
        if new:
            self.last_grid_seq = board["sequence"]
        self.ground_stable_required = AUTO_GROUND_ALIGN_STABLE_OBSERVATIONS

        angle_ok = (value["angle_valid"] and
                    abs(value["angle_error_x10"]) <= AUTO_GROUND_ALIGN_ANGLE_DEADZONE_X10)
        strict = (angle_ok and
                  abs(value["dx"]) <= AUTO_GROUND_ALIGN_X_DEADZONE_PX_640 and
                  abs(value["center_dy"]) <= AUTO_GROUND_ALIGN_CENTER_Y_DEADZONE_PX_480)
        relaxed = (angle_ok and
                   abs(value["dx"]) <= AUTO_GROUND_ALIGN_RELAXED_X_DEADZONE_PX_640 and
                   abs(value["center_dy"]) <=
                   AUTO_GROUND_ALIGN_RELAXED_CENTER_Y_DEADZONE_PX_480)

        if relaxed and value["contained"]:
            action = "stable_strict" if strict else "stable_relaxed"
            required = (AUTO_GROUND_ALIGN_STABLE_OBSERVATIONS if strict else
                        AUTO_GROUND_ALIGN_RELAXED_STABLE_OBSERVATIONS)
            self.rover.stop()
            self.ground_stable_required = required
            if new:
                self.ground_stable += 1
        elif deadline_expired:
            # 到期帧先获得一次完成判定机会；仍不在接受区时才报告超时。
            self._ground_align_timeout()
            return
        elif (value["angle_valid"] and
              abs(value["angle_error_x10"]) > AUTO_GROUND_ALIGN_ANGLE_DEADZONE_X10):
            action = "pivot_right" if value["angle_error_x10"] > 0 else "pivot_left"
            speed = AUTO_GROUND_ALIGN_PIVOT_SPEED_RAD_S
            self.rover.pivot_turn((speed if value["angle_error_x10"] > 0 else -speed) *
                                  AUTO_GROUND_ALIGN_PIVOT_SIGN,
                                  acc_rad_s2=AUTO_DRIVE_ACC_RAD_S2)
            self.ground_stable = 0
        elif abs(value["dx"]) > AUTO_GROUND_ALIGN_RELAXED_X_DEADZONE_PX_640:
            action = "right" if value["dx"] > 0 else "left"
            speed = AUTO_GROUND_ALIGN_TRANSLATE_SPEED_RAD_S
            self.rover.drive(speed, 90 if value["dx"] > 0 else -90,
                             acc_rad_s2=AUTO_DRIVE_ACC_RAD_S2)
            self.ground_stable = 0
        elif abs(value["center_dy"]) > AUTO_GROUND_ALIGN_RELAXED_CENTER_Y_DEADZONE_PX_480:
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
            # 只可能是边界安全条件尚未满足；透视比例仅保留作诊断。
            action = "backward_for_margin"
            self.rover.stop()
            self.ground_stable = 0
        if new:
            print("自动对正：已见=%d 完整=%d 水平差=%d 垂直差=%d 尺寸=%dx%d 角度=%d 角度差=%d 透视=%d/%d 透视诊断=%d 稳定=%d/%d 动作=%s"
                  % (value["observed"], 1 if value["complete"] else 0, value["dx"],
                     value["center_dy"], value["width_640"], value["height_480"],
                     value["angle_x10"], value["angle_error_x10"],
                     value["top_bottom_x1000"], value["left_right_x1000"],
                     1 if value["perspective_ok"] else 0, self.ground_stable,
                     self.ground_stable_required, _action_text(action)))
        if self.ground_stable >= self.ground_stable_required:
            self.rover.stop()
            self.rover.center_chassis_servos()
            if self.session == "dock" and not self.task:
                self.rover.disable()
                self._start_camera_transition(CAMERA_MANUAL_ANGLE_DEG, "aligned")
                print("底盘电机已失能，相机开始偏转到 +90°")
            else:
                self._begin_grid_snapshot(data)
            return
        if deadline_expired:
            self._ground_align_timeout()

    def _update_ground_search(self, data, now):
        """完整框暂失时，仅依靠至少六个真实色块执行限次微前移搜索。"""
        packet = data.get("blocks")
        count = 0
        if self._fresh(packet, AUTO_BOARD_COMMAND_TIMEOUT_MS):
            count = sum(1 for item in packet.get("items", ()) if item.get("pixels", 0) > 0)
        if count < AUTO_GROUND_SEARCH_MIN_BLOCKS:
            if self.ground_search_phase not in ("idle", "insufficient"):
                print("重捕获搜索暂停：真实色块不足%d，车辆保持停车" %
                      AUTO_GROUND_SEARCH_MIN_BLOCKS)
            self.ground_search_phase = "insufficient"
            self.rover.stop()
            return
        if self.ground_search_phase in ("idle", "insufficient"):
            self.ground_search_phase = "wait"
            self.ground_search_deadline = ticks_add(now, AUTO_GROUND_SEARCH_INITIAL_WAIT_MS)
            self.rover.stop()
            print("九宫格暂失但仍见%d个真实色块：停车等待%d毫秒后微前移搜索" %
                  (count, AUTO_GROUND_SEARCH_INITIAL_WAIT_MS))
            return
        if self.ground_search_phase == "wait":
            self.rover.stop()
            if ticks_diff(now, self.ground_search_deadline) < 0:
                return
            if self.ground_search_pulses >= AUTO_GROUND_SEARCH_MAX_PULSES:
                self.ground_search_phase = "exhausted"
                print("微前移搜索已达%d次上限，等待视觉重捕获" %
                      AUTO_GROUND_SEARCH_MAX_PULSES)
                return
            self.ground_search_phase = "pulse"
            self.ground_search_pulses += 1
            self.ground_search_deadline = ticks_add(now, AUTO_GROUND_SEARCH_PULSE_MS)
            print("重捕获微前移：第%d/%d次" %
                  (self.ground_search_pulses, AUTO_GROUND_SEARCH_MAX_PULSES))
        if self.ground_search_phase == "pulse":
            if ticks_diff(now, self.ground_search_deadline) < 0:
                self.rover.drive(AUTO_GROUND_SEARCH_SPEED_RAD_S, 0,
                                 acc_rad_s2=AUTO_DRIVE_ACC_RAD_S2)
                return
            self.rover.stop()
            self.ground_search_phase = "settle"
            self.ground_search_deadline = ticks_add(now, AUTO_GROUND_SEARCH_SETTLE_MS)
            return
        if self.ground_search_phase == "settle":
            self.rover.stop()
            if ticks_diff(now, self.ground_search_deadline) >= 0:
                self.ground_search_phase = "wait"
                self.ground_search_deadline = now
            return
        self.rover.stop()

    def _ground_align_timeout(self):
        """输出最后一次有效姿态，随后进入自动对正超时故障。"""
        value = self.last_ground_value
        if value is None:
            print("自动对正超时诊断：没有可用的稳定九宫格姿态")
        else:
            print("自动对正超时诊断：水平差=%d 垂直差=%d 角度差=%d 边界=%d 透视=%d/%d 透视诊断=%d 稳定=%d/%d"
                  % (value["dx"], value["center_dy"], value["angle_error_x10"],
                     1 if value["contained"] else 0, value["top_bottom_x1000"],
                     value["left_right_x1000"], 1 if value["perspective_ok"] else 0,
                     self.ground_stable, self.ground_stable_required))
        self._fail("ground_align_timeout")

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
                "自动靠近：水平差=%d像素，画面高度=%d，目标高度=%d，"
                "距离差=%d，动作=%s"
                % (
                    geometry["dx"], geometry["height_480"],
                    AUTO_GRID_DOCK_TARGET_HEIGHT_PX_480,
                    geometry["range_error"], _action_text(action),
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
        if len(self.frozen_grid) != 9 or not self.pick_queue:
            self._fail("grid_snapshot_missing")
            return
        if self.pick_index >= len(self.pick_queue):
            self.mode = "complete"
            return
        target = self.pick_queue[self.pick_index]
        print("抓取目标：序号=%d/%d，颜色=%s，格位=%s" %
              (self.pick_index + 1, len(self.pick_queue),
               target["color"], str(target["cell"])))
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
        self.pick_index += 1
        self.task_index = self.pick_index
        if self.pick_index >= len(self.pick_queue):
            self.rover.stop()
            self.mode = "complete"
            print("二维码任务完成，车辆保持停车")
        else:
            # 上一轮结束在料斗开爪位；下一轮直接前往下一个格位悬停。
            self.mode = "grid"

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
        elif self.mode == "grid_snapshot_wait":
            self._update_grid_snapshot_wait(data)
        elif self.mode == "settle":
            self.rover.stop()
            if ticks_diff(ticks_ms(), self.deadline) >= 0:
                self.last_grid_seq = -1
                self.mode = "grid"
        elif self.mode == "grid":
            self._update_grid(data)
        elif self.mode == "arm":
            self._update_arm()
        elif self.mode in ("fault", "complete", "aligned"):
            self.rover.stop()
