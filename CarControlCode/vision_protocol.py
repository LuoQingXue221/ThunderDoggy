"""ESP32端原始视觉串口协议解析。"""

import time

VALID_COLORS = ("red", "yellow", "blue", "pink", "purple")
_BLOCK_LOG_QUANTUM_PX = 8


def _now():
    return time.ticks_ms() if hasattr(time, "ticks_ms") else int(time.time() * 1000)


def _ticks_diff(a, b):
    return time.ticks_diff(a, b) if hasattr(time, "ticks_diff") else a - b


def new_camera_data():
    started = _now()
    return {"qr": None, "line": None, "blocks": None, "board": None,
            "calibration": None,
            "status": None,
            "target": None, "qr_version": 0, "line_version": 0,
            "blocks_version": 0, "board_version": 0, "status_version": 0,
            "calibration_version": 0,
            "last_message": "", "last_kind": "", "link_started_ms": started,
            "last_rx_ms": 0, "last_byte_ms": 0, "vision_online": False,
            "rx_frames": 0, "rx_bytes": 0, "rx_errors": 0}


def _frame_size(width, height):
    if not (80 <= width <= 1920 and 60 <= height <= 1080):
        raise ValueError("视觉分辨率越界")


def parse_vision_message(line):
    text = str(line).strip()
    if not text.startswith("@"):
        return None
    head = text.split(",", 1)[0].lower()
    parts = [v.strip().lower() for v in text.split(",")]

    if head == "@vision_status" and len(parts) == 4:
        protocol, sequence, streaming = (int(v) for v in parts[1:])
        if protocol <= 0 or streaming not in (0, 1):
            raise ValueError("VISION_STATUS字段错误")
        return "status", {"protocol": protocol, "sequence": sequence,
                          "streaming": bool(streaming)}

    if head == "@qr_raw" and len(parts) == 3:
        return "qr", {"sequence": int(parts[1]), "payload": parts[2]}

    if head == "@grid_reference" and len(parts) == 26:
        values = [int(v) for v in parts[1:]]
        return "calibration", {"values": values}

    if head == "@line_raw" and len(parts) == 21:
        seq, fw, fh = int(parts[1]), int(parts[2]), int(parts[3])
        _frame_size(fw, fh)
        points, index = [], 4
        for _ in range(3):
            cx, cy, w, h, pixels = (int(v) for v in parts[index:index + 5])
            index += 5
            points.append(None if cx < 0 else {"cx": cx, "cy": cy, "w": w,
                                                "h": h, "pixels": pixels})
        return "line", {"sequence": seq, "frame_w": fw, "frame_h": fh,
                        "points": points, "left_pixels": int(parts[19]),
                        "right_pixels": int(parts[20])}

    if head == "@blocks_raw" and len(parts) >= 5:
        seq, fw, fh, count = map(int, parts[1:5])
        _frame_size(fw, fh)
        if count < 0 or count > 9 or len(parts) != 5 + count * 6:
            raise ValueError("BLOCKS_RAW数量错误")
        blocks, index = [], 5
        for _ in range(count):
            color = parts[index]
            x, y, w, h, pixels = (int(v) for v in parts[index + 1:index + 6])
            index += 6
            if color not in VALID_COLORS or w <= 0 or h <= 0 or pixels < 0:
                raise ValueError("物块字段错误")
            blocks.append({"color": color, "x": x, "y": y, "w": w,
                           "h": h, "pixels": pixels, "cx": x + w // 2,
                           "cy": y + h // 2})
        return "blocks", {"sequence": seq, "frame_w": fw,
                          "frame_h": fh, "items": blocks}

    if head == "@board_raw" and len(parts) == 22:
        seq, fw, fh, valid = (int(v) for v in parts[1:5])
        observed, complete, cx, cy, center_x, center_y = (int(v) for v in parts[5:11])
        angle_x10, angle_valid = int(parts[11]), int(parts[12])
        corners = [[int(parts[i]), int(parts[i + 1])] for i in range(13, 21, 2)]
        pixels = int(parts[21])
        _frame_size(fw, fh)
        if valid:
            xs, ys = [p[0] for p in corners], [p[1] for p in corners]
            board = {"x": min(xs), "y": min(ys), "w": max(xs) - min(xs),
                     "h": max(ys) - min(ys), "pixels": pixels, "cx": cx, "cy": cy,
                     "center_x": center_x, "center_y": center_y, "corners": corners,
                     "observed": observed, "complete": bool(complete),
                     "angle_x10": angle_x10, "angle_valid": bool(angle_valid)}
        else:
            board = None
        return "board", {"sequence": seq, "frame_w": fw, "frame_h": fh, "value": board}

    if head == "@board_raw" and len(parts) in (10, 12):
        seq, fw, fh, valid, x, y, w, h, pixels = (int(v) for v in parts[1:10])
        angle_x10, angle_valid = (0, 0) if len(parts) == 10 else (int(parts[10]), int(parts[11]))
        _frame_size(fw, fh)
        board = None if not valid else {"x": x, "y": y, "w": w, "h": h,
                                        "pixels": pixels, "cx": x + w // 2,
                                        "cy": y + h // 2, "angle_x10": angle_x10,
                                        "angle_valid": bool(angle_valid)}
        return "board", {"sequence": seq, "frame_w": fw,
                         "frame_h": fh, "value": board}
    raise ValueError("未知视觉帧")


def apply_vision_message(data, message, raw_line):
    if message is None:
        return False
    kind, value = message
    now = _now()
    value["rx_ms"] = now
    duplicate_qr = (kind == "qr" and data.get("qr") is not None
                    and data["qr"].get("payload") == value.get("payload"))
    data[kind] = value
    if not duplicate_qr:
        data[kind + "_version"] = data.get(kind + "_version", 0) + 1
    data["last_message"] = raw_line
    data["last_kind"] = kind
    data["last_rx_ms"] = now
    data["vision_online"] = True
    data["rx_frames"] = data.get("rx_frames", 0) + 1
    return not duplicate_qr


def mark_vision_bytes(data, count):
    if count <= 0:
        return
    data["rx_bytes"] = data.get("rx_bytes", 0) + int(count)
    data["last_byte_ms"] = _now()


def mark_vision_error(data):
    data["rx_errors"] = data.get("rx_errors", 0) + 1


def vision_link_timed_out(data, timeout_ms, now=None):
    now = _now() if now is None else now
    reference = data.get("last_rx_ms", 0) or data.get("link_started_ms", now)
    return _ticks_diff(now, reference) > int(timeout_ms)


def blocks_log_signature(packet):
    """生成抗轻微像素抖动的色块日志签名，避免终端重复刷屏。"""
    if packet is None:
        return ()
    return tuple(
        (
            item["color"],
            item["cx"] // _BLOCK_LOG_QUANTUM_PX,
            item["cy"] // _BLOCK_LOG_QUANTUM_PX,
            item["w"] // _BLOCK_LOG_QUANTUM_PX,
            item["h"] // _BLOCK_LOG_QUANTUM_PX,
        )
        for item in packet.get("items", ())
    )


def format_recognition_result(message):
    """把二维码或色块原始识别结果格式化为 ESP32 终端日志。"""
    if message is None:
        return None
    kind, value = message
    if kind == "qr":
        return "视觉二维码识别: seq=%d, payload=%s" % (
            value["sequence"], value["payload"],
        )
    if kind != "blocks":
        return None
    items = value.get("items", ())
    if not items:
        return "视觉色块识别: seq=%d, count=0（色块已消失）" % value["sequence"]
    details = []
    for item in items:
        details.append(
            "%s center=(%d,%d) size=%dx%d pixels=%d"
            % (
                item["color"], item["cx"], item["cy"],
                item["w"], item["h"], item["pixels"],
            )
        )
    return "视觉色块识别: seq=%d, count=%d, %s" % (
        value["sequence"], len(items), "; ".join(details),
    )
