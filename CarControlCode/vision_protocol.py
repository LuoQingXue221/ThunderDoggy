"""ESP32端原始视觉串口协议解析。"""

import time

VALID_COLORS = ("red", "yellow", "blue", "pink", "purple")


def _now():
    return time.ticks_ms() if hasattr(time, "ticks_ms") else int(time.time() * 1000)


def new_camera_data():
    return {"qr": None, "line": None, "blocks": None, "board": None,
            "target": None, "qr_version": 0, "line_version": 0,
            "blocks_version": 0, "board_version": 0, "last_message": ""}


def _frame_size(width, height):
    if not (80 <= width <= 1920 and 60 <= height <= 1080):
        raise ValueError("视觉分辨率越界")


def parse_vision_message(line):
    text = str(line).strip()
    if not text.startswith("@"):
        return None
    head = text.split(",", 1)[0].lower()
    parts = [v.strip().lower() for v in text.split(",")]

    if head == "@qr_raw" and len(parts) == 3:
        return "qr", {"sequence": int(parts[1]), "payload": parts[2]}

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

    if head == "@board_raw" and len(parts) == 10:
        seq, fw, fh, valid, x, y, w, h, pixels = (int(v) for v in parts[1:])
        _frame_size(fw, fh)
        board = None if not valid else {"x": x, "y": y, "w": w, "h": h,
                                        "pixels": pixels, "cx": x + w // 2,
                                        "cy": y + h // 2}
        return "board", {"sequence": seq, "frame_w": fw,
                         "frame_h": fh, "value": board}
    raise ValueError("未知视觉帧")


def apply_vision_message(data, message, raw_line):
    if message is None:
        return
    kind, value = message
    value["rx_ms"] = _now()
    data[kind] = value
    data[kind + "_version"] = data.get(kind + "_version", 0) + 1
    data["last_message"] = raw_line
