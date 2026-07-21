"""五色色块检测、部分九宫格斜框拟合和白色地块背景掩码。"""

import math
from maix import image

FRAME_WIDTH, FRAME_HEIGHT = 640, 480
GROUND_WHITE = [[48, 100, -18, 18, -18, 18]]
COLORS = (
    ("red", [[12, 85, 28, 88, 0, 70]], image.COLOR_RED),
    ("yellow", [[30, 100, -12, 30, 35, 120]], image.COLOR_YELLOW),
    ("blue", [[8, 85, -28, 16, -90, -15]], image.COLOR_BLUE),
    ("pink", [[30, 100, 5, 32, -18, 28]], image.COLOR_RED),
    ("purple", [[5, 85, 5, 55, -85, -5]], image.COLOR_BLUE),
)


def _size(img):
    return int(img.width()), int(img.height())


def detect_ground(img):
    """白色地块仅限制颜色搜索范围，不作为自动对正目标。"""
    fw, fh = _size(img)
    blobs = img.find_blobs(GROUND_WHITE, roi=[0, 0, fw, fh],
                           area_threshold=max(220, fw * fh // 900),
                           pixels_threshold=max(140, fw * fh // 1500), merge=True, margin=8)
    valid = [b for b in blobs if b[2] >= fw * 10 // 100 and b[3] >= fh * 8 // 100
             and b[4] * 100 >= b[2] * b[3] * 18]
    if not valid:
        return None
    b = max(valid, key=lambda v: v[4])
    return {"x": b[0], "y": b[1], "w": b[2], "h": b[3]}


def detect_blocks(img, ground=None):
    """在白色地块附近检测 7 px 以上且小于地块约三成的方块。"""
    fw, fh = _size(img)
    if ground is None:
        # 白区识别失败时仍进行受尺寸上限保护的全画面搜索，避免整条链路归零。
        rx, ry, rw, rh = 0, 0, fw, fh
    else:
        rx, ry, rw, rh = ground["x"], ground["y"], ground["w"], ground["h"]
    if rw < 30 or rh < 30:
        return []
    small, max_w, max_h = max(7, min(rw, rh) // 90), max(20, rw * 27 // 100), max(24, rh * 36 // 100)
    found = []
    for name, thresholds, color in COLORS:
        for b in img.find_blobs(thresholds, roi=[rx, ry, rw, rh], area_threshold=14,
                                pixels_threshold=14, merge=True, margin=2):
            x, y, w, h, pixels, cx, cy = b[0], b[1], b[2], b[3], b[4], b[5], b[6]
            area = w * h
            if (w < small or h < small or w > max_w or h > max_h or not area or
                    w * 100 < h * 40 or w * 100 > h * 235 or pixels * 100 < area * 20):
                continue
            found.append({"color": name, "draw_color": color, "x": x, "y": y,
                          "w": w, "h": h, "pixels": pixels, "cx": cx, "cy": cy})
    unique = []
    for item in sorted(found, key=lambda v: v["pixels"], reverse=True):
        if not any(abs(item["cx"] - old["cx"]) * 3 < max(item["w"], old["w"]) * 2 and
                   abs(item["cy"] - old["cy"]) * 3 < max(item["h"], old["h"]) * 2
                   for old in unique):
            unique.append(item)
    if len(unique) <= 9:
        return unique
    # 多余候选通常是地块外接矩形四角中的背景；保留离白区中心最近的九个。
    gx = ground["x"] + ground["w"] // 2 if ground else fw // 2
    gy = ground["y"] + ground["h"] // 2 if ground else fh // 2
    return sorted(unique, key=lambda v: (v["cx"] - gx) ** 2 + (v["cy"] - gy) ** 2)[:9]


def _point(u, n, s, t):
    return [int(u[0] * s + n[0] * t), int(u[1] * s + n[1] * t)]


def _fit_grid(items):
    """以最靠下且较大的块为近点，找同一直线色块并拟合旋转矩形。"""
    if not items:
        return None
    near = max(items, key=lambda v: v["cy"] * 4 + min(v["w"], v["h"]))
    unit = max(5, sorted(min(v["w"], v["h"]) for v in items)[len(items) // 2])
    if len(items) == 1:
        x, y, w, h = near["x"], near["y"], near["w"], near["h"]
        return {"points": [[x, y], [x + w, y], [x + w, y + h], [x, y + h]],
                "cx": near["cx"], "cy": near["cy"], "center_x": -1, "center_y": -1,
                "angle_x10": 0, "angle_valid": 0, "observed": 1, "complete": 0,
                "pixels": near["pixels"]}
    best = None
    for other in items:
        dx, dy = other["cx"] - near["cx"], other["cy"] - near["cy"]
        length = math.sqrt(dx * dx + dy * dy)
        if length < unit or abs(dx) * 3 < abs(dy):
            continue
        ux, uy = dx / length, dy / length
        if ux < 0:
            ux, uy = -ux, -uy
        hits = [v for v in items if abs((v["cx"] - near["cx"]) * (-uy) +
                                        (v["cy"] - near["cy"]) * ux) <= unit]
        score = len(hits) * 1000 + int(abs(ux) * 100) - int(length / max(1, unit))
        if best is None or score > best[0]:
            best = (score, ux, uy)
    if best is None:
        return None
    u, n = (best[1], best[2]), (-best[2], best[1])
    values = [(v["cx"] * u[0] + v["cy"] * u[1],
               v["cx"] * n[0] + v["cy"] * n[1], v) for v in items]
    s0, s1 = min(v[0] for v in values) - unit * 0.6, max(v[0] for v in values) + unit * 0.6
    t0, t1 = min(v[1] for v in values) - unit * 0.6, max(v[1] for v in values) + unit * 0.6
    points = [_point(u, n, s0, t0), _point(u, n, s1, t0),
              _point(u, n, s1, t1), _point(u, n, s0, t1)]
    groups = []
    for value in sorted(values, key=lambda v: v[1]):
        if not groups or value[1] - sum(v[1] for v in groups[-1]) / len(groups[-1]) > unit * 1.4:
            groups.append([value])
        else:
            groups[-1].append(value)
    complete = len(items) == 9 and len(groups) == 3 and all(len(v) == 3 for v in groups)
    cx, cy = sum(p[0] for p in points) // 4, sum(p[1] for p in points) // 4
    middle = min(items, key=lambda v: (v["cx"] - cx) ** 2 + (v["cy"] - cy) ** 2) if complete else None
    angle = int(math.atan2(u[1], u[0]) * 1800.0 / 3.1415926)
    return {"points": points, "cx": cx, "cy": cy,
            "center_x": middle["cx"] if middle else -1, "center_y": middle["cy"] if middle else -1,
            "angle_x10": angle, "angle_valid": 1, "observed": len(items),
            "complete": 1 if complete else 0, "pixels": sum(v["pixels"] for v in items)}


def detect_board(img, blocks=None):
    return _fit_grid(blocks or [])


def draw(img, blocks, board, ground=None):
    if ground:
        img.draw_rect(ground["x"], ground["y"], ground["w"], ground["h"], image.COLOR_WHITE, 1)
    if board:
        points = board["points"]
        for i in range(4):
            img.draw_line(points[i][0], points[i][1], points[(i + 1) % 4][0],
                          points[(i + 1) % 4][1], image.COLOR_GREEN)
        img.draw_string(max(0, board["cx"] - 35), max(0, board["cy"] - 10),
                        "grid %d/9" % board["observed"], image.COLOR_GREEN)
    for b in blocks:
        img.draw_rect(b["x"], b["y"], b["w"], b["h"], b["draw_color"], 2)
        img.draw_string(b["x"], max(0, b["y"] - 15), b["color"], b["draw_color"])
