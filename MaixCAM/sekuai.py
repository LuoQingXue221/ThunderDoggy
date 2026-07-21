"""白色场地内五色色块检测与完整九宫格拟合。"""

import math
from maix import image

FRAME_WIDTH, FRAME_HEIGHT = 640, 480
GROUND_WHITE = [[52, 95, -10, 10, -10, 12]]
COLORS = (
    # L 覆盖明暗变化；A/B 仅小幅放宽，白色外圈检查负责抑制地毯误检。
    # 实地截图红块顶面：L=29~37、A=38~44、B=22~32；
    # 保留少量余量，但用 A>=32 排除 A 主要低于 30 的棕红地毯。
    # 红块顶面 A 通常为 35~43；红褐地面 A 主要为 27~31。
    # 只提取高饱和红色顶面，避免红块与背景相接时合并为巨大色块。
    ("red", [[18, 48, 35, 60, 10, 45]], image.COLOR_RED),
    ("yellow", [[25, 100, -16, 20, 12, 88]], image.COLOR_YELLOW),
    ("blue", [[3, 95, -36, 22, -100, -7]], image.COLOR_BLUE),
    ("pink", [[25, 100, 6, 36, -20, 30]], image.COLOR_RED),
    ("purple", [[-7, 42, 5, 42, -36, 4]], image.COLOR_BLUE),
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


def _white_supported(img, item):
    """旋转无关的白纸环形投票，允许远处顶面和一个方向的阴影。"""
    fw, fh = _size(img)
    x, y, w, h = item["x"], item["y"], item["w"], item["h"]
    gap = max(8, min(w, h) // 3)

    def white(px, py):
        rgb = img.get_pixel(max(0, min(fw - 1, px)), max(0, min(fh - 1, py)), True)
        if len(rgb) < 3:
            return False
        low, high = min(rgb[0], rgb[1], rgb[2]), max(rgb[0], rgb[1], rgb[2])
        return low >= 55 and high - low <= 35

    cx, cy = x + w / 2.0, y + h / 2.0
    ring = []
    for index in range(16):
        angle = 2.0 * math.pi * index / 16.0
        px = int(cx + (w / 2.0 + gap) * math.cos(angle))
        py = int(cy + (h / 2.0 + gap) * math.sin(angle))
        ring.append(white(px, py))
    opposite = any(ring[index] and ring[index + 8] for index in range(8))
    return sum(ring) >= 5 and opposite


def _color_counts(items):
    """Return detected block counts keyed by color name."""
    counts = {}
    for item in items:
        name = item["color"]
        counts[name] = counts.get(name, 0) + 1
    return counts


def _draw_color(name, fallback):
    for color_name, _, draw_color in COLORS:
        if color_name == name:
            return draw_color
    return fallback


def _force_grid_color_counts(items):
    """Repair a nine-block color split to three distinct colors of three blocks.

    Competition grids use one color per spatial column.  When threshold overlap
    mislabels one or two blocks, choose the distinct color assignment with the
    strongest per-column vote, recolor the whole column, then require the normal
    3x3 geometry validator to accept the repaired result.  Missing blocks,
    surplus candidates, fewer than three observed colors, and non-grid layouts
    are deliberately not fabricated.
    """
    if len(items) != 9:
        return None
    counts = _color_counts(items)
    if len(counts) == 3 and sorted(counts.values()) == [3, 3, 3]:
        return items
    colors = list(counts)
    if len(colors) < 3:
        return None

    # At the calibrated oblique camera pose the three grid columns remain
    # separated on X.  The full rotation-independent geometry check below is
    # still the final authority, so a bad X grouping cannot become valid data.
    ordered = sorted(items, key=lambda value: value["cx"])
    columns = [sorted(ordered[index:index + 3], key=lambda value: value["cy"])
               for index in (0, 3, 6)]
    best = None
    for left in colors:
        for middle in colors:
            if middle == left:
                continue
            for right in colors:
                if right == left or right == middle:
                    continue
                assignment = (left, middle, right)
                votes = [sum(1 for item in column if item["color"] == name)
                         for column, name in zip(columns, assignment)]
                # Never invent a column color that has no supporting detection.
                if min(votes) == 0:
                    continue
                pixels = sum(item.get("pixels", 0)
                             for column, name in zip(columns, assignment)
                             for item in column if item["color"] == name)
                score = (sum(votes), pixels)
                if best is None or score > best[0]:
                    best = (score, assignment)
    if best is None:
        return None

    repaired = []
    for column, name in zip(columns, best[1]):
        for source in column:
            item = source.copy()
            if item["color"] != name:
                item["original_color"] = item["color"]
                item["color"] = name
                item["draw_color"] = _draw_color(name, item["draw_color"])
                item["color_forced"] = 1
            repaired.append(item)
    repaired_counts = _color_counts(repaired)
    if len(repaired_counts) != 3 or sorted(repaired_counts.values()) != [3, 3, 3]:
        return None
    return repaired if _select_grid(repaired) else None


def detect_blocks(img, ground=None):
    """只检测实际白色区域包围的方块，不把白区外接矩形当作掩码。"""
    fw, fh = _size(img)
    if ground is None:
        return []
    rx, ry, rw, rh = ground["x"], ground["y"], ground["w"], ground["h"]
    if rw < 30 or rh < 30:
        return []
    min_w, min_h = max(9, rw * 3 // 100), max(9, rh * 3 // 100)
    min_pixels = max(20, min_w * min_h // 6)
    # 单块即使旋转也不应占到白区的四分之一；双块合并框必须在此处淘汰。
    max_w, max_h = max(24, rw * 24 // 100), max(28, rh * 32 // 100)
    max_area = rw * rh * 6 // 100
    found = []
    for name, thresholds, color in COLORS:
        for b in img.find_blobs(thresholds, roi=[rx, ry, rw, rh], area_threshold=min_pixels,
                                pixels_threshold=min_pixels, merge=False, margin=1):
            x, y, w, h, pixels, cx, cy = b[0], b[1], b[2], b[3], b[4], b[5], b[6]
            area = w * h
            if (w < min_w or h < min_h or w > max_w or h > max_h or not area or area > max_area or
                    pixels < min_pixels or w * 100 < h * 40 or w * 100 > h * 235 or
                    pixels * 100 < area * 28):
                continue
            found.append({"color": name, "draw_color": color, "x": x, "y": y,
                          "w": w, "h": h, "pixels": pixels, "cx": cx, "cy": cy})
    unique = []
    for item in sorted(found, key=lambda v: v["pixels"], reverse=True):
        if not any(abs(item["cx"] - old["cx"]) * 3 < max(item["w"], old["w"]) * 2 and
                   abs(item["cy"] - old["cy"]) * 3 < max(item["h"], old["h"]) * 2
                   for old in unique):
            unique.append(item)
    # 先去重再做像素采样，减少每帧 get_pixel 调用次数。
    unique = [item for item in unique if _white_supported(img, item)]
    repaired = _force_grid_color_counts(unique)
    if repaired is not None:
        unique = repaired
    completed = _infer_missing_blocks(unique)
    selected = _select_grid(completed)
    # 调试显示允许返回不完整候选；车控是否可用仍只由 board.complete 决定。
    return selected if selected else unique[:9]


def _inferred_item(sample, x, y):
    return {"color": sample["color"], "draw_color": sample["draw_color"],
            "x": int(x - sample["w"] / 2), "y": int(y - sample["h"] / 2),
            "w": sample["w"], "h": sample["h"], "pixels": 0,
            "cx": int(x), "cy": int(y), "inferred": 1}


def _infer_missing_blocks(items):
    """两列完整且第三列已有两个同色块时，按3x3投影补出缺失点。"""
    if len(items) != 8:
        return items
    groups = {}
    for item in items:
        groups.setdefault(item["color"], []).append(item)
    if sorted(len(v) for v in groups.values()) != [2, 3, 3]:
        return items
    full = [sorted(v, key=lambda p: p["cy"]) for v in groups.values() if len(v) == 3]
    partial = next(v for v in groups.values() if len(v) == 2)
    full.sort(key=lambda v: sum(p["cx"] for p in v))
    left, right = full
    lx = sum(p["cx"] for p in left) / 3.0
    rx = sum(p["cx"] for p in right) / 3.0
    px = sum(p["cx"] for p in partial) / 2.0
    if abs(rx - lx) < 8:
        return items
    ratio = (px - lx) / (rx - lx)
    if ratio < -1.35 or ratio > 2.35:
        return items
    predicted = [(left[r]["cx"] + ratio * (right[r]["cx"] - left[r]["cx"]),
                  left[r]["cy"] + ratio * (right[r]["cy"] - left[r]["cy"])) for r in range(3)]
    best = None
    for missing in range(3):
        rows = [r for r in range(3) if r != missing]
        for a, b in ((partial[0], partial[1]), (partial[1], partial[0])):
            error = sum((p["cx"] - predicted[r][0]) ** 2 +
                        (p["cy"] - predicted[r][1]) ** 2 for p, r in ((a, rows[0]), (b, rows[1])))
            if best is None or error < best[0]:
                best = (error, missing)
    unit = sorted(min(v["w"], v["h"]) for v in items)[len(items) // 2]
    if best is None or best[0] > 2 * (unit * 1.7) ** 2:
        return items
    return items + [_inferred_item(partial[0], *predicted[best[1]])]


def _point(u, n, s, t):
    return [int(u[0] * s + n[0] * t), int(u[1] * s + n[1] * t)]


def _column_hypotheses(values, unit):
    """Return at most three equally spaced collinear triples at any rotation."""
    result = []
    values = sorted(values, key=lambda v: v["cy"])
    for i in range(len(values) - 2):
        for j in range(i + 1, len(values) - 1):
            for k in range(j + 1, len(values)):
                column = (values[i], values[j], values[k])
                dx1 = column[1]["cx"] - column[0]["cx"]
                dy1 = column[1]["cy"] - column[0]["cy"]
                dx2 = column[2]["cx"] - column[1]["cx"]
                dy2 = column[2]["cy"] - column[1]["cy"]
                d1, d2 = math.sqrt(dx1 * dx1 + dy1 * dy1), math.sqrt(dx2 * dx2 + dy2 * dy2)
                if min(d1, d2) < unit or max(d1, d2) * 10 > min(d1, d2) * 22:
                    continue
                if dx1 * dx2 + dy1 * dy2 <= 0:
                    continue
                cross = abs(dx1 * dy2 - dy1 * dx2)
                if cross * 10 > d1 * d2 * 6:
                    continue
                bend = math.sqrt((column[0]["cx"] - 2 * column[1]["cx"] + column[2]["cx"]) ** 2 +
                                 (column[0]["cy"] - 2 * column[1]["cy"] + column[2]["cy"]) ** 2)
                result.append((int(abs(d1 - d2) + bend * 2), column))
    result.sort(key=lambda value: value[0])
    # 保留更多候选列交给完整3x3评分；避免一个大面积假色块挤掉真实列。
    return result[:8]


def _select_grid(items):
    """Select one geometrically consistent 3x3 lattice; reject all other candidates."""
    if len(items) < 9:
        return []
    unit = max(5, sorted(min(v["w"], v["h"]) for v in items)[len(items) // 2])
    by_color = {}
    for item in items:
        by_color.setdefault(item["color"], []).append(item)
    colors = [(name, _column_hypotheses(values, unit))
              for name, values in by_color.items() if len(values) >= 3]
    colors = [(name, hypotheses) for name, hypotheses in colors if hypotheses]
    best = None
    for i in range(len(colors) - 2):
        for j in range(i + 1, len(colors) - 1):
            for k in range(j + 1, len(colors)):
                for left in colors[i][1]:
                    for middle in colors[j][1]:
                        for right in colors[k][1]:
                            columns = sorted((left, middle, right),
                                             key=lambda value: sum(v["cx"] for v in value[1]))
                            flat = [v for _, column in columns for v in column]
                            if any((flat[a]["cx"] - flat[b]["cx"]) ** 2 +
                                   (flat[a]["cy"] - flat[b]["cy"]) ** 2 < unit * unit
                                   for a in range(9) for b in range(a)):
                                continue
                            centers = [(sum(v["cx"] for v in column) // 3,
                                        sum(v["cy"] for v in column) // 3)
                                       for _, column in columns]
                            hx1, hy1 = centers[1][0] - centers[0][0], centers[1][1] - centers[0][1]
                            hx2, hy2 = centers[2][0] - centers[1][0], centers[2][1] - centers[1][1]
                            hd1 = math.sqrt(hx1 * hx1 + hy1 * hy1)
                            hd2 = math.sqrt(hx2 * hx2 + hy2 * hy2)
                            vx = sum(column[2]["cx"] - column[0]["cx"] for _, column in columns) / 6.0
                            vy = sum(column[2]["cy"] - column[0]["cy"] for _, column in columns) / 6.0
                            vertical = math.sqrt(vx * vx + vy * vy)
                            horizontal = (hd1 + hd2) / 2.0
                            parallel_error = abs(hx1 * hy2 - hy1 * hx2)
                            perpendicular_error = abs((hx1 + hx2) * vx + (hy1 + hy2) * vy)
                            if (min(hd1, hd2) < unit or max(hd1, hd2) * 10 > min(hd1, hd2) * 20 or
                                    parallel_error * 10 > hd1 * hd2 * 6 or
                                    horizontal < vertical * .65 or horizontal > vertical * 2.8 or
                                    perpendicular_error * 10 > 11 * horizontal * vertical):
                                continue
                            row_error = sum(math.sqrt(
                                (columns[0][1][r]["cx"] - 2 * columns[1][1][r]["cx"] +
                                 columns[2][1][r]["cx"]) ** 2 +
                                (columns[0][1][r]["cy"] - 2 * columns[1][1][r]["cy"] +
                                 columns[2][1][r]["cy"]) ** 2) for r in range(3))
                            score = sum(value[0] for value in columns) + abs(hd1 - hd2) + int(row_error * 2)
                            if best is None or score < best[0]:
                                best = (score, [columns[c][1][r] for r in range(3) for c in range(3)])
    return [] if best is None else best[1]


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
    # 完整九宫格按比赛规则拟合：每列同色。先得到三列，再用三行的
    # 平均方向作为九宫格水平轴，避免从任意三个斜向点误选出对角线。
    color_columns = {}
    for item in items:
        color_columns.setdefault(item["color"], []).append(item)
    columns = [sorted(values, key=lambda v: v["cy"])
               for values in color_columns.values() if len(values) == 3]
    if len(items) == 9 and len(columns) == 3:
        columns.sort(key=lambda values: sum(v["cx"] for v in values))
        row_dx = sum(columns[2][row]["cx"] - columns[0][row]["cx"] for row in range(3))
        row_dy = sum(columns[2][row]["cy"] - columns[0][row]["cy"] for row in range(3))
        length = math.sqrt(row_dx * row_dx + row_dy * row_dy)
        if length >= unit:
            ux, uy = row_dx / length, row_dy / length
            if ux < 0:
                ux, uy = -ux, -uy
            u, n = (ux, uy), (-uy, ux)
            values = [(v["cx"] * ux + v["cy"] * uy,
                       v["cx"] * n[0] + v["cy"] * n[1], v) for v in items]
            s0, s1 = min(v[0] for v in values) - unit * .6, max(v[0] for v in values) + unit * .6
            t0, t1 = min(v[1] for v in values) - unit * .6, max(v[1] for v in values) + unit * .6
            points = [_point(u, n, s0, t0), _point(u, n, s1, t0),
                      _point(u, n, s1, t1), _point(u, n, s0, t1)]
            cx, cy = sum(p[0] for p in points) // 4, sum(p[1] for p in points) // 4
            angle = int(math.atan2(uy, ux) * 1800.0 / 3.1415926)
            return {"points": points, "cx": cx, "cy": cy, "center_x": cx,
                    "center_y": cy, "angle_x10": angle, "angle_valid": 1,
                    "observed": sum(0 if v.get("inferred") else 1 for v in items),
                    "complete": 1,
                    "pixels": sum(v["pixels"] for v in items)}
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
    complete = False
    cx, cy = sum(p[0] for p in points) // 4, sum(p[1] for p in points) // 4
    middle = min(items, key=lambda v: (v["cx"] - cx) ** 2 + (v["cy"] - cy) ** 2) if complete else None
    angle = int(math.atan2(u[1], u[0]) * 1800.0 / 3.1415926)
    return {"points": points, "cx": cx, "cy": cy,
            "center_x": middle["cx"] if middle else -1, "center_y": middle["cy"] if middle else -1,
            "angle_x10": angle, "angle_valid": 1, "observed": len(items),
            "complete": 1 if complete else 0, "pixels": sum(v["pixels"] for v in items)}


def detect_board(img, blocks=None):
    items = blocks or []
    selected = _select_grid(items)
    if selected:
        return _fit_grid(selected)
    # 少于九块、混入误检或3x3模型未通过时，不画拟合框也不向车控提供姿态。
    return None


def draw(img, blocks, board, ground=None):
    fw, fh = _size(img)
    img.draw_line(fw // 2 - 10, fh // 2, fw // 2 + 10, fh // 2, image.COLOR_WHITE)
    img.draw_line(fw // 2, fh // 2 - 10, fw // 2, fh // 2 + 10, image.COLOR_WHITE)
    counts = _color_counts(blocks)
    if counts:
        summary = " ".join("%s:%d" % (name, counts[name])
                           for name in sorted(counts))
        img.draw_string(8, 28, summary, image.COLOR_YELLOW)
    if ground:
        img.draw_rect(ground["x"], ground["y"], ground["w"], ground["h"], image.COLOR_WHITE, 1)
    if board:
        points = board["points"]
        for i in range(4):
            img.draw_line(points[i][0], points[i][1], points[(i + 1) % 4][0],
                          points[(i + 1) % 4][1], image.COLOR_GREEN)
        img.draw_string(max(0, board["cx"] - 70), max(0, board["cy"] - 10),
                        "grid %d/9 a=%d dx=%d dy=%d" %
                        (board["observed"], board["angle_x10"],
                         board["cx"] - fw // 2, board["cy"] - fh // 2), image.COLOR_GREEN)
    for b in blocks:
        if b.get("inferred"):
            img.draw_line(b["cx"] - 7, b["cy"], b["cx"] + 7, b["cy"], b["draw_color"])
            img.draw_line(b["cx"], b["cy"] - 7, b["cx"], b["cy"] + 7, b["draw_color"])
            label = b["color"] + "*"
        else:
            img.draw_rect(b["x"], b["y"], b["w"], b["h"], b["draw_color"], 2)
            label = b["color"] + ("!" if b.get("color_forced") else "")
        img.draw_string(b["x"], max(0, b["y"] - 15), label, b["draw_color"])
