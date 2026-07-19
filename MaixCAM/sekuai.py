"""白纸和固定五色色块视觉检测；不计算偏移、距离、格子或抓取顺序。"""

from maix import image

FRAME_WIDTH, FRAME_HEIGHT = 640, 480
COLORS = (
    ("red", [[15, 80, 29, 80, 4, 60]], image.COLOR_RED),
    ("yellow", [[35, 100, -25, 25, 28, 110]], image.COLOR_YELLOW),
    ("blue", [[15, 80, -30, 7, -80, -12]], image.COLOR_BLUE),
    ("pink", [[40, 100, 8, 28, -15, 28]], image.COLOR_RED),
    ("purple", [[10, 80, 8, 55, -70, -5]], image.COLOR_BLUE),
)
PAPER = [[55, 100, -20, 20, -20, 25]]


def _size(img):
    return int(img.width()), int(img.height())


def detect_blocks(img):
    fw, fh = _size(img)
    small, large = max(7, min(fw, fh) // 50), max(70, min(fw, fh) // 2)
    found = []
    for name, thresholds, color in COLORS:
        blobs = img.find_blobs(
            thresholds, roi=[0, 0, fw, fh], area_threshold=max(50, fw * fh // 2500),
            pixels_threshold=max(30, fw * fh // 6000), merge=True, margin=4,
        )
        for b in blobs:
            x, y, w, h, pixels, cx, cy = b[:7]
            area = w * h
            if (w < small or h < small or w > large or h > large or area <= 0
                    or w * 100 < h * 40 or w * 100 > h * 250
                    or pixels * 100 < area * 14):
                continue
            found.append({"color": name, "draw_color": color, "x": x, "y": y,
                          "w": w, "h": h, "pixels": pixels, "cx": cx, "cy": cy})
    unique = []
    for item in sorted(found, key=lambda v: v["pixels"], reverse=True):
        same = any(abs(item["cx"] - old["cx"]) <= min(item["w"], old["w"])
                   and abs(item["cy"] - old["cy"]) <= min(item["h"], old["h"])
                   for old in unique)
        if not same:
            unique.append(item)
    return unique[:9]


def detect_board(img):
    fw, fh = _size(img)
    blobs = img.find_blobs(
        PAPER, roi=[0, 0, fw, fh], area_threshold=max(1200, fw * fh // 40),
        pixels_threshold=max(600, fw * fh // 100), merge=True, margin=6,
    )
    candidates = [b for b in blobs if b[2] >= fw // 4 and b[3] >= fh // 4
                  and not (b[2] > fw * 95 // 100 and b[3] > fh * 90 // 100)]
    if not candidates:
        return None
    b = max(candidates, key=lambda v: v[4])
    return {"x": b[0], "y": b[1], "w": b[2], "h": b[3], "pixels": b[4]}


def draw(img, blocks, board):
    if board is not None:
        img.draw_rect(board["x"], board["y"], board["w"], board["h"], image.COLOR_YELLOW, 2)
    for b in blocks:
        img.draw_rect(b["x"], b["y"], b["w"], b["h"], b["draw_color"], 3)
        img.draw_string(b["x"], max(0, b["y"] - 16), b["color"], b["draw_color"])
