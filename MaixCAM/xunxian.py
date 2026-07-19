"""青绿色线路视觉检测；只输出原始色块矩形和左右区域像素数。"""

from maix import image

LINE_THRESHOLDS = [[30, 92, -58, -4, -38, 16]]


def _size(img):
    return int(img.width()), int(img.height())


def _rois(w, h):
    return (
        [0, h * 76 // 100, w, h * 22 // 100],
        [0, h * 56 // 100, w, h * 18 // 100],
        [0, h * 36 // 100, w, h * 17 // 100],
    )


def _blobs(img, roi):
    w, h = _size(img)
    return img.find_blobs(
        LINE_THRESHOLDS, roi=roi, area_threshold=max(20, w * h // 5000),
        pixels_threshold=max(12, w * h // 9000), merge=True, margin=5,
    )


def detect_line(img):
    w, h = _size(img)
    rois, points = _rois(w, h), []
    for roi in rois:
        blobs = _blobs(img, roi)
        if blobs:
            b = max(blobs, key=lambda v: v[4])
            points.append({"cx": b[5], "cy": b[6], "w": b[2], "h": b[3], "pixels": b[4]})
        else:
            points.append(None)
    left_roi = [0, h * 31 // 100, w * 45 // 100, h * 43 // 100]
    right_roi = [w * 55 // 100, h * 31 // 100, w * 45 // 100, h * 43 // 100]
    left = sum(b[4] for b in _blobs(img, left_roi))
    right = sum(b[4] for b in _blobs(img, right_roi))
    return {"frame_w": w, "frame_h": h, "points": points,
            "left_pixels": left, "right_pixels": right, "rois": rois}


def draw_line(img, result):
    for roi in result["rois"]:
        img.draw_rect(roi[0], roi[1], roi[2], roi[3], image.COLOR_WHITE, 1)
    for point in result["points"]:
        if point is not None:
            img.draw_rect(point["cx"] - 4, point["cy"] - 4, 8, 8, image.COLOR_GREEN, 2)
