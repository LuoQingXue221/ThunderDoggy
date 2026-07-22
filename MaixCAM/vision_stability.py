"""Validate a 3x3 grid across frames before vehicle control may use it."""


def _median(values):
    values = sorted(int(value) for value in values)
    return values[len(values) // 2]


class GridStabilizer:
    def __init__(self, required=5, center_jitter=10, corner_jitter=16,
                 angle_jitter_x10=35, miss_tolerance=2):
        self.required = max(3, int(required))
        self.center_jitter = int(center_jitter)
        self.corner_jitter = int(corner_jitter)
        self.angle_jitter = int(angle_jitter_x10)
        self.miss_tolerance = max(0, int(miss_tolerance))
        self.reset()

    def reset(self):
        self.layout = None
        self.stable_layout = None
        self.frames = []
        self.misses = 0

    def _note_miss(self):
        """保留最多两帧历史；更长的中断才丢弃稳定窗口。"""
        self.misses += 1
        if self.misses > self.miss_tolerance:
            self.layout = None
            self.stable_layout = None
            self.frames = []

    @staticmethod
    def _layout_key(blocks, board):
        if len(blocks) != 9:
            return None
        p0, p1, p3 = board["points"][0], board["points"][1], board["points"][3]
        ux, uy = p1[0] - p0[0], p1[1] - p0[1]
        vx, vy = p3[0] - p0[0], p3[1] - p0[1]
        determinant = ux * vy - uy * vx
        if abs(determinant) < 100:
            return None
        cells = []
        for block in blocks:
            dx, dy = block["cx"] - p0[0], block["cy"] - p0[1]
            column = max(0, min(2, int(3 * (dx * vy - dy * vx) / determinant)))
            row = max(0, min(2, int(3 * (ux * dy - uy * dx) / determinant)))
            cells.append((row, column, block["color"]))
        if len(set((row, column) for row, column, _ in cells)) != 9:
            return None
        return tuple(color for _, _, color in sorted(cells))

    def _near(self, old, new):
        if abs(old["cx"] - new["cx"]) > self.center_jitter:
            return False
        if abs(old["cy"] - new["cy"]) > self.center_jitter:
            return False
        if abs(old["angle_x10"] - new["angle_x10"]) > self.angle_jitter:
            return False
        for old_point, new_point in zip(old["points"], new["points"]):
            if (abs(old_point[0] - new_point[0]) > self.corner_jitter or
                    abs(old_point[1] - new_point[1]) > self.corner_jitter):
                return False
        return True

    def update(self, blocks, board):
        if board is None:
            self._note_miss()
            return None
        output = dict(board)
        output["complete"] = 0
        layout = self._layout_key(blocks, board)
        if not board.get("complete") or layout is None:
            self._note_miss()
            return output
        self.misses = 0
        if layout != self.layout or (self.frames and not self._near(self.frames[-1], board)):
            self.layout, self.stable_layout, self.frames = layout, None, []
        self.frames.append(board)
        if len(self.frames) > self.required:
            self.frames.pop(0)
        if len(self.frames) < self.required:
            return output
        output["cx"] = _median(frame["cx"] for frame in self.frames)
        output["cy"] = _median(frame["cy"] for frame in self.frames)
        output["center_x"] = _median(frame["center_x"] for frame in self.frames)
        output["center_y"] = _median(frame["center_y"] for frame in self.frames)
        output["angle_x10"] = _median(frame["angle_x10"] for frame in self.frames)
        output["points"] = [
            [_median(frame["points"][index][axis] for frame in self.frames)
             for axis in range(2)] for index in range(4)
        ]
        output["complete"] = 1
        self.stable_layout = tuple(self.layout)
        return output

    def get_stable_layout(self):
        """返回已通过连续帧验证的行优先颜色布局；未稳定时返回None。"""
        return None if self.stable_layout is None else tuple(self.stable_layout)
