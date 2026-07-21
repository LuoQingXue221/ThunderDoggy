"""MaixCAM 白纸尺寸过滤的桌面测试。"""

import importlib.util
import os
import sys
import types
import unittest


ROOT = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
MODULE_PATH = os.path.join(ROOT, "MaixCAM", "sekuai.py")


class FakeImageConstants:
    COLOR_RED = 1
    COLOR_YELLOW = 2
    COLOR_BLUE = 3


fake_maix = types.ModuleType("maix")
fake_maix.image = FakeImageConstants
previous_maix = sys.modules.get("maix")
sys.modules["maix"] = fake_maix
try:
    spec = importlib.util.spec_from_file_location("sekuai_under_test", MODULE_PATH)
    sekuai = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(sekuai)
finally:
    if previous_maix is None:
        del sys.modules["maix"]
    else:
        sys.modules["maix"] = previous_maix


class FakeFrame:
    def __init__(self, blob):
        self.blob = blob

    def width(self):
        return 640

    def height(self):
        return 480

    def find_blobs(self, *args, **kwargs):
        return [] if self.blob is None else [self.blob]


class BoardFilterTests(unittest.TestCase):
    def test_square_board_at_default_target_size_is_accepted(self):
        board = sekuai.detect_board(FakeFrame([190, 100, 260, 260, 50000]))
        self.assertIsNotNone(board)
        self.assertEqual(260, board["h"])

    def test_more_distant_board_is_accepted_for_initial_approach(self):
        board = sekuai.detect_board(FakeFrame([230, 150, 180, 140, 18000]))
        self.assertIsNotNone(board)

    def test_board_smaller_than_acquire_threshold_is_rejected(self):
        board = sekuai.detect_board(FakeFrame([245, 180, 150, 110, 12000]))
        self.assertIsNone(board)


def grid_item(color, column, row, pixels=500):
    return {"color": color, "draw_color": 0,
            "x": 85 + column * 100, "y": 85 + row * 100,
            "w": 30, "h": 30, "pixels": pixels,
            "cx": 100 + column * 100, "cy": 100 + row * 100}


class GridColorRepairTests(unittest.TestCase):
    def test_wrong_counts_are_repaired_by_spatial_column_vote(self):
        items = [
            grid_item("red", 0, 0), grid_item("red", 0, 1),
            grid_item("pink", 0, 2),
            grid_item("blue", 1, 0), grid_item("blue", 1, 1),
            grid_item("blue", 1, 2),
            grid_item("yellow", 2, 0), grid_item("yellow", 2, 1),
            grid_item("red", 2, 2),
        ]
        repaired = sekuai._force_grid_color_counts(items)
        self.assertIsNotNone(repaired)
        self.assertEqual({"red": 3, "blue": 3, "yellow": 3},
                         sekuai._color_counts(repaired))
        self.assertEqual(2, sum(item.get("color_forced", 0) for item in repaired))
        self.assertTrue(sekuai._select_grid(repaired))

    def test_valid_three_by_three_counts_are_unchanged(self):
        items = [grid_item(color, column, row)
                 for column, color in enumerate(("red", "blue", "yellow"))
                 for row in range(3)]
        self.assertIs(items, sekuai._force_grid_color_counts(items))

    def test_missing_color_evidence_is_not_fabricated(self):
        items = [grid_item("red" if column != 1 else "blue", column, row)
                 for column in range(3) for row in range(3)]
        self.assertIsNone(sekuai._force_grid_color_counts(items))


if __name__ == "__main__":
    unittest.main()
