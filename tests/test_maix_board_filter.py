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


class BoardFilterTests(unittest.TestCase):
    def test_complete_three_by_three_grid_is_accepted(self):
        items = [grid_item(color, column, row)
                 for column, color in enumerate(("red", "blue", "yellow"))
                 for row in range(3)]
        board = sekuai.detect_board(None, items)
        self.assertIsNotNone(board)
        self.assertTrue(board["complete"])
        self.assertEqual(9, board["observed"])

    def test_incomplete_grid_is_rejected(self):
        items = [grid_item("red", 0, row) for row in range(3)]
        board = sekuai.detect_board(None, items)
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
        matches = {(row, column): item for column in range(3) for row in range(3)
                   for item in items
                   if item["cx"] == 100 + column * 100 and
                   item["cy"] == 100 + row * 100}
        self.assertEqual(("red", "blue", "yellow"),
                         sekuai._column_colors(matches))

    def test_valid_three_by_three_counts_are_unchanged(self):
        items = [grid_item(color, column, row)
                 for column, color in enumerate(("red", "blue", "yellow"))
                 for row in range(3)]
        matches = {(row, column): item for item in items
                   for column in range(3) for row in range(3)
                   if item["cx"] == 100 + column * 100 and
                   item["cy"] == 100 + row * 100}
        self.assertEqual(("red", "blue", "yellow"),
                         sekuai._column_colors(matches))

    def test_missing_color_evidence_is_not_fabricated(self):
        items = [grid_item("red" if column != 1 else "blue", column, row)
                 for column in range(3) for row in range(3)]
        matches = {(row, column): item for item in items
                   for column in range(3) for row in range(3)
                   if item["cx"] == 100 + column * 100 and
                   item["cy"] == 100 + row * 100}
        self.assertIsNone(sekuai._column_colors(matches))


if __name__ == "__main__":
    unittest.main()
