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


if __name__ == "__main__":
    unittest.main()
