"""对正后九宫格颜色快照的稳定窗口测试。"""

import os
import sys
import unittest


ROOT = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
MAIX_DIR = os.path.join(ROOT, "MaixCAM")
if MAIX_DIR not in sys.path:
    sys.path.insert(0, MAIX_DIR)

from vision_stability import GridStabilizer


def board():
    return {
        "complete": 1,
        "cx": 150,
        "cy": 150,
        "center_x": 150,
        "center_y": 150,
        "angle_x10": 0,
        "points": [[0, 0], [300, 0], [300, 300], [0, 300]],
    }


def blocks():
    colors = ("yellow", "red", "blue")
    return [
        {"cx": 50 + column * 100, "cy": 50 + row * 100,
         "color": colors[column]}
        for row in range(3) for column in range(3)
    ]


class GridSnapshotStabilityTests(unittest.TestCase):
    def test_only_exposes_layout_after_new_stable_window(self):
        stabilizer = GridStabilizer(required=3)
        for _ in range(2):
            self.assertEqual(0, stabilizer.update(blocks(), board())["complete"])
            self.assertIsNone(stabilizer.get_stable_layout())

        self.assertEqual(1, stabilizer.update(blocks(), board())["complete"])
        self.assertEqual(("yellow", "red", "blue") * 3,
                         stabilizer.get_stable_layout())

        stabilizer.reset()
        self.assertIsNone(stabilizer.get_stable_layout())
        self.assertEqual(0, stabilizer.update(blocks(), board())["complete"])
        self.assertIsNone(stabilizer.get_stable_layout())


if __name__ == "__main__":
    unittest.main()
