"""MaixCAM BOARD_RAW 发送格式到 ESP32 解析器的端到端桌面测试。"""

import importlib.util
import os
import sys
import types
import unittest


ROOT = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
CONTROL_DIR = os.path.join(ROOT, "CarControlCode")
CHUANKOU_PATH = os.path.join(ROOT, "MaixCAM", "chuankou.py")
QR_PATH = os.path.join(ROOT, "MaixCAM", "code.py")
if CONTROL_DIR not in sys.path:
    sys.path.insert(0, CONTROL_DIR)

from vision_protocol import (apply_vision_message, blocks_log_signature,
                             format_recognition_result, new_camera_data,
                             parse_vision_message, vision_link_timed_out)


class FakeErr:
    @staticmethod
    def check_raise(*_args, **_kwargs):
        pass


class FakePinmap:
    @staticmethod
    def set_pin_function(*_args, **_kwargs):
        return 0


class FakeUartModule:
    class UART:
        pass


fake_maix = types.ModuleType("maix")
fake_maix.err = FakeErr
fake_maix.pinmap = FakePinmap
fake_maix.uart = FakeUartModule
fake_maix.image = types.SimpleNamespace(COLOR_GREEN=1)
previous_maix = sys.modules.get("maix")
sys.modules["maix"] = fake_maix
try:
    spec = importlib.util.spec_from_file_location("chuankou_under_test", CHUANKOU_PATH)
    chuankou = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(chuankou)
    qr_spec = importlib.util.spec_from_file_location("qr_under_test", QR_PATH)
    qr_module = importlib.util.module_from_spec(qr_spec)
    qr_spec.loader.exec_module(qr_module)
finally:
    if previous_maix is None:
        del sys.modules["maix"]
    else:
        sys.modules["maix"] = previous_maix


class CaptureUart:
    def __init__(self):
        self.text = ""

    def write_str(self, value):
        self.text += value
        return len(value)


def sender():
    link = chuankou.VisionSerial.__new__(chuankou.VisionSerial)
    link.uart = CaptureUart()
    link.buffer = ""
    link.tx_frames = 0
    link.tx_bytes = 0
    return link


class FakeQr:
    def __init__(self, text):
        self.text = text

    def w(self):
        return 80

    def h(self):
        return 80

    def payload(self):
        return self.text


class FakeQrFrame:
    def __init__(self, text):
        self.code = FakeQr(text)

    def find_qrcodes(self):
        return [self.code]


class BoardLinkTests(unittest.TestCase):
    def test_maix_uart_matches_known_working_uart0_wiring(self):
        self.assertEqual("/dev/ttyS0", chuankou.DEVICE)
        self.assertEqual(("A16", "A17"), (chuankou.TX_PIN, chuankou.RX_PIN))

    def test_status_frame_marks_link_online_and_timeout_is_detectable(self):
        link = sender()
        link.send_status(30, True)
        data = new_camera_data()
        data["link_started_ms"] = 100
        raw = link.uart.text.strip()
        changed = apply_vision_message(data, parse_vision_message(raw), raw)
        self.assertTrue(changed)
        self.assertTrue(data["vision_online"])
        self.assertEqual(chuankou.PROTOCOL_VERSION, data["status"]["protocol"])
        self.assertFalse(vision_link_timed_out(data, 2500, data["last_rx_ms"] + 2500))
        self.assertTrue(vision_link_timed_out(data, 2500, data["last_rx_ms"] + 2501))

    def test_valid_board_round_trip(self):
        link = sender()
        link.send_board(30, 640, 480, {
            "x": 190, "y": 80, "w": 260, "h": 260, "pixels": 50000,
        })
        data = new_camera_data()
        raw = link.uart.text.strip()
        apply_vision_message(data, parse_vision_message(raw), raw)
        self.assertEqual(30, data["board"]["sequence"])
        self.assertEqual(260, data["board"]["value"]["h"])
        self.assertEqual(320, data["board"]["value"]["cx"])

    def test_invalid_board_round_trip(self):
        link = sender()
        link.send_board(40, 640, 480, None)
        data = new_camera_data()
        raw = link.uart.text.strip()
        apply_vision_message(data, parse_vision_message(raw), raw)
        self.assertEqual(40, data["board"]["sequence"])
        self.assertIsNone(data["board"]["value"])

    def test_qr_recognition_result_format(self):
        message = parse_vision_message("@QR_RAW,15,blue yellow red 1 2 1")
        text = format_recognition_result(message)
        self.assertIn("seq=15", text)
        self.assertIn("payload=blue yellow red 1 2 1", text)

    def test_qr_round_trip_deduplicates_retries(self):
        link = sender()
        data = new_camera_data()
        link.send_qr(15, "blue yellow red 1 2 1")
        raw = link.uart.text.strip()
        self.assertTrue(apply_vision_message(data, parse_vision_message(raw), raw))
        self.assertEqual(1, data["qr_version"])

        retry = "@QR_RAW,45,blue yellow red 1 2 1"
        self.assertFalse(apply_vision_message(data, parse_vision_message(retry), retry))
        self.assertEqual(1, data["qr_version"])
        self.assertEqual(2, data["rx_frames"])

    def test_grid_colors_round_trip_is_explicit_and_deduplicated(self):
        link = sender()
        layout = ("yellow", "red", "blue") * 3
        link.send_grid_colors(7, 42, layout)
        raw = link.uart.text.strip()
        message = parse_vision_message(raw)
        data = new_camera_data()

        self.assertTrue(apply_vision_message(data, message, raw))
        self.assertEqual(7, data["grid_colors"]["request_id"])
        self.assertEqual(
            [(0, 0, "yellow"), (0, 1, "red"), (0, 2, "blue"),
             (1, 0, "yellow"), (1, 1, "red"), (1, 2, "blue"),
             (2, 0, "yellow"), (2, 1, "red"), (2, 2, "blue")],
            [(item["row"], item["column"], item["color"])
             for item in data["grid_colors"]["items"]],
        )
        self.assertFalse(apply_vision_message(data, message, raw))
        self.assertEqual(1, data["grid_colors_version"])

    def test_grid_colors_rejects_duplicate_coordinate(self):
        fields = ["@GRID_COLORS", "1", "2", "9"]
        for index in range(9):
            row, column = divmod(index, 3)
            if index == 8:
                row, column = 0, 0
            fields += [str(row), str(column), "blue"]
        with self.assertRaises(ValueError):
            parse_vision_message(",".join(fields))

    def test_qr_reader_retries_until_matching_ack(self):
        reader = qr_module.QRReader(stable_required=2, retry_detections=2)
        frame = FakeQrFrame("blue yellow")
        self.assertIsNone(reader.detect(frame))
        self.assertEqual("blue yellow", reader.detect(frame))
        reader.note_sent(30, "blue yellow")
        self.assertIsNone(reader.detect(frame))
        self.assertEqual("blue yellow", reader.detect(frame))
        reader.note_sent(60, "blue yellow")
        self.assertFalse(reader.acknowledge(30))
        self.assertTrue(reader.acknowledge(60))
        self.assertIsNone(reader.detect(frame))

    def test_block_recognition_result_and_stable_signature(self):
        first = parse_vision_message(
            "@BLOCKS_RAW,30,640,480,1,blue,100,120,32,40,900"
        )
        jittered = parse_vision_message(
            "@BLOCKS_RAW,40,640,480,1,blue,102,121,32,40,910"
        )
        text = format_recognition_result(first)
        self.assertIn("blue center=(116,140)", text)
        self.assertIn("size=32x40", text)
        self.assertEqual(
            blocks_log_signature(first[1]), blocks_log_signature(jittered[1]),
        )


if __name__ == "__main__":
    unittest.main()
