"""MaixCAM UART1 原始视觉观测协议。"""

from maix import err, pinmap, uart

DEVICE, BAUD = "/dev/ttyS1", 115200


def _pinmux():
    err.check_raise(pinmap.set_pin_function("A19", "UART1_TX"), "UART1 TX pin failed")
    err.check_raise(pinmap.set_pin_function("A18", "UART1_RX"), "UART1 RX pin failed")


class VisionSerial:
    def __init__(self):
        _pinmux()
        self.uart = uart.UART(DEVICE, BAUD)
        self.buffer = ""

    def send(self, fields):
        return self.uart.write_str(",".join(str(v) for v in fields) + "\n")

    def send_line(self, sequence, result):
        fields = ["@LINE_RAW", sequence, result["frame_w"], result["frame_h"]]
        for point in result["points"]:
            fields += [-1, -1, 0, 0, 0] if point is None else [
                point["cx"], point["cy"], point["w"], point["h"], point["pixels"]]
        fields += [result["left_pixels"], result["right_pixels"]]
        return self.send(fields)

    def send_blocks(self, sequence, width, height, blocks):
        fields = ["@BLOCKS_RAW", sequence, width, height, len(blocks)]
        for b in blocks:
            fields += [b["color"], b["x"], b["y"], b["w"], b["h"], b["pixels"]]
        return self.send(fields)

    def send_board(self, sequence, width, height, board):
        if board is None:
            return self.send(["@BOARD_RAW", sequence, width, height, 0, 0, 0, 0, 0, 0])
        return self.send(["@BOARD_RAW", sequence, width, height, 1, board["x"],
                          board["y"], board["w"], board["h"], board["pixels"]])

    def send_qr(self, sequence, text):
        return self.send(["@QR_RAW", sequence, str(text).replace(",", " ")])

    def read_lines(self):
        data = self.uart.read()
        if data:
            self.buffer += data.decode("utf-8", "ignore") if isinstance(data, bytes) else str(data)
            self.buffer = self.buffer[-512:]
        lines = []
        while "\n" in self.buffer:
            line, self.buffer = self.buffer.split("\n", 1)
            if line.strip().startswith("@"):
                lines.append(line.strip().upper())
        return lines
