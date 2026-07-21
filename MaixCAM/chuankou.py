"""MaixCAM UART0 原始视觉观测协议；保持与已验证实车接线兼容。"""

from maix import uart

DEVICE, BAUD = "/dev/ttyS0", 115200
PORT_NAME, TX_PIN, RX_PIN = "UART0", "A16", "A17"
PROTOCOL_VERSION = 3


class VisionSerial:
    def __init__(self):
        # UART0 是旧版实车已验证通道，A16/A17 默认已映射，无需再次 pinmap。
        self.uart = uart.UART(DEVICE, BAUD)
        self.buffer = ""
        self.tx_frames = 0
        self.tx_bytes = 0

    def send(self, fields):
        packet = ",".join(str(v) for v in fields) + "\n"
        packet_bytes = len(packet.encode("utf-8"))
        written = self.uart.write_str(packet)
        if isinstance(written, int) and written != packet_bytes:
            raise OSError("UART short write %d/%d" % (written, packet_bytes))
        self.tx_frames += 1
        self.tx_bytes += packet_bytes if written is None else int(written)
        return packet_bytes if written is None else int(written)

    def send_status(self, sequence, streaming=True):
        return self.send(["@VISION_STATUS", PROTOCOL_VERSION, sequence,
                          1 if streaming else 0])

    def send_blocks(self, sequence, width, height, blocks):
        fields = ["@BLOCKS_RAW", sequence, width, height, len(blocks)]
        for b in blocks:
            fields += [b["color"], b["x"], b["y"], b["w"], b["h"], b["pixels"]]
        return self.send(fields)

    def send_board(self, sequence, width, height, board):
        if board is None:
            return self.send(["@BOARD_RAW", sequence, width, height, 0] + [0] * 17)
        fields = ["@BOARD_RAW", sequence, width, height, 1, board["observed"],
                  board["complete"], board["cx"], board["cy"], board["center_x"],
                  board["center_y"], board["angle_x10"], board["angle_valid"]]
        for point in board["points"]:
            fields += [point[0], point[1]]
        fields += [board["pixels"]]
        return self.send(fields)

    def send_qr(self, sequence, text):
        return self.send(["@QR_RAW", sequence, str(text).replace(",", " ")])

    def send_grid_reference(self, values):
        """把标定中位数送回 ESP32，再由 ESP32 USB 串口转发给电脑。"""
        return self.send(["@GRID_REFERENCE"] + list(values))

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
