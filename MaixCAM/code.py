"""二维码视觉读取；只返回原始文本，不解析任务或控制车辆。"""

from maix import image


class QRReader:
    def __init__(self, stable_required=2, retry_detections=2):
        self.required = max(1, stable_required)
        self.retry_detections = max(1, retry_detections)
        self.last = ""
        self.count = 0
        self.confirmed = ""
        self.pending_text = ""
        self.pending_sequence = -1
        self.retry_wait = 0
        self.qr = None

    def reset(self):
        self.last, self.count, self.confirmed, self.qr = "", 0, "", None
        self.pending_text, self.pending_sequence, self.retry_wait = "", -1, 0

    def note_sent(self, sequence, text):
        """记录一次UART写入；在ESP32确认前仍会定时重发。"""
        if text == self.pending_text:
            self.pending_sequence = int(sequence)
            self.retry_wait = 0

    def acknowledge(self, sequence):
        """只确认当前等待中的发送序号，旧ACK不会误确认新的二维码。"""
        if int(sequence) != self.pending_sequence or not self.pending_text:
            return False
        self.confirmed = self.pending_text
        self.pending_text, self.pending_sequence, self.retry_wait = "", -1, 0
        return True

    def detect(self, img):
        codes = img.find_qrcodes()
        if not codes:
            self.last, self.count, self.qr = "", 0, None
            return None
        self.qr = max(codes, key=lambda q: q.w() * q.h())
        text = self.qr.payload().strip().replace("\n", " ").replace(",", " ")
        if text == self.last:
            self.count += 1
        else:
            self.last, self.count = text, 1
        if self.count < self.required or not text or text == self.confirmed:
            return None
        if text != self.pending_text:
            self.pending_text, self.pending_sequence, self.retry_wait = text, -1, 0
            return text
        if self.pending_sequence < 0:
            return text
        self.retry_wait += 1
        if self.retry_wait >= self.retry_detections:
            self.retry_wait = 0
            return text
        return None

    def draw(self, img):
        if self.qr is None:
            return
        corners = self.qr.corners()
        for i in range(4):
            a, b = corners[i], corners[(i + 1) % 4]
            img.draw_line(a[0], a[1], b[0], b[1], image.COLOR_GREEN)
        img.draw_string(self.qr.x(), max(0, self.qr.y() - 16), self.last, image.COLOR_GREEN)
