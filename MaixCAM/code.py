"""二维码视觉读取；只返回原始文本，不解析任务或控制车辆。"""

from maix import image


class QRReader:
    def __init__(self, stable_required=2):
        self.required = max(1, stable_required)
        self.last = ""
        self.count = 0
        self.sent = ""
        self.qr = None

    def reset(self):
        self.last, self.count, self.sent, self.qr = "", 0, "", None

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
        if self.count >= self.required and text and text != self.sent:
            self.sent = text
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
