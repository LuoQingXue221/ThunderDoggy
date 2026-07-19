"""MaixCAM唯一入口：采集并发送原始视觉观测，不计算车辆动作。"""

from maix import app, camera, display, image
from chuankou import VisionSerial
from code import QRReader
from sekuai import FRAME_HEIGHT, FRAME_WIDTH, detect_blocks, detect_board, draw
from xunxian import detect_line, draw_line

LINE_INTERVAL = 2
BLOCK_INTERVAL = 6
QR_INTERVAL = 10


def main():
    cam = camera.Camera(FRAME_WIDTH, FRAME_HEIGHT, fps=40)
    cam.skip_frames(20)
    disp = display.Display()
    qr, link = QRReader(2), None
    try:
        link = VisionSerial()
    except Exception as error:
        print("UART unavailable:", error)
    frame_no, streaming = 0, True
    line_result, blocks, board = None, [], None

    while not app.need_exit():
        frame_no += 1
        img = cam.read()
        if link is not None:
            for command in link.read_lines():
                if command == "@STREAM_START":
                    streaming = True
                    qr.reset()
                elif command == "@STREAM_STOP":
                    streaming = False

        if frame_no % LINE_INTERVAL == 0:
            line_result = detect_line(img)
            if streaming and link is not None:
                link.send_line(frame_no, line_result)
        if frame_no % BLOCK_INTERVAL == 0:
            blocks, board = detect_blocks(img), detect_board(img)
            if streaming and link is not None:
                link.send_blocks(frame_no, int(img.width()), int(img.height()), blocks)
                link.send_board(frame_no, int(img.width()), int(img.height()), board)
        if frame_no % QR_INTERVAL == 0:
            payload = qr.detect(img)
            if payload is not None and streaming and link is not None:
                link.send_qr(frame_no, payload)

        if line_result is not None:
            draw_line(img, line_result)
        draw(img, blocks, board)
        qr.draw(img)
        img.draw_string(8, 8, "RAW VISION", image.COLOR_YELLOW)
        disp.show(img)


if __name__ == "__main__":
    main()
