"""MaixCAM entry: QR phase followed by per-frame stable grid vision."""

import math
from maix import app, camera, display, image
from chuankou import BAUD, DEVICE, PORT_NAME, RX_PIN, TX_PIN, VisionSerial
from code import QRReader
from sekuai import FRAME_HEIGHT, FRAME_WIDTH, detect_blocks, detect_board, detect_ground, draw
from vision_stability import GridStabilizer

STATUS_INTERVAL = 30
UART_RETRY_INTERVAL = 150
UART_STATS_INTERVAL = 150
SAVE_RAW_ON_START = False
RAW_IMAGE_PATH = "/root/vision_calibration.jpg"


def _distance(a, b):
    return int(math.sqrt((a[0] - b[0]) ** 2 + (a[1] - b[1]) ** 2) + .5)


def _calibration_values(blocks, board):
    if (board is None or not board.get("complete") or len(blocks) != 9 or
            board.get("observed", 0) != 9):
        return None
    points = board["points"]
    top, bottom = _distance(points[0], points[1]), _distance(points[3], points[2])
    left, right = _distance(points[0], points[3]), _distance(points[1], points[2])
    values = [board["cx"], board["cy"], board["angle_x10"],
              (top + bottom) // 2, (left + right) // 2,
              top * 1000 // max(1, bottom), left * 1000 // max(1, right)]
    for block in blocks:
        values += [block["cx"], block["cy"]]
    return values


def _median(values):
    values = sorted(values)
    return values[len(values) // 2]


def main():
    cam = camera.Camera(FRAME_WIDTH, FRAME_HEIGHT, fps=30)
    cam.skip_frames(20)
    disp = display.Display()
    qr = QRReader(2)
    stabilizer = GridStabilizer(5)
    link = None
    mode, task_payload = "qr", ""
    frame_no, streaming, raw_saved = 0, True, False
    calibration_remaining, calibration_samples = 0, []
    calibration_reference_pending = None
    blocks, board, ground = [], None, None
    try:
        link = VisionSerial()
        written = link.send_status(frame_no, streaming)
        print("VISION UART ready: %s device=%s TX=%s RX=%s baud=%d status_bytes=%d"
              % (PORT_NAME, DEVICE, TX_PIN, RX_PIN, BAUD, written))
    except Exception as error:
        print("VISION UART unavailable; retry every 5s:", error)

    while not app.need_exit():
        frame_no += 1
        img = cam.read()
        if link is None and frame_no % UART_RETRY_INTERVAL == 0:
            try:
                link = VisionSerial()
                written = link.send_status(frame_no, streaming)
                print("VISION UART recovered: %s device=%s TX=%s RX=%s baud=%d status_bytes=%d"
                      % (PORT_NAME, DEVICE, TX_PIN, RX_PIN, BAUD, written))
            except Exception as error:
                print("VISION UART retry failed:", error)
                link = None
        if SAVE_RAW_ON_START and not raw_saved and frame_no >= 30:
            try:
                img.save(RAW_IMAGE_PATH)
                print("raw image saved:", RAW_IMAGE_PATH)
            except Exception as error:
                print("raw image save failed:", error)
            raw_saved = True

        if link is not None:
            try:
                for command in link.read_lines():
                    if command == "@STREAM_START":
                        streaming = True
                        print("VISION RX command: STREAM_START")
                    elif command == "@STREAM_STOP":
                        streaming = False
                        print("VISION RX command: STREAM_STOP")
                    elif command == "@CALIBRATE_GRID":
                        mode = "blocks"
                        stabilizer.reset()
                        blocks, board, ground = [], None, None
                        calibration_remaining, calibration_samples = 30, []
                        print("GRID_CAL start: forced BLOCKS mode; hold the ideal vehicle pose still for 30 valid frames")
                    elif command.startswith("@QR_ACK,"):
                        try:
                            sequence = int(command.split(",", 1)[1])
                        except (ValueError, IndexError):
                            print("VISION RX invalid QR_ACK:", command)
                            continue
                        if qr.acknowledge(sequence):
                            print("VISION QR ACK: seq=%d" % sequence)
            except Exception as error:
                print("VISION UART RX failed; reopening:", error)
                link = None

        payload = None
        if mode == "qr":
            payload = qr.detect(img)
            if payload is not None:
                task_payload = payload
                mode = "blocks"
                stabilizer.reset()
                blocks, board, ground = [], None, None
                print("VISION QR locked locally; mode=BLOCKS payload=%s" % payload)
        if mode == "blocks":
            # Every captured frame produces a fresh grid observation.
            ground = detect_ground(img)
            blocks = detect_blocks(img, ground)
            board = stabilizer.update(blocks, detect_board(img, blocks))
            if calibration_remaining:
                sample = _calibration_values(blocks, board)
                if sample is not None:
                    calibration_samples.append(sample)
                    calibration_remaining -= 1
                    print("GRID_CAL_SAMPLE,%d,%s" %
                          (30 - calibration_remaining, ",".join(str(v) for v in sample)))
                    if calibration_remaining == 0:
                        reference = [_median([row[index] for row in calibration_samples])
                                     for index in range(len(calibration_samples[0]))]
                        print("GRID_REFERENCE,%s" % ",".join(str(v) for v in reference))
                        calibration_reference_pending = reference

        if link is not None:
            try:
                if frame_no % STATUS_INTERVAL == 0:
                    link.send_status(frame_no, streaming)
                if streaming and mode == "blocks":
                    link.send_blocks(frame_no, int(img.width()), int(img.height()), blocks)
                    link.send_board(frame_no, int(img.width()), int(img.height()), board)
                qr_retry = (task_payload and not qr.confirmed and
                            (payload is not None or frame_no % 15 == 0))
                if streaming and qr_retry:
                    written = link.send_qr(frame_no, task_payload)
                    qr.note_sent(frame_no, task_payload)
                    print("VISION QR TX: seq=%d bytes=%d payload=%s"
                          % (frame_no, written, task_payload))
                if calibration_reference_pending is not None:
                    written = link.send_grid_reference(calibration_reference_pending)
                    print("GRID_REFERENCE UART TX: bytes=%d" % written)
                    calibration_reference_pending = None
                if frame_no % UART_STATS_INTERVAL == 0:
                    print("VISION UART TX stats: frames=%d bytes=%d streaming=%d mode=%s"
                          % (link.tx_frames, link.tx_bytes,
                             1 if streaming else 0, mode))
            except Exception as error:
                print("VISION UART TX failed; reopening:", error)
                link = None

        if mode == "blocks":
            draw(img, blocks, board, ground)
        else:
            qr.draw(img)
        img.draw_string(8, 8, "VISION U0 %s" % mode.upper(), image.COLOR_YELLOW)
        disp.show(img)


if __name__ == "__main__":
    main()
