"""MaixCAM entry: QR phase followed by per-frame stable grid vision."""

import math
import time
from maix import app, camera, display, image
from chuankou import BAUD, DEVICE, PORT_NAME, RX_PIN, TX_PIN, VisionSerial
from code import QRReader
from sekuai import FRAME_HEIGHT, FRAME_WIDTH, detect_blocks, detect_board, detect_ground, draw
from vision_stability import GridStabilizer

STATUS_INTERVAL = 30
UART_RETRY_INTERVAL = 150
UART_STATS_INTERVAL = 150
# 相机仍每帧识别；仅将完整文本观测限为约 10Hz，避免 UART0 写入堵住主循环。
VISION_TX_INTERVAL_FRAMES = 3
GRID_SNAPSHOT_RETRY_FRAMES = 15
GROUND_CACHE_FRAMES = 2
GROUND_REACQUIRE_MIN_BLOCKS = 6
PERF_REPORT_INTERVAL = 60
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


def _now_ms():
    """兼容 MaixPy 的 ticks_ms 与桌面语法检查环境。"""
    if hasattr(time, "ticks_ms"):
        return time.ticks_ms()
    return int(time.time() * 1000)


def _elapsed_ms(start):
    now = _now_ms()
    if hasattr(time, "ticks_diff"):
        return time.ticks_diff(now, start)
    return now - start


def _perf_add(perf, name, elapsed):
    perf[name] = perf.get(name, 0) + max(0, elapsed)


def _perf_report(perf, frames):
    labels = (("读取", "read"), ("白区", "ground"), ("识别拟合", "vision"),
              ("串口", "uart"), ("绘制", "draw"), ("显示", "show"))
    values = ["%s=%dms" % (label, perf.get(key, 0) // max(1, frames))
              for label, key in labels]
    print("VISION 性能均值/%d帧: %s" % (frames, " ".join(values)))


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
    snapshot_request_id = None
    snapshot_vision_seq = -1
    snapshot_layout = None
    snapshot_acked = False
    snapshot_started_frame = -1
    snapshot_last_tx_frame = -GRID_SNAPSHOT_RETRY_FRAMES
    blocks, board, ground = [], None, None
    ground_cache_age = GROUND_CACHE_FRAMES
    performance = {}
    try:
        link = VisionSerial()
        written = link.send_status(frame_no, streaming)
        print("VISION UART ready: %s device=%s TX=%s RX=%s baud=%d status_bytes=%d"
              % (PORT_NAME, DEVICE, TX_PIN, RX_PIN, BAUD, written))
    except Exception as error:
        print("VISION UART unavailable; retry every 5s:", error)

    while not app.need_exit():
        frame_no += 1
        stage_start = _now_ms()
        img = cam.read()
        _perf_add(performance, "read", _elapsed_ms(stage_start))
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
                        # 新自动会话不得复用上一会话已缓存的颜色快照。
                        snapshot_request_id = None
                        snapshot_vision_seq = -1
                        snapshot_layout = None
                        snapshot_acked = False
                        snapshot_started_frame = -1
                        print("VISION RX command: STREAM_START")
                    elif command == "@STREAM_STOP":
                        streaming = False
                        print("VISION RX command: STREAM_STOP")
                    elif command == "@CALIBRATE_GRID":
                        mode = "blocks"
                        stabilizer.reset()
                        blocks, board, ground = [], None, None
                        ground_cache_age = GROUND_CACHE_FRAMES
                        calibration_remaining, calibration_samples = 30, []
                        print("GRID_CAL start: forced BLOCKS mode; hold the ideal vehicle pose still for 30 valid frames")
                    elif command.startswith("@GRID_SNAPSHOT_REQ,"):
                        try:
                            request_id = int(command.split(",", 1)[1])
                            if request_id < 0:
                                raise ValueError
                        except (ValueError, IndexError):
                            print("VISION RX invalid GRID_SNAPSHOT_REQ:", command)
                            continue
                        if request_id == snapshot_request_id and snapshot_layout is not None:
                            snapshot_acked = False
                            snapshot_last_tx_frame = frame_no - GRID_SNAPSHOT_RETRY_FRAMES
                            print("GRID snapshot duplicate request: resend id=%d" % request_id)
                        elif request_id != snapshot_request_id:
                            mode = "blocks"
                            snapshot_request_id = request_id
                            snapshot_vision_seq = -1
                            snapshot_layout = None
                            snapshot_acked = False
                            snapshot_started_frame = frame_no
                            snapshot_last_tx_frame = frame_no - GRID_SNAPSHOT_RETRY_FRAMES
                            stabilizer.reset()
                            print("GRID snapshot start: id=%d; collect new stable frames" % request_id)
                    elif command.startswith("@GRID_COLORS_ACK,"):
                        try:
                            _, request_text, sequence_text = command.split(",")
                            request_id = int(request_text)
                            vision_seq = int(sequence_text)
                        except (ValueError, IndexError):
                            print("VISION RX invalid GRID_COLORS_ACK:", command)
                            continue
                        if (request_id == snapshot_request_id and
                                vision_seq == snapshot_vision_seq and
                                snapshot_layout is not None):
                            snapshot_acked = True
                            print("GRID snapshot ACK: id=%d seq=%d" %
                                  (request_id, vision_seq))
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
                ground_cache_age = GROUND_CACHE_FRAMES
                print("VISION QR locked locally; mode=BLOCKS payload=%s" % payload)
        if mode == "blocks":
            # Every captured frame produces a fresh grid observation.
            stage_start = _now_ms()
            refresh_ground = ground is None or ground_cache_age >= GROUND_CACHE_FRAMES
            if refresh_ground:
                ground = detect_ground(img)
                ground_cache_age = 0
            else:
                ground_cache_age += 1
            _perf_add(performance, "ground", _elapsed_ms(stage_start))
            stage_start = _now_ms()
            blocks = detect_blocks(img, ground)
            # 缓存白区若已随车身移动而失配，立即全图重找后重做当前帧。
            if ground is not None and len(blocks) < GROUND_REACQUIRE_MIN_BLOCKS and not refresh_ground:
                ground_start = _now_ms()
                ground = detect_ground(img)
                ground_cache_age = 0
                _perf_add(performance, "ground", _elapsed_ms(ground_start))
                blocks = detect_blocks(img, ground)
            detected_board = detect_board(img, blocks)
            # 请求命令是在本轮图像采集后读到的；跳过这一帧，确保快照只使用请求后的图像。
            if (snapshot_request_id is not None and snapshot_layout is None and
                    frame_no == snapshot_started_frame):
                board = None
            else:
                board = stabilizer.update(blocks, detected_board)
            if (snapshot_request_id is not None and snapshot_layout is None and
                    board is not None and board.get("complete")):
                stable_layout = stabilizer.get_stable_layout()
                if stable_layout is not None and len(stable_layout) == 9:
                    snapshot_layout = stable_layout
                    snapshot_vision_seq = frame_no
                    snapshot_last_tx_frame = frame_no - GRID_SNAPSHOT_RETRY_FRAMES
                    print("GRID snapshot stable: id=%d seq=%d" %
                          (snapshot_request_id, snapshot_vision_seq))
            _perf_add(performance, "vision", _elapsed_ms(stage_start))
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
            stage_start = _now_ms()
            try:
                if frame_no % STATUS_INTERVAL == 0:
                    link.send_status(frame_no, streaming)
                # 块和九宫格来自同一识别快照；限频同步发送，不积压旧帧。
                if (streaming and mode == "blocks" and
                        frame_no % VISION_TX_INTERVAL_FRAMES == 0):
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
                if (streaming and snapshot_layout is not None and not snapshot_acked and
                        frame_no - snapshot_last_tx_frame >= GRID_SNAPSHOT_RETRY_FRAMES):
                    written = link.send_grid_colors(
                        snapshot_request_id, snapshot_vision_seq, snapshot_layout)
                    snapshot_last_tx_frame = frame_no
                    print("GRID_COLORS UART TX: id=%d seq=%d bytes=%d" %
                          (snapshot_request_id, snapshot_vision_seq, written))
                if frame_no % UART_STATS_INTERVAL == 0:
                    print("VISION UART TX stats: frames=%d bytes=%d streaming=%d mode=%s"
                          % (link.tx_frames, link.tx_bytes,
                             1 if streaming else 0, mode))
            except Exception as error:
                print("VISION UART TX failed; reopening:", error)
                link = None
            _perf_add(performance, "uart", _elapsed_ms(stage_start))

        stage_start = _now_ms()
        if mode == "blocks":
            draw(img, blocks, board, ground)
        else:
            qr.draw(img)
        img.draw_string(8, 8, "VISION U0 %s" % mode.upper(), image.COLOR_YELLOW)
        _perf_add(performance, "draw", _elapsed_ms(stage_start))
        stage_start = _now_ms()
        disp.show(img)
        _perf_add(performance, "show", _elapsed_ms(stage_start))
        if frame_no % PERF_REPORT_INTERVAL == 0:
            _perf_report(performance, PERF_REPORT_INTERVAL)
            performance = {}


if __name__ == "__main__":
    main()
