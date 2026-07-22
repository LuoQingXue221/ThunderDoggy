"""ESP32车控入口：接收原始视觉量，运动计算由车控模块完成。"""

import _thread
import machine
import time
from esp32 import CAN
from machine import UART

from arm_control import RobotArm
from chassis_control import LunarRover
from motor_lib import MotorBus
from ps2_control import ps2_loop
from ps2_lib import PS2Controller, PS2Receiver
from robot_config import (CAMERA_LINK_TIMEOUT_MS, CAMERA_PROBE_INTERVAL_MS,
                          CAMERA_UART_BAUD, CAMERA_UART_ID, CAMERA_UART_RX,
                          CAMERA_UART_TX, CAN_BAUDRATE, CAN_BUS_ID, CAN_RX, CAN_TX,
                          PS2_CLK, PS2_CS, PS2_DI, PS2_DO, RUN_MODE,
                          SERVO_UART_BAUD, SERVO_UART_ID, SERVO_UART_RX,
                          SERVO_UART_TX)
from servo_control import ServoControl, get_all_servo_ids
from servo_lib import ServoBus
from vision_protocol import (apply_vision_message, format_recognition_result, mark_vision_bytes,
                             mark_vision_error, new_camera_data,
                             parse_vision_message, vision_link_timed_out)

time.sleep(3)
camera_data = new_camera_data()
camera_buffer = ""

servo_uart = UART(SERVO_UART_ID, SERVO_UART_BAUD, tx=SERVO_UART_TX,
                  rx=SERVO_UART_RX, timeout=64)
camera_uart = UART(CAMERA_UART_ID, CAMERA_UART_BAUD, tx=CAMERA_UART_TX,
                   rx=CAMERA_UART_RX, timeout=64)
try:
    can = CAN(CAN_BUS_ID, mode=CAN.NORMAL, baudrate=CAN_BAUDRATE,
              tx=CAN_TX, rx=CAN_RX)
except Exception:
    print("CAN硬件占用，系统软复位")
    time.sleep(1)
    machine.reset()
can.clear_rx_queue()

motor_bus = MotorBus(can)
servo_bus = ServoBus(servo_uart)
servo_bus.reset_turns_polling(get_all_servo_ids())
servo_bus.lock_all(get_all_servo_ids())
servo_control = ServoControl(servo_bus)
servo_control.init_reserve_servos()
arm = RobotArm(servo_control)
rover = LunarRover(motor_bus, servo_control, arm=arm)


def receive_vision(serial):
    global camera_buffer
    offline_reported = False
    camera_data["link_started_ms"] = time.ticks_ms()
    last_probe_ms = time.ticks_add(camera_data["link_started_ms"],
                                   -CAMERA_PROBE_INTERVAL_MS)
    print("视觉串口接收线程已启动: UART%d baud=%d TX=GPIO%d RX=GPIO%d"
          % (CAMERA_UART_ID, CAMERA_UART_BAUD, CAMERA_UART_TX, CAMERA_UART_RX))
    while True:
        try:
            now = time.ticks_ms()
            if (not camera_data.get("vision_online", False) and
                    time.ticks_diff(now, last_probe_ms) >= CAMERA_PROBE_INTERVAL_MS):
                probe = b"@STREAM_START\n"
                last_probe_ms = now
                written = serial.write(probe)
                if camera_data.get("rx_frames", 0) == 0:
                    print("视觉链路握手已发送: bytes=%s，等待MaixCAM状态帧"
                          % str(written))

            available = serial.any()
            if available:
                chunk = serial.read(available)
                if chunk:
                    mark_vision_bytes(camera_data, len(chunk))
                    camera_buffer += chunk.decode("utf-8", "ignore")
                    while "\n" in camera_buffer:
                        line, camera_buffer = camera_buffer.split("\n", 1)
                        line = line.strip()
                        if not line.startswith("@"):
                            if line:
                                mark_vision_error(camera_data)
                                print("丢弃非协议视觉文本:", line[:120])
                            continue
                        try:
                            message = parse_vision_message(line)
                            was_online = camera_data.get("vision_online", False)
                            is_new = apply_vision_message(camera_data, message, line)
                            if message is not None:
                                kind, value = message
                                if not was_online:
                                    offline_reported = False
                                    if kind == "status":
                                        print("视觉链路已上线: protocol=%d seq=%d streaming=%d"
                                              % (value["protocol"], value["sequence"],
                                                 1 if value["streaming"] else 0))
                                    else:
                                        print("视觉链路已上线: first_frame=%s seq=%d"
                                              % (kind, value.get("sequence", -1)))
                                if kind == "qr":
                                    ack = ("@QR_ACK,%d\n" % value["sequence"]).encode("utf-8")
                                    ack_written = serial.write(ack)
                                    if ack_written is not None and ack_written != len(ack):
                                        print("二维码ACK短写: %s/%d"
                                              % (str(ack_written), len(ack)))
                                    if is_new:
                                        print(format_recognition_result(message))
                                elif kind == "grid_colors":
                                    ack = ("@GRID_COLORS_ACK,%d,%d\n" %
                                           (value["request_id"], value["vision_seq"])).encode("utf-8")
                                    ack_written = serial.write(ack)
                                    if ack_written is not None and ack_written != len(ack):
                                        print("九宫格颜色ACK短写: %s/%d" %
                                              (str(ack_written), len(ack)))
                                    if is_new:
                                        print("九宫格颜色快照已接收: request=%d seq=%d" %
                                              (value["request_id"], value["vision_seq"]))
                                elif kind == "blocks":
                                    # 保留逐帧解析和缓存，供自动对正使用；终端不再输出色块明细。
                                    pass
                                elif kind == "calibration":
                                    print("GRID_REFERENCE,%s" % ",".join(
                                        str(v) for v in value["values"]))
                                    print("GRID_CAL：标定结果已由相机经 UART0 转发成功；请复制上一整行。")
                        except (ValueError, TypeError) as error:
                            mark_vision_error(camera_data)
                            print("丢弃视觉帧: %s raw=%s" % (str(error), line[:160]))
                    if len(camera_buffer) > 2048:
                        mark_vision_error(camera_data)
                        print("视觉接收缓冲区溢出，丢弃未闭合数据: bytes=", len(camera_buffer))
                        marker = camera_buffer.rfind("@")
                        camera_buffer = camera_buffer[marker:] if marker >= 0 else ""

            now = time.ticks_ms()
            if vision_link_timed_out(camera_data, CAMERA_LINK_TIMEOUT_MS, now):
                camera_data["vision_online"] = False
                if not offline_reported:
                    recent_bytes = (camera_data.get("last_byte_ms", 0) and
                                    time.ticks_diff(now, camera_data["last_byte_ms"])
                                    <= CAMERA_LINK_TIMEOUT_MS)
                    detail = "收到原始字节但没有合法帧" if recent_bytes else "未收到任何字节"
                    print("视觉链路离线: %dms内%s; rx_bytes=%d rx_frames=%d errors=%d buffer=%d"
                          % (CAMERA_LINK_TIMEOUT_MS, detail,
                             camera_data.get("rx_bytes", 0),
                             camera_data.get("rx_frames", 0),
                             camera_data.get("rx_errors", 0), len(camera_buffer)))
                    offline_reported = True
            time.sleep_ms(3)
        except Exception as error:
            print("视觉串口异常:", error)
            time.sleep_ms(100)


def main():
    rover.prepare()
    if RUN_MODE != "ps2":
        print("RUN_MODE不是ps2，车辆保持停车")
        while True:
            rover.stop()
            time.sleep_ms(500)
    controller = PS2Controller(di=PS2_DI, do=PS2_DO, cs=PS2_CS, clk=PS2_CLK)
    controller.init_vibration()
    ps2 = PS2Receiver(controller, 30, True)
    ps2.start()
    try:
        ps2_loop(rover, ps2, camera_data, camera_uart)
    finally:
        ps2.stop()
        rover.disable()


_thread.start_new_thread(receive_vision, (camera_uart,))
if __name__ == "__main__":
    main()
