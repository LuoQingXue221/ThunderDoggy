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
from robot_config import (CAMERA_UART_BAUD, CAMERA_UART_ID, CAMERA_UART_RX,
                          CAMERA_UART_TX, CAN_BAUDRATE, CAN_BUS_ID, CAN_RX, CAN_TX,
                          PS2_CLK, PS2_CS, PS2_DI, PS2_DO, RUN_MODE,
                          SERVO_UART_BAUD, SERVO_UART_ID, SERVO_UART_RX,
                          SERVO_UART_TX)
from servo_control import ServoControl, get_all_servo_ids
from servo_lib import ServoBus
from vision_protocol import apply_vision_message, new_camera_data, parse_vision_message

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
    while True:
        try:
            if serial.any():
                chunk = serial.read()
                if chunk:
                    camera_buffer += chunk.decode("utf-8", "ignore")
                    camera_buffer = camera_buffer[-1024:]
                    while "\n" in camera_buffer:
                        line, camera_buffer = camera_buffer.split("\n", 1)
                        line = line.strip()
                        if not line.startswith("@"):
                            continue
                        try:
                            apply_vision_message(camera_data,
                                                 parse_vision_message(line), line)
                        except (ValueError, TypeError) as error:
                            print("丢弃视觉帧:", error)
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
