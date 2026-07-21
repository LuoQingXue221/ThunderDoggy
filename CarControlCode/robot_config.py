"""
月球小车公共配置。

学生通常只需要改这里的运行模式、引脚、速度上限、舵机限位、机械臂尺寸等参数。

作者 王笑
日期 20260528
"""

def clamp(value, low, high):
    return max(low, min(high, value))


# =============================================================================
# 硬件端口与运行模式
# =============================================================================

RUN_MODE = "ps2"
"""程序运行模式："ps2" 使用 PS2 手柄遥控，"idle" 执行 main.py 中的学生示例程序。"""

SERVO_UART_ID    = 2
"""舵机总线使用的 ESP32 硬件 UART 编号；必须与当前固件支持的 UART 外设一致。"""
SERVO_UART_BAUD  = 115200
"""舵机 UART 通信波特率，单位 baud；必须与 Fashion Star 舵机总线配置一致。"""
SERVO_UART_TX    = 16
"""舵机 UART 发送引脚 GPIO 编号，连接舵机总线的 RX/数据输入端。"""
SERVO_UART_RX    = 17
"""舵机 UART 接收引脚 GPIO 编号，连接舵机总线的 TX/数据输出端。"""

CAMERA_UART_ID   = 1
"""相机通信使用的 ESP32 硬件 UART 编号；不能与舵机 UART 编号冲突。"""
CAMERA_UART_BAUD = 115200
"""相机 UART 通信波特率，单位 baud；必须与相机端程序的串口配置一致。"""
CAMERA_UART_TX   = 5
"""相机 UART 发送引脚 GPIO 编号，用于向相机发送应答或控制数据。"""
CAMERA_UART_RX   = 6
"""相机 UART 接收引脚 GPIO 编号，用于接收相机识别结果。"""
CAMERA_LINK_TIMEOUT_MS = 2500
"""超过该时间没有收到任何合法视觉帧时，报告相机链路离线。"""
CAMERA_PROBE_INTERVAL_MS = 2000
"""视觉链路离线时，ESP32 重发 STREAM_START 握手命令的间隔。"""

CAN_BUS_ID = 0
"""驱动电机使用的 ESP32 CAN 控制器编号；当前项目使用 CAN0。"""
CAN_BAUDRATE = 1000000
"""CAN 总线通信速率，单位 bit/s；所有电机驱动器必须配置为相同速率。"""
CAN_TX = 8
"""CAN 控制器发送引脚 GPIO 编号，应连接外部 CAN 收发器的 TXD。"""
CAN_RX = 18
"""CAN 控制器接收引脚 GPIO 编号，应连接外部 CAN 收发器的 RXD。"""

PS2_DI = 9
"""PS2 接收器 DI 引脚对应的 ESP32 GPIO；数据方向为手柄接收器到 ESP32。"""
PS2_DO = 10
"""PS2 接收器 DO 引脚对应的 ESP32 GPIO；数据方向为 ESP32 到手柄接收器。"""
PS2_CS = 11
"""PS2 接收器片选 CS 引脚对应的 ESP32 GPIO，低电平时选中手柄。"""
PS2_CLK = 12
"""PS2 接收器时钟 CLK 引脚对应的 ESP32 GPIO，由软件模拟 SPI 时钟。"""


# =============================================================================
# 底盘配置
# =============================================================================

MAX_MOTOR_RPM = 200.0
"""单个驱动电机允许的最大转速，单位 r/min；PS2 油门会按此值换算角速度。"""
DEFAULT_ACC_RAD_S2 = 20.0
"""驱动电机默认角加速度，单位 rad/s^2；数值越大，启停和变速响应越快。"""

MAX_STEER_ANGLE_DEG = 90.0
"""普通行驶时允许的最大转向输入角，单位度；实际舵机角度仍会受各自限位约束。"""
PIVOT_SPEED_SCALE = 0.3
"""原地旋转最大速度相对普通最大电机速度的比例，范围建议为 0.0～1.0。"""


# =============================================================================
# 舵机配置
# =============================================================================

BASE_SERVO_IDS = (1, 2, 3, 4, 5, 6, 7, 8, 9, 10, 11)
"""项目固定安装的全部基础舵机 ID；应与舵机调试软件中写入的 ID 保持一致。"""
CAMERA_SERVO_ID = 8
"""相机云台舵机 ID；该 ID 来自实际舵机配置，非更换硬件或重新编号时不要修改。"""

RESERVE_SERVO_ENABLED = False
"""是否初始化并允许控制预留舵机；未安装预留舵机时必须保持 False。"""
RESERVE_SERVO_IDS = (14,)
"""预留舵机 ID 元组；启用后可配置一个或多个 ID，例如 (13, 14)。"""
RESERVE_SERVO_SIGNS = {14: 1}
"""每个预留舵机的方向系数：1 表示同向，-1 表示反向；键必须对应预留舵机 ID。"""
RESERVE_SERVO_INIT_ANGLE_DEG = {14: 0.0}
"""每个预留舵机上电初始化的逻辑角度，单位度；键必须对应预留舵机 ID。"""
RESERVE_SERVO_MIN_DEG = {14: -90.0}
"""每个预留舵机允许的最小逻辑角度，单位度；用于限制控制命令以保护机构。"""
RESERVE_SERVO_MAX_DEG = {14: 90.0}
"""每个预留舵机允许的最大逻辑角度，单位度；必须大于或等于对应最小角度。"""

STEER_ANGLE_MIN_DEG = -90.0
"""六个底盘转向舵机允许的最小逻辑角度，单位度。"""
STEER_ANGLE_MAX_DEG = 90.0
"""六个底盘转向舵机允许的最大逻辑角度，单位度。"""
CAMERA_ANGLE_MIN_DEG =0
"""相机舵机允许的最小逻辑角度，单位度；设置目标角时会裁剪到此下限。"""
CAMERA_ANGLE_MAX_DEG = 90.0
"""相机舵机允许的最大逻辑角度，单位度。"""
CAMERA_MANUAL_ANGLE_DEG = 90.0
"""手动操作、自动抓取和完成提示时的相机偏移角。"""
CAMERA_VISION_ANGLE_DEG = 0.0
"""二维码识别成功后及九宫格自动对正期间的相机回正角。"""

# 视觉水平对准参数：MaixCAM发送dx像素误差，车控按比例小步调整相机舵机。
CAMERA_TRACK_ENABLED = True
"""是否允许视觉结果自动控制相机水平舵机。"""
CAMERA_TRACK_DEADZONE_PX = 15
"""目标中心距离画面中心不超过该像素数时停止转动。"""
CAMERA_TRACK_KP_DEG_PER_PX = 0.025
"""水平像素误差换算为单次舵机角度增量的比例。"""
CAMERA_TRACK_MAX_STEP_DEG = 3.0
"""每条视觉消息允许的最大角度变化，防止舵机突然大幅转动。"""
CAMERA_TRACK_DIRECTION = -1.0
"""dx>0表示目标在画面右侧；现有手动映射中向右为角度减小，所以为-1。"""
CAMERA_TRACK_SPEED_DEG_S = 40.0
"""视觉对准时的相机舵机速度。"""
ARM_ROLL_MIN_DEG = -180.0
"""机械臂 Roll 关节允许的最小逻辑角度，单位度。"""
ARM_ROLL_MAX_DEG = 180.0
"""机械臂 Roll 关节允许的最大逻辑角度，单位度。"""
ARM_PITCH1_MIN_DEG = -90.0
"""机械臂 Pitch1 关节允许的最小逻辑角度，单位度。"""
ARM_PITCH1_MAX_DEG = 90.0
"""机械臂 Pitch1 关节允许的最大逻辑角度，单位度。"""
ARM_PITCH2_MIN_DEG = -150.0
"""机械臂 Pitch2 关节允许的最小逻辑角度，单位度。"""
ARM_PITCH2_MAX_DEG = 150.0
"""机械臂 Pitch2 关节允许的最大逻辑角度，单位度。"""
ARM_PITCH3_MIN_DEG = -150.0
"""机械臂 Pitch3 关节允许的最小逻辑角度，单位度。"""
ARM_PITCH3_MAX_DEG = 150.0
"""机械臂 Pitch3 关节允许的最大逻辑角度，单位度。"""


# =============================================================================
# 机械臂配置
# =============================================================================

ARM_SERVO_SPEED_DEG_S = 60.0
"""机械臂各关节执行角度命令时的默认运动速度，单位 deg/s。"""

ARM_INIT_ROLL_DEG = 0.0
"""机械臂回初始姿态时 Roll 关节的目标逻辑角度，单位度。"""
ARM_INIT_PITCH1_DEG = 50
"""机械臂回初始姿态时 Pitch1 关节的目标逻辑角度，单位度。"""
ARM_INIT_PITCH2_DEG = -140.0
"""机械臂回初始姿态时 Pitch2 关节的目标逻辑角度，单位度。"""
ARM_INIT_PITCH3_DEG = 0.0
"""机械臂回初始姿态时 Pitch3 关节的目标逻辑角度，单位度。"""


# =============================================================================
# 自动巡线、直角转弯和九宫格停车
# =============================================================================

AUTO_LINE_SPEED_RAD_S = 0.8
"""自动直线巡线轮速。刻意低于原示例2.0 rad/s，实车仍需测试最低稳定速度。"""
AUTO_DOCK_SPEED_RAD_S = 0.35
"""九宫格前后微调轮速，只允许极低速。"""
AUTO_PIVOT_SPEED_RAD_S = 0.45
"""直角原地转向轮速，正右负左。"""
AUTO_DRIVE_ACC_RAD_S2 = 3.0
"""自动模式加速度，独立于PS2手动模式的20 rad/s^2。"""
AUTO_LINE_KP_DEG_PER_PX = 0.08
"""线路水平像素误差换算为底盘转向角的比例。"""
AUTO_LINE_MAX_STEER_DEG = 22.0
"""低速巡线最大转向角；直角不依赖该角度，而是停车后原地旋转。"""
AUTO_LINE_MIN_CONFIDENCE = 12
"""低于该视觉置信度立即停车。"""
AUTO_LINE_COMMAND_TIMEOUT_MS = 450
"""超过该时间没有新线路帧时立即停车。"""
AUTO_BOARD_COMMAND_TIMEOUT_MS = 800
"""白纸/色块视觉帧新鲜度；当前 MaixCAM 约 3 Hz，需留出处理抖动余量。"""
AUTO_TURN_TIMEOUT_MS = 6500
"""原地转向最长持续时间，超时强制停车。"""
AUTO_DOCK_X_DEADZONE_PX = 20
"""白纸中心水平对位死区。"""
AUTO_DOCK_RANGE_DEADZONE_PX = 14
"""白纸表观高度误差死区。"""
AUTO_CAMERA_FORWARD_ANGLE_DEG = CAMERA_VISION_ANGLE_DEG
"""巡线与九宫格固定标定时的相机角度；现场确认后只改此处。"""
AUTO_CAMERA_SETTLE_TOLERANCE_DEG = 2.0
"""自动任务开始前，相机舵机读回角与标定角允许的最大误差。"""
AUTO_CAMERA_SETTLE_OBSERVATIONS = 2
"""相机连续到位次数；避免舵机经过目标角时过早启动车辆。"""
AUTO_CAMERA_VERIFY_INTERVAL_MS = 120
"""自动任务开始前轮询相机舵机角度的间隔。"""
AUTO_CAMERA_SETTLE_TIMEOUT_MS = 4500
"""相机回标定角并完成读回确认的最长时间。"""

# 原始视觉量换算参数；所有像素阈值统一折算到640×480后再比较。
AUTO_CORNER_CONFIRM_OBSERVATIONS = 3
AUTO_CORNER_SIDE_MIN_PIXELS = 100
AUTO_CORNER_SIDE_RATIO_X100 = 135
AUTO_LINE_ALIGN_DEADZONE_PX = 22
AUTO_TURN_MIN_MS = 400
AUTO_TURN_ALIGNED_OBSERVATIONS = 4
AUTO_GRID_ENTRY_HEIGHT_PX_480 = 105
AUTO_GRID_DOCK_TARGET_HEIGHT_PX_480 = 260
AUTO_GRID_DOCK_STABLE_OBSERVATIONS = 5
AUTO_GRID_SETTLE_MS = 500
AUTO_GRID_LAYOUT_STABLE_OBSERVATIONS = 3
AUTO_GRID_TARGET_TIMEOUT_MS = 6000
AUTO_BOARD_ACQUIRE_TIMEOUT_MS = 5000
"""手动巡线后启动局部自动时，停车等待白纸出现的最长时间。"""
AUTO_BOARD_TIMEOUT_MS = 1500

# 九宫格自动对正（由 START 完整自动或 L3 局部自动流程进入）。
AUTO_GROUND_ALIGN_TIMEOUT_MS = 20000
AUTO_GROUND_ALIGN_STABLE_OBSERVATIONS = 5
AUTO_GROUND_ALIGN_X_DEADZONE_PX_640 = 5
AUTO_GROUND_ALIGN_CENTER_Y_DEADZONE_PX_480 = 5
AUTO_GROUND_ALIGN_BOTTOM_DEADZONE_PX_480 = 12
AUTO_GROUND_ALIGN_MARGIN_X_PX_640 = 12
AUTO_GROUND_ALIGN_MARGIN_Y_PX_480 = 12
AUTO_GROUND_ALIGN_BOTTOM_RATIO_X1000 = 900
AUTO_GROUND_ALIGN_ANGLE_DEADZONE_X10 = 10
AUTO_GROUND_ALIGN_TRANSLATE_SPEED_RAD_S = 0.16
AUTO_GROUND_ALIGN_SEARCH_SPEED_RAD_S = 0.13
AUTO_GROUND_ALIGN_PIVOT_SPEED_RAD_S = 0.14
# 若第一次实车测试发现旋转方向相反，只改为 -1，不要改控制流程。
AUTO_GROUND_ALIGN_PIVOT_SIGN = 1

# 自动对正的几何目标固定为画面正中心且九宫格水平；标定采样中的
# (315,257,-2.3°) 是摆放误差，不能写成车辆目标位姿。
AUTO_GRID_REFERENCE_CX_PX_640 = 320
AUTO_GRID_REFERENCE_CY_PX_480 = 240
AUTO_GRID_REFERENCE_ANGLE_X10 = 0
# 30 个稳定帧仍可用于确定合适的画面尺度和近似透视比例。
AUTO_GRID_REFERENCE_WIDTH_PX_640 = 407
AUTO_GRID_REFERENCE_HEIGHT_PX_480 = 299
AUTO_GRID_REFERENCE_TOP_BOTTOM_X1000 = 997
AUTO_GRID_REFERENCE_LEFT_RIGHT_X1000 = 1003
AUTO_GRID_REFERENCE_SCALE_TOLERANCE_PERCENT = 3
AUTO_GRID_REFERENCE_PERSPECTIVE_TOLERANCE_X1000 = 60


# =============================================================================
# 九宫格固定机械臂动作表
# =============================================================================

ARM_GRID_CALIBRATED = False
"""九个格子的关节角全部实测填好后才能改True；False时拒绝驱动机械臂。"""
ARM_GRID_ACTION_MODE = "touch"
"""当前机械爪未安装，使用touch：悬停->轻触顶面->收回。"""
ARM_GRID_MOVE_SPEED_DEG_S = 20.0
"""自动机械臂动作速度，保持低速。"""
ARM_GRID_HOME_WAIT_MS = 1800
ARM_GRID_HOVER_WAIT_MS = 1200
ARM_GRID_TOUCH_WAIT_MS = 700

# 每一项都必须填写四关节绝对角度：(Roll, Pitch1, Pitch2, Pitch3)。
# row=0是画面上方远处，row=2是画面下方近处；column=0/1/2为左/中/右。
# 在九格实测完成之前保持None，程序会通过@GRAB_FAIL明确报告“uncalibrated”。
ARM_GRID_POSES = {
    (0, 0): {"hover": None, "touch": None},
    (0, 1): {"hover": None, "touch": None},
    (0, 2): {"hover": None, "touch": None},
    (1, 0): {"hover": None, "touch": None},
    (1, 1): {"hover": None, "touch": None},
    (1, 2): {"hover": None, "touch": None},
    (2, 0): {"hover": None, "touch": None},
    (2, 1): {"hover": None, "touch": None},
    (2, 2): {"hover": None, "touch": None},
}

