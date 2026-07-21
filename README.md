# ThunderDoggy 火星车

ThunderDoggy 是一套由 **MaixCAM 视觉端**和 **ESP32 MicroPython 车控端**组成的低速自动火星车工程。系统同时支持两条流程：`START` 从二维码开始执行自动巡线、对位和目标轻触；也可由人工巡线，由 ESP32 持续缓存视觉端识别到的二维码任务，到九宫格附近后按 `L3`，只让车辆接管白纸对位和九宫格目标轻触。

> 当前机械爪尚未安装，九宫格机械臂动作处于安全锁定状态。软件控制链已经建立，但必须完成实车标定后才能解除锁定。

## 当前架构原则

本工程采用“相机只识别、ESP32 负责决策和动作”的分工：

```text
MaixCAM
  ├─ 二维码解码
  ├─ 青绿色线路色块检测
  ├─ 白纸检测
  └─ 红/黄/蓝/粉/紫色块检测
            │
            │ UART1 原始视觉数据
            ▼
ESP32
  ├─ 二维码任务解析
  ├─ 巡线误差与置信度计算
  ├─ 直角方向与转弯结束判断
  ├─ 白纸中心和远近误差计算
  ├─ 九宫格编号、最近物块和中心距离计算
  ├─ 低速底盘闭环
  └─ 固定机械臂动作与安全保护
```

ESP32 无法直接读取 MaixCAM 图像，因此 LAB 阈值和 `find_blobs` 必须留在相机端；坐标、距离、巡线和动作状态机属于轻量计算，全部放到 ESP32。

## 仓库结构

```text
ThunderDoggy/
├─ README.md
├─ CarControlCode_API接口说明书.pdf
├─ MaixCAM/
│  ├─ app.yaml          # MaixCAM 应用清单
│  ├─ main.py           # 唯一应用入口，仅调度识别和发送原始量
│  ├─ code.py           # 二维码原文识别
│  ├─ xunxian.py        # 青绿色线路原始色块检测
│  ├─ sekuai.py         # 白纸和固定五色色块检测
│  └─ chuankou.py       # MaixCAM UART0 原始视觉协议
└─ CarControlCode/
   ├─ boot.py
   ├─ main.py           # ESP32 硬件初始化和视觉串口接收线程
   ├─ robot_config.py   # 引脚、速度、限位、自动参数和九格姿态
   ├─ vision_protocol.py# 原始视觉帧解析
   ├─ autonomous_control.py # 几何计算、任务状态机和自动动作
   ├─ ps2_control.py    # PS2 手动/自动模式、急停和标定
   ├─ ps2_lib.py
   ├─ chassis_control.py
   ├─ arm_control.py
   ├─ motor_lib.py
   ├─ servo_control.py
   └─ servo_lib.py
```

## 已经完成的功能

### MaixCAM 视觉端

- 应用只有一个入口 `MaixCAM/main.py`，其他文件保持模块分工。
- 视觉端代码已经精简，不包含车辆状态机、偏移控制或抓取决策。
- 支持红、黄、蓝、粉、紫五种固定颜色的 LAB 色块识别。
- 支持单个物块测试，不再要求至少出现两个物块。
- 检测区域根据实际画面尺寸生成，兼容 320×240 和 640×480 坐标。
- 对色块进行尺寸、长宽比、像素密度和重复框过滤，不识别黑色背景框。
- 识别白纸外接矩形，但不在相机端计算停车误差或九宫格行列。
- 使用三个水平采样带检测青绿色线路，只返回色块中心、宽高和像素数。
- 读取二维码原文，例如 `blue yellow red 1 2 1`，不在相机端解析任务。
- 使用实车旧版已验证的 UART0 `/dev/ttyS0`；A16/A17 是对应的板载 TX/RX。
- 视觉端以 30 FPS 采集：线路每 10 帧（约 3 Hz）、白纸和物块在同一帧每 10 帧（约 3 Hz）、二维码每 15 帧（约 2 Hz），避免用旧白纸位置配合新物块数据。

### ESP32 车控端

- 支持原始视觉协议的半包、粘包和异常帧过滤。
- 将二维码 `blue yellow red 1 2 1` 解析为：

  ```text
  blue → yellow → yellow → red
  ```

- 使用三个线路采样带的原始中心进行 5:3:1 加权巡线。
- 将不同分辨率的像素量归一化到 640×480 基准参数。
- 在 ESP32 中计算线路误差、置信度、丢线状态和直角方向。
- 直角确认后停车并低速原地旋转；新线路连续居中后结束旋转。
- 白纸出现后计算水平中心误差和表观高度误差，先旋转对中，再前后低速对距。
- 在 ESP32 中把白纸内部划分为 3×3 九宫格。
- 计算每个物块的 `dx`、`dy` 和像素距离 `distance`。
- 同色多个目标按画面 `cy` 从大到小排序，即从画面下方向上、近处优先。
- 已完成格子会被记录，二维码包含重复颜色时不会重复选择同一格。
- 自动速度刻意限制为低速，丢线、串口超时、转弯超时或白纸丢失会停车。
- 手动阶段持续缓存最新有效二维码；局部自动启动后冻结本次任务。
- ESP32 收到二维码后打印原始 `payload`；收到色块后打印颜色、中心坐标、宽高和像素数。色块日志会过滤轻微像素抖动，并在全部色块消失时输出一次。
- PS2 `START` 切换完整自动模式，`L3` 切换对位轻触模式；两种模式互斥。
- `×` 随时取消自动任务并失能驱动电机；PS2 丢线也会停止车辆。
- 机械臂固定动作框架为：初始位 → 悬停 → 轻触 → 悬停 → 初始位。
- 九宫格未标定时强制拒绝机械臂动作，不会使用猜测角度。

## 相机与 ESP32 通信协议

所有帧使用 UTF-8 文本，以 `@` 开头、换行结尾，波特率为 115200。

### 线路原始量

```text
@LINE_RAW,seq,frame_w,frame_h,
cx0,cy0,w0,h0,pixels0,
cx1,cy1,w1,h1,pixels1,
cx2,cy2,w2,h2,pixels2,
left_pixels,right_pixels
```

三个点依次对应近、中、远采样带。未识别到的点使用 `-1,-1,0,0,0`。

### 色块原始量

```text
@BLOCKS_RAW,seq,frame_w,frame_h,count,
color,x,y,w,h,pixels,...
```

每个物块只包含颜色和外接矩形；中心、距离、格子和近远顺序由 ESP32 计算。

### 白纸原始量

```text
@BOARD_RAW,seq,frame_w,frame_h,valid,x,y,w,h,pixels
```

`valid=0` 表示未识别到可靠白纸。

### 二维码原文

```text
@QR_RAW,seq,blue yellow red 1 2 1
```

### ESP32 控制相机数据流

```text
@STREAM_START
@STREAM_STOP
@QR_ACK,seq
```

MaixCAM 默认持续发送原始视觉量。ESP32 启动任一自动任务时发送 `@STREAM_START` 以确保串流开启并重置二维码去重状态；普通取消或完成后不停止串流，便于手动阶段继续识别二维码。

MaixCAM 每秒发送一次链路状态帧：

```text
@VISION_STATUS,protocol_version,seq,streaming
```

ESP32 上电后会自动发送 `@STREAM_START` 握手，并在 2.5 秒没有收到任何合法视觉帧时报告链路离线；离线期间每 2 秒重试握手。二维码在收到对应的 `@QR_ACK` 前会定时重发，ESP32 会对重发帧去重，避免同一任务被重复缓存。

## 当前硬件配置

硬件参数集中在 `CarControlCode/robot_config.py`。

| 外设 | 当前配置 |
| --- | --- |
| 舵机总线 | ESP32 UART2，115200，TX=16，RX=17 |
| MaixCAM 通信 | ESP32 UART1，115200，TX=5，RX=6 |
| 电机 CAN | CAN0，1 Mbps，TX=8，RX=18 |
| PS2 | DI=9，DO=10，CS=11，CLK=12 |
| 运行模式 | `RUN_MODE = "ps2"` |

### MaixCAM UART0 接线

| MaixCAM | ESP32 |
| --- | --- |
| A16 / UART0_TX | GPIO6 / UART1_RX |
| A17 / UART0_RX | GPIO5 / UART1_TX |
| GND | GND |

TX 与 RX 必须交叉连接并共地。当前选择 UART0 是为了保持与已经完成实车通信测试的旧版 MaixCAM 程序和既有接线一致；ESP32接收器会忽略 UART0 启动阶段的非协议日志。参考：[MaixPy UART 文档](https://wiki.sipeed.com/maixpy/doc/en/peripheral/uart.html)。

## 软件环境

### MaixCAM

- MaixCAM / MaixCAM-Pro
- MaixPy v4
- MaixVision

MaixCAM 应用根目录必须包含 `app.yaml` 和 `main.py`。参考：[MaixCAM 应用开发文档](https://wiki.sipeed.com/maixpy/doc/en/basic/app.html)。

### ESP32

车控代码不是普通 CPython，需要包含以下扩展的 ESP32 MicroPython 固件：

- `machine.Pin`
- `machine.UART`
- `esp32.CAN`
- `_thread`
- `ustruct`

桌面编辑器提示无法导入 `machine` 或 `esp32` 不代表开发板运行失败。

## 部署 MaixCAM

### MaixVision 在线调试

1. 在 MaixVision 中打开 `MaixCAM/main.py`。
2. 确认同目录存在 `code.py`、`xunxian.py`、`sekuai.py` 和 `chuankou.py`。
3. 连接 MaixCAM 后运行 `main.py`。
4. 图像左上角显示 `RAW VISION U0 v4.10.4`，表示当前运行的是使用已验证 UART0 接线的修复版本。
5. 观察黄色白纸框、五色色块框和三段线路采样框。

### 打包安装

可以在 `MaixCAM` 目录使用 MaixVision 打包，或使用 `maixtool release`。安装同版本应用前应卸载相机中的旧 `num` 应用，避免继续运行缓存代码。

当前应用版本为 `4.10.4`，用于避免设备继续使用旧 `4.10.3` 缓存。当前调试阶段没有开启通电自动启动；应先人工打开应用并完成在线验证。

## 部署 ESP32

必须把 `CarControlCode` 内全部 `.py` 文件上传到 ESP32 根目录。特别注意新增文件：

- `vision_protocol.py`
- `autonomous_control.py`

使用 Thonny 时，选择整个 `CarControlCode` 目录中的文件并上传到设备根目录。使用 `mpremote` 时，可先确认端口：

```powershell
python -m mpremote connect list
```

假设端口为 COM6：

```powershell
python -m mpremote connect COM6 fs cp CarControlCode/*.py :
python -m mpremote connect COM6 reset
```

不同版本的 Windows shell 对通配符处理可能不同；如果上传失败，请在 Thonny 中逐文件上传。

## 使用流程

### 完整自动

1. 架空车轮，连接 ESP32、MaixCAM、PS2 和串口。
2. 启动 MaixCAM 应用，确认画面显示 `RAW VISION`。
3. 启动 ESP32，观察终端是否出现视觉串口异常。
4. 按 PS2 `△` 使能电机。
5. 用 PS2 手动把车移动到起始位置。
6. 按 `START` 进入完整自动模式。
7. 相机读取二维码，ESP32 终端打印任务序列。
8. 车辆开始低速巡线；`×` 可随时急停并失能驱动电机。
9. 看到九宫格后车辆执行低速停车对位。
10. 未完成机械臂标定时，系统会报告 `uncalibrated` 并保持停车。

再次按 `START` 可取消完整自动模式；发生故障后应先按 `×`，检查原因再按 `△` 重新使能。

### 手动巡线后自动对位

1. 保持手动模式，让相机看到二维码，确认终端输出“二维码任务已缓存”。
2. 用 PS2 人工巡线，将车驶到九宫格白纸附近。
3. 单按 `L3`：车辆立即停车并锁定本次任务。相机先回到 `AUTO_CAMERA_FORWARD_ANGLE_DEG`（当前 0°），连续两次读回确认到位后丢弃运动期间的旧画面。
4. 相机到位后才开始独立的 5 秒白纸等待；白纸出现后车辆自动水平对中、前后对距，再按二维码展开顺序轻触全部目标。
5. 完成或故障后车辆保持停车；再按 `L3` 确认返回手动模式。

没有有效二维码缓存时，`L3` 仍会启动白纸对正，但对正完成后直接保持停车、自动失能驱动电机，不进入选块和机械臂动作。再按 `L3` 后即可返回手动模式进行机械臂标定；此时底盘电机仍保持失能，如需重新移动车辆必须明确按 `△`。局部自动中按 `START` 不会切换模式，完整自动中按 `L3` 也不会切换模式。

## PS2 操作

### 底盘与安全

| 操作 | 功能 |
| --- | --- |
| 左摇杆全方向 | 底盘二维平移（前后/左右） |
| 右摇杆上下 | 底盘原地旋转 |
| 右摇杆左右 | 相机舵机旋转 |
| START | 启动/取消完整自动模式 |
| L3 | 无二维码时启动白纸对正；有缓存任务时启动自动对位轻触；再次按下取消/确认退出 |
| × | 取消自动任务、急停并失能驱动电机 |
| △ | 重新使能驱动电机 |
| L3 + R3 | 机械臂、相机、底盘转向舵机和夹爪复位 |
| SELECT | 退出 PS2 控制 |

### 机械臂和标定

| 操作 | 功能 |
| --- | --- |
| 方向键左/右 | Roll 减小/增加 |
| 方向键上/下 | Pitch1 增加/减小 |
| L1 / L2 | Pitch2 增加/减小 |
| R1 / R2 | Pitch3 增加/减小 |
| □ / ○ | 收紧/放松夹爪 |
| R3 | 打印当前四关节 `GRID_POSE` 和白纸标定高度 `BOARD_CAL` |

## 九宫格机械臂标定

`CarControlCode/robot_config.py` 当前保持：

```python
ARM_GRID_CALIBRATED = False
```

九个格子的 `hover` 和 `touch` 均为 `None`。首次实车标定分三阶段进行。

### 阶段 A：标定白纸目标高度

1. 架空车轮，确认电机动力可单独断开，操作范围内无人，并把手放在 `×` 急停键附近。
2. 启动 ESP32。程序上电后底盘电机默认失能，并会尝试读取四个关节舵机的实际角度。必须看到“机械臂真实角度同步成功。”；若出现 `read_failed`、`state_unsynced` 或角度越界，禁止点动机械臂，先检查舵机连接、ID、方向和限位。
3. 按 `△` 使能底盘，手动把车停到机械臂确认能够安全覆盖九宫格的位置，同时保证白纸完整进入画面。
4. 单按 `R3`，记录终端 `BOARD_CAL` 行中的 `suggested_target`。如果显示 `unavailable`，先检查白纸轮廓、光照、相机安装角度和视觉串口数据。
5. 把 `suggested_target` 填入 `CarControlCode/robot_config.py` 的 `AUTO_GRID_DOCK_TARGET_HEIGHT_PX_480`，重新部署 ESP32 程序并重启。

### 阶段 B：验证自动对正

1. 不让相机识别二维码，按 `△` 使能底盘，再单按 `L3` 启动纯白纸对正。
2. 终端应依次显示“相机回自动标定角”和“相机已到自动标定角”。只有到位确认成功后，车辆才会使用新的白纸画面。
3. 车辆应先水平旋转对中，再低速前后对距。首次必须架空车轮或留出充分空间，分别核对旋转和前后方向；方向错误立即按 `×`，不要等待状态机自行纠正。
4. 稳定完成后终端应提示“白纸对正完成，底盘电机已失能”；此时车辆保持停车。
5. 再按一次 `L3` 返回手动模式进行机械臂标定。底盘电机仍然失能，不会因退出局部自动而恢复动力。

自动对正故障码：

- `motors_disabled`：底盘尚未按 `△` 使能，L3 被拒绝。
- `camera_settle_timeout`：相机舵机未在 4.5 秒内到达标定角，检查 ID 8、舵机串口、供电和角度方向。
- `board_link_timeout`：相机到位后没有收到新的 `BOARD_RAW`，检查 MaixCAM 应用、115200 波特率、TX/RX 交叉接线和共地。
- `board_not_detected_timeout`：已经收到 `BOARD_RAW`，但 `valid=0`；检查 MaixCAM 画面是否出现黄色白纸框、白纸尺寸和 LAB 阈值。
- `board_lost`：开始对位后白纸连续丢失。

全局视觉链路日志：

- `视觉链路已上线`：ESP32 已收到并解析至少一条 MaixCAM 状态或视觉帧。
- `视觉链路离线: ...未收到任何字节`：ESP32 RX 没有电气数据，检查应用启动、TX/RX 交叉接线和共地。
- `视觉链路离线: ...收到原始字节但没有合法帧`：RX 有数据但协议无法解析，检查波特率、串扰和两端程序版本。
- MaixCAM 的 `VISION UART TX stats`：发送端累计帧数和字节数；可用于确认相机应用仍在持续写 UART。

### 阶段 C：采集九格姿态

1. 确认自动模式已关闭、底盘电机已失能，并移除机械臂路径中的手、线缆和易碎物品。
2. 使用方向键、`L1/L2` 和 `R1/R2` 以每次 2° 的步长手动移动机械臂。
3. 到达某格上方安全悬停位置，按 `R3`，记录终端输出的四关节角度作为 `hover`。
4. 缓慢移动到顶端舵机刚好轻触物块的位置，再按 `R3`，记录为 `touch`。
5. 对九个格子重复以上步骤，共采集 18 组姿态，并逐项填入 `ARM_GRID_POSES`。
6. 保持 `ARM_GRID_CALIBRATED = False`，先逐格执行受控测试；全部姿态确认没有越界、碰撞或反向运动后，才能把它改为 `True`。

数据格式：

```text
(0,0): hover=(Roll, Pitch1, Pitch2, Pitch3), touch=(...)
...
(2,2): hover=(...), touch=(...)
```

`row=0` 是画面上方远处，`row=2` 是画面下方近处；`column=0/1/2` 分别为左/中/右。

首次标定自动对距参数时，先用手动模式把车停在机械臂确认能覆盖整个九宫格的位置，并保证白纸完整可见。单按 `R3` 会同时输出：

```text
GRID_POSE=(12.0, 35.0, -108.0, 18.0)
BOARD_CAL: height_480=243, current_target=260, suggested_target=243, raw_height=243/480
```

将 `suggested_target` 的值填入 `AUTO_GRID_DOCK_TARGET_HEIGHT_PX_480`。该高度会统一折算到 480 像素高的画面，因此相机使用 320×240 或 640×480 时都可直接使用 `height_480`。

`R3` 当前只把关节姿态和白纸高度打印到串口，不会自动修改或保存 `robot_config.py`。这是有意保留的安全限制：程序并不知道本次记录属于哪个格子以及 `hover` 还是 `touch`，必须由操作者确认后手动填写，避免姿态写错位置。

## 需要继续完成的工作

### 必须完成

- 使用 MaixCAM 原始画面在比赛现场光照下微调五色 LAB 阈值。
- 固定相机曝光、白平衡、安装角度和焦距，避免阈值随自动曝光漂移。
- 检验线路误差正负与实际转向方向是否一致。
- 实测青绿色线路阈值、巡线比例 `AUTO_LINE_KP_DEG_PER_PX` 和最低稳定轮速。
- 实测左/右直角判定和原地旋转方向；必要时调整方向符号。
- 标定白纸目标高度 `AUTO_GRID_DOCK_TARGET_HEIGHT_PX_480`。
- 采集九宫格所有 `hover/touch` 姿态并解除机械臂安全锁。
- 在安装机械爪后增加张开、闭合、抬起和放入车顶篮子的动作。
- 进行完整实车联调：二维码 → 巡线 → 直角 → 九宫格 → 目标序列 → 动作完成。

### 后续功能

- 增加赛道终点/返回路线识别；当前任务在二维码目标序列完成后停车。
- 根据机械爪结构增加抓取失败检测和重试策略。
- 增加离线开机自动启动；当前阶段保留人工启动，防止调试时通电即运动。
- 保存实车串口日志和原始相机截图，建立每次参数调整的可追踪记录。
- 增加硬件在环测试；当前自动测试只验证协议和纯计算逻辑。

## 已执行的软件验证

桌面回归命令：

```powershell
python -B -m unittest discover -s tests -v
```

- 所有 MaixCAM 和 ESP32 Python 源文件通过桌面 AST 语法检查。
- 相机生成的 `LINE_RAW/BLOCKS_RAW/BOARD_RAW/QR_RAW` 已直接交给 ESP32 解析器进行联合测试。
- 已模拟验证居中线路、线路置信度、左直角判断、白纸中心、九宫格编号和最近同色块排序。
- 已验证二维码任务展开为 `blue, yellow, yellow, red`。
- 已通过 31 项纯桌面回归测试，覆盖已验证 UART0 接线锁定、视觉状态心跳、链路超时、二维码 ACK/重发去重、二维码缓存/冻结、二维码与色块日志格式、无任务独立对正、L3 启停、双模式互斥、相机到位读回、旧画面清除、相机/UART/白纸检测超时分类、MaixCAM 到 ESP32 的 `BOARD_RAW` 有效/无效帧往返、白纸高度标定输出、对位稳定计数、未标定安全锁、驱动未使能拒绝自动启动、机械臂读回同步保护、上电驱动默认失能和远距离白纸尺寸过滤。
- 已验证 `L3+R3` 复位后不会误触发单按 L3，且 UP 恢复为 Pitch1 正向点动。
- MaixCAM 应用压缩包已通过 ZIP 完整性检查。

这些验证不能替代真车测试，因为桌面环境没有 CAN、电机、舵机、PS2 和真实 MaixCAM 图像。

## 安全要求

- 第一次运行任何新底盘代码时必须架空车轮。
- `×` 急停和失能必须在每次实验前先验证。
- 上电后底盘电机默认失能；只有确认环境安全后才能按 `△` 使能。
- 未看到机械臂关节读回同步成功时禁止使用手动关节点动。
- 电机动力电源和逻辑电源应能独立断开。
- 未标定九宫格时禁止把 `ARM_GRID_CALIBRATED` 改为 `True`。
- 机械臂动作区域内不得放置手、线缆或易碎物品。
- 不要使用 5V TTL 直接连接 MaixCAM/ESP32 UART。
- 修改舵机 ID、方向、限位或初始角前必须核对实际机构。

## 重要参数位置

| 参数 | 文件 |
| --- | --- |
| 五色 LAB 阈值 | `MaixCAM/sekuai.py` |
| 青绿色线路阈值 | `MaixCAM/xunxian.py` |
| 相机采样间隔 | `MaixCAM/main.py` |
| UART/CAN/PS2 引脚 | `CarControlCode/robot_config.py` |
| 巡线速度与比例 | `CarControlCode/robot_config.py` |
| 直角与停车阈值 | `CarControlCode/robot_config.py` |
| 九格机械臂姿态 | `CarControlCode/robot_config.py` |
| 视觉协议解析 | `CarControlCode/vision_protocol.py` |
| 自动任务状态机 | `CarControlCode/autonomous_control.py` |

## 作者与状态

原始车控代码作者：王笑。

当前版本处于在线调试和实车标定阶段，尚不是可直接参加比赛的最终离线版本。
