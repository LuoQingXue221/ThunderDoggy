# ThunderDoggy 火星车

ThunderDoggy 是一套由 **MaixCAM 视觉端**和 **ESP32 MicroPython 车控端**组成的低速自动火星车工程。目前目标流程是：PS2 手动把车移动到起始位置，启动自动模式，车辆读取二维码任务、沿青绿色直角路线低速行驶、识别九宫格白纸和固定五色色块、完成停车对位，并按九宫格固定姿态触碰目标物块。

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
│  └─ chuankou.py       # MaixCAM UART1 原始视觉协议
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
- 使用 UART1 `/dev/ttyS1`，A19/A18 自动复用为 UART1 TX/RX。
- 视觉采样分频：线路每 2 帧、物块和白纸每 6 帧、二维码每 10 帧，降低卡顿和串口占用。

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
- PS2 `START` 切换自动模式，`R1` 随时急停；PS2 丢线也会停止车辆。
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
```

## 当前硬件配置

硬件参数集中在 `CarControlCode/robot_config.py`。

| 外设 | 当前配置 |
| --- | --- |
| 舵机总线 | ESP32 UART2，115200，TX=16，RX=17 |
| MaixCAM 通信 | ESP32 UART1，115200，TX=5，RX=6 |
| 电机 CAN | CAN0，1 Mbps，TX=8，RX=18 |
| PS2 | DI=9，DO=10，CS=11，CLK=12 |
| 运行模式 | `RUN_MODE = "ps2"` |

### MaixCAM UART1 接线

| MaixCAM | ESP32 |
| --- | --- |
| A19 / UART1_TX | GPIO6 / UART1_RX |
| A18 / UART1_RX | GPIO5 / UART1_TX |
| GND | GND |

TX 与 RX 必须交叉连接并共地。UART1 是 MaixCAM 官方推荐的自定义通信串口；UART0 会输出系统日志并可能影响启动。参考：[MaixPy UART 文档](https://wiki.sipeed.com/maixpy/doc/en/peripheral/uart.html)。

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
4. 图像左上角显示 `RAW VISION` 表示当前运行的是新原始视觉架构。
5. 观察黄色白纸框、五色色块框和三段线路采样框。

### 打包安装

可以在 `MaixCAM` 目录使用 MaixVision 打包，或使用 `maixtool release`。安装同版本应用前应卸载相机中的旧 `num` 应用，避免继续运行缓存代码。

当前调试阶段没有开启通电自动启动；应先人工打开应用并完成在线验证。

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

1. 架空车轮，连接 ESP32、MaixCAM、PS2 和串口。
2. 启动 MaixCAM 应用，确认画面显示 `RAW VISION`。
3. 启动 ESP32，观察终端是否出现视觉串口异常。
4. 按 PS2 `△` 使能电机。
5. 用 PS2 手动把车移动到起始位置。
6. 按 `START` 进入自动模式。
7. 相机读取二维码，ESP32 终端打印任务序列。
8. 车辆开始低速巡线；`R1` 可随时急停。
9. 看到九宫格后车辆执行低速停车对位。
10. 未完成机械臂标定时，系统会报告 `uncalibrated` 并保持停车。

再次按 `START` 可取消自动模式；发生故障后应先按 `R1`，检查原因再重新启动。

## PS2 操作

### 底盘与安全

| 操作 | 功能 |
| --- | --- |
| 右摇杆上下 | 手动前进/后退 |
| 左摇杆左右 | 手动转向 |
| R2 + 右摇杆左右 | 手动原地旋转 |
| START | 启动/取消视觉自动模式 |
| R1 | 自动或手动模式立即停车 |
| × | 失能驱动电机 |
| △ | 重新使能驱动电机 |
| SELECT | 退出 PS2 控制 |

### 机械臂和标定

按住 `L2` 进入机械臂手动模式：

| 操作 | 功能 |
| --- | --- |
| 右摇杆左右 | Roll |
| 右摇杆上下 | Pitch1 |
| 左摇杆上下 | Pitch2 |
| 方向键上下 | Pitch3 |
| 方向键左右 | 相机舵机 |
| ○ | 机械臂回初始位，相机回零 |
| L2 + □ | 打印当前四关节 `GRID_POSE` |

## 九宫格机械臂标定

`CarControlCode/robot_config.py` 当前保持：

```python
ARM_GRID_CALIBRATED = False
```

九个格子的 `hover` 和 `touch` 均为 `None`。标定步骤：

1. 让车辆通过视觉停在最终对位位置，并固定车辆位置。
2. 保持自动模式关闭，使用 `L2 + 摇杆/方向键` 手动移动机械臂。
3. 到达某格上方安全悬停位置，按 `L2 + □`。
4. 记录终端输出的四关节角度作为 `hover`。
5. 缓慢移动到顶端舵机刚好轻触物块的位置，再按 `L2 + □`。
6. 记录为 `touch`。
7. 对所有可能放置物块的格子重复以上步骤。
8. 将角度填入 `ARM_GRID_POSES`。
9. 逐格架空/断电保护测试通过后，才能把 `ARM_GRID_CALIBRATED` 改为 `True`。

数据格式：

```text
(0,0): hover=(Roll, Pitch1, Pitch2, Pitch3), touch=(...)
...
(2,2): hover=(...), touch=(...)
```

`row=0` 是画面上方远处，`row=2` 是画面下方近处；`column=0/1/2` 分别为左/中/右。

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

- 所有 MaixCAM 和 ESP32 Python 源文件通过桌面 AST 语法检查。
- 相机生成的 `LINE_RAW/BLOCKS_RAW/BOARD_RAW/QR_RAW` 已直接交给 ESP32 解析器进行联合测试。
- 已模拟验证居中线路、线路置信度、左直角判断、白纸中心、九宫格编号和最近同色块排序。
- 已验证二维码任务展开为 `blue, yellow, yellow, red`。
- MaixCAM 应用压缩包已通过 ZIP 完整性检查。

这些验证不能替代真车测试，因为桌面环境没有 CAN、电机、舵机、PS2 和真实 MaixCAM 图像。

## 安全要求

- 第一次运行任何新底盘代码时必须架空车轮。
- `R1` 急停必须在每次实验前先验证。
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
