# APM / ArduPilot 2.74b 使用、启动流程与功能模块梳理

## 0. 这份资料是什么

当前目录 `APM源代码原理图/ArduPlane-2.74b` 是一份早期 ArduPilot 源码，包含：

- `ArduPlane`：固定翼飞控。
- `ArduCopter`：多旋翼飞控。
- `APMrover2`：无人车/船控制。
- `libraries`：传感器、MAVLink、参数、日志、HAL、数学、控制器等公共库。
- `APM_v280_RELEASE.sch`：APM 2.8.0 Eagle 原理图。

这份源码是老式 Arduino sketch 风格，主程序文件是 `.pde`，通过 `make` 或打补丁的 ArduPilot Arduino IDE 构建。

## 1. 硬件与连接方式

APM 2.8 的核心硬件：

- 主 MCU：`ATmega2560`
- USB 转串口/PPM 辅助 MCU：`ATmega32U2`
- IMU：`MPU6000`
- 气压计：`MS5611`
- 磁力计：`HMC5883L`
- DataFlash：`AT45DB161D`，16Mbit，约 2MB

### 1.1 电脑 USB 连接

连接链路：

```text
电脑 USB
  ↓
Micro USB 插座
  ↓
ATmega32U2
  ↓
UART0
  ↓
ATmega2560
```

飞控源码里对应的是：

- `hal.uartA`：UART0，通常用于 USB/console/MAVLink。
- `gcs0`：绑定到 `hal.uartA` 的 MAVLink 通道。

如果灯亮但电脑没有出现 `/dev/ttyACM*` 或 `/dev/ttyUSB*`，说明 USB 供电到了，但 USB 数据没有枚举成功。

### 1.2 Telemetry / WiFi / 数传连接

APM 2.8 本身不跑 WiFi 协议栈。所谓 WiFi 接入通常是外接 WiFi 串口透传模块：

```text
地面站 / 伴随计算机
  ↓ WiFi TCP/UDP
WiFi 模块
  ↓ UART 串口
APM Telemetry 口
  ↓
MAVLink
```

因此飞控源码里没有 `wifi.cpp`。飞控只处理 UART 上的 MAVLink。

常见参数：

- `SERIAL0_BAUD`：USB/console 串口波特率。
- `SERIAL3_BAUD`：telemetry 串口波特率。
- `SR0_*`：MAVLink 通道 0 的数据流频率。
- `SR3_*`：MAVLink 通道 3/telemetry 的数据流频率。

## 2. 构建与启动

源码 README 给出的构建方式：

```bash
cd APM源代码原理图/ArduPlane-2.74b/APMrover2
make configure
make
```

默认构建 APM2。也可以：

```bash
make apm1
make px4
make upload
```

注意：

- 这份源码较老，不兼容标准 Arduino IDE，需要 ArduPilot patched Arduino IDE。
- Linux 下建议走 `make`。
- 当前项目主要用于读源码和理解架构，不建议直接把老固件作为现代无人机项目基础。

## 3. APMrover2 启动流程

以 `APMrover2` 为例，主入口在：

```text
APM源代码原理图/ArduPlane-2.74b/APMrover2/APMrover2.pde
```

### 3.1 全局对象初始化

主文件中创建核心全局对象：

- `Parameters g`：全局参数表。
- `AP_Scheduler scheduler`：任务调度器。
- `RCMapper rcmap`：遥控通道映射。
- `DataFlash`：日志存储。
- `GPS g_gps_driver`：GPS 驱动。
- `AP_AHRS_DCM ahrs`：姿态估计。
- `GCS_MAVLINK gcs0 / gcs3`：MAVLink 地面站通道。

### 3.2 `setup()`

位置：

```text
APMrover2/APMrover2.pde
```

主要做：

1. 初始化 HAL。
2. 设置串口。
3. 调用 `init_ardupilot()`。
4. 初始化调度器。

`init_ardupilot()` 在：

```text
APMrover2/system.pde
```

它负责：

- 判断 USB mux 状态。
- 初始化 `hal.uartA`。
- 初始化 GPS 串口 `hal.uartB`。
- 加载 EEPROM 参数。
- 初始化 GCS/MAVLink。
- 初始化 DataFlash。
- 初始化 GPS、罗盘、AHRS、RC 输入输出。
- 设置 failsafe。

核心串口初始化片段：

```cpp
hal.uartA->begin(SERIAL0_BAUD, 128, 128);
hal.uartB->begin(115200, 128, 16);
gcs0.init(hal.uartA);

// 如果有第二 telemetry 口：
hal.uartC->begin(map_baudrate(g.serial3_baud, SERIAL3_BAUD), 128, 128);
gcs3.init(hal.uartC);
```

### 3.3 `loop()`

`loop()` 是主循环。

核心逻辑：

```text
如果到了 20ms 周期
  ↓
执行 fast_loop()
  ↓
调度非快速任务 scheduler.run()
```

APMrover2 的惯性传感器采样率是 50Hz，因此主快循环约 20ms 一次。

### 3.4 `fast_loop()`

`fast_loop()` 负责高频控制：

- 读取/更新 AHRS。
- 更新遥控输入。
- 更新当前模式。
- 执行当前模式控制。
- 更新 GCS。
- 发送 MAVLink 数据流。
- 更新云台。

可以理解为：

```text
传感器更新
  ↓
姿态/位置估计
  ↓
遥控与模式判断
  ↓
控制输出
  ↓
MAVLink 通信
```

## 4. 调度器任务

APMrover2 的定时任务表在：

```text
APMrover2/APMrover2.pde
```

关键任务：

```cpp
static const AP_Scheduler::Task scheduler_tasks[] PROGMEM = {
    { update_GPS,       5,  2500 },
    { update_compass,   5,  2000 },
    { update_commands,  5,  1000 },
    { update_logging,   5,  1000 },
    { update_events,   15,  1000 },
};
```

含义：

- `update_GPS`：读取 GPS，更新位置。
- `update_compass`：读取罗盘。
- `update_commands`：处理任务/航点命令。
- `update_logging`：写日志。
- `update_events`：处理 repeat servo/relay 等事件。

## 5. 通信模块

### 5.1 MAVLink 入口

APMrover2 MAVLink 文件：

```text
APMrover2/GCS_Mavlink.pde
```

核心类：

```cpp
GCS_MAVLINK
```

核心函数：

- `GCS_MAVLINK::init()`：绑定 UART 到 MAVLink 通道。
- `GCS_MAVLINK::update()`：读取串口字节，解析 MAVLink 消息。
- `GCS_MAVLINK::handleMessage()`：处理地面站发来的 MAVLink 消息。
- `GCS_MAVLINK::data_stream_send()`：按配置频率发送遥测数据。
- `gcs_update()`：更新所有 GCS 通道。
- `gcs_data_stream_send()`：发送所有 GCS 数据流。

### 5.2 接收消息流程

```text
UART 收到字节
  ↓
comm_receive_ch()
  ↓
mavlink_parse_char()
  ↓
解析出 mavlink_message_t
  ↓
handleMessage()
```

### 5.3 常见 MAVLink 消息

源码里处理的典型消息包括：

- `REQUEST_DATA_STREAM`：请求遥测流。
- `COMMAND_LONG`：执行命令。
- `SET_MODE`：切换模式。
- `MISSION_*`：任务/航点上传下载。
- `PARAM_REQUEST_LIST`：请求参数列表。
- `PARAM_SET`：设置参数。
- `RC_CHANNELS_OVERRIDE`：遥控通道覆盖。

### 5.4 WiFi 模块应接在哪里

如果外接 ESP8266/ESP32/串口 WiFi 模块：

```text
WiFi 模块 TX -> APM Telemetry RX
WiFi 模块 RX -> APM Telemetry TX
WiFi 模块 GND -> APM GND
电源按模块要求供电
```

WiFi 模块只做透明传输。APM 侧看到的仍然是 MAVLink 串口数据。

## 6. 参数模块

参数定义在：

```text
APMrover2/Parameters.h
APMrover2/Parameters.pde
```

核心对象：

```cpp
static Parameters g;
```

常见参数：

- `SERIAL0_BAUD`
- `SERIAL3_BAUD`
- `SYSID_THISMAV`
- `FS_ACTION`
- `FS_TIMEOUT`
- `LOG_BITMASK`
- `MODE1` 到 `MODE6`
- `CRUISE_SPEED`
- `CRUISE_THROTTLE`
- 转向和油门 PID 参数

参数通过 MAVLink 可以读写，处理位置在：

```text
GCS_Mavlink.pde
```

消息：

- `PARAM_REQUEST_LIST`
- `PARAM_REQUEST_READ`
- `PARAM_SET`
- `PARAM_VALUE`

## 7. 传感器模块

### 7.1 GPS

库：

```text
libraries/AP_GPS
```

APMrover2 中：

```cpp
g_gps->init(hal.uartB, GPS::GPS_ENGINE_AIRBORNE_4G);
```

更新函数：

```text
update_GPS()
```

### 7.2 IMU

库：

```text
libraries/AP_InertialSensor
```

APM2 常见配置使用：

```text
AP_InertialSensor_MPU6000
```

### 7.3 罗盘

库：

```text
libraries/AP_Compass
```

APM2 常见使用：

```text
AP_Compass_HMC5843 / HMC5883L 类
```

### 7.4 AHRS

库：

```text
libraries/AP_AHRS
```

作用：

- 融合 IMU、GPS、罗盘等。
- 输出姿态、航向、位置相关状态。

## 8. 控制与模式模块

APMrover2 的控制文件：

- `Steering.pde`：转向与油门控制。
- `control_modes.pde`：模式切换。
- `navigation.pde`：导航相关计算。
- `commands_logic.pde`：任务命令逻辑。
- `commands_process.pde`：任务处理流程。
- `radio.pde`：遥控输入。
- `events.pde`：重复舵机/继电器事件。

典型控制链：

```text
RC 输入 / MAVLink 命令 / mission
  ↓
模式判断
  ↓
导航目标
  ↓
转向/油门控制
  ↓
RCOutput 输出到舵机/电调
```

## 9. 日志模块

当前打开的文件：

```text
APMrover2/Log.pde
```

作用：

- 定义日志类型。
- 写入姿态、GPS、控制、性能等日志。
- 通过 DataFlash 存储。

底层库：

```text
libraries/DataFlash
```

APM2 对应：

```cpp
DataFlash_APM2 DataFlash;
```

日志写入入口之一：

```text
update_logging()
```

## 10. HAL 硬件抽象层

HAL 库位置：

```text
libraries/AP_HAL
libraries/AP_HAL_AVR
libraries/AP_HAL_AVR_SITL
libraries/AP_HAL_PX4
```

作用是屏蔽硬件差异，提供统一接口：

- UART
- I2C
- SPI
- GPIO
- RC input
- RC output
- Scheduler
- Storage

APM2 对应：

```text
libraries/AP_HAL_AVR/HAL_AVR_APM2_Class.cpp
```

其中映射：

```cpp
&avrUart0Driver  // uartA
&avrUart1Driver  // uartB
&avrUart2Driver  // uartC
```

## 11. 模型驱动系统如何接入

不建议把模型接进 APM 源码内部。

推荐方式：

```text
Stabilize-ai SDK / 伴随计算机
  ↓
MAVLink
  ↓
APM telemetry UART
  ↓
GCS_MAVLINK::handleMessage()
  ↓
飞控模式、参数、任务、RC override
```

也就是说，模型系统应当作为“地面站/伴随计算机”，通过 MAVLink 控制飞控，而不是修改飞控固件来跑模型。

### 11.1 可以通过 MAVLink 做的事

- 读取姿态、GPS、高度、电池、电机输出。
- 请求/设置参数。
- 切换模式。
- 上传任务/航点。
- 发送 RC override。
- 发送控制命令。

### 11.2 不建议直接做的事

- 高频写 PID 参数。
- 模型直接输出 PWM。
- 绕过飞控 failsafe。
- 在 APM 8-bit MCU 上跑大模型。

## 12. 快速定位表

| 需求 | 文件 |
|---|---|
| 主入口 | `APMrover2/APMrover2.pde` |
| 系统初始化 | `APMrover2/system.pde` |
| MAVLink 通信 | `APMrover2/GCS_Mavlink.pde` |
| 参数定义 | `APMrover2/Parameters.h`, `APMrover2/Parameters.pde` |
| 遥控输入 | `APMrover2/radio.pde` |
| 转向/油门控制 | `APMrover2/Steering.pde` |
| 导航 | `APMrover2/navigation.pde` |
| 任务逻辑 | `APMrover2/commands_logic.pde` |
| 日志 | `APMrover2/Log.pde` |
| 事件 | `APMrover2/events.pde` |
| HAL 串口 | `libraries/AP_HAL_AVR/UARTDriver.cpp` |
| APM2 HAL 映射 | `libraries/AP_HAL_AVR/HAL_AVR_APM2_Class.cpp` |
| MAVLink 库 | `libraries/GCS_MAVLink` |
| DataFlash 库 | `libraries/DataFlash` |
| 原理图 | `APM_v280_RELEASE.sch` |

## 13. 阅读源码建议顺序

如果要系统读懂它，建议按这个顺序：

1. `APMrover2/APMrover2.pde`
2. `APMrover2/system.pde`
3. `APMrover2/GCS_Mavlink.pde`
4. `APMrover2/Parameters.h`
5. `APMrover2/radio.pde`
6. `APMrover2/Steering.pde`
7. `APMrover2/navigation.pde`
8. `APMrover2/Log.pde`
9. `libraries/AP_HAL_AVR`
10. `libraries/GCS_MAVLink`

## 14. 一句话总结

这份 APM/ArduPilot 源码的核心架构是：

```text
HAL 负责硬件
参数系统负责配置
传感器库负责状态来源
AHRS 负责姿态估计
模式/导航/控制负责决策
MAVLink 负责外部通信
DataFlash 负责日志
Scheduler 负责节奏
```

WiFi、模型 SDK、地面站等外部系统都应该通过 MAVLink 串口进入，而不是直接改飞控内部通信模型。

