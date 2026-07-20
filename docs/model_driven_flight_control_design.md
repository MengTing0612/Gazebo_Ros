# 模型驱动飞控调参与控制系统设计框架

## 0. 结论

当前方案可以成立，但边界要划清楚：

- 稳定飞行不能依赖模型直接裸控电机或舵机。
- 模型 SDK 更适合作为策略推理服务，输出高层动作、调参建议或风险评分。
- 飞控继续承担姿态稳定、混控、PWM 输出、失控保护和人工接管。
- 参数自动调整必须有安全层、限幅、评估窗口和回滚机制。
- APM 2.8 这类老飞控不适合承载模型，只适合作为底层飞控或学习原理的平台。

推荐工程架构是：

```text
传感器/飞控状态/任务目标/日志
        ↓
状态构造与归一化
        ↓
Stabilize-ai SDK 推理服务
        ↓
动作解释层
        ↓
安全守卫层
        ↓
MAVLink 飞控接口
        ↓
ArduPilot / PX4 / APM
```

## 1. 方案工程评估

### 1.1 可行性

方案在工程上可行，前提是将模型放在伴随计算机或地面站运行，而不是飞控 MCU 上运行。

Stabilize-ai SDK 文档说明它提供本地 TCP 推理服务，通信方式是逐行 JSON：

```json
{"obs": [...], "feedback": [...]}
```

成功响应：

```json
{"status": "success", "action": [...]}
```

这意味着它天然适合做独立推理模块。外部程序负责构造观测、调用 SDK、解释动作并写入飞控。

### 1.2 关键风险

| 风险 | 说明 | 处理方式 |
|---|---|---|
| 直接改飞控参数导致失稳 | PID、TECS、L1 参数不当会引起震荡或失控 | 只允许小幅低频调整，设置白名单、上下限、回滚 |
| 模型输出不可解释 | action 含义若不固定，难以审查 | 定义 action schema，不允许自由输出 |
| 推理延迟 | 模型 150MB，实时性取决于伴随计算机 | 控制频率分层，高层 5-20Hz，底层飞控 50-400Hz |
| 通信丢包或 SDK 挂掉 | 模型服务可能断开、续期失败或超时 | watchdog、超时保持、自动切回飞控原模式 |
| 仿真到真实差异 | 离线优化参数在真机不一定安全 | 先 SITL/HIL，再系留/低风险飞行 |
| APM 2.8 硬件老旧 | USB、算力、固件生态都偏旧 | 原型验证可用，正式方案建议 Pixhawk 类飞控 |

### 1.3 不建议的做法

- 不建议把 150MB 模型部署到 APM 2.8。
- 不建议让模型直接输出 PWM。
- 不建议真机飞行时连续高频自动改 PID 参数。
- 不建议跳过传统飞控基础调参，直接上模型控制。

## 2. 系统角色划分

### 2.1 模型 SDK

职责：

- 加载加密模型。
- 接收观测 `obs` 和反馈 `feedback`。
- 返回动作 `action`。

不负责：

- 直接连接飞控。
- 直接写参数。
- 判断动作是否安全。
- 执行失控保护。

### 2.2 飞控

职责：

- 传感器融合。
- 姿态稳定。
- 电机/舵机混控。
- PWM 输出。
- failsafe、地理围栏、遥控接管。

飞控应当是最后一道安全边界。

### 2.3 桥接程序

桥接程序是本方案的核心工程模块。

职责：

- 连接 SDK。
- 连接飞控。
- 构造模型输入。
- 解释模型输出。
- 应用安全限制。
- 记录完整日志。
- 执行参数写入或控制指令发送。

## 3. 推荐软件架构

```text
flight_ai_system/
  config/
    sdk.json
    vehicle.yaml
    safety_limits.yaml
    tuning_params.yaml
  flight_bridge/
    sdk_client.py
    mavlink_adapter.py
    observation_builder.py
    action_mapper.py
    safety_guard.py
    parameter_manager.py
    logger.py
    main.py
  experiments/
    replay_logs.py
    sitl_eval.py
    tune_offline.py
  docs/
    model_driven_flight_control_design.md
```

### 3.1 `sdk_client.py`

职责：

- 启动或连接 `sdk_client` 服务。
- 完成登录、resume 握手。
- 发送 `obs`、`feedback`。
- 解析返回的 `action`。
- 处理超时、断线和重连。

接口示例：

```python
class StabilizeSdkClient:
    def infer(self, obs: list[float], feedback: list[float]) -> list[float]:
        ...
```

### 3.2 `mavlink_adapter.py`

职责：

- 使用 MAVLink 连接飞控。
- 读取状态：姿态、高度、速度、GPS、电池、模式、RC 输入。
- 写参数：`PARAM_SET`。
- 发控制：航点、速度目标、姿态目标或 RC override。

推荐依赖：

- 原型阶段：`pymavlink`
- 更现代的 PX4 场景：`MAVSDK`

### 3.3 `observation_builder.py`

职责：

- 将飞控状态、任务目标、历史误差转成模型 `obs`。
- 做归一化、裁剪、缺失值处理。
- 保证输入维度稳定。

建议观测字段：

```text
roll, pitch, yaw
roll_rate, pitch_rate, yaw_rate
altitude, vertical_speed
airspeed 或 ground_speed
cross_track_error
heading_error
target_altitude_error
servo_output_saturation
battery_voltage
flight_mode_id
```

### 3.4 `action_mapper.py`

职责：

- 将模型输出 `action` 映射到明确动作。
- 不允许 action 含义模糊。

两类 action 建议分开设计。

#### A. 调参 action

```text
action = [
  delta_RLL2SRV_P,
  delta_RLL2SRV_D,
  delta_PTCH2SRV_P,
  delta_PTCH2SRV_D,
  delta_NAVL1_PERIOD,
  delta_TECS_TIME_CONST
]
```

#### B. 控制 action

```text
action = [
  target_heading_delta,
  target_altitude_delta,
  target_speed_delta
]
```

第一阶段建议只做调参建议或高层控制，不做 PWM。

### 3.5 `safety_guard.py`

职责：

- 参数白名单。
- 参数上下限。
- 单次变化幅度限制。
- 单位时间变化次数限制。
- 模型置信度/动作幅度检查。
- 飞控模式检查。
- 急停和回滚。

示例限制：

```yaml
parameters:
  RLL2SRV_P:
    min: 0.2
    max: 2.0
    max_step_ratio: 0.05
  PTCH2SRV_P:
    min: 0.2
    max: 2.0
    max_step_ratio: 0.05
  NAVL1_PERIOD:
    min: 12.0
    max: 25.0
    max_step_abs: 1.0
```

### 3.6 `parameter_manager.py`

职责：

- 读取当前参数。
- 应用安全后的参数更新。
- 写入飞控。
- 保存变更记录。
- 支持回滚到上一次稳定参数。

参数写入必须具备事务感：

```text
读取当前参数
    ↓
保存 checkpoint
    ↓
写入候选参数
    ↓
评估窗口
    ↓
通过则提交，失败则回滚
```

## 4. 调参闭环设计

### 4.1 离线日志调参

输入：

- 飞行日志。
- 人工标注的稳定/不稳定片段。
- 姿态误差、超调、震荡频率、控制饱和率。

输出：

- 参数调整建议。
- 风险评分。

优点：

- 不影响真机安全。
- 可以快速迭代模型。
- 适合早期验证 SDK 输入输出。

### 4.2 SITL 仿真调参

流程：

```text
启动 SITL
    ↓
设置初始参数
    ↓
运行标准任务
    ↓
采集误差指标
    ↓
模型输出参数调整
    ↓
更新参数并复测
```

评价指标：

- 姿态 RMS error。
- 高度 RMS error。
- 航向误差。
- 横向偏航距离。
- 控制输出饱和率。
- 震荡指标。
- 任务完成率。

### 4.3 真机受限调参

真机只允许：

- 人工授权后写参数。
- 每次只改少量参数。
- 改动幅度小。
- 有遥控器随时接管。
- 有回滚参数。

不允许：

- 起飞初期自动大幅改参数。
- 电机解锁前后无检查写参数。
- 模型服务超时时继续自动控制。

## 5. 控制闭环设计

### 5.1 推荐控制粒度

优先级从安全到激进：

1. 模型输出航点或任务目标。
2. 模型输出速度/航向/高度目标。
3. 模型输出姿态目标。
4. 模型输出 PWM。

建议阶段 1-2 只使用前两种。

### 5.2 控制频率

```text
飞控姿态环：50-400Hz
飞控导航环：5-50Hz
模型高层控制：5-20Hz
模型调参：0.01-0.2Hz
```

调参是慢环，不应该和姿态环抢实时性。

## 6. 数据协议

### 6.1 SDK 请求

```json
{
  "obs": [0.0, 0.0, 0.0],
  "feedback": [0.0]
}
```

### 6.2 SDK 响应

```json
{
  "status": "success",
  "action": [0.0, 0.0, 0.0]
}
```

### 6.3 内部观测包

建议桥接程序内部使用结构化数据，然后最后再展平成数组。

```python
@dataclass
class FlightObservation:
    roll_rad: float
    pitch_rad: float
    yaw_rad: float
    altitude_m: float
    ground_speed_mps: float
    heading_error_rad: float
    altitude_error_m: float
    control_saturation: float
```

## 7. 日志与可追溯性

必须记录：

- 原始飞控状态。
- 构造后的 obs。
- feedback。
- SDK action。
- safety guard 修改前后的 action。
- 写入的参数。
- 飞控响应。
- 评估指标。
- 是否回滚。

建议日志格式：

```json
{
  "timestamp": 123456.0,
  "mode": "FBWA",
  "obs": [],
  "feedback": [],
  "raw_action": [],
  "safe_action": [],
  "param_changes": {},
  "metrics": {},
  "decision": "commit"
}
```

## 8. 安全状态机

```text
INIT
  ↓
CONNECT_SDK
  ↓
CONNECT_FC
  ↓
READ_ONLY
  ↓
SUGGEST_ONLY
  ↓
AUTHORIZED_WRITE
  ↓
EVALUATE
  ↓
COMMIT 或 ROLLBACK
```

默认状态应是 `READ_ONLY`。只有明确授权才进入 `AUTHORIZED_WRITE`。

## 9. 阶段路线

### 阶段 0：文档与接口固化

产出：

- obs schema。
- action schema。
- 参数白名单。
- 安全限制表。
- SDK 最小调用 demo。

### 阶段 1：日志回放

产出：

- 日志转 obs。
- SDK 推理调用。
- action 可视化。
- 不写飞控，只生成建议。

验收：

- 能解释每个 action 为什么产生。
- 没有维度错、NaN、超界输出。

### 阶段 2：SITL 自动调参

产出：

- SITL 任务脚本。
- 自动参数写入。
- 指标评估。
- 自动回滚。

验收：

- 至少 20 次重复任务无失控。
- 参数变化在边界内。
- 任务指标优于 baseline 或不显著变差。

### 阶段 3：半实物/地面测试

产出：

- 真实飞控连接。
- 读状态。
- 写非危险参数或只读建议。
- 人工确认写入。

验收：

- SDK 掉线能停止动作。
- 飞控断连能进入安全状态。
- 日志完整可追溯。

### 阶段 4：真机低风险飞行

产出：

- 低速、低高度、受控场地测试。
- 遥控器随时接管。
- 小幅低频参数调整。

验收：

- 任意异常可切回人工/稳定模式。
- 回滚参数有效。
- 没有连续高频参数写入。

## 10. 推荐技术选择

### 10.1 飞控

原理验证：

- APM 2.8 可以学习架构，但不建议作为正式模型驱动平台。

正式平台：

- Pixhawk 4/6X/CubeOrange 等 ArduPilot/PX4 兼容飞控。
- 带 SD 卡，方便日志。
- 带稳定 MAVLink telemetry。

### 10.2 伴随计算机

根据模型大小和延迟选择：

- Raspberry Pi 5：轻量 CPU 推理。
- Jetson Orin Nano：视觉模型或较大策略模型。
- x86 小主机：地面/台架验证。

### 10.3 软件依赖

- SDK 通信：Python socket。
- 飞控通信：`pymavlink`。
- 实验记录：JSONL + CSV。
- 可视化：Matplotlib 或 Plotly。

## 11. 最小原型伪代码

```python
sdk = StabilizeSdkClient(host="127.0.0.1", port=9000)
fc = MavlinkAdapter(device="/dev/ttyACM0", baud=115200)
guard = SafetyGuard("config/safety_limits.yaml")
params = ParameterManager(fc)

baseline = params.snapshot()

while True:
    state = fc.read_state()
    obs = build_observation(state)
    feedback = compute_feedback(state)

    raw_action = sdk.infer(obs, feedback)
    candidate_update = map_action_to_params(raw_action)
    safe_update = guard.filter_param_update(candidate_update, current=params.current())

    if not operator_authorized():
        log_suggestion(obs, raw_action, safe_update)
        continue

    params.apply(safe_update)
    metrics = evaluate_window(duration_s=30)

    if guard.accept(metrics):
        params.commit()
    else:
        params.rollback(baseline)
```

## 12. 需要尽快确认的问题

1. 模型的 `obs` 长度和每一维语义是什么？
2. `feedback` 是奖励、误差、人工评分，还是环境反馈？
3. `action` 当前训练时对应的真实含义是什么？
4. 模型推理延迟是多少？
5. 是否需要保持 recurrent state？`resume=true/false` 对策略状态有什么影响？
6. 目标机型是固定翼、多旋翼，还是通用？
7. 初期要优化的是参数，还是直接输出控制目标？
8. 是否有 SITL 环境或历史飞控日志？

## 13. 工程判断

这个方案的正确落点不是“AI 替代飞控”，而是：

> AI 做高层策略和调参建议，飞控做实时稳定和安全兜底。

如果按这个边界推进，方案有落地价值；如果试图让模型直接接管底层控制，工程风险会快速失控。

