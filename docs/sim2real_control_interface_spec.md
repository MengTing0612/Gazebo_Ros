# Flight Bridge Sim2Real Control Interface v0.2

本文是当前项目的 Sim2Real 通信接口规范。它已经按本地代码重新对齐：在线通信帧直接复用 `flight_bridge/main.py` Phase 1 的 JSONL 记录结构，不再另起一套 `sim_state`、`command.safe`、`sdk_latency_ms` 等字段名。

核心原则：

- `flight_bridge` 现有日志字段就是控制通信 payload 的源格式。
- 真机端只能执行 `safe_command`，不能执行 `action` 或 `candidate_command`。
- v0.2 首选 `control == "rc"` 的 RC residual 路径，对应 `RcCommand.to_list()`。
- `control == "guided"` 只保留为后续扩展，当前 ArduCopter 2.74b 主路径不依赖它。
- 真机代理必须做本地二次安全验收，不能只相信仿真侧 `SafetyShield`。

## 1. 本地代码源

本规范以这些文件为准：

| 文件 | 对齐内容 |
| --- | --- |
| `flight_bridge/main.py` | Phase 1 主循环、JSONL 帧字段、`cmd_method` 语义 |
| `flight_bridge/command_types.py` | `RcCommand`、`VelocityCmd`、`ShieldInfo` 字段和 `to_list()` 顺序 |
| `flight_bridge/mavlink_adapter.py` | `FlightState.to_log_dict()`、`CommandLog`、MAVLink command method |
| `flight_bridge/state_builder.py` | `CopterState`、obs16、feedback1 的单位和坐标系 |
| `flight_bridge/action_adapter.py` | action10 到 RC/velocity 的映射和 residual 语义 |
| `flight_bridge/safety_shield.py` | `shield` 日志字段、限幅、滤波、超时验收 |
| `flight_bridge/config.yaml` | `limits_sim`、`limits_real`、`control.frequency_hz`、`residual_lambda` |

实现通信代码时，应优先复用这些 dataclass 和 helper，不要手写一份独立字段模型。

## 2. 分层架构

```text
Gazebo odometry / MAVLink state / real_state
    -> CopterState
    -> obs16 + feedback1
    -> Stabilize/Gaia SDK
    -> action10
    -> ActionAdapter
    -> candidate command
    -> SafetyShield
    -> safe_command
    -> Sim2Real transport
    -> real_vehicle_agent local safety gate
    -> MAVLink RC_CHANNELS_OVERRIDE
    -> real flight controller
```

真实飞控负责姿态环、角速度环、电机混控和底层 failsafe。通信链路只传受限的外环修正或 RC residual，不传电机级 PWM/RPM。

## 3. 传输层

v0.2 使用 UTF-8 JSON Lines over TCP：

- 真机代理默认监听 `0.0.0.0:9100`。
- 控制主机作为 TCP client 发起连接。
- 每条消息是一个 JSON 对象，以 `\n` 结尾。
- 控制频率默认 20 Hz，对齐 `config.control.frequency_hz` 和 CLI `--rate-hz`。
- 单条消息建议小于 16 KiB。
- 控制端必须记录发送前的原始 JSONL，真机代理必须记录收到的原始 JSONL 和验收结果。

后续可以增加 UDP/QUIC，但应用层字段和安全规则保持不变。

## 4. 协议外壳

在线消息在现有 Phase 1 日志记录外面包一层传输头。传输头用于 session、序号、ACK 和代理验收；`frame_record` 保持本地日志字段不改名。

```json
{
  "protocol": "flight_bridge_sim2real",
  "schema_version": 1,
  "msg_type": "control_frame",
  "session_id": "20260611T120000Z-x3-to-real",
  "seq": 42,
  "sent_time_unix_s": 1781160002.100,
  "monotonic_ms": 2100.0,
  "execution_mode": "shadow",
  "frame_record": {}
}
```

字段要求：

| 字段 | 类型 | 要求 |
| --- | --- | --- |
| `protocol` | string | 固定为 `flight_bridge_sim2real` |
| `schema_version` | int | 当前固定为 `1` |
| `msg_type` | string | `hello`、`hello_ack`、`heartbeat`、`control_frame`、`command_ack`、`real_state`、`agent_status`、`safety_event` |
| `session_id` | string | 一次实验唯一 ID；重连重新生成 |
| `seq` | int | 同一 `session_id` 内单调递增 |
| `sent_time_unix_s` | float | 发送时 Unix 时间，秒 |
| `monotonic_ms` | float | 发送端单调时钟，毫秒 |
| `execution_mode` | string | `shadow`、`dry_run_real_state`、`bench_no_prop`、`tether_low_alt`、`free_low_alt` |
| `frame_record` | object | 仅 `control_frame` 必填；内容为 Phase 1 JSONL 记录 |

接收端必须拒绝同一 `session_id` 下 `seq` 不递增的控制消息。

协议外壳参数语义：

| 参数 | 语义 | 生产方 | 消费方 |
| --- | --- | --- | --- |
| `protocol` | 应用层协议名，用来避免把其他 JSONL 流误当成飞控控制流。 | 控制主机/真机代理 | 双方 |
| `schema_version` | 协议外壳版本，不等于模型版本或飞控版本。字段删除、单位变化、命令语义变化时必须升级。 | 控制主机/真机代理 | 双方 |
| `msg_type` | 消息用途。`control_frame` 是控制主机到真机代理的主消息；其他类型用于握手、状态和安全事件。 | 双方 | 双方 |
| `session_id` | 一次实验/连接的唯一标识，用于把日志、ACK 和安全事件归到同一次试验。重连应换新值。 | 控制主机 | 真机代理/日志分析 |
| `seq` | 传输层消息序号，用于丢包、乱序、重复消息检测。它不是 `frame_record.frame`，二者可以不同。 | 双方 | 双方 |
| `sent_time_unix_s` | 墙钟发送时间，用于跨进程/跨机器日志对齐和命令年龄检查。时钟不同步时只作诊断，真机端仍要依赖接收时刻做本地超时。 | 双方 | 双方 |
| `monotonic_ms` | 发送进程内单调时间，主要用于本机延迟诊断；不能跨机器直接比较绝对值。 | 双方 | 日志分析 |
| `execution_mode` | 本次发送方声明的测试阶段。真机代理只能把它作为意图，最终是否执行由真机本地配置决定。 | 控制主机 | 真机代理 |
| `frame_record` | 与现有 Phase 1 JSONL 日志逐字段一致的控制帧。协议层不重命名内部字段。 | 控制主机 | 真机代理/日志分析 |

## 5. Phase 1 Frame Record

`frame_record` 必须直接对齐当前 `flight_bridge/main.py` 的 `logger.write({...})` 字段。典型控制帧：

```json
{
  "ts": 1781160002.095,
  "frame": 42,
  "phase": 1,
  "mode": "sitl",
  "control": "rc",
  "rc_control_mode": "residual",
  "vehicle_mode": "STABILIZE",
  "armed": true,
  "state": {
    "roll": 0.01,
    "pitch": -0.02,
    "yaw": 0.0,
    "roll_speed": 0.0,
    "pitch_speed": 0.0,
    "yaw_speed": 0.0,
    "heading": 0.0,
    "groundspeed": 0.0,
    "climb": 0.0,
    "lat": 47.397742,
    "lon": 8.545594,
    "relative_alt": 0.85,
    "mode": "STABILIZE",
    "armed": true,
    "battery_voltage": 12000.0,
    "heading_error_deg": 0.0,
    "attitude_ts": 1781160002.070,
    "position_ts": 1781160002.070,
    "vfr_ts": 1781160002.070,
    "heartbeat_ts": 1781160001.900
  },
  "gazebo_odom": null,
  "copter_state": {
    "x_err": 0.02,
    "y_err": -0.01,
    "z_err": 0.15,
    "vx": 0.03,
    "vy": 0.0,
    "vz": -0.02,
    "roll": 0.01,
    "pitch": -0.02,
    "yaw_err": 0.0,
    "p": 0.0,
    "q": 0.0,
    "r": 0.0,
    "altitude_m": 0.85
  },
  "obs": [0.0, 0.0, 0.0, 0.0],
  "feedback": [0.5],
  "feedback_diagnostics": {},
  "action": [0.0, 0.0, 0.2, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0],
  "latency_ms": 18.4,
  "state_age_ms": 35.0,
  "gazebo_age_ms": null,
  "timeout_discard": false,
  "baseline_rc": [1500, 1500, 1700, 1500],
  "candidate_command": [1500, 1500, 1712, 1500],
  "safe_command": [1500, 1500, 1704, 1500],
  "rc_delta": [0.0, 0.0, 0.008, 0.0],
  "gazebo_reset": null,
  "reset_recovery": null,
  "shield": {
    "filtered": true,
    "saturated": false,
    "safety_triggered": false,
    "clipped_fields": []
  },
  "sent": true,
  "cmd_method": "RC_CHANNELS_OVERRIDE",
  "error": null
}
```

字段要求：

| 字段 | 对齐代码 | 说明 |
| --- | --- | --- |
| `ts` | `time.time()` | 源端生成记录的 Unix 时间 |
| `frame` | `frame_index` | 控制循环帧号 |
| `phase` | literal `1` | Sim2Real 只使用 Phase 1 控制帧 |
| `mode` | CLI `--mode` | 当前代码为 `dry-run` 或 `sitl` |
| `control` | CLI `--control` | `rc`、`guided`、`off` |
| `rc_control_mode` | CLI `--rc-control-mode` | `residual`、`sdk-direct`、`baseline-only` |
| `vehicle_mode` | `FlightState.mode` | 飞控模式字符串 |
| `armed` | `FlightState.armed` | 源端看到的解锁状态 |
| `state` | `FlightState.to_log_dict()` | MAVLink/仿真原始飞行状态 |
| `gazebo_odom` | `GazeboOdom.to_log_dict()` | SITL truth，可为 `null` |
| `copter_state` | `_copter_state_to_log()` | 统一 body-FLU 控制状态 |
| `obs` | `CopterStateBuilder.build_hover_obs()` | 16 维，RC 模式最后 4 维为上一帧 `rc_delta` |
| `feedback` | `HoverFeedbackBuilder.build()` | 当前为 1 维 |
| `feedback_diagnostics` | `HoverFeedbackDiagnostics.to_log_dict()` | 可为 `null`，恢复/保护帧常见 |
| `action` | SDK 输出 | 必须保留 10 维；异常/恢复帧可为空 |
| `latency_ms` | SDK 推理耗时 | 当前字段名不是 `sdk_latency_ms` |
| `state_age_ms` | `FlightState.fast_state_age_ms()` | 真机端必须用于 stale gate |
| `gazebo_age_ms` | Gazebo odom age | 非 Gazebo truth 可为 `null` |
| `timeout_discard` | bool | `SafetyShield.should_accept_action()` 或状态新鲜度失败 |
| `baseline_rc` | `RcCommand.to_list()` | RC baseline，顺序见第 7 节 |
| `candidate_command` | `CommandLog.candidate` | 只用于日志，禁止真机执行 |
| `safe_command` | `CommandLog.safe` | 真机端唯一可执行命令 |
| `rc_delta` | derived | `(safe - baseline_rc) / 500.0` |
| `gazebo_reset` | reset info | 真机端必须拒绝这类仿真复位帧 |
| `reset_recovery` | recovery info | 真机端默认拒绝或转本地 hold |
| `shield` | `ShieldInfo` | `filtered`、`saturated`、`safety_triggered`、`clipped_fields` |
| `sent` | `CommandLog.sent` | 源端本地 adapter 是否发送；不是远端执行结果 |
| `cmd_method` | `CommandLog.method` | 源端方法/控制阶段标签 |
| `error` | string/null | 源端异常 |

顶层控制帧参数语义：

| 参数 | 语义 | 真机端使用规则 |
| --- | --- | --- |
| `ts` | 源端生成这一帧记录的时间。它比协议外壳的 `sent_time_unix_s` 更早，表示控制循环内部时间点。 | 只用于日志对齐，不单独作为执行条件。 |
| `frame` | Phase 1 控制循环帧号，每循环一次加一。 | 用于 ACK 和日志索引；不能替代传输层 `seq` 做乱序检测。 |
| `phase` | 桥接阶段。Sim2Real 控制只接受 `1`，Phase 0 是 dry-run/inference，不应写真机。 | 不是 `1` 时拒绝执行。 |
| `mode` | 源端车辆模式，当前为 `dry-run` 或 `sitl`。 | 真机执行通常只接受经过 shadow 验证的 `sitl` 帧；`dry-run` 不写飞控。 |
| `control` | 源端控制后端。`rc` 表示 `safe_command` 是 RC PWM；`guided` 表示速度命令；`off` 表示只读。 | v0.2 默认只执行 `rc`。 |
| `rc_control_mode` | RC 路径的控制策略：baseline、SDK residual 或 SDK direct。 | 真机低风险阶段优先 `baseline-only`，再逐步允许 `residual`；`sdk-direct` 默认拒绝。 |
| `vehicle_mode` | 源端看到的飞控模式，例如 `STABILIZE`、`LOITER`。 | 必须在真机代理本地白名单内。 |
| `armed` | 源端看到的解锁状态。 | 只作参考；真机端必须读本地飞控真实 armed 状态。 |
| `state` | `MavlinkAdapter` 缓存的飞控原始状态。 | 真机端不直接用它执行命令；可用于一致性检查和日志。 |
| `gazebo_odom` | Gazebo truth odometry。 | 真机端不得用 Gazebo truth 控制真实飞机。 |
| `copter_state` | 控制器统一后的机体系状态，是 obs/feedback 的物理来源。 | 用于诊断和重放；真机执行仍以 `safe_command` 为准。 |
| `obs` | 发给 SDK 的观测向量。 | 只用于记录和模型行为复现，禁止真机端解释成控制量。 |
| `feedback` | 发给 SDK 的任务反馈。 | 只用于记录和模型行为复现。 |
| `feedback_diagnostics` | feedback 的内部代价项和滤波信息。 | 用于离线分析，不参与执行。 |
| `action` | SDK 输出的 action10。 | 禁止真机端直接执行。必须只看 ActionAdapter 和 SafetyShield 后的 `safe_command`。 |
| `latency_ms` | SDK 推理耗时。 | 超过本地或源端阈值时拒绝执行。 |
| `state_age_ms` | ATT/GLOBAL/VFR 等 fast state 的最老消息年龄。 | stale gate 的核心参数，超过阈值拒绝执行。 |
| `gazebo_age_ms` | Gazebo odom 的消息年龄。 | 只用于 SITL truth 诊断；真机端不依赖该值执行。 |
| `timeout_discard` | 源端已判定这帧 action 因超时或状态陈旧被丢弃。 | 为 `true` 时拒绝执行。 |
| `baseline_rc` | 传统 baseline controller 给出的 RC 基准。 | 与 `safe_command` 一起计算 residual；真机端可做范围和偏移诊断。 |
| `candidate_command` | SafetyShield 之前的候选命令。 | 禁止执行，只用于观察模型/适配器输出。 |
| `safe_command` | SafetyShield 之后的安全命令。 | 唯一可执行控制量，还必须经过真机代理二次验收。 |
| `rc_delta` | `safe_command` 相对 `baseline_rc` 的归一化 RC 偏移，公式为 `(safe - baseline) / 500`。 | 用于 obs 历史和残差大小监控，不直接执行。 |
| `gazebo_reset` | SITL episode reset 信息。 | 非空时默认拒绝执行或进入本地 hold。 |
| `reset_recovery` | 仿真 reset 后恢复阶段信息。 | 非空时默认拒绝执行或进入本地 hold。 |
| `shield` | 源端 SafetyShield 的输出状态。 | 作为拒绝规则输入，但不能代替真机本地 safety gate。 |
| `sent` | 源端是否向本地 SITL `MavlinkAdapter` 发送成功。 | 不能表示真机已执行；真机执行结果只看 `command_ack.executed`。 |
| `cmd_method` | 源端发送方法或控制阶段标签。 | 用于白名单和风险分级。 |
| `error` | 源端异常文本。 | 非空时拒绝执行。 |

`state` 参数语义：

| 参数 | 语义 | 单位/坐标 |
| --- | --- | --- |
| `roll` | 飞机横滚角，右滚为正，来自 MAVLink `ATTITUDE.roll`。 | rad |
| `pitch` | 飞机俯仰角，抬头/低头符号按 MAVLink 姿态定义。 | rad |
| `yaw` | 飞机航向角，按 MAVLink yaw 定义。 | rad |
| `roll_speed` | 横滚角速度。 | rad/s |
| `pitch_speed` | 俯仰角速度。 | rad/s |
| `yaw_speed` | 偏航角速度。 | rad/s |
| `heading` | 航向角，来自 `VFR_HUD.heading`。 | deg |
| `groundspeed` | 水平地速标量。 | m/s |
| `climb` | 垂直速度，当前代码约定正值向上。 | m/s |
| `lat` | 纬度。 | deg |
| `lon` | 经度。 | deg |
| `relative_alt` | 相对起飞点/ home 的高度。 | m |
| `mode` | 飞控模式字符串。 | enum string |
| `armed` | 源端 MAVLink heartbeat 解出的 armed 状态。 | bool |
| `battery_voltage` | 电压读数。SITL/旧 MAVLink 中可能是 mV，真机代理需要在本地明确单位。 | implementation-defined |
| `heading_error_deg` | 当前 heading 到目标 heading 的误差。 | deg |
| `attitude_ts` | 最近姿态消息到达时间。 | Unix s |
| `position_ts` | 最近位置消息到达时间。 | Unix s |
| `vfr_ts` | 最近 VFR/HUD 消息到达时间。 | Unix s |
| `heartbeat_ts` | 最近 heartbeat 到达时间。 | Unix s |

`copter_state` 参数语义：

| 参数 | 语义 | 单位/坐标 |
| --- | --- | --- |
| `x_err` | 飞机相对目标/锚点的前向误差。正值表示飞机在目标前方，baseline 通常需要向后修正。 | m, body-FLU |
| `y_err` | 飞机相对目标/锚点的左向误差。正值表示飞机在目标左侧。 | m, body-FLU |
| `z_err` | 目标高度减当前高度后的上向误差。正值通常表示需要爬升。 | m, body-FLU |
| `vx` | 机体系前向速度。 | m/s, body-FLU |
| `vy` | 机体系左向速度。 | m/s, body-FLU |
| `vz` | 机体系上向速度。 | m/s, body-FLU |
| `roll` | 控制状态使用的横滚角。 | rad |
| `pitch` | 控制状态使用的俯仰角。 | rad |
| `yaw_err` | 当前航向到目标航向的误差。 | rad |
| `p` | 横滚角速度。 | rad/s |
| `q` | 俯仰角速度。 | rad/s |
| `r` | 偏航角速度。 | rad/s |
| `altitude_m` | 当前相对高度/任务高度估计。 | m |

`obs` 向量语义：

| 索引 | 来源 | 语义 |
| --- | --- | --- |
| `0..2` | `tanh([x_err, y_err, z_err] / 2.0)` | 位置误差归一化，保留误差方向。 |
| `3..5` | `tanh([vx, vy, vz] / 1.0)` | 速度归一化。 |
| `6..8` | `tanh([roll, pitch, yaw_err] / 30deg)` | 姿态/航向误差归一化。 |
| `9..11` | `tanh([p, q, r] / 180deg/s)` | 角速度归一化。 |
| `12..15` | `last_cmd[0..3]` | RC 模式为上一帧 `rc_delta`；guided 模式为上一帧速度命令。 |

`feedback_diagnostics` 参数语义：

| 参数 | 语义 |
| --- | --- |
| `mode` | feedback 构造模式，例如 `reward_01`。 |
| `value` | 最终传给 SDK 的 `feedback[0]`。 |
| `cost` | 未滤波 hover cost，越大表示状态越差。 |
| `filtered_cost` | 一阶滤波后的 cost。 |
| `previous_filtered_cost` | 上一帧滤波 cost，用来计算 improvement。 |
| `improvement` | `previous_filtered_cost - filtered_cost`，正值表示状态改善。 |
| `terms` | 分项 cost，包括 `xy`、`z`、`vxy`、`vz`、`tilt`、`rate`、`yaw`、`command`、`saturation`、`safety`。 |

## 6. 坐标系与单位

当前代码约定：

- `CopterState`：body-FLU，x 向前，y 向左，z 向上。
- `VelocityCmd`：body-FLU，`[vx, vy, vz, yaw_rate]`。
- `FlightState.climb`：m/s，正值向上。
- 角度字段默认为弧度，`heading` 和 `heading_error_deg` 为度。
- `relative_alt`、`altitude_m`、`x_err`、`y_err`、`z_err` 单位为米。
- `vx`、`vy`、`vz` 单位为 m/s。
- `p`、`q`、`r`、`roll_speed`、`pitch_speed`、`yaw_speed` 单位为 rad/s。
- RC PWM 范围为 `[1000, 2000]`，横滚/俯仰/偏航中位为 1500。

不要在通信层把 `copter_state.x_err` 改名为 `x_err_m`，也不要把 `latency_ms` 改名为 `sdk_latency_ms`。单位由本规范和字段表定义。

## 7. 命令语义

### 7.1 RC path

当 `frame_record.control == "rc"` 时：

```text
safe_command = [roll_pwm, pitch_pwm, throttle_pwm, yaw_pwm]
baseline_rc  = [roll_pwm, pitch_pwm, throttle_pwm, yaw_pwm]
```

这个顺序来自 `RcCommand.to_list()`。

v0.2 真机执行只支持该路径。真机代理将 `safe_command` 转成 MAVLink `RC_CHANNELS_OVERRIDE` 或等价真实链路。

RC 执行规则：

- `safe_command` 必须是长度为 4 的有限整数列表。
- 每个 PWM 必须在 `[1000, 2000]`。
- 真机代理必须再做本地通道方向、通道范围和油门保护。
- 真机悬停油门必须来自真机标定，不能直接沿用 X3 的 `hover_throttle`。
- `candidate_command` 和 `action` 永远不能被执行。

RC 通道语义：

| 索引 | 字段 | 语义 | 单位/范围 |
| --- | --- | --- | --- |
| `0` | `roll` | 横滚通道。1500 为中位，方向必须在真机台架阶段确认。 | PWM us, `[1000, 2000]` |
| `1` | `pitch` | 俯仰通道。1500 为中位，方向必须在真机台架阶段确认。 | PWM us, `[1000, 2000]` |
| `2` | `throttle` | 油门/高度通道。中位不是固定 1500，悬停值必须由真机标定。 | PWM us, `[1000, 2000]` |
| `3` | `yaw` | 偏航通道。1500 为中位，方向必须在真机台架阶段确认。 | PWM us, `[1000, 2000]` |

RC 相关参数语义：

| 参数 | 语义 |
| --- | --- |
| `baseline_rc` | 确定性 baseline 控制器输出，用来维持起飞/高度/弱位置回拉。 |
| `candidate_command` | SDK residual 或 baseline 经过 ActionAdapter 后、SafetyShield 前的命令。 |
| `safe_command` | SafetyShield 后的命令，是源端认为可发送的安全值。 |
| `rc_delta` | 归一化 residual，便于观察 SDK/安全层到底给 baseline 加了多大修正。 |

### 7.2 Guided velocity path

当 `frame_record.control == "guided"` 时：

```text
safe_command = [vx, vy, vz, yaw_rate]
```

这个顺序来自 `VelocityCmd.to_list()`，单位为 body-FLU m/s 和 rad/s。

当前项目文档和 `docs/current_status.md` 都指出 ArduCopter 2.74b 的可靠首选入口是 RC override，所以真机代理默认必须拒绝 `guided`。只有换成已实测支持 guided/offboard 的飞控后，才能通过本地白名单启用。

### 7.3 `cmd_method`

`cmd_method` 是源端控制阶段或发送方法标签。当前常见值：

| 值 | 真机端默认处理 |
| --- | --- |
| `RC_CHANNELS_OVERRIDE` | 可按普通 RC 控制帧验收 |
| `BASELINE_ONLY` | 可按普通 RC 控制帧验收，适合低风险 baseline 测试 |
| `LOITER_TRANSITION`、`LOITER_NEUTRAL` | 默认 shadow，真机 LOITER 测试需单独白名单 |
| `SET_POSITION_TARGET_LOCAL_NED` | 默认拒绝，除非启用 guided |
| `POST_RESET_GUARD` | 默认不执行；可映射为本地 hold |
| `RESET_RECOVERY_*` | 默认不执行；属于仿真 reset 恢复 |
| `AUTO_REARM` | 真机端禁止远程自动解锁 |
| `gazebo_reset` | 真机端禁止执行 |

## 8. 真机代理验收规则

真机代理收到 `control_frame` 后，按顺序执行这些检查：

1. 外壳 `protocol == "flight_bridge_sim2real"`。
2. `schema_version == 1`。
3. `session_id` 当前有效。
4. `seq` 严格递增。
5. 控制帧年龄不超过本地 `max_command_age_ms`。
6. 心跳未超时。
7. 本地 `execution_mode` 允许写飞控。
8. 本地人工使能开关打开。
9. `frame_record.phase == 1`。
10. `frame_record.control == "rc"`，除非本地显式启用 guided。
11. `frame_record.timeout_discard == false`。
12. `frame_record.state_age_ms <= state_age_max_ms`。
13. `frame_record.error is null`。
14. `frame_record.gazebo_reset is null`。
15. `frame_record.reset_recovery is null`，除非本地映射为 hold。
16. `frame_record.shield.safety_triggered == false`。
17. `frame_record.safe_command` 类型、长度和范围合法。
18. `frame_record.vehicle_mode` 在本地白名单内，例如 `STABILIZE`、`ALT_HOLD`、`LOITER`。
19. 未检测到人工接管、低电压 failsafe、通信 failsafe、姿态过大、地理围栏越界。

任一条件失败时，真机代理不得写飞控，并应发送 `command_ack` 说明原因。

`frame_record.sent` 只说明源端 `MavlinkAdapter` 是否发送过命令。Sim2Real 真机代理不能把它当成真实机执行结果。

验收参数语义：

| 参数 | 语义 | 拒绝建议 |
| --- | --- | --- |
| `max_command_age_ms` | 真机代理从收到帧或对比发送时间计算出的最大允许命令年龄。 | 超过即 stale reject。 |
| `heartbeat_timeout_ms` | 控制主机心跳允许中断的最长时间。 | 超过进入 fallback。 |
| `state_age_max_ms` | `frame_record.state_age_ms` 最大允许值。 | 超过拒绝执行。 |
| `manual_enable` | 本地人工使能开关，不应由网络消息远程开启。 | 未开启时 shadow/拒绝。 |
| `manual_override` | 遥控器/安全员接管状态。 | 检测到接管时释放 override 或 hold。 |
| `failsafe_active` | 真机飞控或代理检测到 failsafe。 | 进入本地 fallback，并拒绝网络控制。 |

## 9. 真机回传消息

### 9.1 `real_state`

`real_state` 应复用 `FlightState.to_log_dict()` 的字段形状，方便控制主机以后把真机状态接回 `CopterStateBuilder`。

```json
{
  "protocol": "flight_bridge_sim2real",
  "schema_version": 1,
  "msg_type": "real_state",
  "session_id": "20260611T120000Z-x3-to-real",
  "seq": 100,
  "sent_time_unix_s": 1781160002.120,
  "monotonic_ms": 2150.0,
  "vehicle_id": "quad-alpha",
  "state": {
    "roll": 0.01,
    "pitch": -0.02,
    "yaw": 0.0,
    "roll_speed": 0.0,
    "pitch_speed": 0.0,
    "yaw_speed": 0.0,
    "heading": 0.0,
    "groundspeed": 0.0,
    "climb": 0.0,
    "lat": 47.397742,
    "lon": 8.545594,
    "relative_alt": 0.85,
    "mode": "STABILIZE",
    "armed": true,
    "battery_voltage": 15.2,
    "heading_error_deg": 0.0,
    "attitude_ts": 1781160002.100,
    "position_ts": 1781160002.100,
    "vfr_ts": 1781160002.100,
    "heartbeat_ts": 1781160001.900
  },
  "manual_override": false,
  "failsafe_active": false
}
```

控制主机进入真机闭环后，应优先用 `real_state.state` 构造控制状态，而不是继续使用 `gazebo_odom` 控制真实飞机。

`real_state` 参数语义：

| 参数 | 语义 |
| --- | --- |
| `vehicle_id` | 真机代理给当前飞机的稳定标识，用于区分多机和日志。 |
| `state` | 与 `FlightState.to_log_dict()` 对齐的真机飞控状态。 |
| `manual_override` | 真机代理检测到人工接管或 RC override 应释放。 |
| `failsafe_active` | 真机飞控/代理检测到低电压、通信丢失、姿态异常等 failsafe。 |

### 9.2 `command_ack`

真机代理必须对拒绝、执行、模式变化和安全触发发送 ACK。正常执行可按 5 Hz 到 20 Hz 汇总。

```json
{
  "protocol": "flight_bridge_sim2real",
  "schema_version": 1,
  "msg_type": "command_ack",
  "session_id": "20260611T120000Z-x3-to-real",
  "seq": 101,
  "sent_time_unix_s": 1781160002.130,
  "monotonic_ms": 2160.0,
  "control_seq": 42,
  "frame": 42,
  "accepted": true,
  "executed": false,
  "reason": "shadow_mode",
  "command_age_ms": 30.0,
  "last_executed_seq": null
}
```

`command_ack` 参数语义：

| 参数 | 语义 |
| --- | --- |
| `control_seq` | 被 ACK 的 `control_frame.seq`。 |
| `frame` | 被 ACK 的 `frame_record.frame`，用于和源端控制循环对齐。 |
| `accepted` | 代理是否接受该帧进入验收流程；格式错误、session 错误等为 `false`。 |
| `executed` | 代理是否真的写入飞控。真机执行结果只看这个字段。 |
| `reason` | 未执行或拒绝的原因，例如 `shadow_mode`、`stale`、`manual_override`。 |
| `command_age_ms` | 代理侧计算的命令年龄。 |
| `last_executed_seq` | 最近一次成功写入飞控的控制帧 `seq`。 |

### 9.3 `agent_status`

```json
{
  "protocol": "flight_bridge_sim2real",
  "schema_version": 1,
  "msg_type": "agent_status",
  "session_id": "20260611T120000Z-x3-to-real",
  "seq": 102,
  "sent_time_unix_s": 1781160002.200,
  "monotonic_ms": 2200.0,
  "execution_mode": "shadow",
  "connected_to_fcu": true,
  "manual_enable": false,
  "write_permission": false,
  "last_control_seq": 42,
  "last_executed_seq": null,
  "reject_counts": {
    "stale": 0,
    "out_of_range": 0,
    "manual_override": 0,
    "mode_not_allowed": 0,
    "safety_triggered": 0
  }
}
```

`agent_status` 参数语义：

| 参数 | 语义 |
| --- | --- |
| `execution_mode` | 代理本地真实运行阶段。以代理本地配置为准，不以控制主机声明为准。 |
| `connected_to_fcu` | 代理是否连接真实飞控。 |
| `manual_enable` | 安全员是否允许网络控制写入飞控。 |
| `write_permission` | 当前阶段和状态是否允许写控制命令。 |
| `last_control_seq` | 最近收到的控制帧序号。 |
| `last_executed_seq` | 最近实际执行的控制帧序号。 |
| `reject_counts` | 按原因累计的拒绝次数，用于联调定位。 |

### 9.4 `safety_event`

```json
{
  "protocol": "flight_bridge_sim2real",
  "schema_version": 1,
  "msg_type": "safety_event",
  "session_id": "20260611T120000Z-x3-to-real",
  "seq": 103,
  "sent_time_unix_s": 1781160002.300,
  "monotonic_ms": 2300.0,
  "severity": "warning",
  "event": "command_timeout",
  "detail": "No executable control frame for 280 ms",
  "fallback": {
    "activated": true,
    "mode": "release_rc_override"
  }
}
```

`critical` 事件发生后，真机代理必须拒绝后续控制帧，直到本地复位。

`safety_event` 参数语义：

| 参数 | 语义 |
| --- | --- |
| `severity` | 安全事件级别。`critical` 表示必须人工/本地复位。 |
| `event` | 机器可读事件名，例如 `command_timeout`、`manual_override`、`out_of_range`。 |
| `detail` | 人类可读细节，用于现场排查。 |
| `fallback.activated` | 是否已经进入本地 fallback。 |
| `fallback.mode` | 本地 fallback 行为，例如释放 RC override、切 hold、切 LOITER。 |

## 10. 心跳与握手

控制主机连接后发送 `hello`，代理回复 `hello_ack`。双方每 100 ms 到 500 ms 发送 `heartbeat`。

`hello` 示例：

```json
{
  "protocol": "flight_bridge_sim2real",
  "schema_version": 1,
  "msg_type": "hello",
  "session_id": "20260611T120000Z-x3-to-real",
  "seq": 0,
  "sent_time_unix_s": 1781160000.0,
  "monotonic_ms": 10.0,
  "client": {
    "name": "flight_bridge",
    "control_rate_hz": 20.0
  },
  "requested_capabilities": ["rc_override", "real_state", "agent_status"]
}
```

`hello_ack` 示例：

```json
{
  "protocol": "flight_bridge_sim2real",
  "schema_version": 1,
  "msg_type": "hello_ack",
  "session_id": "20260611T120000Z-x3-to-real",
  "seq": 0,
  "sent_time_unix_s": 1781160000.02,
  "monotonic_ms": 50.0,
  "accepted": true,
  "agent": {
    "name": "real_vehicle_agent",
    "vehicle_id": "quad-alpha",
    "mavlink_connection": "serial:/dev/ttyUSB0:57600"
  },
  "capabilities": ["rc_override", "hold", "real_state", "agent_status"],
  "limits": {
    "max_rate_hz": 50.0,
    "max_command_age_ms": 100.0,
    "control_timeout_ms": 250.0,
    "heartbeat_timeout_ms": 500.0
  }
}
```

无线真机测试必须运行在可信网络、VPN 或 TLS 隧道内。不要在公共网络裸露控制端口。

## 11. 分阶段启用

### Stage 0: shadow

- 真机代理连接真实飞控或 mock 飞控。
- 控制主机发送完整 `control_frame`。
- 真机代理只 ACK 和记录，不写飞控。
- 验证 `frame_record` 与本地 JSONL 日志逐字段一致。

### Stage 1: real-state dry run

- 真机代理回传 `real_state`。
- 控制主机支持用 `real_state.state` 构造 `CopterState`。
- 仍不写飞控。
- 对比真机状态和 SITL/Gazebo 状态的尺度、方向、延迟。

### Stage 2: bench no prop

- 拆除桨叶。
- 真机代理允许写 RC override。
- 只允许 `BASELINE_ONLY` 或极小 `residual_lambda`。
- 验证通道顺序、通道方向、人工接管和超时 fallback。

### Stage 3: tether low altitude

- 系留低空。
- 使用 `flight_bridge/config.yaml` 的 `limits_real`。
- `residual_lambda` 从非常小的值开始。
- 每次只做短时间悬停。

### Stage 4: free low altitude

- 低空自由飞。
- 必须有人工接管和物理 kill switch。
- 逐步扩大限幅和任务复杂度。

## 12. 实现建议

建议新增：

```text
flight_bridge/sim2real_protocol.py
  ProtocolHeader dataclass
  control_frame encode/decode
  frame_record validation
  RcCommand/VelocityCmd list validators

flight_bridge/sim2real_client.py
  TCP JSONL client
  hello/heartbeat/control_frame send
  ack/status/real_state receive

real_vehicle_agent/main.py
  TCP JSONL server
  MAVLink connection
  local safety gate
  RC override execution
  real_state/command_ack/agent_status/safety_event publisher
```

第一版不要把真机代理塞进现有 `MavlinkAdapter`。先让通信边界独立存在，跑通 shadow 和字段对齐，再决定是否把它变成 `MavlinkAdapter` 的一个 backend。

## 13. 对齐检查清单

实现前后都要检查：

- `frame_record` 能直接追加进现有 JSONL 日志，不需要字段转换。
- `safe_command` 在 RC 模式下等于 `RcCommand.to_list()` 顺序。
- guided 模式下的 `safe_command` 等于 `VelocityCmd.to_list()` 顺序。
- `shield` 字段只包含当前 `ShieldInfo` 语义。
- 真机代理 ACK 不改写源端 `sent`，只通过 `command_ack.executed` 表示真实执行。
- `real_state.state` 能被映射回 `FlightState`。
- `action` 保留完整 10 维。
- `timeout_discard`、`state_age_ms`、`error`、`gazebo_reset`、`reset_recovery` 都参与真机拒绝规则。
