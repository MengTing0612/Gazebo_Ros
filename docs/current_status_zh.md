# Flight Bridge 当前状态

本文是当前 `flight_bridge` 工作流的简短事实来源。较早的 Rover-only MVP 说明已经归档到 `docs/archive/`。

## 当前范围

当前主路径是 Gazebo X3 + ArduCopter 2.74b SITL + Stabilize/Gaia SDK。
SDK 被用作一个小权重的外环 residual 信号。ArduCopter 仍然负责姿态稳定、姿态控制和电机混控。

主控制链路：

```text
Gazebo odometry / MAVLink state
    -> CopterState
    -> obs16 + sdk_feedback1
    -> Stabilize/Gaia SDK
    -> action10
    -> ActionAdapter
    -> RC residual command
    -> SafetyShield
    -> MAVLink RC_CHANNELS_OVERRIDE
    -> ArduCopter SITL
```

代码里保留了现代 guided velocity 控制作为实验后端，但它不是 ArduCopter 2.74b 的首选路径。当前更可靠的一线控制入口是：围绕 ArduCopter baseline 的 RC override。

## 主要文档

当前设计参考优先看这些：

- `docs/stabilize_ai_sdk_drone_control_design.md`：总体架构、安全策略和控制策略。
- `docs/gaia_drone_task_adaptation_design.md`：Gaia 下游无人机任务适配策略，包括分阶段 obs/feedback、reset 处理和 residual adapter 方向。
- `docs/fastapi_control_service.md`：FastAPI 控制服务操作指南，包括 HTTP API 示例、数据传输链路、日志字段和 SITL/SDK 用法。

分析附录：

- `docs/gaia_action_contribution_implementation.md`：action10 贡献度拟合说明，以及 learned adapter 的 A/B 判断标准。
- `docs/hover_anchor_residual_plan.md`：当前 hover 任务定义、reset/home XY anchor 方案、baseline-first 测试顺序和 SDK residual 推进标准。
- `docs/open_source_control_integration_research.md`：成熟开源飞控集成方案，以及逐步迁移出手写 RC XY 控制的推荐路线。
- `docs/sim2real_control_interface_spec.md`：v0.2 通信契约，用于把仿真/SDK 侧 safe command 发给真实载具 agent。

归档的 Rover 阶段文档：

- `docs/archive/flight_bridge_implementation.md`
- `docs/archive/flight_bridge_project_plan.md`

## 常用命令

Dry-run SDK 冒烟测试：

```bash
python -m flight_bridge.main --phase 0 --mode dry-run --control off --sdk-mode mock --iterations 100
```

Phase 1 SITL，RC residual 路径：

```bash
python -m flight_bridge.main \
  --phase 1 \
  --mode sitl \
  --control rc \
  --sdk-mode mock \
  --target-alt 3.0 \
  --reset-on-flip
```

推荐的无限 SITL 运行方式：Gaia SDK、Gazebo GUI、residual RC、Gazebo truth、post-reset guard 和实时可视化：

```bash
scripts/start_residual_infinite_test.sh
```

`scripts/start_sdk_direct_infinite_test.sh` 作为兼容入口保留，但现在它也会启动 residual 路径。`sdk-direct` 只作为诊断模式保留；它会把 `action[0..3]` 直接解释成飞机摇杆量，不应该作为主稳定性评估方式。

如果要使用真实 SDK server，把手动命令里的 `--sdk-mode mock` 替换成配置好的 SDK 模式和凭据。`--control rc` 要保持显式填写；控制方式不会被隐式推断。

实时可视化：

```bash
python tools/visualize_feedback.py --watch logs/
```

## 运行时 Feedback 语义

这里有两个相关但不同的量：

- `feedback`：输入给 SDK 的一维 feedback。Phase 1 中由 `HoverFeedbackBuilder` 根据 hover 质量构造，是 reward/cost 标量。
- `distance_m`：从 `copter_state` 还原出的真实三维物理距离：

```text
distance_m = sqrt(x_err^2 + y_err^2 + z_err^2)
```

实时可视化工具画的是物理距离 `distance_m`，不是 SDK feedback 标量，也不是 tanh 归一化后的值。

## 关键 JSONL 字段

每个 Phase 1 帧是一行 JSON object，用于在线调试和离线 action 贡献度分析。字段按“原始状态 -> 统一状态 -> SDK 输入输出 -> 控制命令 -> 安全与发送结果”的顺序保存。

### 顶层字段格式

| 字段 | 类型 / 格式 | 单位 / 取值 | 说明 |
|---|---|---|---|
| `ts` | `float` | Unix 秒 | 写日志时的墙钟时间。 |
| `frame` | `int` | 从 0 开始 | 控制循环帧号。 |
| `phase` | `int` | 当前为 `1` | Phase 1 SDK-in-the-loop 控制。 |
| `mode` | `string` | `"dry-run"` / `"sitl"` | 桥接程序运行模式。 |
| `control` | `string` | 当前主路径为 `"rc"` | 控制后端。 |
| `rc_control_mode` | `string` | `"residual"` / `"sdk-direct"` / `"baseline-only"` | RC 路径动作解释模式。 |
| `vehicle_mode` | `string` | 例如 `"STABILIZE"`、`"LOITER"` | 飞控当前模式，来自 MAVLink heartbeat。 |
| `armed` | `bool` | `true` / `false` | 飞控是否解锁。 |
| `state` | `object` | 见下表 | 原始 MAVLink/Dry-run 状态。 |
| `gazebo_odom` | `object` / `null` | 见下表 | Gazebo odometry 真值；未启用或无数据时为 `null`。 |
| `copter_state` | `object` | 见下表 | 统一后的 body-FLU 状态，是 obs/feedback 的主要来源。 |
| `obs` | `array[float]`，长度 16 | 大多在 `[-1, 1]` | 输入给 SDK 的观测向量。 |
| `feedback` | `array[float]`，长度 1 | 通常 `[0, 1]` | 输入给 SDK 的反馈标量数组。 |
| `feedback_diagnostics` | `object` / `null` | 见下表 | feedback 的分项诊断。 |
| `action` | `array[float]`，长度 10 | 通常 `[-1, 1]` | SDK 输出动作；贡献度分析必须保留完整 10 维。 |
| `latency_ms` | `float` | ms | SDK 推理耗时。 |
| `state_age_ms` | `float` | ms | 最旧 fast MAVLink 状态的年龄。 |
| `gazebo_age_ms` | `float` / `null` | ms | Gazebo odometry 数据年龄。 |
| `timeout_discard` | `bool` | `true` / `false` | SDK 延迟、状态过旧或 Gazebo 数据过旧时丢弃 action。 |
| `baseline_rc` | `array[int]`，长度 4 | PWM | baseline 控制器输出的 RC 命令。 |
| `candidate_command` | `array[int]`，长度 4 | PWM | baseline + SDK residual 后、安全层前的候选 RC 命令。 |
| `safe_command` | `array[int]`，长度 4 | PWM | 经过 `SafetyShield` 后最终允许发送的 RC 命令。 |
| `rc_delta` | `array[float]`，长度 4 | 归一化 RC 偏移 | `safe_command` 相对 neutral 的差值，约按 `/500` 归一化。 |
| `shield` | `object` | 见下表 | 安全层处理结果。 |
| `sent` | `bool` | `true` / `false` | 是否实际写入 MAVLink；dry-run 中应为 `false`。 |
| `cmd_method` | `string` | 例如 `"RC_CHANNELS_OVERRIDE"` | 实际发送方法；未发送时可为空字符串。 |
| `error` | `string` / `null` | - | 本帧错误信息。 |

### `state` 原始状态格式

`state` 来自 `FlightState.to_log_dict()`。角度除 `heading` 外均为弧度。

| 字段 | 类型 | 单位 | 说明 |
|---|---|---|---|
| `roll` / `pitch` / `yaw` | `float` | rad | 姿态角。 |
| `roll_speed` / `pitch_speed` / `yaw_speed` | `float` | rad/s | 机体角速度。 |
| `heading` | `float` | deg | 航向角。 |
| `groundspeed` | `float` | m/s | 地速。 |
| `climb` | `float` | m/s | 垂直速度，向上为正。 |
| `lat` / `lon` | `float` | deg | GPS 经纬度。 |
| `relative_alt` | `float` | m | 相对高度。 |
| `mode` | `string` | - | 飞控模式。 |
| `armed` | `bool` | - | 是否解锁。 |
| `battery_voltage` | `float` | mV | 电压字段，具体来源取决于 MAVLink 消息。 |
| `heading_error_deg` | `float` | deg | Phase 0/旧路径使用的航向误差。 |
| `attitude_ts` / `position_ts` / `vfr_ts` / `heartbeat_ts` | `float` | Unix 秒 | 各类 MAVLink 消息最近更新时间。 |

### `gazebo_odom` 真值格式

启用 Gazebo truth 时记录；否则为 `null`。

| 字段 | 类型 | 单位 | 说明 |
|---|---|---|---|
| `x` / `y` / `z` | `float` | m | Gazebo world 坐标位置。 |
| `vx` / `vy` / `vz` | `float` | m/s | Gazebo world 线速度。 |
| `wx` / `wy` / `wz` | `float` | rad/s | Gazebo world 角速度。 |
| `roll` / `pitch` / `yaw` | `float` | rad | 从 Gazebo quaternion 转出的姿态角。 |
| `updated_at` | `float` | Unix 秒 | odometry 最近更新时间。 |
| `rel_x` / `rel_y` / `rel_z` | `float` | m | 如果已记录 home，则是相对 home 的位移。 |

### `copter_state` 统一状态格式

`copter_state` 是控制和学习侧最重要的结构，坐标系为 body-FLU：x 前、y 左、z 上。

| 字段 | 类型 | 单位 | 说明 |
|---|---|---|---|
| `x_err` | `float` | m | 机体系前向位置误差。 |
| `y_err` | `float` | m | 机体系左向位置误差。 |
| `z_err` | `float` | m | 向上高度误差，目标高度减当前高度。 |
| `vx` | `float` | m/s | 机体系前向速度。 |
| `vy` | `float` | m/s | 机体系左向速度。 |
| `vz` | `float` | m/s | 向上速度。 |
| `roll` | `float` | rad | 横滚角。 |
| `pitch` | `float` | rad | 俯仰角。 |
| `yaw_err` | `float` | rad | 航向误差。 |
| `p` | `float` | rad/s | 横滚角速度。 |
| `q` | `float` | rad/s | 俯仰角速度。 |
| `r` | `float` | rad/s | 偏航角速度。 |
| `altitude_m` | `float` | m | 当前高度。 |

### `obs` 数组格式

`obs` 固定 16 维，传给 SDK：

| 下标 | 含义 | 来源 | 归一化 |
|---:|---|---|---|
| 0 | `x_err` | `copter_state.x_err` | `tanh(value / 2.0m)` |
| 1 | `y_err` | `copter_state.y_err` | `tanh(value / 2.0m)` |
| 2 | `z_err` | `copter_state.z_err` | `tanh(value / 2.0m)` |
| 3 | `vx` | `copter_state.vx` | `tanh(value / 1.0m/s)` |
| 4 | `vy` | `copter_state.vy` | `tanh(value / 1.0m/s)` |
| 5 | `vz` | `copter_state.vz` | `tanh(value / 1.0m/s)` |
| 6 | `roll` | `copter_state.roll` | `tanh(value / 30deg)` |
| 7 | `pitch` | `copter_state.pitch` | `tanh(value / 30deg)` |
| 8 | `yaw_err` | `copter_state.yaw_err` | `tanh(value / 30deg)` |
| 9 | `p` | `copter_state.p` | `tanh(value / 180deg/s)` |
| 10 | `q` | `copter_state.q` | `tanh(value / 180deg/s)` |
| 11 | `r` | `copter_state.r` | `tanh(value / 180deg/s)` |
| 12 | 上一帧 roll RC delta | `last_rc_delta[0]` | 已归一化 |
| 13 | 上一帧 pitch RC delta | `last_rc_delta[1]` | 已归一化 |
| 14 | 上一帧 throttle RC delta | `last_rc_delta[2]` | 已归一化 |
| 15 | 上一帧 yaw RC delta | `last_rc_delta[3]` | 已归一化 |

### `feedback` 与 `feedback_diagnostics`

`feedback` 固定是长度为 1 的数组，例如：

```json
"feedback": [0.5]
```

`feedback_diagnostics` 记录该值的来源：

| 字段 | 类型 | 说明 |
|---|---|---|
| `mode` | `string` | feedback 模式，例如 `"reward_01"`。 |
| `value` | `float` | 实际写入 `feedback[0]` 的值。 |
| `cost` | `float` | 当前帧 hover cost。 |
| `filtered_cost` | `float` | 低通后的 cost。 |
| `previous_filtered_cost` | `float` / `null` | 上一帧低通 cost。 |
| `improvement` | `float` | cost 改善量。 |
| `terms` | `object` | 分项 cost，例如 `xy`、`z`、`vxy`、`vz`、`tilt`、`rate`、`yaw`、`command`、`saturation`、`safety`。 |

### RC 命令数组格式

`baseline_rc`、`candidate_command`、`safe_command` 都是 4 维 PWM 数组：

```text
[roll_pwm, pitch_pwm, throttle_pwm, yaw_pwm]
```

含义：

| 下标 | 字段 | 类型 | 单位 / 范围 | 说明 |
|---:|---|---|---|---|
| 0 | `roll_pwm` | `int` | 1000-2000 | 横滚通道，1500 为中位。 |
| 1 | `pitch_pwm` | `int` | 1000-2000 | 俯仰通道，1500 为中位。 |
| 2 | `throttle_pwm` | `int` | 1000-2000 | 油门通道，hover 通常不是 1500，本项目 X3 常在约 1700 附近。 |
| 3 | `yaw_pwm` | `int` | 1000-2000 | 偏航通道，1500 为中位。 |

三者区别：

| 字段 | 说明 |
|---|---|
| `baseline_rc` | 基于当前高度误差、垂直速度、XY 误差和水平速度算出的保守 baseline。 |
| `candidate_command` | SDK action 经过 `ActionAdapter` 后与 baseline 混合得到的候选命令。 |
| `safe_command` | `candidate_command` 经过限幅、低通、死区和速率限制后的最终命令。 |

`rc_delta` 也是 4 维数组：

```text
[
  (safe_roll - neutral_roll) / 500,
  (safe_pitch - neutral_pitch) / 500,
  (safe_throttle - neutral_throttle) / 500,
  (safe_yaw - neutral_yaw) / 500
]
```

### `shield` 安全层格式

| 字段 | 类型 | 说明 |
|---|---|---|
| `filtered` | `bool` | 是否经过低通后发生变化。 |
| `saturated` | `bool` | 是否发生限幅或速率限制。 |
| `safety_triggered` | `bool` | 是否触发安全 fallback。 |
| `clipped_fields` | `array[string]` | 被限幅或保护的字段，例如 `rc_throttle_rate`。 |
| `in_deadband` | `bool` | 是否进入死区处理。 |

### 示例 JSONL 单帧

```json
{
  "ts": 0.0,
  "frame": 0,
  "phase": 1,
  "mode": "sitl",
  "control": "rc",
  "rc_control_mode": "residual",
  "vehicle_mode": "STABILIZE",
  "armed": true,
  "state": {},
  "gazebo_odom": {},
  "copter_state": {
    "x_err": 0.0,
    "y_err": 0.0,
    "z_err": 0.0,
    "vx": 0.0,
    "vy": 0.0,
    "vz": 0.0,
    "roll": 0.0,
    "pitch": 0.0,
    "yaw_err": 0.0,
    "p": 0.0,
    "q": 0.0,
    "r": 0.0,
    "altitude_m": 0.0
  },
  "obs": [],
  "feedback": [],
  "feedback_diagnostics": {},
  "action": [],
  "latency_ms": 0.0,
  "state_age_ms": 0.0,
  "gazebo_age_ms": 0.0,
  "timeout_discard": false,
  "baseline_rc": [1500, 1500, 1700, 1500],
  "candidate_command": [1500, 1500, 1700, 1500],
  "safe_command": [1500, 1500, 1700, 1500],
  "rc_delta": [0.0, 0.0, 0.0, 0.0],
  "shield": {
    "filtered": false,
    "saturated": false,
    "safety_triggered": false,
    "clipped_fields": []
  },
  "sent": true,
  "cmd_method": "RC_CHANNELS_OVERRIDE",
  "error": null
}
```

## 实时可视化面板

`tools/visualize_feedback.py` 显示三个面板：

1. 顶部：`3D distance (m)`，由 `copter_state` 计算；绿色线是目标距离 0。
2. 中部：红色 `z_err (m)` 和橙色 `tilt (deg)`。
3. 底部：绿色 `throttle PWM` 和紫色 `action saturation %`。

Action saturation 表示绝对值大于 `0.9` 的 action 维度占比。

## Reset 触发条件

启用 `--reset-on-flip` 后，Gazebo episode reset 可以由以下条件触发：

- 倾角过大。
- 水平 runaway。
- 地面卡住。

推荐的 SITL 脚本使用 Gazebo odometry truth 做 reset gate 和可视化指标。水平 runaway gate 只使用 XY 误差：

```text
xy_error_m = sqrt(x_err^2 + y_err^2)
```

它不包含高度误差。当前放宽后的阈值是 `--reset-xy-runaway-m 20.0`，并且需要持续 `--reset-runaway-hold-s 2.0` 秒。

当使用 Gazebo truth 且显式设置 `--target-x/--target-y` 时，XY error 相对于该目标点计算。当使用 `--no-use-gazebo-truth` 时，XY error 回退到启动或 reset 后记录的 MAVLink home 位置。

Reset 之后：

```text
触发帧:     feedback = terminal_value = 0.0
后续帧:     feedback warm-up = neutral_value = 0.5
guard窗口: --post-reset-guard-s 0.75 抑制 SDK，并发送 baseline RC
历史量:     清空 last_rc_delta/action history
```

## 当前优先级

1. 默认使用 reset/home XY 作为 hover anchor；除非显式测试固定点任务，不要硬编码 world `(0, 0)`。
2. 在提高 SDK 权重之前，先保持 baseline RC 高度控制和 XY 限速行为稳定。
3. 收集数据时使用较低 residual 权重。
4. 用日志指标评估 `front-4` 和 learned `W @ action10`：`success_rate`、`distance_p95`、`tilt_p95`、saturation 和 safety triggers。
5. 不要把 X3 参数或 learned adapter gain 当作可直接迁移到真实自定义四旋翼的参数。
