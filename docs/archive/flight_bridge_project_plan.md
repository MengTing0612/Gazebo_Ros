# Flight Bridge 项目推进计划

## 0. 当前状态

当前项目还没有连接无人车/无人机查看实际输出，因此现阶段目标不是闭环控制，而是先把 **SDK 推理输出可观测、可记录、可复现** 做扎实。

本计划与 `docs/flight_bridge_implementation.md` 保持一致：

- Phase 0/1/2 默认 `--control off`，不发送任何 MAVLink 写控制。
- `RC_CHANNELS_OVERRIDE` 只作为 Phase 3 的低速调试通道。
- 本地 APMrover2 默认通道按 `ch1=steering, ch3=throttle` 处理，实际写控制前必须再校准 RCMAP、trim 和正反向。

## 1. 总体原则

1. 先观测，再控制。
2. 先 dry-run，再 SITL，只读通过后再考虑写控制。
3. 所有推理输入、输出和延迟必须落盘记录，避免只看终端输出做判断。
4. action 语义未确认前，不把任意 action 维度直接映射成 steering/throttle。
5. 真机接入第一阶段也只读，不能直接启用 RC override。

## 2. 阶段路线

### Phase 0: 本地 SDK dry-run

目标：不连接飞控，只验证 SDK 通信、obs 构造、action 输出和推理延迟。

输入：

- dry-run 模拟 `FlightState`
- 临时 8 维 obs
- 临时 feedback

输出：

- 终端打印 obs/action/latency
- JSONL 日志
- action 维度和值域统计

建议命令：

```bash
./examples/retina-server-x86_64
python flight_bridge/main.py --mode dry-run --control off
```

不启动 SDK 服务时，可以先用 mock 客户端做本地冒烟测试：

```bash
python flight_bridge/main.py --sdk-mode mock --iterations 20 --rate-hz 20 --control off
```

验收标准：

- SDK 能连续推理 5-10 分钟。
- 每帧记录 `timestamp`、`obs`、`feedback`、`action`、`latency_ms`。
- 确认 action 维度。
- 确认 action 值域是否接近 `[-1, 1]`。
- p95 推理延迟满足目标控制频率，例如 10-20Hz。
- 无任何 MAVLink 写控制路径被调用。

### Phase 1: Rover SITL 只读

目标：连接 Rover SITL，只读 MAVLink 状态，并继续观察 SDK action。

输入：

- SITL 的 `HEARTBEAT`
- `ATTITUDE`
- `VFR_HUD`
- `GLOBAL_POSITION_INT`
- `RC_CHANNELS`
- `SERVO_OUTPUT_RAW`

输出：

- 真实飞控状态
- 真实状态构造出的 obs
- SDK action
- 延迟与丢包/超时记录

建议命令：

```bash
cd ~/ardupilot/Rover
../Tools/autotest/sim_vehicle.py --map --console
python flight_bridge/main.py --mode sitl --control off
```

验收标准：

- 能稳定收到 HEARTBEAT。
- 能识别 mode、armed、heading、groundspeed。
- obs 构造不出现 NaN/Inf。
- action 日志连续。
- `--control off` 下不会发送 `RC_CHANNELS_OVERRIDE`。

### Phase 1.5: RC/PWM 校准

目标：为后续 Phase 3 的低速 RC override 建立真实安全边界。

必须确认：

- steering 通道。
- throttle 通道。
- steering center/min/max。
- throttle stop/trim/min/max。
- throttle 正反向。
- 停发 RC override 后飞控行为。
- STEERING 模式下 RC override 是否被接受。

当前本地 APMrover2 参考：

```text
ch1 = steering
ch3 = throttle
```

但最终以 SITL 参数和 RCMAP 为准。

验收标准：

- `config/safety_limits.yaml` 中的通道和值来自校准，不是猜测。
- emergency stop 使用 `throttle=stop/trim`，不把 `1000` 直接当成安全停止。
- 形成一份校准记录，包括每个 PWM 对应的车辆响应。

### Phase 3: Rover SITL 低速 RC override

目标：只在 SITL 中启用低速 RC override，验证闭环链路。

启用条件：

- Phase 1 只读稳定。
- Phase 1.5 RC/PWM 校准完成。
- Phase 2 action 语义分析完成。
- safety limits 已更新。
- 命令显式带 `--control rc`。

建议初始限制：

```yaml
steering:
  channel: 1
  center: 1500
  min: 1300
  max: 1700
throttle:
  channel: 3
  stop: 1500
  min: 1450
  max: 1550
```

建议命令：

```bash
python flight_bridge/main.py --mode sitl --control rc
```

验收标准：

- 车辆在 SITL 地图中响应方向合理。
- 超时后能回中/停发。
- 模式切到非 STEERING 时拒绝发送。
- 日志能对应每次 action、safe command 和车辆状态变化。

### Phase 4: 后续扩展

目标：根据 action 语义决定是否进入更正规的飞控接入方式。

优先方向：

- 如果 action 可解释成目标速度/航向/局部目标，转向 Guided setpoint 后端。
- 如果 action 只能解释成低层摇杆量，继续限制在 SITL 或低风险实车测试。
- 参数调优、真机闭环、Copter 都必须单独评审。

## 3. 近期任务清单

### 第 1 批：本地可运行骨架

- 创建 `flight_bridge/` 包。
- 实现 `sdk_client.py`，复用 `examples/sdk_comm/stabilize_sdk_tcp_demo.py` 的 TCP JSON Lines 协议。
- 实现 `mavlink_adapter.py` 的 dry-run 模式。
- 实现 `observation_builder.py`。
- 实现 `main.py --mode dry-run --control off`。
- 实现 JSONL logger。

完成标准：

- 不启动 SITL 也能跑通 SDK 推理。
- 能生成一份可分析的日志文件。

### 第 2 批：action 探测工具

- 这批暂缓，等后续需要重新评估 action 语义时再恢复。

### 第 3 批：SITL 只读

- 安装或确认 ArduPilot SITL。
- 实现 `mavlink_adapter.py` 的 SITL 连接。
- 解析 `HEARTBEAT`、`ATTITUDE`、`VFR_HUD`、`GLOBAL_POSITION_INT`。
- 记录真实 state -> obs -> action。

完成标准：

- `--mode sitl --control off` 稳定运行。
- 不发送任何 RC override。

### 第 4 批：安全控制准备

- 实现 `action_mapper.py`，但默认不启用。
- 实现 `safety_guard.py`。
- 创建 `config/safety_limits.yaml`。
- 加入 `--control off|rc` 硬开关。
- 加入模式检查和超时处理。

完成标准：

- `control=off` 下任何发送函数都会拒绝写控制。
- 单元测试或 smoke test 覆盖限幅、通道映射、非 STEERING 拒绝。

## 4. 日志字段规范

每帧至少记录：

```json
{
  "ts": 0.0,
  "mode": "dry-run",
  "control": "off",
  "vehicle_mode": "STEERING",
  "armed": false,
  "state": {
    "heading": 45.0,
    "groundspeed": 0.0,
    "roll": 0.0,
    "pitch": 0.0,
    "yaw": 0.0
  },
  "obs": [],
  "feedback": [],
  "action": [],
  "latency_ms": 0.0,
  "candidate_command": null,
  "safe_command": null,
  "sent": false,
  "error": null
}
```

Phase 3 才允许 `sent=true`。

## 5. 风险与门槛

| 风险 | 处理方式 |
|---|---|
| action 语义未知 | Phase 2 前不写控制 |
| throttle 方向不确定 | Phase 1.5 校准，默认 stop=trim |
| ch2/ch3 通道混淆 | 本地 APMrover2 按 ch1/ch3，实际以 RCMAP 为准 |
| SDK 延迟过高 | 先记录 latency p50/p95/p99，再决定控制频率 |
| 模式误判 | 只允许 STEERING 写控制 |
| SDK 断连 | control=off 只记录；control=rc 回中/停发 |
| 真机风险 | 真机第一阶段只读，不启用 RC override |

## 6. 下一步建议

下一步先实现 Phase 0 的最小闭环：

```text
dry-run FlightState
  -> ObservationBuilder
  -> SdkClient.infer()
  -> JSONL logger
  -> terminal summary
```

这一步完成后，再决定是否进入 SITL 只读。当前不建议直接连接真机，也不建议实现 RC override 的发送路径作为默认行为。
