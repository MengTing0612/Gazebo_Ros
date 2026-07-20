# FastAPI 控制服务操作与数据传输说明

本文说明 `flight_bridge/api.py` 提供的 FastAPI 控制服务怎么启动、怎么调用、数据如何在 HTTP、SDK、MAVLink 和日志之间传输。

## 1. 服务定位

FastAPI 服务是现有 Phase 1 控制链路的 HTTP 包装层。它不替代 ArduCopter，也不改变原来的 `python -m flight_bridge.main` 命令行入口。

主链路如下：

```text
HTTP /control/start
    -> FlightControlService 后台线程
    -> MavlinkAdapter.read_state()
    -> FlightState
    -> CopterState
    -> obs16 + feedback1
    -> Stabilize/Gaia SDK
    -> action10
    -> ActionAdapter
    -> baseline_rc / candidate_rc
    -> SafetyShield.filter_rc()
    -> safe_rc
    -> MavlinkAdapter.send_rc_cmd()
    -> MAVLink RC_CHANNELS_OVERRIDE
    -> ArduCopter
```

默认安全策略：

- `mode` 默认是 `"dry-run"`，不会写 MAVLink。
- `sdk_mode` 默认是 `"mock"`，不需要真实 SDK 服务。
- 只有 `/control/start` 请求显式传 `mode: "sitl"` 时，服务才会连接 MAVLink 并发送 RC override。
- 当前 API 主输出后端固定为 RC override，对应 `control="rc"`。

## 2. 启动服务

在项目根目录运行：

```bash
python3 -m uvicorn flight_bridge.api:app --host 127.0.0.1 --port 8000
```

启动成功后会看到类似输出：

```text
Uvicorn running on http://127.0.0.1:8000
```

浏览器可打开：

```text
http://127.0.0.1:8000/docs
```

这里是 FastAPI 自动生成的接口页面，可以直接填写 JSON 请求体调用接口。

## 3. 接口概览

| 接口 | 方法 | 作用 | 是否写飞控 |
|---|---|---|---|
| `/health` | GET | 查看服务是否活着 | 否 |
| `/control/start` | POST | 启动后台控制循环 | 只有 `mode="sitl"` 才写 |
| `/control/stop` | POST | 停止后台控制循环 | SITL 模式停止时发送低油门保护 |
| `/control/status` | GET | 查看当前帧、状态、obs/action/safe command | 否 |
| `/control/step` | POST | 单帧结构化状态调试 | 默认否，运行中的 SITL 会话且 `send=true` 才可能写 |

## 4. Dry-run 操作流程

Dry-run 用于确认 API、数据结构、obs、feedback、action、日志都正常，不连接飞控。

启动后台循环：

```bash
curl -X POST http://127.0.0.1:8000/control/start \
  -H "Content-Type: application/json" \
  -d '{
    "mode": "dry-run",
    "sdk_mode": "mock",
    "rate_hz": 20.0,
    "iterations": 100,
    "target_alt_m": 3.0
  }'
```

返回示例：

```json
{
  "running": true,
  "mode": "dry-run",
  "control": "rc",
  "sdk_mode": "mock",
  "rate_hz": 20.0,
  "log_file": "logs/flight_bridge_api_20260617-152119.jsonl",
  "writes_enabled": false
}
```

查看当前状态：

```bash
curl http://127.0.0.1:8000/control/status
```

重点字段：

- `running`：后台循环是否还在运行。
- `frame`：最新帧号。
- `latest_state`：原始 `FlightState`。
- `latest_copter_state`：统一后的机体坐标状态。
- `latest_obs`：发送给 SDK 的 16 维观测。
- `latest_feedback`：发送给 SDK 的 1 维反馈。
- `latest_action`：SDK 返回的动作。
- `latest_safe_command`：经过安全层后的 RC PWM。
- `latest_error`：最近错误。

停止后台循环：

```bash
curl -X POST http://127.0.0.1:8000/control/stop
```

Dry-run 日志中 `sent` 应为 `false`，表示没有写飞控。

## 5. SITL 操作流程

SITL 模式会连接 MAVLink，并通过 RC override 写入 ArduCopter。建议先确认 SITL 已启动并能从 `tcp:127.0.0.1:5760` 收到 heartbeat。

可先用已有命令检查 MAVLink：

```bash
python3 -c "from pymavlink import mavutil; mav=mavutil.mavlink_connection('tcp:127.0.0.1:5760', robust_parsing=True); msg=mav.recv_match(type='HEARTBEAT', blocking=True, timeout=12); print(msg)"
```

确认后启动 API 控制：

```bash
curl -X POST http://127.0.0.1:8000/control/start \
  -H "Content-Type: application/json" \
  -d '{
    "mode": "sitl",
    "connect": "tcp:127.0.0.1:5760",
    "sdk_mode": "mock",
    "rate_hz": 20.0,
    "target_alt_m": 3.0,
    "rc_control_mode": "residual",
    "use_gazebo_truth": false
  }'
```

返回中的 `writes_enabled` 会是 `true`：

```json
{
  "running": true,
  "mode": "sitl",
  "control": "rc",
  "sdk_mode": "mock",
  "rate_hz": 20.0,
  "log_file": "logs/flight_bridge_api_20260617-153000.jsonl",
  "writes_enabled": true
}
```

查看状态：

```bash
curl http://127.0.0.1:8000/control/status
```

SITL 正常写入时，JSONL 日志里一般会看到：

```json
{
  "mode": "sitl",
  "control": "rc",
  "sent": true,
  "cmd_method": "RC_CHANNELS_OVERRIDE",
  "safe_command": [1500, 1500, 1700, 1500]
}
```

停止控制：

```bash
curl -X POST http://127.0.0.1:8000/control/stop
```

停止时，如果当前是 SITL 写入模式，服务会发送多次低油门 `emergency_stop()`，避免 RC override 停在上一次高油门。

## 6. 使用真实 SDK

真实 SDK 连接参数通过 `/control/start` 请求体传入：

```json
{
  "mode": "sitl",
  "connect": "tcp:127.0.0.1:5760",
  "sdk_mode": "real",
  "host": "127.0.0.1",
  "port": 9000,
  "timeout_s": 5.0,
  "username": "admin",
  "password": "admin123",
  "resume": true,
  "rate_hz": 20.0,
  "target_alt_m": 3.0
}
```

SDK 传输协议仍是原来的 JSON-lines TCP：

请求：

```json
{"obs":[...16 floats...],"feedback":[...1 float...]}
```

响应：

```json
{"status":"success","action":[...10 floats...]}
```

FastAPI 不直接替代 SDK。FastAPI 是外层控制服务；SDK 仍由 `flight_bridge/sdk_client.py` 用 TCP JSON-lines 调用。

## 7. 单帧 step 调试

`/control/step` 用于外部直接提交结构化 `CopterState`，服务返回一帧 obs、feedback、action 和 safe RC command。

示例：

```bash
curl -X POST http://127.0.0.1:8000/control/step \
  -H "Content-Type: application/json" \
  -d '{
    "sdk_mode": "mock",
    "x_err_m": 0.0,
    "y_err_m": 0.0,
    "z_err_m": 0.8,
    "vx_mps": 0.0,
    "vy_mps": 0.0,
    "vz_mps": 0.0,
    "roll_rad": 0.0,
    "pitch_rad": 0.0,
    "yaw_err_rad": 0.0,
    "p_rad_s": 0.0,
    "q_rad_s": 0.0,
    "r_rad_s": 0.0,
    "altitude_m": 0.2,
    "armed": true,
    "mode": "STABILIZE"
  }'
```

返回示例：

```json
{
  "obs": [],
  "feedback": [0.5],
  "feedback_diagnostics": {},
  "action": [],
  "latency_ms": 0.0,
  "baseline_command": [1500, 1500, 1764, 1500],
  "candidate_command": [1500, 1500, 1764, 1500],
  "candidate_command_struct": {
    "roll_pwm": 1500,
    "pitch_pwm": 1500,
    "throttle_pwm": 1764,
    "yaw_pwm": 1500
  },
  "safe_command": [1500, 1500, 1764, 1500],
  "safe_command_struct": {
    "roll_pwm": 1500,
    "pitch_pwm": 1500,
    "throttle_pwm": 1764,
    "yaw_pwm": 1500
  },
  "shield": {
    "saturated": false,
    "safety_triggered": false,
    "clipped_fields": [],
    "filtered": false,
    "in_deadband": false
  },
  "sent": false,
  "cmd_method": "",
  "error": null
}
```

`step` 默认不写飞控。只有当后台服务已经用 `mode="sitl"` 启动，并且 `step` 请求体里显式传 `send: true` 时，才会复用当前会话发送一帧 RC command。

## 8. 数据字段说明

### 8.1 HTTP start 请求字段

| 字段 | 默认值 | 说明 |
|---|---:|---|
| `mode` | `"dry-run"` | `"dry-run"` 不写飞控，`"sitl"` 连接 MAVLink 并允许发送 |
| `connect` | `""` | MAVLink 地址，空值使用 `tcp:127.0.0.1:5760` |
| `sdk_mode` | `"mock"` | `"mock"` 本地假 SDK，`"real"` 连接真实 SDK |
| `host` | `"127.0.0.1"` | SDK TCP host |
| `port` | `9000` | SDK TCP port |
| `timeout_s` | `5.0` | SDK socket 超时 |
| `rate_hz` | `20.0` | 后台控制频率 |
| `iterations` | `0` | 循环帧数，`0` 表示一直运行 |
| `target_alt_m` | `3.0` | 目标悬停高度 |
| `target_heading_deg` | `0.0` | 目标航向 |
| `config` | `flight_bridge/config.yaml` | 控制、安全、反馈配置 |
| `log_file` | `null` | 指定 JSONL 路径；为空则自动写到 `logs/` |
| `use_real_limits` | `false` | 是否使用 real limits |
| `rc_control_mode` | `"residual"` | `"residual"`、`"sdk-direct"`、`"baseline-only"` |
| `adapter_mode` | `"front-4"` | `"front-4"` 或 `"learned"` |
| `gaia_adapter_config` | `flight_bridge/gaia_adapter.yaml` | learned adapter YAML |
| `use_gazebo_truth` | `false` | 是否读取 Gazebo odometry 作为真值 |
| `gazebo_odom_topic` | `/model/x3/odometry` | Gazebo odometry topic |

### 8.2 CopterState 输入字段

`/control/step` 的结构化状态字段使用明确单位：

| 字段 | 单位 | 说明 |
|---|---|---|
| `x_err_m` | m | 机体系前向位置误差 |
| `y_err_m` | m | 机体系左向位置误差 |
| `z_err_m` | m | 向上高度误差 |
| `vx_mps` | m/s | 前向速度 |
| `vy_mps` | m/s | 左向速度 |
| `vz_mps` | m/s | 向上速度 |
| `roll_rad` | rad | 横滚 |
| `pitch_rad` | rad | 俯仰 |
| `yaw_err_rad` | rad | 航向误差 |
| `p_rad_s` | rad/s | 横滚角速度 |
| `q_rad_s` | rad/s | 俯仰角速度 |
| `r_rad_s` | rad/s | 偏航角速度 |
| `altitude_m` | m | 当前高度 |
| `armed` | bool | 飞控是否解锁 |
| `mode` | string | 飞控模式名 |

### 8.3 后台循环内部字段

后台循环读到 MAVLink 后会先得到 `FlightState`，再转换为 `CopterState`。

`obs` 是 16 维：

```text
0..2   tanh(position error / scale)
3..5   tanh(velocity / scale)
6..8   tanh(attitude / scale)
9..11  tanh(angular rate / scale)
12..15 previous RC delta, normalized around neutral
```

`feedback` 是 1 维，由 `HoverFeedbackBuilder` 根据悬停误差、速度、倾角、饱和、安全触发等计算。

`action` 是 SDK 返回的动作，当前按 10 维保留。

RC 输出字段：

```text
baseline_rc       稳定悬停基线 RC
candidate_command baseline + SDK residual 后的候选 RC
safe_command      经过 SafetyShield 后的最终 RC
rc_delta          safe_command 相对 neutral 的归一化差值
```

## 9. 日志传输与保存

每帧写一行 JSON 到 `log_file`。默认路径类似：

```text
logs/flight_bridge_api_YYYYMMDD-HHMMSS.jsonl
```

单帧日志包含：

```json
{
  "ts": 0.0,
  "frame": 0,
  "phase": 1,
  "api": true,
  "mode": "sitl",
  "control": "rc",
  "rc_control_mode": "residual",
  "vehicle_mode": "STABILIZE",
  "armed": true,
  "state": {},
  "gazebo_odom": null,
  "copter_state": {},
  "obs": [],
  "feedback": [],
  "feedback_diagnostics": {},
  "action": [],
  "latency_ms": 0.0,
  "state_age_ms": 0.0,
  "gazebo_age_ms": null,
  "timeout_discard": false,
  "baseline_rc": [1500, 1500, 1700, 1500],
  "candidate_command": [1500, 1500, 1700, 1500],
  "safe_command": [1500, 1500, 1700, 1500],
  "rc_delta": [0.0, 0.0, 0.0, 0.0],
  "shield": {
    "filtered": false,
    "saturated": false,
    "safety_triggered": false,
    "clipped_fields": [],
    "in_deadband": false
  },
  "sent": true,
  "cmd_method": "RC_CHANNELS_OVERRIDE",
  "error": null
}
```

可以继续用已有可视化工具观察日志：

```bash
python tools/visualize_feedback.py --watch logs/
```

## 10. 排错

### 10.1 `/control/start` 返回 409

说明后台控制循环已经在运行。先停止：

```bash
curl -X POST http://127.0.0.1:8000/control/stop
```

### 10.2 `/control/start` 返回 MAVLink 连接失败

检查：

- SITL 是否已经启动。
- `connect` 是否正确。
- `tcp:127.0.0.1:5760` 是否能收到 `HEARTBEAT`。

### 10.3 status 里 `latest_error` 非空

常见原因：

- 真实 SDK 没启动，但请求用了 `"sdk_mode": "real"`。
- SDK host/port/用户名/密码错误。
- MAVLink 状态太旧，触发 fallback。
- Gazebo truth 开启但 odometry topic 没有数据。

### 10.4 Dry-run 里 `sent=false` 是正常的

Dry-run 只计算 `safe_command`，不写飞控。判断是否真写 MAVLink，看：

```json
{
  "mode": "sitl",
  "sent": true,
  "cmd_method": "RC_CHANNELS_OVERRIDE"
}
```

### 10.5 `fastapi.testclient.TestClient` 卡住

这只影响本地自动测试方式，不影响真实运行。真实服务走 `uvicorn`，真机数据读取走 `MavlinkAdapter + pymavlink`，不经过 `TestClient`。

## 11. 最小推荐流程

先跑 dry-run：

```bash
python3 -m uvicorn flight_bridge.api:app --host 127.0.0.1 --port 8000
curl -X POST http://127.0.0.1:8000/control/start \
  -H "Content-Type: application/json" \
  -d '{"mode":"dry-run","sdk_mode":"mock","rate_hz":20,"iterations":100}'
curl http://127.0.0.1:8000/control/status
```

确认 `latest_obs`、`latest_action`、`latest_safe_command` 都存在后，再进入 SITL：

```bash
curl -X POST http://127.0.0.1:8000/control/start \
  -H "Content-Type: application/json" \
  -d '{
    "mode":"sitl",
    "connect":"tcp:127.0.0.1:5760",
    "sdk_mode":"mock",
    "rate_hz":20,
    "target_alt_m":3.0
  }'
```

最后再把 `sdk_mode` 从 `"mock"` 切成 `"real"`，接入真实 SDK。

