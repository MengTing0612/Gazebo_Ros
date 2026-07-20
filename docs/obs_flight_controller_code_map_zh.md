# obs 到飞控代码位置梳理

本文整理 `flight_bridge` 中从飞控状态读取、构造 `obs`、SDK 推理，再写回飞控的主要代码位置。

当前主路径是 Phase 1 的 RC residual 控制链路：

```text
MAVLink / Gazebo 状态
    -> FlightState
    -> CopterState
    -> obs16 + feedback1
    -> Stabilize/Gaia SDK
    -> action
    -> ActionAdapter
    -> SafetyShield
    -> MAVLink RC_CHANNELS_OVERRIDE
    -> ArduCopter SITL / 飞控
```

现代 ArduCopter 的 guided velocity 后端仍在代码中保留，但对当前 APM 2.74b / ArduCopter 2.74b 实验来说，首选控制入口是 `RC_CHANNELS_OVERRIDE`。

## 1. 主循环入口

文件：

- `flight_bridge/main.py`

关键函数：

- `_main_phase1()`：Phase 1 初始化入口，负责创建 MAVLink adapter、状态构造器、SDK client、动作适配器和安全层。
- `run_phase1_loop()`：主控制循环，串起 `raw_state -> obs -> action -> 飞控命令`。

主循环里的关键代码位置：

```python
raw_state = adapter.read_state()
```

位置：`flight_bridge/main.py:253-255`

含义：从 `MavlinkAdapter` 读取飞控状态，得到 `FlightState`。这是后续 `obs` 的飞控来源。

```python
copter_state = _flightstate_to_copterstate(...)
```

位置：`flight_bridge/main.py:279-290`

含义：把 MAVLink 原始飞控状态转换成统一的 `CopterState`。`CopterState` 使用 body-FLU 坐标，后面直接用于构造 SDK 观测。

```python
obs = state_builder.build_hover_obs(copter_state, last_cmd_for_obs, rc_mode=True)
```

位置：`flight_bridge/main.py:643-650`

含义：构造真正发给 SDK 的 16 维 `obs`。

```python
feedback = feedback_builder.build(...)
```

位置：`flight_bridge/main.py:651-655`

含义：构造 SDK 的 1 维 feedback。

```python
action = sdk.infer(obs, feedback)
```

位置：`flight_bridge/main.py:772-775`

含义：把 `obs` 和 `feedback` 发给 SDK，拿回 action。

```python
cmd_log = adapter.send_rc_cmd(safe_rc)
```

位置：`flight_bridge/main.py:827`

含义：RC residual 主路径下，把经过安全层过滤后的 RC PWM 命令写回飞控。

## 2. 飞控状态读取

文件：

- `flight_bridge/mavlink_adapter.py`

关键类：

- `FlightState`
- `MavlinkAdapter`

`FlightState` 是桥接层保存飞控状态的数据结构，字段包括姿态、角速度、航向、地速、爬升率、GPS、高度、模式、解锁状态和时间戳。

位置：`flight_bridge/mavlink_adapter.py:31-96`

主要字段：

```python
roll, pitch, yaw
roll_speed, pitch_speed, yaw_speed
heading, groundspeed, climb
lat, lon, relative_alt
mode, armed
attitude_ts, position_ts, vfr_ts, heartbeat_ts
```

状态读取入口：

```python
def read_state(self) -> FlightState:
```

位置：`flight_bridge/mavlink_adapter.py:182-186`

在 SITL / 飞控连接模式下，它会调用：

```python
def _read_mavlink_state(self) -> FlightState:
```

位置：`flight_bridge/mavlink_adapter.py:216-288`

这里从 MAVLink 消息更新 `FlightState`：

- `ATTITUDE` -> roll / pitch / yaw / angular rates
- `GLOBAL_POSITION_INT` -> lat / lon / relative_alt
- `VFR_HUD` -> heading / groundspeed / climb
- `HEARTBEAT` -> armed / mode

飞控连接和数据流请求在：

```python
def connect(self, connection_string: str = "") -> bool:
```

位置：`flight_bridge/mavlink_adapter.py:150-176`

这里会通过 `pymavlink` 建立连接，并请求 MAVLink data streams。

## 3. FlightState 转 CopterState

文件：

- `flight_bridge/main.py`

关键函数：

```python
def _flightstate_to_copterstate(...):
```

位置：`flight_bridge/main.py:945-1034`

作用：把飞控/MAVLink 读到的 `FlightState` 转成统一的 `CopterState`，用于后续 obs 构造。

它会处理三种来源：

- 有 Gazebo truth：使用 Gazebo odometry 的位置、速度、姿态和角速度。
- dry-run：生成模拟漂移，让 obs / feedback 不全为零。
- SITL MAVLink：用经纬度和 home 点估计本地位置误差，再旋转到 body-FLU。

核心输出字段：

```python
x_err, y_err, z_err
vx, vy, vz
roll, pitch, yaw_err
p, q, r
altitude_m
armed, mode
```

## 4. obs 具体构造

文件：

- `flight_bridge/state_builder.py`

关键类：

- `CopterState`
- `CopterStateBuilder`
- `HoverFeedbackBuilder`

`CopterState` 定义在：

```python
@dataclass
class CopterState:
```

位置：`flight_bridge/state_builder.py:24-55`

`obs` 构造函数：

```python
def build_hover_obs(
    self,
    state: CopterState,
    last_cmd: VelocityCmd | list[float] | None = None,
    rc_mode: bool = False,
) -> list[float]:
```

位置：`flight_bridge/state_builder.py:81-121`

当前 `obs` 固定 16 维：

```text
obs[0]   x_err
obs[1]   y_err
obs[2]   z_err
obs[3]   vx
obs[4]   vy
obs[5]   vz
obs[6]   roll
obs[7]   pitch
obs[8]   yaw_err
obs[9]   p
obs[10]  q
obs[11]  r
obs[12]  last_cmd[0]
obs[13]  last_cmd[1]
obs[14]  last_cmd[2]
obs[15]  last_cmd[3]
```

前 12 维会用 `tanh(value / scale)` 归一化到近似 `[-1, 1]`。后 4 维是上一帧命令历史：

- RC 模式下：上一帧 RC delta，已经是归一化量。
- guided velocity 模式下：上一帧速度/yaw-rate 命令。

注意：`flight_bridge/observation_builder.py` 是 Phase 0 的旧 8 维临时 obs 构造，不是当前 Phase 1 飞控闭环的主要 obs 入口。

## 5. SDK 通信

文件：

- `flight_bridge/sdk_client.py`

关键接口：

```python
def infer(self, obs: list[float], feedback: list[float]) -> list[float]:
```

位置：`flight_bridge/sdk_client.py:75-86`

它会发送 JSON-lines 请求：

```json
{"obs":[...],"feedback":[...]}
```

并从 SDK 响应中读取：

```json
{"action":[...]}
```

主循环里调用 SDK 的位置在 `flight_bridge/main.py:772-775`。

## 6. action 转飞控命令

文件：

- `flight_bridge/action_adapter.py`

关键类：

- `ActionAdapter`

当前 RC 主路径的关键函数：

```python
def to_rc_command(self, action: list[float]) -> RcCommand:
```

位置：`flight_bridge/action_adapter.py:102-145`

作用：把 SDK action 的前 4 维解释为 RC PWM：

```text
action[0] -> roll
action[1] -> pitch
action[2] -> throttle
action[3] -> yaw
```

残差融合函数：

```python
def residual_blend_rc(self, baseline: RcCommand, sdk_cmd: RcCommand) -> RcCommand:
```

位置：`flight_bridge/action_adapter.py:147-161`

作用：将 baseline RC 和 SDK 修正量融合：

```text
final_pwm = baseline_pwm + residual_lambda * (sdk_pwm - neutral_pwm)
```

baseline RC 由当前高度误差、垂直速度和 XY 偏差产生：

```python
def baseline_rc_for_altitude(...):
```

位置：`flight_bridge/action_adapter.py:168-215`

## 7. 安全层

文件：

- `flight_bridge/safety_shield.py`

关键类：

- `SafetyShield`

RC 主路径过滤函数：

```python
def filter_rc(self, raw_cmd: RcCommand) -> tuple[RcCommand, ShieldInfo]:
```

位置：`flight_bridge/safety_shield.py:140-225`

它会对 RC PWM 命令做：

- NaN / inf 检查
- PWM 范围裁剪
- 低通滤波
- 死区处理
- 单帧变化率限制

SDK 延迟和飞控状态新鲜度检查：

```python
def should_accept_action(self, latency_ms: float, state_age_ms: float) -> bool:
```

位置：`flight_bridge/safety_shield.py:124-134`

如果 SDK 延迟太高、状态过旧或 Gazebo truth 过旧，主循环会丢弃 SDK action，回退到 baseline。

## 8. 最终写回飞控

文件：

- `flight_bridge/mavlink_adapter.py`

RC 写回入口：

```python
def send_rc_cmd(self, cmd: RcCommand) -> CommandLog:
```

位置：`flight_bridge/mavlink_adapter.py:294-315`

底层 MAVLink 发送函数：

```python
def send_rc_override(...):
```

位置：`flight_bridge/mavlink_adapter.py:340-358`

最终调用：

```python
mav.mav.rc_channels_override_send(
    mav.target_system, mav.target_component,
    roll, pitch, throttle, yaw, ch5, ch6, ch7, ch8,
)
```

这就是当前 RC residual 路径真正写入飞控的位置。

guided velocity 写回入口也保留在同一文件：

```python
def send_velocity_cmd(self, cmd: VelocityCmd) -> CommandLog:
```

位置：`flight_bridge/mavlink_adapter.py:317-338`

底层发送：

```python
def _send_guided_velocity(self, cmd: VelocityCmd) -> None:
```

位置：`flight_bridge/mavlink_adapter.py:455` 起。

## 9. FastAPI 控制服务中的同一链路

如果不是从命令行运行 `flight_bridge/main.py`，而是通过 FastAPI 控制服务跑后台控制，入口在：

- `flight_bridge/api.py`

关键位置：

```python
raw_state = session.adapter.read_state()
```

位置：`flight_bridge/api.py:470`

```python
copter_state = _flightstate_to_copterstate(...)
```

位置：`flight_bridge/api.py:494-506`

```python
obs = session.state_builder.build_hover_obs(...)
```

位置：`flight_bridge/api.py:541-545`

```python
action = session.sdk.infer(obs, feedback)
```

位置：`flight_bridge/api.py:560-563`

```python
cmd_log = session.adapter.send_rc_cmd(safe_rc)
```

位置：`flight_bridge/api.py:588-590`

FastAPI 路径和命令行路径共用这些核心模块：

- `MavlinkAdapter`
- `CopterStateBuilder`
- `HoverFeedbackBuilder`
- `ActionAdapter`
- `SafetyShield`

## 10. 一句话定位

如果只想看 `obs` 和飞控的关系，优先按这个顺序打开：

1. `flight_bridge/main.py`
   - `raw_state = adapter.read_state()`
   - `copter_state = _flightstate_to_copterstate(...)`
   - `obs = state_builder.build_hover_obs(...)`
   - `action = sdk.infer(obs, feedback)`
   - `cmd_log = adapter.send_rc_cmd(safe_rc)`
2. `flight_bridge/mavlink_adapter.py`
   - `FlightState`
   - `_read_mavlink_state()`
   - `send_rc_cmd()`
   - `send_rc_override()`
3. `flight_bridge/state_builder.py`
   - `CopterState`
   - `build_hover_obs()`
4. `flight_bridge/action_adapter.py`
   - `to_rc_command()`
   - `residual_blend_rc()`
   - `baseline_rc_for_altitude()`
5. `flight_bridge/safety_shield.py`
   - `filter_rc()`
   - `should_accept_action()`

