# SITL + MAVLink 桥接 SDK 飞控 — v1 MVP 实现文档

## 0. 定位与范围

### v1 MVP 一句话目标

> **Rover SITL 下，先跑通 SDK 推理与 MAVLink 状态读取；RC_CHANNELS_OVERRIDE 只作为 Phase 3 低速调试通道，不启用参数调节和自动模式切换。**

### 明确不做（v1 范围外）

- 不调 PID 参数（`PARAM_SET`）
- 不切换飞行模式（`SET_MODE`）
- 不发 GUIDED 航点（`MISSION_ITEM`）
- 不支持 Copter 多旋翼（仅 Rover/无人车）
- 不做闭环自动优化（先跑通开环数据流）
- Phase 0/1/2 不发送任何 MAVLink 写控制消息
- action 语义未确认前，不把任意 action 维度直接当成舵量/油门闭环使用

### 为什么这么收缩

| 当前确定 | 当前不确定 |
|---|---|
| SDK TCP 通信链路 | action 10 维语义 |
| MAVLink 只读状态接口 | obs 是否匹配模型训练分布 |
| RC override 可作为调试通道 | feedback 真实含义 |
| 模块拆分方案 | 推理延迟与频率匹配 |
| 本地 APMrover2 默认 steering/throttle 通道 | RC trim、正反向、刹车语义 |

在 action 语义、RC 通道校准、推理延迟和安全接管机制搞清楚之前，任何写控制路径都必须保持显式开关和低速限幅。

---

## 1. 分阶段落地路线

```
Phase 0: SDK Dry-run      ← 本次实现
  │  模拟状态 → SDK 推理 → 打印 obs/action/latency
  │  控制开关: --control off，不连接飞控、不写 MAVLink
  │  目标: 确认 obs/action 维度、延迟、action 统计分布
  │
Phase 1: Rover SITL 只读   ← 依赖 SITL 安装
  │  连接 Rover SITL → 读取真实 MAVLink 状态 → SDK 推理 → 只打印
  │  控制开关: --control off，禁止 RC override
  │  目标: 确认状态读取正常、obs 构造与 SITL 数据对齐
  │
Phase 1.5: RC/PWM 校准
  │  确认 RCMAP、trim、stop PWM、正反向、油门/刹车语义
  │  目标: 形成 safety_limits.yaml 的真实安全范围
  │
Phase 2: action/obs 语义分析
  │  系统性地变化 obs → 观察 action 变化规律
  │  尝试不同 feedback → 观察 action 响应
  │  目标: 建立 obs → action 的映射理解，决定 action_index / invert / scale
  │
Phase 3: Rover SITL 低速 RC override
  │  Rover SITL + 显式 --control rc + 低速 steering/throttle RC override
  │  目标: 首次闭环控制，限速限角度，遥控器随时接管
  │
Phase 4: 扩展
       GUIDED setpoint 后端 / 参数调优 / Copter / 真机
```

**当前只做 Phase 0。** Phase 1 仍然必须保持只读；Phase 3 需要完成 RC/PWM 校准和 action 语义分析并单独评审后再启用。

---

## 2. 系统架构

```
┌──────────────────────────────────────────────┐
│          retina-server-x86_64                 │
│         127.0.0.1:9000  TCP JSON Lines        │
│         login → resume → infer(obs, feedback) │
│                      → action (10-dim)         │
└──────────────────┬───────────────────────────┘
                   │ TCP socket
┌──────────────────▼───────────────────────────┐
│              flight_bridge/                    │
│                                               │
│  main.py                  ← 主循环            │
│  sdk_client.py            ← SDK TCP 封装      │
│  mavlink_adapter.py       ← MAVLink + dry-run │
│  observation_builder.py   ← 状态 → obs       │
│  action_mapper.py         ← action → 候选 RC 命令 │
│  safety_guard.py          ← RC 限幅/模式检查  │
│                                               │
│  config/safety_limits.yaml ← RC 安全范围      │
└──────────────────┬───────────────────────────┘
                   │ MAVLink UDP (Phase 1/2 只读, Phase 3+ 可写)
┌──────────────────▼───────────────────────────┐
│          ArduPilot Rover SITL                 │
│         STEERING 模式                         │
│         ← ATTITUDE, VFR_HUD, HEARTBEAT, ...  │
│         → RC_CHANNELS_OVERRIDE (Phase 3 only) │
│         姿态环 + 混控 (飞控自己跑)            │
└──────────────────────────────────────────────┘
```

---

## 3. SDK 通信协议

### 3.1 连接流程

```
1. TCP connect → 127.0.0.1:9000
2. → {"login": {"username": "admin", "password": "admin123"}}
   ← {"status": "ok", "token": "..."}
3. → {"resume": true}
   ← {"status": "ok"}
4. → {"obs": [8 floats], "feedback": [1 float]}
   ← {"status": "success", "action": [10 floats]}
```

### 3.2 维度（待验证）

| 字段 | 假设维度 | 备注 |
|---|---|---|
| obs | 8 | 用不同长度试探 SDK 报错边界来确认 |
| feedback | 1 | 跟踪误差或其他标量信号 |
| action | 10 | 语义完全未知，Phase 0 重点观察 |

---

## 4. MAVLink 控制方式（v1 用 RC override 作为调试通道）

### RC_CHANNELS_OVERRIDE — Phase 3 调试通道

```python
# 模拟遥控器摇杆输入
# 本地 APMrover2 默认: ch1=steering, ch3=throttle。
# 实际实现必须读取/配置 RCMAP，不能硬编码 ch2 为 throttle。
mav.mav.rc_channels_override_send(
    target_system, target_component,
    steer_pwm,    # ch1: steering，方向需校准
    0,            # ch2: 本地 APMrover2 未使用
    thr_pwm,      # ch3: throttle，1500 通常为 trim/stop，正反向需校准
    0, 0, 0, 0, 0
)
```

- **层级**: 摇杆输入层，飞控自己跑姿态环 + 混控
- **频率**: 10-20Hz
- **默认状态**: `--control off`，Phase 0/1/2 不发送
- **启用条件**: Phase 3 显式 `--control rc`，且完成 RC/PWM 校准和 action 语义分析
- **安全**: 飞控 failsafe 有效，遥控器随时接管；桥接端也必须有超时回中/停发策略

### 后续才启用的控制方式（v1 不实现，只列出来）

| 方式 | 用途 | 何时启用 |
|---|---|---|
| Guided setpoint (`SET_POSITION_TARGET_*`) | 发局部/全局目标或速度 | Phase 4，action 语义可抽象成目标后 |
| `MISSION_ITEM` (GUIDED) | 发任务航点 | Phase 4，任务导航需求明确后 |
| `PARAM_SET` | 调 PID/导航参数 | Phase 4，且需要独立的参数安全评审 |
| `SET_MODE` | 切换飞行模式 | 仅紧急情况需要，v1 人工切换 |

---

## 5. 目标平台：Rover

### Rover vs Copter 的差异

| | Rover (v1 目标) | Copter (v1 不做) |
|---|---|---|
| 控制通道 | 本地 APMrover2 默认 ch1=steering, ch3=throttle；实际以 RCMAP 为准 | ch1=roll, ch2=pitch, ch3=throttle, ch4=yaw |
| 危险动作 | 撞墙 | 炸机 |
| 测试成本 | 低 | 高 |
| 模式 | v1 写控制仅允许 STEERING；GUIDED 只作为后续 setpoint 后端 | STABILIZE / GUIDED / LOITER |
| neutral PWM | steering=1500, throttle=1500 | roll/pitch/yaw=1500, throttle=1000 (怠速) |

### Rover STEERING 模式

- 飞控接受 steering + throttle 的 RC 输入
- 飞控自己跑转向 PID 和油门控制
- 适合外部模型通过 RC override 做高层路径跟踪
- 本地 APMrover2 注释和 `set_control_channels()` 显示 throttle 走 `rcmap.throttle()`，默认是 ch3；不要把 ch2 当油门
- throttle 的停止、前进、后退、刹车语义必须通过 SITL/实车校准确认，不能只按 1000/1500/2000 猜测

### emergency_stop 按载具类型

```python
EMERGENCY_STOP_PWM = {
    # Rover 默认按本地 APMrover2: ch1 steer, ch3 throttle。
    # throttle_stop_pwm 先用 1500/trim，完成校准后再按车辆配置调整。
    "rover":  [1500, 0, 1500, 0, 0, 0, 0, 0],
    "copter": [1500, 1500, 1000, 1500, 0, 0, 0, 0],  # roll/pitch=中, thr=怠速, yaw=中
}
```

---

## 6. 模块详细设计

### 6.1 `sdk_client.py`

基于 `examples/sdk_comm/stabilize_sdk_tcp_demo.py`，接口精简为:

```python
class SdkClient:
    def __init__(self, host="127.0.0.1", port=9000, timeout=5.0): ...
    def __enter__(self) -> "SdkClient": ...   # connect + login + resume
    def infer(self, obs: list[float], feedback: list[float]) -> list[float]: ...
    def __exit__(self, ...) -> None: ...      # logout + close
```

连接断开自动重试最多 3 次，超时可配置。`infer()` 是唯一对外接口。

### 6.2 `mavlink_adapter.py`

```python
from dataclasses import dataclass

@dataclass
class FlightState:
    """从 MAVLink 消息解析的结构化状态。所有字段都有明确来源。"""
    # 来源: ATTITUDE 消息
    roll: float           # rad
    pitch: float          # rad
    yaw: float            # rad
    roll_speed: float     # rad/s
    pitch_speed: float    # rad/s
    yaw_speed: float      # rad/s

    # 来源: VFR_HUD 消息
    heading: float        # deg (0=N, 90=E, 180=S, 270=W)
    groundspeed: float    # m/s

    # 来源: GLOBAL_POSITION_INT 消息
    lat: float            # deg * 1e7
    lon: float            # deg * 1e7
    relative_alt: float   # mm

    # 来源: HEARTBEAT 消息
    mode: str             # "MANUAL" | "STEERING" | "HOLD" | "GUIDED" | "AUTO" | "RTL"
    armed: bool

    # 来源: SYS_STATUS 消息
    battery_voltage: float  # mV

    # 计算值 (非 MAVLink 直接提供)
    heading_error_deg: float = 0.0  # 目标航向 - 当前航向，由上层设置


class MavlinkAdapter:
    def __init__(self, vehicle_type: str = "rover", mode: str = "dry-run", control: str = "off"):
        """
        vehicle_type: "rover" | "copter"
        mode: "dry-run" | "sitl"
        control: "off" | "rc"
        """

    # --- 连接 ---
    def connect(self, connection_string: str = "") -> bool: ...

    # --- 状态读取 ---
    def read_state(self) -> FlightState: ...

    # --- 控制指令 (v1 仅此一个) ---
    def send_rc_override(self, channels: list[int]) -> None:
        """发送 8 通道 RC override PWM。control=off 时拒绝发送，dry-run 模式仅打印。"""

    # --- 紧急停止 ---
    def emergency_stop(self) -> None:
        """发送载具对应的停止 PWM。"""

    # --- 后续扩展 (v1 不调用) ---
    # def send_guided_point(...)
    # def set_mode(...)
    # def set_param(...)
```

#### Dry-Run 模式

`read_state()` 返回一组合理的 Rover 巡航模拟值（微小 steering 偏角、中等速度）。`control=off` 时不允许写控制；`dry-run + control=rc` 也只能打印，不发送 MAVLink。

#### SITL 模式 (Phase 1+)

通过 pymavlink `mavutil.mavlink_connection("udp:127.0.0.1:14550")` 连接。

### 6.3 `observation_builder.py`

```python
class ObservationBuilder:
    """Rover 飞控状态 → 临时 8 维 obs 向量 (归一化到 [-1, 1])"""

    # 各维度归一化范围
    _RANGES: list[tuple[float, float]] = [
        (-3.14, 3.14),     # 0: roll rad
        (-3.14, 3.14),     # 1: pitch rad
        (-3.14, 3.14),     # 2: yaw rad
        (-6.28, 6.28),     # 3: roll_speed rad/s
        (-6.28, 6.28),     # 4: pitch_speed rad/s
        (-6.28, 6.28),     # 5: yaw_speed rad/s
        (-1.0, 1.0),       # 6: 归一化 heading_error (用 sin 映射到 [-1,1])
        (0.0, 15.0),       # 7: groundspeed m/s
    ]

    def build(self, state: FlightState) -> list[float]:
        raw = [
            state.roll,
            state.pitch,
            state.yaw,
            state.roll_speed,
            state.pitch_speed,
            state.yaw_speed,
            math.sin(math.radians(state.heading_error_deg)),
            state.groundspeed,
        ]
        return [self._normalize(v, lo, hi) for v, (lo, hi) in zip(raw, self._RANGES)]
```

**说明:** 6 维 IMU + 1 维 heading_error + 1 维速度 = 8 维。这是 Phase 0 的临时探测 schema，用来确认 SDK obs 维度和 action 响应，不代表模型训练时的真实语义。Rover 正式控制更可能需要 yaw/yaw_rate、groundspeed、目标速度、航向误差、横向误差等字段；若 SDK 实际期望的 obs 语义不同，必须替换 schema 后再进入闭环。

### 6.4 `action_mapper.py`

v1 只实现一种候选映射: **action → steering + throttle PWM**。它只能在 Phase 3 低速实验中启用；Phase 0/1/2 只记录 action，不使用 action 驱动车辆。

```python
@dataclass
class FlightCommand:
    """v1 只有 RC override"""
    rc_channels: list[int]  # 8 通道 PWM, 本地 APMrover2 默认 ch1(steer) ch3(thr) 非零

class ActionMapper:
    def __init__(self, vehicle_type: str = "rover", steering_index: int = 0, throttle_index: int = 1): ...

    def map(self, action: list[float], state: FlightState) -> FlightCommand:
        """
        Phase 3 候选策略: action[steering_index] → steering,
        action[throttle_index] → throttle

        action 值域假设: [-1, 1]
        steering: -1=左满舵, 0=直行, +1=右满舵
        throttle:  -1=倒车/刹车方向, 0=trim/stop, +1=前进方向

        映射到 PWM:
          steering: [-1,1] → [1000, 2000], 中心 1500
          throttle:  [-1,1] → [1000, 2000], 中心 1500，正反向需校准
        """
        steer_pwm = self._map_to_pwm(action[self.steering_index], out_min=1000, out_max=2000, center=1500)
        thr_pwm   = self._map_to_pwm(action[self.throttle_index], out_min=1000, out_max=2000, center=1500)
        return FlightCommand(rc_channels=[steer_pwm, 0, thr_pwm, 0, 0, 0, 0, 0])
```

**v1 不实现** Guided 映射和 param_tune 映射，但保留 `guided_setpoint` 后端接口位置，等 action 语义明确后再加。

### 6.5 `safety_guard.py`

v1 只做 RC override 安全:

```python
class SafetyGuard:
    def __init__(self, config_path: str, control: str = "off"): ...

    def filter(self, cmd: FlightCommand, state: FlightState) -> FlightCommand:
        """
        1. 控制开关: control != "rc" → 拒绝写入
        2. 模式检查: 非 STEERING 模式 → 拒绝写入, 返回 None 或 safe cmd
        3. steering 限幅: 不允许满舵, 限制在 [1100, 1900] (v1 低速)
        4. throttle 限幅: 围绕 stop/trim 小范围限速，范围必须来自校准
        5. 变化率限制: 相邻两次 steer/thr 变化不超过 max_step
        6. 超时保护: 超过 N 秒未收到新指令 → 回中/停发
        """

    @property
    def emergency_stop(self) -> FlightCommand:
        """返回载具对应的停止指令"""
```

#### `config/safety_limits.yaml` (v1 精简版)

```yaml
vehicle_type: rover
control_default: off

# v1: 仅 RC override
rc_limits:
  steering:
    channel: 1
    center: 1500
    min: 1100       # v1 限 55% 舵量
    max: 1900
    max_step: 50    # 单次最大变化 50 PWM
  throttle:
    channel: 3      # 本地 APMrover2 默认 throttle 在 ch3，实际以 RCMAP 校准为准
    stop: 1500      # 初始按 trim/stop，Phase 1.5 校准后更新
    min: 1450       # v1 低速倒车/刹车侧，需校准
    max: 1550       # v1 低速前进侧，需校准
    max_step: 30    # 单次最大变化 30 PWM

# 允许 RC override 写入的模式
allowed_modes:
  - STEERING

# 超时保护
rc_timeout_s: 1.0   # 超过 1 秒无指令回中/停发

# 后续 Phase 才启用 (v1 不读取)
# guided_setpoint: {}
# param_whitelist: {}
# max_param_changes_per_minute: 0
```

### 6.6 `main.py`

```python
import time
import argparse

def wrap_deg(angle: float) -> float:
    return (angle + 180.0) % 360.0 - 180.0


def main():
    args = parse_args()

    # 1. 初始化
    sdk = SdkClient(host="127.0.0.1", port=9000, timeout=5.0)
    if args.mode == "sitl":
        fc = MavlinkAdapter(vehicle_type="rover", mode="sitl", control=args.control)
        fc.connect(args.connection)
    else:
        fc = MavlinkAdapter(vehicle_type="rover", mode="dry-run", control=args.control)

    guard = SafetyGuard("config/safety_limits.yaml", control=args.control)
    obs_builder = ObservationBuilder()
    mapper = ActionMapper(vehicle_type="rover")

    # 2. 主循环
    with sdk:
        while True:
            state = fc.read_state()

            # 计算 tracking error (Phase 0: 给一个固定的虚拟目标航向，只用于观测 action)
            target_heading = 0.0  # 正北
            state.heading_error_deg = wrap_deg(target_heading - state.heading)

            obs = obs_builder.build(state)
            feedback = [abs(state.heading_error_deg) / 180.0]  # 归一化跟踪误差

            try:
                action = sdk.infer(obs, feedback)
                print(f"action={[f'{a:.3f}' for a in action]}")  # Phase 0: 重点观察
            except RuntimeError:
                if args.control == "rc":
                    fc.emergency_stop()
                continue

            if args.control != "rc":
                time.sleep(0.05)
                continue

            cmd = mapper.map(action, state)
            safe_cmd = guard.filter(cmd, state)

            if safe_cmd is not None:
                fc.send_rc_override(safe_cmd.rc_channels)

            time.sleep(0.05)  # 20Hz


def parse_args():
    p = argparse.ArgumentParser()
    p.add_argument("--mode", choices=["dry-run", "sitl"], default="dry-run")
    p.add_argument("--control", choices=["off", "rc"], default="off")
    p.add_argument("--connection", default="udp:127.0.0.1:14550")
    return p.parse_args()
```

---

## 7. 数据流

### 一次主循环 (50ms, 20Hz)

```
fc.read_state()
  → 解析 MAVLink 或返回 dry-run 模拟值
  → FlightState(heading=45.2, groundspeed=3.1, mode="STEERING", ...)

obs_builder.build(state)
  → [0.002, -0.010, 0.790, 0.0, 0.0, 0.0, -0.707, 0.207]  (8-dim, 归一化)

sdk.infer(obs, feedback=[0.251])
  → TCP round-trip
  ← action = [0.984, -0.872, 0.116, ...]  (10-dim)

control=off:
  → 只记录 obs/action/latency，不生成或发送控制指令

control=rc (Phase 3 only):
mapper.map(action, state)
  → action[steering_index]→steer_pwm=1984
  → action[throttle_index]→thr_pwm=1264
  → FlightCommand(rc_channels=[1984, 0, 1264, 0, 0, 0, 0, 0])

guard.filter(cmd, state)
  → steer: 1984→1900 (限幅), throttle: 1264→1450 (校准后的低速下限示例)
  → FlightCommand(rc_channels=[1900, 0, 1450, 0, 0, 0, 0, 0])

fc.send_rc_override([1900, 0, 1450, 0, 0, 0, 0, 0])
  → MAVLink RC_CHANNELS_OVERRIDE → 飞控
```

### 异常路径

```
SDK 超时/断连 + control=off  → 记录错误, 不发送控制
SDK 超时/断连 + control=rc   →  emergency_stop() → steer=1500, throttle=stop/trim
SDK 返回非 success  → 跳过本次, control=rc 时停发或回中
飞控模式变为 MANUAL  →  guard.filter() 拒绝, 不发送
```

---

## 8. Phase 0/2: action/obs 语义分析

这是 v1 最重要的非代码工作。Phase 0 先确认维度、延迟和值域；Phase 2 再做系统扰动实验，为 Phase 3 的 action 映射提供依据。

### 8.1 obs 维度确认

```python
# 试探: 用不同长度的 obs 调用 SDK, 看是否报错
for dim in [6, 7, 8, 10, 12, 16]:
    obs = [0.0] * dim
    try:
        action = sdk.infer(obs, [0.0])
        print(f"obs_dim={dim}: ok, action_dim={len(action)}")
    except Exception as e:
        print(f"obs_dim={dim}: rejected — {e}")
```

### 8.2 action 语义探测

```python
# 单变量变化实验
# 1. 固定 obs 全零 → 记录 baseline action
# 2. 逐个维度扰动 obs[i] += 1.0 → 看 action 哪些维度变化
# 3. 极端值测试: obs 全最大 / 全最小 → 看 action 值域

# 结果记录为:
# | 扰动 obs[i] | action 变化最大的维度 | 变化量 | 推测含义 |
```

### 8.3 feedback 影响

```python
# 同一 obs, 不同 feedback
for fb in [-1.0, -0.5, 0.0, 0.5, 1.0]:
    action = sdk.infer(obs, [fb])
    print(f"feedback={fb:.1f} action[0]={action[0]:.3f} action[1]={action[1]:.3f}")
```

### 8.4 recurrent state 检测

```python
# 同一 obs 多次 infer (resume=true)
obs = [0.0] * 8
for i in range(5):
    action = sdk.infer(obs, [0.0])
    print(f"call {i}: action[0]={action[0]:.3f}")
# 如果 5 次结果不同 → 模型有内部状态
```

---

## 9. 依赖安装

```bash
# Phase 0 即可安装
pip install pymavlink pyserial

# Phase 1+ 需要
git clone https://github.com/ArduPilot/ardupilot.git ~/ardupilot
cd ~/ardupilot
./Tools/environment_install/install-prereqs-ubuntu.sh -y
```

---

## 10. 验证步骤

### Phase 0: Dry-run (本次)

```bash
# 终端 1: 启动 SDK 服务
./examples/retina-server-x86_64 &

# 终端 2: 跑桥接 dry-run，只观测不写控制
python flight_bridge/main.py --mode dry-run --control off

# 预期输出:
#   mode=STEERING, armed=True, groundspeed=3.2m/s
#   obs=[0.002, -0.010, 0.79, 0.0, 0.0, 0.0, -0.71, 0.21]
#   action=[0.984, -0.872, 0.116, -0.531, ...]
#   latency_ms=...
```

### Phase 1: Rover SITL 只读

```bash
# 终端 1: Rover SITL
cd ~/ardupilot/Rover
../Tools/autotest/sim_vehicle.py --map --console

# 终端 2: SDK 服务
./examples/retina-server-x86_64 &

# 终端 3: 桥接，只读
python flight_bridge/main.py --mode sitl --control off
# 输出真实 MAVLink state + obs/action/latency，不发送 RC override
```

### Phase 1.5: RC/PWM 校准

```bash
# 确认 RCMAP_ROLL / RCMAP_THROTTLE 或旧版 rcmap 默认值
# 确认 steering center/min/max、throttle stop/min/max、正反向
# 更新 config/safety_limits.yaml 后才能进入 Phase 3
```

### Phase 2: action/obs 语义分析

```bash
# 使用 --control off 运行 obs 维度、action 扰动、feedback 影响实验
# 记录 action_index / invert / scale 的依据
```

### Phase 3: Rover SITL 低速 RC override

```bash
# 确认 SITL 中 Rover 处于 STEERING 模式
# 确认 safety_guard 限幅生效
# 显式启用写控制
python flight_bridge/main.py --mode sitl --control rc
# 观察 SITL 地图中车辆轨迹是否响应
```

---

## 11. 文件清单

```
flight_bridge/
  __init__.py              # 空文件
  sdk_client.py            # SDK TCP 通信 (重构自 examples/sdk_comm/)
  mavlink_adapter.py       # MAVLink + dry-run 双模式适配器
  observation_builder.py   # Rover 状态 → 8 维 obs
  action_mapper.py         # action → 候选 steering/throttle PWM
  safety_guard.py          # RC 限幅 + 模式检查 + 超时保护
  main.py                  # 主循环

config/
  safety_limits.yaml       # v1 RC 安全参数

docs/
  flight_bridge_implementation.md  # 本文档
```
