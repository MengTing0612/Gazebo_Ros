# Stabilize-AI SDK 接入 Gazebo X3 + ArduCopter SITL + 真机部署控制方案

## 1. 项目目标

当前系统目标是将预训练模型 SDK 接入四旋翼无人机控制系统中，先在 Gazebo 仿真环境中完成验证，后续迁移到本地真机部署。

现有系统基础包括：

- Gazebo X3 四旋翼仿真模型；
- ArduCopter SITL 飞控仿真；
- 本地飞控参数；
- external motor UDP 接口；
- Stabilize-AI SDK 本地推理服务；
- 后续需要部署到自研 / 手搓真实四旋翼。

注意：当前 Gazebo X3 只是临时测试模型，用于验证接口、控制流程、日志系统和 Safety Shield。它不能代表真实手搓无人机的质量、惯量、电机 / 桨效率、重心、ESC 延迟和传感器噪声。任何在 X3 上得到的控制参数都只能作为仿真初值，不能直接迁移到真机。

本方案的核心目标不是让预训练模型 SDK 直接控制电机，而是让 SDK 作为外环智能反馈模块，根据观测状态和任务反馈输出控制建议，再由传统飞控或控制器完成稳定控制。

核心原则：

> 模型 SDK 负责“感知、判断、反馈修正”，飞控负责“稳定、执行、姿态闭环、电机控制”。

---

## 2. 总体设计思路

推荐系统采用分层控制架构：

```text
Gazebo X3 / 真机传感器
相机、IMU、姿态、高度、速度、目标误差
        ↓
State Builder
构造 obs 向量
        ↓
Stabilize-AI SDK
obs + feedback → action
        ↓
Action Adapter
将 action 映射为高层控制量
        ↓
Safety Shield
限幅、滤波、异常检测、超时保护、模式切换
        ↓
ArduCopter / PID / MPC / LQR
位置环、速度环、姿态环、电机混控
        ↓
Gazebo 电机模型 / 真机 ESC
```

其中：

- SDK 不直接输出电机 PWM / RPM；
- SDK 输出高层控制修正量；
- 飞控保留姿态稳定和电机混控权限；
- Safety Shield 必须作为 SDK 和飞控之间的中间层；
- 仿真阶段和真机阶段尽量使用一致的控制接口。
- ArduCopter 2.74b 属于较老固件，进入 SDK 闭环前必须先验证当前固件实际支持的 MAVLink 控制入口；不能默认现代 `SET_POSITION_TARGET_LOCAL_NED` / `SET_ATTITUDE_TARGET` 行为完整可用。

### 2.1 当前系统的控制入口选择

根据 `ArduPlane-2.74b/ArduCopter` 源码核对，首版应按以下顺序验证和接入控制入口：

```text
1. ArduCopter 原生基础模式：STABILIZE / ALT_HOLD
2. RC override：roll / pitch / throttle / yaw 小幅 residual
3. ArduCopter 原生定点模式：LOITER
4. 老式 Guided waypoint：MISSION_ITEM current == 2
5. MAVLink Guided velocity / attitude target：需要补 handler 或升级固件后再使用
6. external motor UDP：仅限 Gazebo 仿真和电机级调试
```

`LOITER` 是 APM 原生定点控制路径，应整合进来；但它依赖 GPS / home / navigation 健康状态。因此首版顺序是先用 `STABILIZE / ALT_HOLD + RC_CHANNELS_OVERRIDE` 验证姿态、高度和外部 residual 链路，再在导航状态可靠后接入 `LOITER`。

当前 2.74b 的 `GCS_Mavlink.pde` 未确认支持现代 `SET_POSITION_TARGET_LOCAL_NED` / `SET_ATTITUDE_TARGET` handler，所以不能把 Guided velocity 或 attitude target 当作首版主入口。

---

## 3. 仿真模型边界与手搓机建模

当前 X3 模型的作用是：

```text
1. 验证 Gazebo ↔ ArduCopter SITL ↔ SDK 的数据链路；
2. 验证 obs / feedback / action / cmd 日志格式；
3. 验证 Safety Shield 的限幅、超时和 fallback；
4. 验证控制入口，例如 RC override / APM 原生 LOITER 是否可用；
5. 做低风险的电机级和坐标系调试。
```

X3 模型不应承担：

```text
1. 代表真实飞机动力学；
2. 给出可直接上真机的 PID / MPC / SDK 权重；
3. 验证真实电机顺序、桨方向、ESC 响应；
4. 验证真实重心偏移和机架柔性；
5. 验证电池电压下降、振动、IMU 噪声等真实因素。
```

### 3.1 手搓机最小参数清单

进入真机迁移前，需要为自研飞机建立一份独立参数表：

```text
mass_kg                 整机起飞重量
arm_length_m            电机到重心距离
inertia_ixx_iyy_izz     三轴转动惯量
center_of_gravity       重心相对机架中心偏移
motor_layout            电机位置、编号、转向、桨方向
motor_thrust_curve      PWM / RPM / throttle 到推力曲线
motor_torque_curve      反扭矩曲线
esc_delay_ms            ESC + 电机响应延迟
max_thrust_per_motor    单电机最大推力
hover_throttle          真机悬停油门
imu_noise_vibration     IMU 噪声和振动水平
battery_voltage_range   电池满电 / 中段 / 低电压响应
```

这些参数应进入自定义 Gazebo SDF / URDF 或独立动力学配置，而不是继续使用 X3 默认模型。

### 3.2 参数辨识顺序

建议按风险从低到高做：

```text
1. 桨叶拆除：确认电机编号、转向、MAVLink / RC 通道方向；
2. 单电机推力台：测 PWM / RPM / throttle → thrust 曲线；
3. 整机称重：测 mass 和重心偏移；
4. 简化几何估算：得到初始 ixx / iyy / izz；
5. 固定架低油门：观察 roll / pitch / yaw 力矩方向；
6. 绑绳低空：估计 hover_throttle 和姿态响应；
7. 低空自由悬停：微调飞控参数和外环限幅。
```

### 3.3 X3 到手搓机的迁移策略

迁移时按三层处理：

```text
接口层：保持 obs / feedback / action / Safety Shield 不变；
动力学层：从 X3 模型切换到手搓机模型；
控制层：重新标定 hover_throttle、速度限幅、姿态限幅和 SDK residual_lambda。
```

SDK 接入时应先采用小权重 residual：

```text
X3 仿真通过 ≠ 真机可飞
手搓机仿真通过 ≠ 真机可飞
真机无桨通过 ≠ 真机可飞
```

只有在手搓机模型和真机低风险测试都通过后，才允许逐步提高 SDK 权重。

---

## 4. SDK 在系统中的角色

Stabilize-AI SDK 是一个本地推理服务。典型调用方式为：

```json
{"obs": [...], "feedback": [...]}
```

返回：

```json
{"status": "success", "action": [...]}
```

因此，它适合作为独立的策略服务或反馈服务，而不是嵌入式飞控模块。

在本系统中，SDK 的角色为：

```text
状态观测 + 任务反馈
        ↓
SDK 推理
        ↓
输出动作建议 action
        ↓
控制适配层解释 action
```

SDK 主要负责：

1. 根据飞机状态输出姿态或速度修正建议；
2. 根据视觉目标误差输出跟踪修正；
3. 根据 feedback 判断上一轮控制效果；
4. 作为外环策略模块参与控制。

SDK 不建议负责：

1. 直接输出四电机转速；
2. 直接替代 ArduCopter 姿态环；
3. 直接控制真机 ESC；
4. 运行高频 400 Hz 以上电机控制闭环。

---

## 5. 推荐控制层级

### 5.1 第一优先级：APM 原生模式 + RC residual

这是当前 APM 2.74b 源码下最可靠的首版部署方案。

SDK 输出的 action 被解释为：

```text
action = [a0, a1, a2, a3]
```

映射为小幅 RC residual：

```text
a0 → roll_residual
a1 → pitch_residual
a2 → throttle_or_climb_residual
a3 → yaw_residual
```

对应控制量为：

```text
roll_residual              roll 通道小幅修正
pitch_residual             pitch 通道小幅修正
throttle_or_climb_residual STABILIZE 中是油门修正，ALT_HOLD 中是爬升率修正
yaw_residual               yaw 通道小幅修正
```

控制链路：

```text
SDK action
    ↓
Action Adapter
    ↓
[roll, pitch, throttle/climb, yaw] residual
    ↓
Safety Shield
    ↓
MAVLink RC_CHANNELS_OVERRIDE
    ↓
ArduCopter STABILIZE / ALT_HOLD
    ↓
APM 原生姿态环 / 高度环 / 电机混控
```

STABILIZE 适合先验证姿态方向、电机混控、油门范围；ALT_HOLD 适合验证高度环稳定后做低空悬停。

优点：

- 源码已确认 `RC_CHANNELS_OVERRIDE` 可用；
- 不破坏 ArduCopter 内部姿态环、角速度环和电机混控；
- SDK 权限小，容易限幅和回退；
- 与真机遥控链路语义一致，方便做保护和人工接管；
- 适合低空悬停的第一阶段闭环。

---

### 5.2 第二优先级：APM 原生 LOITER 定点控制

`LOITER` 是 APM 2.74b 原生定点控制模式，不是外部模型方案。它的控制链路是：

```text
position error
    ↓
HLD_LAT / HLD_LON 位置 PI
    ↓
desired horizontal velocity
    ↓
LOITER_LAT / LOITER_LON 速度 PID
    ↓
desired roll / pitch
    ↓
STB_RLL / STB_PIT 姿态 PI
    ↓
RATE_RLL / RATE_PIT 角速度 PID
    ↓
AP_Motors 电机混控
```

在整体方案中，`LOITER` 应作为 APM 原生 baseline controller 整合进来。SDK 不替代 `LOITER_*` / `HLD_*` 参数，而是用于：

```text
1. 给 Loiter target 或 RC roll/pitch/yaw 小幅 residual；
2. 根据视觉 / 任务误差提供外层偏置；
3. 监督 GPS/home/nav/姿态/高度健康；
4. 异常时降级到 ALT_HOLD 或 STABILIZE。
```

启用条件：

```text
1. STABILIZE 姿态方向和电机混控已确认；
2. ALT_HOLD 高度环不冲高、不掉高、不震荡；
3. GPS / home / navigation 状态可靠；
4. RCOU 没有长期饱和；
5. Safety Shield 具备降级和人工接管逻辑。
```

---

### 5.3 第三优先级：补充现代 Guided velocity / attitude target

这不是当前 2.74b 源码下的首版主线。若后续补充 MAVLink handler 或升级飞控固件，可以让 SDK 输出速度或姿态目标：

```text
velocity action = [vx_cmd, vy_cmd, vz_cmd, yaw_rate_cmd]
attitude action = [roll_cmd, pitch_cmd, yaw_rate_cmd, thrust_cmd]
```

控制链路：

```text
SDK action
    ↓
Action Adapter
    ↓
[velocity or attitude target]
    ↓
Safety Shield
    ↓
MAVLink SET_POSITION_TARGET_LOCAL_NED / SET_ATTITUDE_TARGET
    ↓
ArduCopter 对应 handler
    ↓
APM 原生姿态环 / 高度环 / 电机混控
```

初始限制建议：

```text
roll_cmd   ∈ [-5°, 5°]
pitch_cmd  ∈ [-5°, 5°]
yaw_rate   ∈ [-10°/s, 10°/s]
thrust     ∈ [hover_thrust - 5%, hover_thrust + 5%]
```

注意：当前源码核对结果是 `GCS_Mavlink.pde` 未确认处理现代 velocity / attitude target 消息。实现前必须先补 handler 或升级固件，并通过 SITL 单独验证每个字段是否被解析和执行。

---

### 5.4 第四优先级：电机级控制

即：

```text
action = [m1, m2, m3, m4]
```

对应：

```text
m1, m2, m3, m4 → 四个电机 RPM / PWM / normalized motor command
```

该方案只建议用于 Gazebo 或 PyBullet 仿真测试，不建议作为真机首版部署方案。

原因：

1. SDK 输出异常会直接影响飞机姿态；
2. 绕过了飞控姿态环；
3. 真机电机响应、ESC 延迟、电池电压和仿真存在差异；
4. 安全兜底困难；
5. 对模型实时性和稳定性要求过高。

因此，external motor UDP 可以保留为仿真调试接口，但真机部署应优先走 MAVLink `RC_CHANNELS_OVERRIDE / SET_MODE`，并使用 APM 原生 `STABILIZE / ALT_HOLD / LOITER` 控制链路。

---

## 6. obs 向量设计

SDK 的 obs 是模型输入观测向量。obs 的设计应尽量保持低维、稳定、可归一化，并与任务强相关。

### 6.1 低空悬停任务 obs

适用于定点悬停、基础稳定控制：

```text
obs = [
    x_err, y_err, z_err,
    vx, vy, vz,
    roll, pitch, yaw_err,
    p, q, r,
    last_vx_cmd, last_vy_cmd, last_vz_cmd, last_yaw_rate_cmd
]
```

含义：

| 字段 | 含义 |
|---|---|
| x_err, y_err, z_err | 当前相对目标点的位置误差 |
| vx, vy, vz | 当前速度 |
| roll, pitch, yaw_err | 当前姿态误差 |
| p, q, r | 机体系角速度 |
| last_vx_cmd, last_vy_cmd, last_vz_cmd, last_yaw_rate_cmd | 上一时刻控制命令 |

建议所有 obs 做归一化，例如：

```text
位置误差 / 2.0 m
速度 / 1.0 m/s
角度 / 30°
角速度 / 180°/s
控制量 / 最大控制量
```

### 6.1.1 坐标系契约

obs 必须在 `State Builder` 中完成统一坐标转换。不能混用 Gazebo 世界系、MAVLink NED 和机体系字段。

首版建议采用：

```text
位置误差：body-FLU
速度：body-FLU
高度 z / 垂直速度 vz：up 为正
roll / pitch / yaw_err：机体系姿态误差
p / q / r：机体系角速度
last_cmd：上一轮发送给飞控的高层命令
```

其中：

```text
body-FLU:
  x = forward
  y = left
  z = up

MAVLink NED:
  x = north
  y = east
  z = down
```

Gazebo / MAVLink / 真机状态进入 SDK 前必须统一到同一套坐标系。每个字段都应在日志中打印原始值和归一化值，至少包括：

```text
x_err_body, y_err_body, z_err
vx_body, vy_body, vz
roll, pitch, yaw_err
p, q, r
```

坐标系验证必须独立于 SDK 完成，例如：

```text
1. 人为给目标点 x 正方向偏差，确认 obs.x_err_body 为正；
2. 人为给目标点 y 左方向偏差，确认 obs.y_err_body 为正；
3. 机体向前运动，确认 obs.vx_body 为正；
4. 机体向左运动，确认 obs.vy_body 为正；
5. roll / pitch / yaw_err 与飞控日志 ATT 一致。
```

---

### 6.2 视觉目标跟踪任务 obs

适用于目标居中、跟随、降落、对准等任务：

```text
obs = [
    target_u_err,
    target_v_err,
    target_area_err,
    target_confidence,
    z,
    vx, vy, vz,
    roll, pitch, yaw_rate,
    last_vx_cmd, last_vy_cmd, last_vz_cmd, last_yaw_rate_cmd
]
```

含义：

| 字段 | 含义 |
|---|---|
| target_u_err | 目标中心相对图像中心的水平偏差 |
| target_v_err | 目标中心相对图像中心的垂直偏差 |
| target_area_err | 目标框面积与期望面积的误差 |
| target_confidence | 目标检测置信度 |
| z | 当前高度 |
| vx, vy, vz | 当前速度 |
| roll, pitch, yaw_rate | 姿态和偏航角速度 |
| last_cmd | 上一时刻控制命令 |

视觉误差建议归一化到 [-1, 1]。

---

## 7. feedback 向量设计

feedback 不建议简单传上一帧 action，而应传递“上一轮控制是否有效”的反馈信息。

推荐首版 feedback：

```text
feedback = [
    tracking_error,
    hover_error,
    control_saturation,
    safety_flag
]
```

实际实现时必须以当前 SDK 模型的 feedback 输入维度为准。如果 SDK 只接受一维 feedback，则首版使用：

```text
feedback = [total_error]
```

其中：

```text
total_error =
    w_hover * hover_error +
    w_tracking * tracking_error +
    w_saturation * control_saturation +
    w_safety * safety_flag
```

只有在 SDK 服务确认支持多维 feedback 后，才启用四维 feedback。避免出现维度不匹配，例如：

```text
expected feedback shape (1,), called with (4,)
```

含义：

| 字段 | 含义 |
|---|---|
| tracking_error | 视觉目标误差，例如 sqrt(u_err² + v_err²) |
| hover_error | 悬停误差，例如 sqrt(x_err² + y_err² + z_err²) |
| control_saturation | 上一轮控制是否被限幅，0 或 1 |
| safety_flag | 是否触发安全保护，0 或 1 |

feedback 的作用：

```text
让模型知道：
1. 上一轮动作是否让误差变小；
2. 是否导致控制量过大；
3. 是否触发了安全机制；
4. 当前是否处于不稳定状态。
```

---

## 8. action 映射设计

假设 SDK 返回：

```json
{"status": "success", "action": [a0, a1, a2, a3]}
```

### 8.1 RC residual 映射

推荐首版使用 RC residual 映射，对应 APM 原生 `STABILIZE / ALT_HOLD`：

```python
roll_pwm_delta = clip(a0 * roll_pwm_max, -roll_pwm_max, roll_pwm_max)
pitch_pwm_delta = clip(a1 * pitch_pwm_max, -pitch_pwm_max, pitch_pwm_max)
thr_pwm_delta = clip(a2 * thr_pwm_max, -thr_pwm_max, thr_pwm_max)
yaw_pwm_delta = clip(a3 * yaw_pwm_max, -yaw_pwm_max, yaw_pwm_max)
```

首版推荐将 SDK action 解释为 RC 通道小幅修正：

```text
CH1 = 1500 + roll_pwm_delta
CH2 = 1500 + pitch_pwm_delta
CH3 = base_throttle_pwm + thr_pwm_delta
CH4 = 1500 + yaw_pwm_delta
```

在 `ALT_HOLD` 中，CH3 不再是直接电机油门，而是映射为 APM 内部爬升率请求，因此建议让 `thr_pwm_delta` 围绕 1500 小幅变化：

```text
CH3 = 1500 + climb_pwm_delta
```

仿真阶段初始参数：

```text
roll_pwm_max  = 40
pitch_pwm_max = 40
thr_pwm_max   = 30
yaw_pwm_max   = 30
```

真机阶段初始参数：

```text
roll_pwm_max  = 20
pitch_pwm_max = 20
thr_pwm_max   = 15
yaw_pwm_max   = 15
```

### 8.2 LOITER residual 映射

当 GPS / home / navigation 可靠后，接入 APM 原生 `LOITER`。此时 SDK 不替代 `HLD_*` / `LOITER_*`，只做外层 residual：

```text
1. 通过 RC roll / pitch 小幅移动 loiter target；
2. 通过 yaw 小幅修正机头方向；
3. 通过 CH3 中位附近小幅修正爬升率；
4. 通过 Safety Shield 监督 GPS/home/nav/高度/姿态健康。
```

在 `ROLL_PITCH_LOITER` 中，APM 源码会把 roll / pitch 输入解释为目标水平速度，而不是直接姿态角。因此 LOITER 阶段的 RC residual 比 STABILIZE 阶段更接近“位置/速度外环修正”。

### 8.3 现代 velocity / attitude target 扩展

如果后续补充 MAVLink handler 或升级固件，可扩展为速度控制映射：

```python
vx_cmd = clip(a0 * vx_max, -vx_max, vx_max)
vy_cmd = clip(a1 * vy_max, -vy_max, vy_max)
vz_cmd = clip(a2 * vz_max, -vz_max, vz_max)
yaw_rate_cmd = clip(a3 * yaw_rate_max, -yaw_rate_max, yaw_rate_max)
```

但在当前 2.74b 源码下，这不是首版主入口。

### 8.4 residual control 设计

为了避免 SDK 初期直接主导控制，建议使用 residual control：

```text
final_cmd = baseline_cmd + λ * sdk_cmd
```

其中：

```text
baseline_cmd = ArduCopter / PID / MPC 给出的基础悬停控制
sdk_cmd      = SDK 输出的修正控制量
λ            = SDK 权重
```

建议从小权重开始：

```text
λ = 0.1 → 0.2 → 0.3 → 0.5
```

只有当仿真稳定后，才逐步增加 SDK 权重。

---

## 9. Safety Shield 安全保护层

Safety Shield 是本方案的关键模块，所有 SDK action 必须经过该层后才能发送给飞控。

### 9.1 action 限幅

```python
vx_cmd = clip(vx_cmd, -0.3, 0.3)
vy_cmd = clip(vy_cmd, -0.3, 0.3)
vz_cmd = clip(vz_cmd, -0.2, 0.2)
yaw_rate_cmd = clip(yaw_rate_cmd, -10_deg, 10_deg)
```

真机阶段进一步收紧：

```python
vx_cmd = clip(vx_cmd, -0.15, 0.15)
vy_cmd = clip(vy_cmd, -0.15, 0.15)
vz_cmd = clip(vz_cmd, -0.10, 0.10)
yaw_rate_cmd = clip(yaw_rate_cmd, -5_deg, 5_deg)
```

---

### 9.2 低通滤波

防止模型输出跳变：

```python
cmd_filtered = alpha * cmd_new + (1 - alpha) * cmd_prev
```

建议：

```text
alpha = 0.2 ~ 0.4
```

---

### 9.3 死区控制

目标偏差较小时不进行修正，减少抖动：

```python
if abs(target_u_err) < 0.03:
    vy_cmd = 0.0

if abs(target_v_err) < 0.03:
    vz_cmd = 0.0
```

---

### 9.4 超时保护

如果 SDK 推理超时，立即切换到悬停或保持模式：

```text
SDK timeout > 100 ms:
    cmd = [0, 0, 0, 0]
    mode = HOLD / LOITER / GUIDED hover
```

状态数据也必须做过期检查：

```text
Gazebo / MAVLink state age > 100 ms:
    discard SDK action
    cmd = hover_cmd()
```

也就是说，SDK 新鲜但状态过期时同样不能继续闭环。

---

### 9.5 异常 action 检查

以下情况应丢弃 action：

```text
1. action 维度错误；
2. action 中存在 NaN；
3. action 中存在 inf；
4. action 超过合理范围；
5. SDK 返回 status != success；
6. 推理延迟过大；
7. 连续多帧 action 剧烈跳变。
```

需要同时限制控制命令变化率：

```text
abs(cmd_new - cmd_prev) / dt < cmd_rate_limit
```

初始建议：

```text
vx_rate_limit       = 0.30 m/s^2
vy_rate_limit       = 0.30 m/s^2
vz_rate_limit       = 0.20 m/s^2
yaw_rate_rate_limit = 20 deg/s^2
```

处理方式：

```text
discard action
use previous safe command
or switch to hover mode
```

---

### 9.6 模式切换保护

建议保留至少三种控制模式：

```text
Mode 0: Manual / RC 接管
Mode 1: Hover baseline，仅飞控稳定悬停
Mode 2: SDK residual control，SDK 小权重修正
Mode 3: SDK guided control，SDK 主导外环速度控制
```

真机测试时应始终允许人工立即切回 Manual / Loiter / Brake。

---

## 10. 仿真实施路线

### 阶段 1：基础 hover baseline

目标：

```text
Gazebo X3 + ArduCopter SITL 可以低空稳定悬停
```

验证指标：

```text
1. 高度 z 稳定；
2. roll / pitch 不发散；
3. yaw 不快速漂移；
4. 电机输出不过饱和；
5. 低空 0.5m ~ 1.0m 可持续悬停。
```

此阶段不接入 SDK，只验证 X3 模型和飞控参数。

阶段 1 的输出物必须包括：

```text
1. 可重复运行的 hover baseline 脚本；
2. 10 轮低空悬停日志；
3. 每轮最大 roll / pitch / z_error / xy_error；
4. SERVO_OUTPUT_RAW 或 RC OUT 是否饱和；
5. 可复用的 hover 参数集。
```

未完成阶段 1 前，不进入 SDK residual control。

---

### 阶段 2：SDK 旁路推理

目标：

```text
SDK 接收 obs / feedback 并返回 action，但 action 不参与控制
```

链路：

```text
Gazebo state
    ↓
State Builder
    ↓
obs + feedback
    ↓
SDK
    ↓
action
    ↓
logger
```

记录内容：

```text
1. obs；
2. feedback；
3. action；
4. SDK 推理耗时；
5. action 是否连续；
6. action 是否存在异常值；
7. 当前飞机状态。
```

此阶段用于判断 SDK 输出是否稳定。

---

### 阶段 3：SDK residual control

目标：

```text
SDK 小权重参与外环修正
```

控制公式：

```text
final_cmd = baseline_cmd + λ * sdk_cmd
```

初始：

```text
λ = 0.1
```

逐步测试：

```text
λ = 0.1 → 0.2 → 0.3 → 0.5
```

如果出现明显振荡、漂移或失稳，立即降低 λ 或回到纯 hover baseline。

---

### 阶段 4：APM 原生 LOITER 接入测试

目标：

```text
在 GPS/home/nav 可靠后，使用 APM 原生 LOITER 作为定点 baseline
```

链路：

```text
SDK action
    ↓
RC / loiter target residual
    ↓
Safety Shield
    ↓
APM LOITER
    ↓
HLD_LAT/HLD_LON + LOITER_LAT/LOITER_LON + 姿态/角速度环
```

只有在 STABILIZE / ALT_HOLD residual control 稳定，并且 navigation 健康检查通过后再进入该阶段。

### 阶段 4b：现代 Guided velocity 扩展

如果补充 `SET_POSITION_TARGET_LOCAL_NED` handler 或升级固件，再评估 SDK guided velocity control。该阶段不作为 APM 2.74b 当前源码下的主线。

---

### 阶段 5：扰动鲁棒性测试

在 Gazebo 中加入：

```text
1. mass ±10%;
2. inertia ±10%;
3. motor constant ±15%;
4. sensor noise;
5. camera latency 50~150 ms;
6. SDK inference delay;
7. wind disturbance;
8. battery voltage drop approximation.
```

通过条件：

```text
1. 无发散；
2. 无明显翻滚；
3. 控制命令不过饱和；
4. SDK 超时时能安全切回 hover；
5. 多轮随机扰动下仍能保持低空稳定。
```

---

## 11. 真机部署路线

真机部署不建议走 external motor UDP 直控电机。当前 APM 2.74b 源码下，真机首版应走 MAVLink `RC_CHANNELS_OVERRIDE` + APM 原生 `STABILIZE / ALT_HOLD / LOITER`，保留飞控内部 PID 和电机混控。现代 velocity / attitude setpoint 只有在补 handler 或升级固件后再考虑。

推荐链路：

```text
机载计算机 / 本地计算机
    ↓
Stabilize-AI SDK 服务
    ↓
control_client.py
    ↓
MAVLink RC_CHANNELS_OVERRIDE / SET_MODE
    ↓
ArduCopter STABILIZE / ALT_HOLD / LOITER
    ↓
ESC / 电机
```

### 11.1 真机测试顺序

```text
1. 桨叶拆除，验证 MAVLink 指令方向和模式切换；
2. 固定机架，低油门测试控制响应；
3. 保护架 / 绑绳低空测试；
4. 0.3m 悬停；
5. 0.5m 悬停；
6. 1.0m 悬停；
7. 小范围目标跟踪；
8. 扩大速度和 yaw 限幅；
9. 逐步增加 SDK 权重。
```

### 11.2 真机初始安全参数

```text
vx_max       = 0.15 m/s
vy_max       = 0.15 m/s
vz_max       = 0.10 m/s
yaw_rate_max = 5 deg/s
roll_max     = 3°
pitch_max    = 3°
SDK timeout  = 100 ms
```

### 11.3 真机必须保留的安全机制

```text
1. RC 人工接管；
2. 一键切回 Loiter / Brake / Stabilize；
3. SDK 超时自动悬停；
4. action 异常自动丢弃；
5. 电池低压保护；
6. 高度上限；
7. 速度上限；
8. 姿态角上限；
9. 地理围栏或测试区域边界；
10. 日志记录与复盘。
```

---

## 12. 工程目录设计

推荐工程结构：

```text
drone_sdk_control/
├── sdk_client.py
├── state_builder.py
├── action_adapter.py
├── safety_shield.py
├── mavlink_bridge.py
├── gazebo_bridge.py
├── logger.py
├── config.yaml
└── main.py
```

模块职责：

| 文件 | 职责 |
|---|---|
| sdk_client.py | 连接 Stabilize-AI SDK，发送 obs / feedback，读取 action |
| state_builder.py | 从 Gazebo / MAVLink / vision 构造 obs |
| action_adapter.py | 将 action 映射成速度或姿态命令 |
| safety_shield.py | 限幅、滤波、超时、fallback |
| mavlink_bridge.py | 向 ArduCopter 发送 velocity / attitude setpoint |
| gazebo_bridge.py | 读取 Gazebo 状态或发送仿真控制指令 |
| logger.py | 记录 obs、feedback、action、cmd、state、latency |
| config.yaml | 管理所有控制参数和安全参数 |
| main.py | 主控制循环 |

---

## 13. 核心伪代码

```python
while True:
    # 1. 读取 Gazebo / 真机状态
    state = read_gazebo_or_mavlink_state()
    vision = read_vision_result()

    # 2. 构造 obs
    obs = build_obs(
        state=state,
        vision=vision,
        last_cmd=last_cmd
    )

    # 3. 构造 feedback
    feedback = build_feedback(
        tracking_error=vision.error,
        hover_error=state.position_error,
        control_saturation=last_saturation,
        safety_flag=last_safety_flag
    )

    # 4. 调用 SDK
    ok, action, latency = sdk_client.infer(
        obs=obs,
        feedback=feedback,
        timeout_ms=100
    )

    # 5. SDK 异常则 fallback
    if not ok:
        cmd = hover_cmd()
        safety_flag = 1
    else:
        # 6. action → 高层控制命令
        raw_cmd = action_to_velocity_cmd(action)

        # 7. 安全保护
        cmd, safety_info = safety_shield.filter(raw_cmd)

    # 8. 发送 MAVLink 控制命令
    send_mavlink_velocity_cmd(cmd)

    # 9. 记录日志
    logger.write(
        state=state,
        vision=vision,
        obs=obs,
        feedback=feedback,
        action=action,
        cmd=cmd,
        latency=latency
    )

    # 10. 更新上一时刻状态
    last_cmd = cmd
    last_saturation = safety_info.saturation
    last_safety_flag = safety_info.safety_flag
```

---

## 14. config.yaml 示例

```yaml
sdk:
  host: "127.0.0.1"
  port: 9000
  timeout_ms: 100

control:
  mode: "velocity"
  residual_lambda: 0.1
  frequency_hz: 20

limits_sim:
  vx_max: 0.30
  vy_max: 0.30
  vz_max: 0.20
  yaw_rate_max_deg: 10.0
  roll_max_deg: 5.0
  pitch_max_deg: 5.0

limits_real:
  vx_max: 0.15
  vy_max: 0.15
  vz_max: 0.10
  yaw_rate_max_deg: 5.0
  roll_max_deg: 3.0
  pitch_max_deg: 3.0

filter:
  alpha: 0.3
  deadband_u: 0.03
  deadband_v: 0.03

safety:
  enable_action_check: true
  enable_timeout_fallback: true
  enable_manual_override: true
  fallback_mode: "hover"
```

---

## 15. 推荐开发优先级

```text
P0: Gazebo X3 + ArduCopter SITL 稳定 hover，用于验证接口链路
P1: 验证 ArduCopter 2.74b 支持的 MAVLink / RC 控制入口
P2: 建立手搓机参数表和自定义仿真模型初版
P3: SDK client 打通 TCP JSON 推理
P4: State Builder 坐标系统一与日志验证
P5: obs / feedback / action 日志系统
P6: SDK 旁路推理，不接入控制
P7: baseline RC override 外环控制器
P8: Safety Shield
P9: 手搓机模型 hover baseline
P10: SDK residual control
P11: APM 原生 LOITER 接入测试
P12: Gazebo 扰动鲁棒性测试
P13: 真机无桨验证
P14: 真机固定架 / 保护架低空测试
```

---

## 16. 最终结论

本项目最合理的设计不是：

```text
SDK → 直接输出四电机控制
```

而是：

```text
Gazebo / 真机状态
    ↓
obs + feedback
    ↓
Stabilize-AI SDK
    ↓
action
    ↓
速度 / yaw / 姿态修正
    ↓
Safety Shield
    ↓
ArduCopter Guided / Attitude Control
    ↓
飞控完成姿态和电机闭环
```

这样可以同时满足三个目标：

1. 利用预训练模型 SDK 的 feedback 能力；
2. 保留 ArduCopter 原有稳定控制能力；
3. 让仿真结果更容易迁移到本地真机。

一句话总结：

> SDK 做外环智能反馈，飞控做内环稳定控制；先在 Gazebo 中做 residual control，再逐步迁移到真机低空测试。
