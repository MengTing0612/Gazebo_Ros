# Hover Anchor Residual Plan

本文档记录当前无人机悬停任务的推荐定义和下一步推进方案。

## 1. 任务定义

当前不建议把悬停任务定义成固定世界坐标点：

```text
target = (0, 0, target_alt)
```

更接近真实无人机悬停的定义是：

```text
target_xy = 起飞/重置后的当前位置
target_z  = target_xy 上方的指定高度
```

也就是起飞后在当前点上方定高悬停，而不是强制回到 Gazebo 世界坐标原点。

在当前 `flight_bridge.main` 中，如果不传 `--target-x` 和 `--target-y`，
Gazebo truth 路径会使用 `gazebo_home` 作为 XY anchor；如果传了
`--target-x 0 --target-y 0`，任务会变成回到世界原点定点悬停。

推荐下一轮实验去掉：

```bash
--target-x 0
--target-y 0
```

只保留：

```bash
--target-alt 3.0
```

## 2. 传感器假设

真实无人机不一定天然知道绝对 XY 位置。

常见 XY 位置来源包括：

```text
室外: GPS / RTK-GPS
室内: 光流 + 高度计
高级方案: VIO / SLAM
实验室: 动捕 / UWB
```

只有 IMU、陀螺仪、气压计时，通常可以获得姿态、高度和短时运动趋势，
但无法长期稳定获得 XY 位置。当前项目使用：

```text
Gazebo odometry -> x/y/z truth
```

因此当前实验等价于有一个理想仿真定位传感器。后续如果要更贴近真实
飞行器，可以在 Gazebo odometry 上加入噪声、延迟或丢包，或改成 GPS /
光流 / VIO 风格观测。

## 3. XY 漂移处理

去掉固定世界坐标目标不等于不处理 XY 漂移。

推荐处理方式是：

```text
以 reset/home 位置作为 hover anchor
用 x_err/y_err/vx/vy 衡量相对 anchor 的漂移和速度
feedback 惩罚 XY 漂移
baseline 做保守限速和轻微回拉
SDK residual 只做小修正
```

也就是说，XY 目标仍然存在，只是它不是硬编码世界原点，而是每轮 episode
的起飞/重置 anchor。

当前 baseline 的 XY 逻辑是 matrix-free 的保守外环：

```text
pitch_delta = -(rc_xy_kp_pwm_per_m * x_err + rc_xy_kd_pwm_per_mps * vx)
roll_delta  = -(rc_xy_kp_pwm_per_m * y_err + rc_xy_kd_pwm_per_mps * vy)
```

并通过 `rc_xy_max_delta_pwm` 限幅。它的定位不是精确定点控制器，而是：

```text
限制横向速度
漂远时轻微回拉
避免越飞越快
```

## 4. 当前问题判断

最近 residual 测试说明：

```text
SDK 接入链路正常
action_dim = 10
RC_CHANNELS_OVERRIDE 正常发送
高度基本可以被 baseline 控住
主要问题是 XY 继续漂移
action 长期接近饱和
```

因此当前不应通过增大 `residual_lambda` 解决问题。若 action 语义尚未验证，
更大的 residual 只会把满幅 action 变成更强扰动。

建议把 `residual_lambda` 回到小权重区间：

```yaml
control:
  residual_lambda: 0.01  # very conservative diagnostic
  # or
  residual_lambda: 0.03  # previous conservative residual value
```

不建议在 action 贡献度未验证前使用较大值作为主评估。

## 5. 推荐推进顺序

### Step 1: 修正悬停目标

启动脚本中去掉 `--target-x 0 --target-y 0`，使用 reset/home anchor 作为
XY 悬停目标。

### Step 2: baseline-only 实验

先跑：

```text
rc_control_mode = baseline-only
target_xy = reset/home anchor
target_alt = 3.0
```

观察指标：

```text
是否稳定起飞
高度是否接近 target_alt
XY drift 是否快速发散
vxy 是否被压住
tilt 是否保持小角度
reset 是否合理触发
```

baseline-only 的目标不是精准悬停，而是先确认基础外环不会把飞机越推越快。

### Step 3: 调整保守 baseline

如果 baseline-only 仍然明显外漂，优先调整：

```yaml
copter:
  rc_xy_kp_pwm_per_m: ...
  rc_xy_kd_pwm_per_mps: ...
  rc_xy_max_delta_pwm: ...
```

原则：

```text
先增加速度阻尼 kd
再小幅增加位置回拉 kp
最后才放大 max_delta
```

避免一次性把 roll/pitch baseline 调得过猛，导致姿态振荡。

### Step 4: 小 lambda residual

baseline-only 不快速发散后，再启用 SDK residual：

```text
rc_control_mode = residual
residual_lambda = 0.01 ~ 0.03
```

此阶段只判断 SDK 是否改善：

```text
distance_p95
xy_p95
vxy_p95
tilt_p95
reset_count
action_saturation
safety_triggered
```

如果 residual 让 reset、速度或 tilt 明显变差，说明当前 action 映射仍不适合
直接控制。

### Step 5: action10 贡献度分析

当前 front-4 映射是假设：

```text
action[0] -> roll
action[1] -> pitch
action[2] -> throttle
action[3] -> yaw
```

但 Gaia action 原始语义并不是无人机四通道杆量，因此需要离线验证。

收集日志字段：

```text
action10
copter_state
baseline_rc
candidate_command
safe_command
rc_delta
feedback_diagnostics
future distance / future xy / future vxy
```

分析目标：

```text
哪些 action 维度与未来距离降低相关
哪些 action 维度与横向速度降低相关
哪些维度只是在饱和但没有正贡献
是否能拟合 W @ action10 -> rc_delta
```

最终使用 learned W 替代 front-4 硬映射：

```text
action10 -> W @ action10 -> bounded rc_delta -> baseline_rc + rc_delta
```

## 6. 推荐命令

baseline-only 原地定高悬停：

```bash
python3 -m flight_bridge.main \
  --phase 1 \
  --mode sitl \
  --control rc \
  --host 127.0.0.1 \
  --port 9000 \
  --rc-control-mode baseline-only \
  --iterations 800 \
  --target-alt 3.0 \
  --reset-on-flip \
  --reset-tilt-deg 85.0 \
  --no-reset-recovery \
  --post-reset-guard-s 0.75 \
  --reset-pose-z 0.35 \
  --reset-xy-runaway-m 20.0 \
  --reset-runaway-hold-s 2.0 \
  --log-file logs/baseline_anchor_hover_800f.jsonl
```

小 lambda residual 原地定高悬停：

```bash
python3 -m flight_bridge.main \
  --phase 1 \
  --mode sitl \
  --control rc \
  --host 127.0.0.1 \
  --port 9000 \
  --rc-control-mode residual \
  --iterations 800 \
  --target-alt 3.0 \
  --reset-on-flip \
  --reset-tilt-deg 85.0 \
  --no-reset-recovery \
  --post-reset-guard-s 0.75 \
  --reset-pose-z 0.35 \
  --reset-xy-runaway-m 20.0 \
  --reset-runaway-hold-s 2.0 \
  --log-file logs/residual_anchor_hover_800f.jsonl
```

实时可视化：

```bash
python3 tools/visualize_feedback.py --watch logs/
```

## 7. 下一步待改

建议先做两处小改动：

```text
1. 一键启动脚本默认去掉 --target-x 0 --target-y 0
2. residual_lambda 默认回到 0.03 或提供脚本参数覆盖
```

之后按 baseline-only -> 小 residual -> action contribution 的顺序推进。
