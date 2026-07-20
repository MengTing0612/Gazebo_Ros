# Gaia 10 维 Action 贡献度分析与 Adapter 反推实施文档

## 1. 背景

当前 Gaia / Stabilize-AI SDK 已经可以接入飞控闭环，MAVLink / RC override / 电机响应链路也已经打通。最近日志显示：

- SDK 每帧返回 10 维无量纲 `action`；
- 当前 adapter 临时取前 4 维映射为 `roll / pitch / throttle / yaw`；
- 电机确认响应 SDK 命令；
- 但距离误差仍然发散；
- 多个 action 维度长期接近 `-1` 或 `+1`，存在明显饱和输出。

因此当前问题不再是基础 pipeline 是否通，而是：

```text
10 维无量纲 action 到飞控命令的 decoder / adapter 未知。
```

模型输出去量纲是正常的，但无量纲 action 仍然需要恢复成明确控制语义。`action[i]` 不天然等于某个 RC 通道，也不天然等于某个电机输出。必须通过日志、扰动实验和监督分析，反推出哪些维度对悬停任务有有效贡献，并拟合一个安全的小残差 adapter。

本实施文档定义如何从现有日志和后续小扰动实验中，得到：

```text
[roll_residual, pitch_residual, throttle_residual, yaw_residual]
    = W @ action10
```

其中 `W` 是一个受限的小增益矩阵，只作为稳定 baseline 上的 residual，不直接接管飞控。

## 2. 目标与非目标

### 2.1 目标

1. 判断 Gaia 10 维 action 中哪些维度与悬停误差改善相关；
2. 区分“变化大”的 action 维度和“真的有帮助”的 action 维度；
3. 拟合一个 `10 -> 4` 的线性 adapter 矩阵 `W`；
4. 将 Gaia 从主控改为小残差控制；
5. 建立 A/B 测试流程：

```text
baseline only
front-4 hard mapping
learned W mapping
```

6. 明确何时判定 Gaia 对当前 X3 / 手搓机动力学不可用，需要微调或重训。

### 2.2 非目标

1. 不让 Gaia 直接控制电机；
2. 不把 PCA 的主成分直接当控制通道；
3. 不在 baseline 尚不稳定时采集贡献度数据；
4. 不在 action 语义未确认前上真机；
5. 不用一次实验结果直接迁移到手搓机。

## 3. 核心原则

### 3.1 不是单纯 PCA

PCA 只能回答：

```text
action 的哪些组合变化最大？
```

但悬停控制真正关心的是：

```text
action 的哪些组合让未来误差变小？
```

所以 PCA 只能作为诊断工具。主分析必须使用未来误差改善作为监督标签。

推荐方法优先级：

```text
1. 延迟监督回归：future improvement ~ action10
2. permutation importance：打乱某维 action 看预测能力下降
3. PLS / supervised low-rank：找和目标最相关的低维 action 子空间
4. PCA：只用于发现 action 是否低秩、饱和、模式坍缩
```

### 3.2 Gaia 只做 residual

飞控稳定性必须由传统控制器保证：

```text
APM / PID / LOITER baseline
        +
small Gaia residual
        +
Safety Shield
```

初始残差建议：

```text
residual_lambda = 0.03 ~ 0.08
roll_residual_max = 10 ~ 30 PWM
pitch_residual_max = 10 ~ 30 PWM
throttle_residual_max = 20 ~ 50 PWM
yaw_residual_max = 10 ~ 30 PWM
```

当 action 贡献度未验证前，禁止使用 `residual_lambda = 1.0` 作为主控。

### 3.3 必须做延迟对齐

无人机响应不是瞬时的。一个 action 在 `t` 时刻发出，对距离、高度、姿态的影响通常出现在：

```text
t + 100 ms
t + 200 ms
t + 300 ms
```

贡献度分析必须同时评估多个延迟窗口，不能只看同一帧相关性。

## 4. 数据流设计

### 4.1 在线控制数据流

```text
Gazebo / SITL / 真机状态
        ↓
CopterState
        ↓
obs16 + feedback1
        ↓
Gaia SDK
        ↓
action10
        ↓
ActionContributionAdapter
        ↓
rc4 residual
        ↓
Safety Shield
        ↓
RC_CHANNELS_OVERRIDE / APM baseline
```

### 4.2 离线分析数据流

```text
jsonl logs
        ↓
align frames by timestamp
        ↓
build delayed labels
        ↓
fit contribution models
        ↓
rank action dims
        ↓
fit W matrix
        ↓
save adapter config
        ↓
A/B test
```

## 5. 日志采集要求

每一帧至少保存以下字段。

### 5.1 时间与模式

```json
{
  "ts": 1780564813.12,
  "frame": 123,
  "mode": "sitl",
  "control": "rc",
  "vehicle_mode": "STABILIZE",
  "armed": true
}
```

### 5.2 状态

建议保存 raw state 和已转换后的 copter state。

```json
{
  "copter_state": {
    "x_err": 0.42,
    "y_err": -0.18,
    "z_err": 0.07,
    "vx": 0.03,
    "vy": -0.02,
    "vz": 0.01,
    "roll": 0.03,
    "pitch": -0.02,
    "yaw_err": 0.12,
    "p": 0.01,
    "q": -0.03,
    "r": 0.02,
    "altitude_m": 1.02
  }
}
```

最低要求是能重建：

```text
distance = sqrt(x_err^2 + y_err^2 + z_err^2)
tilt = max(abs(roll), abs(pitch))
vertical_error = abs(z_err)
horizontal_error = sqrt(x_err^2 + y_err^2)
```

### 5.3 SDK 输入输出

```json
{
  "obs": [0.0, 0.0, 0.9, "..."],
  "feedback": [0.9],
  "action": [0.97, -0.96, -0.99, "... 10 dims"],
  "latency_ms": 8.3
}
```

必须记录完整 10 维 action，不能只记录前 4 维。

### 5.4 命令

```json
{
  "baseline_rc": [1500, 1500, 1450, 1500],
  "candidate_rc": [1512, 1494, 1460, 1501],
  "safe_command": [1510, 1495, 1458, 1500],
  "rc_delta": [0.02, -0.01, 0.016, 0.0],
  "sent": true
}
```

其中：

```text
rc_delta = (safe_command - baseline_rc) / 500
```

油门也统一用相对 baseline 的 PWM delta。

### 5.5 Safety 信息

```json
{
  "shield": {
    "filtered": true,
    "saturated": false,
    "safety_triggered": false,
    "clipped_fields": []
  },
  "timeout_discard": false,
  "state_age_ms": 50.0
}
```

后续训练 / 分析默认剔除以下帧：

```text
sent == false
timeout_discard == true
safety_triggered == true
state_age_ms > 200
tilt > 25 deg
```

## 6. 实验阶段

### 6.1 Phase A：baseline 稳定性验证

目标：确认没有 Gaia 时，飞机能在低空保持有限误差。

设置：

```text
SDK residual disabled
control = PID / ALT_HOLD / LOITER
duration = 30 ~ 60 s
target_alt = 0.8 ~ 1.2 m
```

通过标准：

```text
horizontal_error_p95 < 1.0 m
vertical_error_p95 < 0.4 m
tilt_p95 < 10 deg
crash_count = 0
```

如果 baseline 自己不稳定，不进入后续 Gaia 贡献度分析。

### 6.2 Phase B：Gaia 小残差被动采集

目标：在稳定 baseline 上采集 Gaia action 与未来误差变化。

设置：

```text
residual_lambda = 0.03 ~ 0.08
roll/pitch residual <= 30 PWM
throttle residual <= 50 PWM
yaw residual <= 30 PWM
duration = 60 ~ 180 s
```

注意：这一阶段不是为了飞得最好，而是为了获得安全、连续、有响应的数据。

### 6.3 Phase C：正交小扰动实验

仅靠 Gaia 自然输出会有状态相关偏差。需要叠加小扰动来提高可辨识性。

扰动方式：

```text
u = baseline_rc + gaia_small_residual + probe
```

`probe` 使用 PRBS 或分段正交信号：

```text
roll_probe     = +/- 8 PWM
pitch_probe    = +/- 8 PWM
throttle_probe = +/- 12 PWM
yaw_probe      = +/- 8 PWM
```

每段持续：

```text
0.3 ~ 0.8 s
```

限制：

```text
probe_total_duration <= 120 s
tilt > 20 deg 立即停止 probe
distance > 3 m 立即 fallback
```

该阶段用于估计：

```text
RC residual -> future state change
```

这能帮助后续把 Gaia action 映射到正确的 RC 轴。

## 7. 标签构造

### 7.1 基础误差

每帧定义：

```text
e_pos = sqrt(x_err^2 + y_err^2 + z_err^2)
e_xy  = sqrt(x_err^2 + y_err^2)
e_z   = abs(z_err)
e_att = abs(roll) + abs(pitch)
```

综合误差：

```text
E = w_xy * e_xy + w_z * e_z + w_att * e_att
```

推荐初值：

```text
w_xy = 1.0
w_z = 1.0
w_att = 0.3
```

### 7.2 未来改善标签

对延迟窗口 `k`：

```text
improve_k(t) = E(t) - E(t + k)
```

含义：

```text
improve_k > 0  误差变小，action 可能有帮助
improve_k < 0  误差变大，action 可能有害
```

建议窗口：

```text
k = 100 ms, 200 ms, 300 ms
```

如果日志是 20 Hz：

```text
100 ms = 2 frames
200 ms = 4 frames
300 ms = 6 frames
```

### 7.3 分轴标签

除了综合误差，还要构造分轴标签：

```text
improve_xy = e_xy(t) - e_xy(t+k)
improve_z  = e_z(t)  - e_z(t+k)
improve_att = e_att(t) - e_att(t+k)
```

用于判断某些 action 维度是否只对高度、横向或姿态有效。

## 8. 分析方法

### 8.1 Action 分布检查

先统计每个 action 维度：

```text
mean
std
min / max
abs(action) > 0.9 的饱和比例
自相关
维度之间相关系数
```

如果某个维度长期饱和，说明它不能直接用大增益接入。

输出表：

| dim | mean | std | min | max | saturation_pct |
|---|---:|---:|---:|---:|---:|
| a0 | 0.27 | 0.91 | -0.99 | 0.99 | 94% |

### 8.2 PCA 诊断

对 `action10` 做 PCA，得到：

```text
explained_variance_ratio
principal_components
```

用途：

1. 判断 action 是否低秩；
2. 判断模型是否模式坍缩；
3. 找到主要变化方向；
4. 辅助后续降维。

不要把 PCA 主成分直接当控制命令。

### 8.3 延迟监督回归

对每个延迟窗口拟合：

```text
improve_k = beta0 + beta_action @ action10 + beta_state @ state_features
```

建议先使用 Ridge：

```text
Ridge(alpha=1.0)
```

再使用 Lasso / ElasticNet 做稀疏筛选：

```text
Lasso(alpha=0.001)
ElasticNet(alpha=0.001, l1_ratio=0.5)
```

为什么要加入 `state_features`：

```text
action 和状态强相关。
如果不控制 state，会把“当前已经快恢复了”误判成 action 有贡献。
```

推荐 state features：

```text
x_err, y_err, z_err,
vx, vy, vz,
roll, pitch, yaw_err,
p, q, r
```

### 8.4 Permutation Importance

训练好模型后，对每个 action 维度做：

```text
随机打乱 action[i]
重新计算验证集预测误差
误差增加越多，说明该维度越重要
```

输出：

| dim | importance_100ms | importance_200ms | importance_300ms |
|---|---:|---:|---:|
| a2 | 0.018 | 0.031 | 0.024 |

### 8.5 PLS：监督版 PCA

如果 action 维度高度相关，PCA 可能能看到主方向，但不知道方向是否有用。可以补充 PLS：

```text
PLSRegression(n_components=2~4)
```

PLS 找的是：

```text
与 future improvement 最相关的低维 action 子空间
```

这比 PCA 更接近本任务。

## 9. W 矩阵拟合

### 9.1 Adapter 形式

最终 adapter 使用：

```text
rc_delta = W @ action10
```

其中：

```text
W shape = [4, 10]
rc_delta = [roll_delta, pitch_delta, throttle_delta, yaw_delta]
```

实际 PWM：

```text
rc_pwm = baseline_pwm + rc_delta_pwm
```

限幅：

```text
roll_delta_pwm     ∈ [-30, 30]
pitch_delta_pwm    ∈ [-30, 30]
throttle_delta_pwm ∈ [-50, 50]
yaw_delta_pwm      ∈ [-30, 30]
```

### 9.2 两种拟合路线

#### 路线 A：贡献度筛选 + 手工轴映射

如果分析显示：

```text
a2 对高度改善最强
a0/a4 对横向改善最强
a3 主要引入 yaw 干扰
```

则先构造稀疏矩阵：

```text
roll_delta     = k0 * a0 + k4 * a4
pitch_delta    = k1 * a1 + k5 * a5
throttle_delta = k2 * a2
yaw_delta      = 0
```

优点：

```text
可解释、安全、容易调试。
```

缺点：

```text
不能充分利用 action 组合信息。
```

#### 路线 B：用 desired correction 拟合 W

先用 baseline controller 计算一个小的目标修正：

```text
u_desired = [
  kx * x_err + kvx * vx,
  ky * y_err + kvy * vy,
  kz * z_err + kvz * vz,
  kyaw * yaw_err
]
```

再拟合：

```text
u_desired ≈ W @ action10
```

使用 Ridge：

```text
W = argmin ||A W^T - U||^2 + alpha ||W||^2
```

其中：

```text
A = action matrix [N, 10]
U = desired residual matrix [N, 4]
```

注意：这条路线得到的是“Gaia action 是否能表达 baseline 想要的修正”。如果拟合分数很低，说明 Gaia action 与悬停修正方向关系弱。

### 9.3 矩阵正则与裁剪

拟合后必须裁剪：

```text
abs(W[row, col]) <= row_max_pwm
```

并加入行级归一：

```text
sum(abs(W[roll]))     <= 30 PWM
sum(abs(W[pitch]))    <= 30 PWM
sum(abs(W[throttle])) <= 50 PWM
sum(abs(W[yaw]))      <= 30 PWM
```

这样即使 action 全部饱和，输出也不会超过安全 residual。

## 10. 在线接入方式

### 10.1 Adapter 配置文件

建议新增：

```text
flight_bridge/gaia_adapter.yaml
```

示例：

```yaml
adapter:
  type: learned_linear
  action_dim: 10
  output: rc_delta_pwm
  residual_lambda: 0.05
  max_delta_pwm:
    roll: 30
    pitch: 30
    throttle: 50
    yaw: 30
  W:
    roll:     [3.1, 0.0, 0.0, -1.2, 2.4, 0.0, 0.0, 0.0, 0.0, 0.0]
    pitch:    [0.0, 2.7, 0.0, 0.0, 0.0, -1.5, 0.0, 0.0, 0.0, 0.0]
    throttle: [0.0, 0.0, 14.0, 0.0, 0.0, 0.0, -4.0, 0.0, 0.0, 0.0]
    yaw:      [0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0]
```

### 10.2 在线计算伪代码

```python
action = sdk.infer(obs, feedback)

raw_delta = W @ action
raw_delta = clip_by_axis(raw_delta, max_delta_pwm)
raw_delta = residual_lambda * raw_delta

candidate_rc = baseline_rc + raw_delta
safe_rc = safety_shield.filter_rc(candidate_rc)

send_rc_override(safe_rc)
```

### 10.3 发散 gate

如果以下条件任意成立，SDK residual 立即淡出：

```text
feedback 连续 1.0 s 增大
tilt > 20 deg
abs(action[i]) > 0.95 的维度比例 > 70%
state_age_ms > 200
SDK latency_ms > 200
safe_command 被连续 rate-limit / clip
```

淡出方式：

```text
lambda = max(0, lambda - 0.01 per frame)
```

恢复方式：

```text
feedback 稳定下降 2 s 后，lambda 缓慢恢复到目标值
```

## 11. A/B 测试设计

### 11.1 三组实验

```text
A: baseline only
B: front-4 hard mapping
C: learned W mapping
```

每组至少：

```text
10 runs
每轮 20 ~ 30 s
同一 target_alt
同一初始位置范围
发生侧翻 / crash 立即 reset
```

### 11.2 指标

| 指标 | 含义 |
|---|---|
| success_rate | 未 crash 且保持在安全范围内 |
| distance_mean | 平均距离误差 |
| distance_p95 | 95 分位距离误差 |
| horizontal_p95 | 横向误差 95 分位 |
| vertical_p95 | 高度误差 95 分位 |
| tilt_p95 | 姿态倾角 95 分位 |
| action_saturation_pct | action 饱和比例 |
| rc_clip_count | Safety Shield 裁剪次数 |
| fallback_count | SDK residual 被关闭次数 |

### 11.3 判定标准

`learned W mapping` 必须满足：

```text
success_rate >= baseline only
distance_p95 <= baseline only * 0.9
tilt_p95 <= baseline only * 1.1
fallback_count 不显著增加
```

否则不认为 Gaia 对当前悬停任务有有效增益。

如果 `front-4 hard mapping` 明显差于 baseline，而 `learned W` 接近或优于 baseline，则说明问题主要是 action decoder，而不是 pipeline。

如果 `learned W` 仍明显差于 baseline，则说明 Gaia 当前模型对 X3 / 当前 obs schema 缺少可用控制信号，需要换模型、微调或重训。

## 12. 脚本拆分建议

### 12.1 `collect_gaia_action_dataset.py`

职责：

```text
运行 baseline + small Gaia residual
采集完整 jsonl
可选 PRBS probe
```

关键参数：

```text
--duration
--target-alt
--residual-lambda
--probe-enable
--probe-amp-roll
--probe-amp-pitch
--probe-amp-throttle
--log-file
```

### 12.2 `analyze_gaia_action_contribution.py`

职责：

```text
读取 jsonl
构造 delayed labels
输出 action 分布、PCA、Ridge/Lasso、Permutation importance
生成 markdown / csv 报告
```

输出：

```text
logs/gaia_action_contribution_report.md
logs/gaia_action_importance.csv
logs/gaia_action_pca.csv
```

### 12.3 `fit_gaia_action_adapter.py`

职责：

```text
根据贡献度结果和 desired correction 拟合 W
保存 gaia_adapter.yaml
```

输出：

```text
flight_bridge/gaia_adapter.yaml
```

### 12.4 `eval_gaia_adapter.py`

职责：

```text
执行 A/B 测试
统计 success_rate / distance_p95 / tilt_p95
输出对比报告
```

输出：

```text
logs/gaia_adapter_ab_report.md
logs/gaia_adapter_ab_metrics.csv
```

## 13. 最小可执行算法伪代码

### 13.1 贡献度分析

```python
frames = load_jsonl(log_files)
frames = filter_bad_frames(frames)

X_action = action10(frames)
X_state = state_features(frames)

for delay_ms in [100, 200, 300]:
    y = build_future_improvement(frames, delay_ms)
    X = concat([X_action, X_state])

    model = Ridge(alpha=1.0).fit(X_train, y_train)
    score = model.score(X_val, y_val)

    action_coef = model.coef_[:10]
    perm = permutation_importance(model, X_val, y_val, columns=range(10))

    report(delay_ms, score, action_coef, perm)
```

### 13.2 W 拟合

```python
frames = load_jsonl(log_files)
frames = filter_bad_frames(frames)

A = action10(frames)
S = state_features(frames)

U = desired_rc_residual(S)

ridge = Ridge(alpha=alpha, fit_intercept=False)
ridge.fit(A, U)

W = ridge.coef_  # shape [4, 10]
W = clip_and_normalize_rows(W, row_limits_pwm)

save_yaml(W)
```

### 13.3 在线 adapter

```python
def gaia_action_to_rc(action10, baseline_rc, W, cfg):
    action = np.asarray(action10[:10], dtype=float)
    action = np.clip(action, -1.0, 1.0)

    delta = W @ action
    delta = clip_axis(delta, cfg.max_delta_pwm)
    delta = cfg.residual_lambda * delta

    return RcCommand(
        roll=int(baseline_rc.roll + delta[0]),
        pitch=int(baseline_rc.pitch + delta[1]),
        throttle=int(baseline_rc.throttle + delta[2]),
        yaw=int(baseline_rc.yaw + delta[3]),
    )
```

## 14. 风险与对应处理

| 风险 | 表现 | 处理 |
|---|---|---|
| action 长期饱和 | `abs(action)>0.9` 比例很高 | 降低 lambda，增加 saturation gate |
| action 维度语义不稳定 | 不同 run 贡献度符号变化 | 不接入该维度 |
| baseline 不稳定 | 无 SDK 也漂移 / 侧翻 | 先修 PID / LOITER / 电机映射 |
| 延迟错位 | 同帧相关强但未来效果差 | 固定用 100/200/300ms 标签 |
| probe 过大 | 姿态超限 | probe 限幅，tilt gate |
| W 过拟合 | 日志内有效，A/B 无效 | Ridge 正则、交叉验证、多 run |
| X3 到真机不一致 | 仿真有效，真机无效 | 真机重新采集数据，重新拟合 W |

## 15. 实施里程碑

### M1：日志字段补齐

完成标准：

```text
jsonl 中包含 action10、baseline_rc、candidate_rc、safe_command、
copter_state、feedback、shield、state_age_ms。
```

### M2：baseline only 通过

完成标准：

```text
10 轮 baseline 测试无 crash；
distance_p95 和 tilt_p95 进入安全范围。
```

### M3：贡献度报告

完成标准：

```text
输出 action saturation、PCA、Ridge/Lasso、Permutation importance；
明确列出建议启用 / 禁用的 action 维度。
```

### M4：W adapter 初版

完成标准：

```text
生成 gaia_adapter.yaml；
在线接入时 residual 不超过配置限幅；
发散 gate 可触发 fallback。
```

### M5：A/B 测试报告

完成标准：

```text
baseline only / front-4 / learned W 三组完成 10 轮；
输出 success_rate、distance_p95、tilt_p95、fallback_count；
给出是否继续使用 Gaia 的结论。
```

## 16. 决策规则

### 16.1 继续使用 Gaia

满足：

```text
learned W mapping 优于 baseline only；
多轮贡献度符号一致；
action 饱和不会导致发散；
fallback 少量触发即可恢复。
```

### 16.2 Gaia 只作为诊断 / 调参模型

满足任一：

```text
action 贡献度弱；
W mapping 与 baseline 持平；
高饱和维度无法稳定解释；
模型无法提供稳定 residual。
```

### 16.3 需要微调或重训

满足任一：

```text
learned W mapping 明显差于 baseline；
不同 run 的重要维度随机变化；
action 与 future improvement 几乎无相关；
必须大幅 residual 才有可见响应，但一放大就发散。
```

此时应采集 X3 / 手搓机 rollouts，使用真实 mass、inertia、电机常数、延迟和传感器噪声重新训练或微调策略。

## 17. 推荐下一步

优先实现：

```text
1. analyze_gaia_action_contribution.py
2. fit_gaia_action_adapter.py
3. flight_bridge/gaia_adapter.yaml
4. ActionAdapter learned_linear 分支
5. eval_gaia_adapter.py
```

在这之前，运行闭环时建议临时配置：

```yaml
control:
  residual_lambda: 0.05
```

并把 SDK residual 限制在：

```text
roll/pitch <= 30 PWM
throttle <= 50 PWM
yaw <= 30 PWM
```

这样可以继续收集有效数据，同时避免把饱和 action 直接变成主控命令。
