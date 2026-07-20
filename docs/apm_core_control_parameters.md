# APM 2.74b ArduCopter 核心控制参数整理

## 1. 范围

本文只整理 `APM源代码原理图/ArduPlane-2.74b` 中 ArduCopter 飞控核心相关内容：

- 飞行模式与控制模式切换；
- 姿态环、角速度环、高度环、Loiter 环参数；
- MAVLink / RC 外部控制入口；
- 四旋翼电机混控和输出；
- SDK residual control 更适合接入的飞控层级。

本文不整理 Gazebo X3 模型动力学参数、Raptor / Stabilize-AI SDK 细节，也不把 X3 参数当作真机参数。

## 2. 核心控制链路

APM 2.74b 的 ArduCopter 是传统级联控制结构：

```text
RC / MAVLink / 飞行模式
    ↓
模式逻辑：STABILIZE / ALT_HOLD / LOITER / GUIDED
    ↓
外层位置 / 高度 / 速度控制
    ↓
目标 roll / pitch / yaw / throttle
    ↓
姿态角 PI：STB_RLL / STB_PIT / STB_YAW
    ↓
角速度 PID：RATE_RLL / RATE_PIT / RATE_YAW
    ↓
AP_MotorsMatrix 电机混控
    ↓
PWM 输出到电调 / 电机
```

核心源码位置：

| 内容 | 文件 |
| --- | --- |
| 主循环和调度器 | `ArduCopter/ArduCopter.pde` |
| 模式切换 | `ArduCopter/system.pde` |
| 姿态 / 角速度 / 高度控制 | `ArduCopter/Attitude.pde` |
| 参数定义 | `ArduCopter/config.h` |
| MAVLink 输入处理 | `ArduCopter/GCS_Mavlink.pde` |
| 四旋翼电机布局 | `libraries/AP_Motors/AP_MotorsQuad.cpp` |
| 电机混控 | `libraries/AP_Motors/AP_MotorsMatrix.cpp` |

## 3. 控制频率

| 控制任务 | 频率 | 源码依据 |
| --- | --- | --- |
| `fast_loop()` | 100 Hz | `ArduCopter.pde` 注释 `Main loop - 100hz` |
| 姿态角目标更新 | 100 Hz | `update_roll_pitch_mode()` |
| 角速度 PID | 100 Hz | `run_rate_controllers()` |
| 电机输出 | 100 Hz | `set_servos_4()` → `motors.output()` |
| 油门 / 高度控制 | 50 Hz | `fifty_hz_loop()` → `update_throttle_mode()` |
| MAVLink 输入检查 | 50 Hz | scheduler 中 `gcs_check_input` 周期 2 个 10ms tick |
| GPS / navigation 更新 | 10 Hz 左右 | scheduler 中 `update_navigation` 周期 10 个 10ms tick |

注意：`fast_loop()` 中当前周期先执行 `run_rate_controllers()` 和 `set_servos_4()`，然后才读取 RC 并更新下一周期的 roll / pitch / yaw target。因此外部 RC override 的变化会在后续控制周期体现。

## 4. 飞行模式与控制模式

### 4.1 模式枚举

`ArduCopter/defines.h` 中主要模式：

| 模式 | 数值 | 含义 |
| --- | ---: | --- |
| `STABILIZE` | 0 | 自稳，姿态角控制，油门基本手动 |
| `ACRO` | 1 | 角速度手控 |
| `ALT_HOLD` | 2 | 姿态手控 + 高度保持 |
| `AUTO` | 3 | 自动任务 |
| `GUIDED` | 4 | 老式 guided waypoint |
| `LOITER` | 5 | GPS 定点悬停 |
| `RTL` | 6 | 返航 |
| `CIRCLE` | 7 | 绕圈 |
| `POSITION` | 8 | 位置模式 |
| `LAND` | 9 | 降落 |
| `OF_LOITER` | 10 | 光流悬停 |

### 4.2 Roll / Pitch 控制模式

| 控制模式 | 含义 |
| --- | --- |
| `ROLL_PITCH_STABLE` | RC roll / pitch 作为目标姿态角 |
| `ROLL_PITCH_ACRO` | RC roll / pitch 作为目标角速度 |
| `ROLL_PITCH_AUTO` | 自动导航输出目标 roll / pitch |
| `ROLL_PITCH_STABLE_OF` | RC 姿态角叠加光流修正 |
| `ROLL_PITCH_LOITER` | RC 输入作为目标水平速度，Loiter 导航输出目标 roll / pitch |

### 4.3 Throttle 控制模式

| 控制模式 | 含义 |
| --- | --- |
| `THROTTLE_MANUAL` | 手动油门，直接映射到电机输出 |
| `THROTTLE_MANUAL_TILT_COMPENSATED` | 手动油门 + 倾角补偿 |
| `THROTTLE_HOLD` | 高度保持，油门杆映射为爬升率 |
| `THROTTLE_AUTO` | 自动高度控制，目标高度来自导航 |
| `THROTTLE_LAND` | 降落油门控制 |

### 4.4 常用模式实际组合

| 飞行模式 | Roll / Pitch | Throttle | Nav | 适合当前 SDK residual 吗 |
| --- | --- | --- | --- | --- |
| `STABILIZE` | `ROLL_PITCH_STABLE` | `THROTTLE_MANUAL_TILT_COMPENSATED` | `NAV_NONE` | 适合第一阶段姿态 / 油门小幅 residual |
| `ALT_HOLD` | `ROLL_PITCH_STABLE` | `THROTTLE_HOLD` | `NAV_NONE` | 适合第一阶段低空高度 + 姿态 residual |
| `LOITER` | `ROLL_PITCH_LOITER` | `THROTTLE_HOLD` | `NAV_LOITER` | APM 原生定点路径，需 GPS / home / nav 可靠后接入 |
| `GUIDED` | `ROLL_PITCH_AUTO` | `THROTTLE_AUTO` | `NAV_WP` | 只支持老式 waypoint guided，不是现代 velocity offboard |

`system.pde` 中 `mode_requires_GPS()` 明确要求 `AUTO / GUIDED / LOITER / RTL / CIRCLE / POSITION` 需要 GPS。GPS 不满足时会退回 `STABILIZE`。

## 5. 姿态与角速度 PID 参数

### 5.1 姿态角 PI

源码默认值在 `ArduCopter/config.h`：

| 参数 | 默认值 | 作用 |
| --- | ---: | --- |
| `STABILIZE_ROLL_P` | `4.5` | roll 角度误差 → 目标 roll rate |
| `STABILIZE_ROLL_I` | `0.0` | roll 角度环积分 |
| `STABILIZE_ROLL_IMAX` | `8.0 deg` | roll 角度环积分限幅 |
| `STABILIZE_PITCH_P` | `4.5` | pitch 角度误差 → 目标 pitch rate |
| `STABILIZE_PITCH_I` | `0.0` | pitch 角度环积分 |
| `STABILIZE_PITCH_IMAX` | `8.0 deg` | pitch 角度环积分限幅 |
| `STABILIZE_YAW_P` | `4.5` | yaw 角度误差 → 目标 yaw rate |
| `STABILIZE_YAW_I` | `0.0` | yaw 角度环积分 |
| `STABILIZE_YAW_IMAX` | `8.0 deg * 100` | yaw 角度环积分限幅 |

参数对外暴露名在 `ArduCopter/Parameters.pde`：

| 内部对象 | 参数前缀 |
| --- | --- |
| `pi_stabilize_roll` | `STB_RLL_` |
| `pi_stabilize_pitch` | `STB_PIT_` |
| `pi_stabilize_yaw` | `STB_YAW_` |

源码链路：

```text
get_stabilize_roll(target_angle)
    ↓ angle error
g.pi_stabilize_roll.get_pi()
    ↓ target_rate
set_roll_rate_target()
```

pitch / yaw 同理。

### 5.2 角速度 PID

源码默认值在 `ArduCopter/config.h`：

| 参数 | 默认值 | 作用 |
| --- | ---: | --- |
| `RATE_ROLL_P` | `0.150` | roll rate 误差 → roll 混控输出 |
| `RATE_ROLL_I` | `0.100` | roll rate 积分补偿 |
| `RATE_ROLL_D` | `0.004` | roll rate 微分阻尼 |
| `RATE_ROLL_IMAX` | `5.0 deg` | roll rate I 限幅 |
| `RATE_PITCH_P` | `0.150` | pitch rate 误差 → pitch 混控输出 |
| `RATE_PITCH_I` | `0.100` | pitch rate 积分补偿 |
| `RATE_PITCH_D` | `0.004` | pitch rate 微分阻尼 |
| `RATE_PITCH_IMAX` | `5.0 deg` | pitch rate I 限幅 |
| `RATE_YAW_P` | `0.200` | yaw rate 误差 → yaw 混控输出 |
| `RATE_YAW_I` | `0.020` | yaw rate 积分补偿 |
| `RATE_YAW_D` | `0.000` | yaw rate 微分阻尼 |
| `RATE_YAW_IMAX` | `8.0 deg` | yaw rate I 限幅 |

参数对外暴露名：

| 内部对象 | 参数前缀 |
| --- | --- |
| `pid_rate_roll` | `RATE_RLL_` |
| `pid_rate_pitch` | `RATE_PIT_` |
| `pid_rate_yaw` | `RATE_YAW_` |

源码链路：

```text
run_rate_controllers()
    ↓
g.rc_1.servo_out = get_rate_roll(roll_rate_target_bf)
g.rc_2.servo_out = get_rate_pitch(pitch_rate_target_bf)
g.rc_4.servo_out = get_rate_yaw(yaw_rate_target_bf)
```

限幅：

| 控制量 | 输出限幅 |
| --- | ---: |
| roll rate PID output | `[-5000, 5000]` |
| pitch rate PID output | `[-5000, 5000]` |
| yaw rate PID output | `[-4500, 4500]` |

角速度环有基本 anti-windup：当电机达到 roll / pitch / yaw 限制时，I 项会冻结，除非当前误差方向能减小积分。

## 6. 高度 / 油门控制参数

### 6.1 油门基准参数

| 参数 | 默认值 | 作用 |
| --- | ---: | --- |
| `THROTTLE_CRUISE` | `450` | 飞控估计的悬停油门，对外参数名 `TRIM_THROTTLE` |
| `THR_MID` | `500` | 油门杆中位对应的 throttle output，范围建议 `300 ~ 700` |
| `PILOT_VELZ_MAX` | `250 cm/s` | 油门杆可请求的最大垂直速度 |
| `ALT_HOLD_ACCEL_MAX` | `250 cm/s/s` | 高度保持速度曲线使用的最大加速度 |

`THR_MID` 只用于手动油门映射，让油门杆中位接近悬停点；它不是完整高度 PID 参数。

### 6.2 高度三级级联

高度链路：

```text
target_alt - current_alt
    ↓ THR_ALT_ PI
desired climb rate
    ↓ THR_RATE_ PID
desired vertical acceleration
    ↓ THR_ACCEL_ PID
throttle output 0 ~ 1000
```

### 6.3 高度位置环

| 参数 | 默认值 | 作用 |
| --- | ---: | --- |
| `ALT_HOLD_P` | `1.0` | 高度误差 → 目标爬升率 |
| `ALT_HOLD_I` | `0.0` | 高度环积分 |
| `ALT_HOLD_IMAX` | `300` | 高度环积分限幅 |

对外参数前缀：

| 内部对象 | 参数前缀 |
| --- | --- |
| `pi_alt_hold` | `THR_ALT_` |

### 6.4 垂直速度环

| 参数 | 默认值 | 作用 |
| --- | ---: | --- |
| `THROTTLE_RATE_P` | `6.0` | 垂直速度误差 → 目标垂直加速度 |
| `THROTTLE_RATE_I` | `0.0` | 垂直速度积分 |
| `THROTTLE_RATE_D` | `0.0` | 垂直速度微分 |
| `THROTTLE_RATE_IMAX` | `300` | 垂直速度 I 限幅 |

对外参数前缀：

| 内部对象 | 参数前缀 |
| --- | --- |
| `pid_throttle_rate` | `THR_RATE_` |

### 6.5 垂直加速度环

| 参数 | 默认值 | 作用 |
| --- | ---: | --- |
| `THROTTLE_ACCEL_P` | `0.75` | 垂直加速度误差 → throttle output |
| `THROTTLE_ACCEL_I` | `1.50` | 垂直加速度积分 |
| `THROTTLE_ACCEL_D` | `0.0` | 垂直加速度微分 |
| `THROTTLE_ACCEL_IMAX` | `500` | 垂直加速度 I 限幅 |

对外参数前缀：

| 内部对象 | 参数前缀 |
| --- | --- |
| `pid_throttle_accel` | `THR_ACCEL_` |

`get_throttle_accel()` 输出会叠加 `g.throttle_cruise`，再约束到 `g.throttle_min ~ g.throttle_max`。

## 7. Loiter 参数

Loiter 是 APM 2.74b ArduCopter 原生定点悬停路径，不是外部模型调研引入的新控制器。它应纳入整体方案，作为 GPS / home / navigation 可靠之后的原生定点控制层。

当前阶段建议把它放在 `STABILIZE` 和 `ALT_HOLD` 之后启用：先确认姿态环、高度环、电机混控和 RC override 都稳定，再切换到 Loiter 做定点悬停。

### 7.1 Loiter 位置环

| 参数 | 默认值 | 作用 |
| --- | ---: | --- |
| `LOITER_P` | `1.0` | 位置误差 → 目标水平速度 |
| `LOITER_I` | `0.0` | 位置环积分 |
| `LOITER_IMAX` | `30 deg` | 位置环 I 限幅 |

对外参数前缀：

| 内部对象 | 参数前缀 |
| --- | --- |
| `pi_loiter_lat` | `HLD_LAT_` |
| `pi_loiter_lon` | `HLD_LON_` |

### 7.2 Loiter 速度环

| 参数 | 默认值 | 作用 |
| --- | ---: | --- |
| `LOITER_RATE_P` | `1.0` | 水平速度误差 → 目标倾角 |
| `LOITER_RATE_I` | `0.5` | 水平速度积分 |
| `LOITER_RATE_D` | `0.0` | 水平速度微分 |
| `LOITER_RATE_IMAX` | `4 m/s/s` | 速度环 I 最大加速度 |

对外参数前缀：

| 内部对象 | 参数前缀 |
| --- | --- |
| `pid_loiter_rate_lat` | `LOITER_LAT_` |
| `pid_loiter_rate_lon` | `LOITER_LON_` |

在 `ROLL_PITCH_LOITER` 中，RC roll / pitch 输入不是直接姿态角，而是用于移动 loiter target，最大水平速度注释为 `5.0 m/s`。

### 7.3 WPNav Loiter 参数

`LOITER` 的 target 移动和导航输出在 `libraries/AC_WPNav` 中实现。需要额外关注以下参数：

| 参数 / 常量 | 默认值 | 作用 |
| --- | ---: | --- |
| `WPNAV_LOITER_SPEED` / `LOIT_SPEED` | `500 cm/s` | Loiter 最大水平速度 |
| `WPNAV_LOITER_ACCEL_MAX` | `250 cm/s/s` | Loiter 最大水平加速度 |
| `WPNAV_LOITER_ACCEL_MIN` | `25 cm/s/s` | Loiter 最小减速项 |
| `WPNAV_LOITER_SPEED_MAX_TO_CORRECT_ERROR` | `200 cm/s` | 用于修正位置误差的最大速度 |

源码执行链路：

```text
set_mode(LOITER)
    ↓
set_roll_pitch_mode(ROLL_PITCH_LOITER)
set_throttle_mode(THROTTLE_HOLD)
set_nav_mode(NAV_LOITER)
    ↓
wp_nav.init_loiter_target(current_position, current_velocity)
    ↓
update_roll_pitch_mode()
    ↓
wp_nav.move_loiter_target(control_roll, control_pitch, 0.01)
    ↓
run_nav_updates() / update_nav_mode()
    ↓
wp_nav.update_loiter()
    ↓
wp_nav.get_desired_roll() / get_desired_pitch()
    ↓
get_stabilize_roll() / get_stabilize_pitch()
```

`move_loiter_target()` 中 RC 输入会被换算为机头前向 / 右向的目标速度：

```text
pilot_vel_forward_cms = -control_pitch * loiter_accel / 4500
pilot_vel_right_cms   =  control_roll  * loiter_accel / 4500
```

所以在 LOITER 阶段，SDK 通过 RC override 给 roll / pitch residual，本质是在小幅移动 Loiter target，而不是直接命令姿态角。这一点和 STABILIZE 不同。

## 8. MAVLink / RC 控制入口

### 8.1 已在源码中确认的入口

| 入口 | 支持情况 | 说明 |
| --- | --- | --- |
| `SET_MODE` | 支持 | 使用 `custom_mode` 切换 STABILIZE / ALT_HOLD / GUIDED / LOITER 等 |
| `RC_CHANNELS_OVERRIDE` | 支持 | 覆盖 1~8 通道 RC PWM，是当前最可靠的外部 residual 入口 |
| `MISSION_ITEM current == 2` | 支持 | 老式 guided waypoint，调用 `do_guided()` |

### 8.2 未在 `GCS_Mavlink.pde` 中确认的现代 offboard 入口

这份 2.74b 源码中没有在 `GCS_Mavlink.pde` 主消息处理里看到以下现代接口的实际处理逻辑：

- `SET_POSITION_TARGET_LOCAL_NED`
- `SET_POSITION_TARGET_GLOBAL_INT`
- `SET_ATTITUDE_TARGET`

因此不能把现代 Guided velocity / attitude target 当作已验证主控制入口。若要使用，需要新增 MAVLink handler 或升级飞控栈。

### 8.3 RC override 通道建议

| RC 通道 | APM 含义 | SDK residual 建议 |
| --- | --- | --- |
| CH1 | roll | 小幅 roll 姿态角 residual |
| CH2 | pitch | 小幅 pitch 姿态角 residual |
| CH3 | throttle | STABILIZE 中可作油门 residual；ALT_HOLD 中映射为爬升率 residual |
| CH4 | yaw | 小幅 yaw / yaw rate residual |
| CH5 | flight mode | 不建议 SDK 高频改动 |

第一阶段建议：

```text
STABILIZE:
  CH1 / CH2 / CH4 = 小幅姿态修正
  CH3 = 人工或安全控制器给定低空油门

ALT_HOLD:
  CH1 / CH2 / CH4 = 小幅姿态修正
  CH3 = 小幅爬升率 residual，中心 1500 附近
```

## 9. 电机混控

### 9.1 四旋翼 X frame 电机布局

`AP_MotorsQuad.cpp` 默认 X frame：

| APM motor | 角度 | yaw factor | test order |
| --- | ---: | --- | ---: |
| `AP_MOTORS_MOT_1` | `45 deg` | `CCW` | 1 |
| `AP_MOTORS_MOT_2` | `-135 deg` | `CCW` | 3 |
| `AP_MOTORS_MOT_3` | `-45 deg` | `CW` | 4 |
| `AP_MOTORS_MOT_4` | `135 deg` | `CW` | 2 |

注意：这只是 APM 内部 motor map。仿真 X3 / 真机 ESC 接线顺序必须单独标定，不能假设和 APM 内部顺序天然一致。

### 9.2 混控公式

`AP_MotorsMatrix::output_armed()` 中核心公式：

```cpp
motor_out[i] =
    throttle.radio_out
  + roll.pwm_out  * roll_factor[i]
  + pitch.pwm_out * pitch_factor[i]
  + yaw.pwm_out   * yaw_factor[i];
```

实际源码中 yaw 会先做约束，再在有余量时追加 excess yaw。最终输出会被约束到 `radio_min ~ radio_max`，然后：

```cpp
hal.rcout->write(_motor_to_channel_map[i], motor_out[i]);
```

这说明 APM 内部已经有：

- 姿态混控；
- yaw 优先级处理；
- throttle 上下限；
- 电机饱和检测；
- I 项 anti-windup 依据。

因此 SDK 首版不应该绕过这层直接控电机。

## 10. SDK residual 接入结论

### 10.1 推荐主线

当前这版 APM 最稳妥的主线分两层。

第一层是低风险基础悬停路径：

```text
SDK obs / feedback
    ↓
SDK action
    ↓
Action Adapter：映射成小幅 RC residual
    ↓
Safety Shield：限幅、滤波、超时、姿态/高度保护
    ↓
MAVLink RC_CHANNELS_OVERRIDE
    ↓
APM STABILIZE / ALT_HOLD
    ↓
APM 原生姿态环、角速度环、高度环、电机混控
```

第二层是 APM 原生定点路径：

```text
SDK obs / feedback
    ↓
SDK action
    ↓
Action Adapter：映射成小幅位置 / 速度 / RC residual
    ↓
Safety Shield：限幅、滤波、超时、GPS/home/nav 健康检查
    ↓
APM LOITER
    ↓
HLD_LAT/HLD_LON 位置环
    ↓
LOITER_LAT/LOITER_LON 速度环
    ↓
STB_RLL/STB_PIT 姿态环
    ↓
RATE_RLL/RATE_PIT 角速度环
    ↓
AP_Motors 电机混控
```

也就是说，`LOITER` 应作为 APM 原生 baseline controller 整合进来。SDK 不替代 `LOITER_*` / `HLD_*` / `THR_*`，而是在外层给小幅 residual、任务偏置或健康监督。

### 10.2 不推荐首版直接接入的层级

| 层级 | 原因 |
| --- | --- |
| 直接电机 PWM / motor UDP | 绕过 APM 姿态 PID 和混控，风险高；只适合仿真标定 |
| `RATE_RLL/PIT/YAW` 内环 residual | 太接近电机层，容易和原生 PID 抢控制权 |
| 现代 Guided velocity / attitude target | 这版源码未确认支持对应 MAVLink handler |
| 跳过 STABILIZE / ALT_HOLD 直接只依赖 LOITER | LOITER 是原生能力，但依赖 GPS/home/nav；需要先验证基础姿态和高度链路 |

### 10.3 推荐调试顺序

```text
1. 确认 RC override 能稳定进入 CH1~CH4；
2. STABILIZE：小幅 roll / pitch step，验证姿态方向；
3. STABILIZE：扫油门，找到安全低空悬停附近范围；
4. ALT_HOLD：CH3 中位附近给小幅爬升率 residual；
5. SDK residual 只接 CH1 / CH2 / CH3 / CH4，限幅很小；
6. 记录 ATT / CTUN / RCIN / RCOU / IMU；
7. GPS/home/nav 稳定后接入 APM 原生 LOITER 定点路径；
8. 如确需 velocity / attitude offboard，再补 MAVLink handler 或升级固件。
```

## 11. 调参优先级

建议按从内到外的顺序处理：

| 阶段 | 参数 | 目标 |
| --- | --- | --- |
| 1 | `RATE_RLL_`, `RATE_PIT_`, `RATE_YAW_` | 角速度稳定，无高频抖动，不过饱和 |
| 2 | `STB_RLL_`, `STB_PIT_`, `STB_YAW_` | 姿态角跟踪平滑，过冲可控 |
| 3 | `THR_MID`, `TRIM_THROTTLE` | 油门中位和悬停油门接近 |
| 4 | `THR_ALT_`, `THR_RATE_`, `THR_ACCEL_` | 高度保持不冲高、不掉高、不震荡 |
| 5 | `HLD_LAT_`, `HLD_LON_`, `LOITER_LAT_`, `LOITER_LON_` | GPS/nav 可靠后再调定点 |
| 6 | SDK residual 权重 | 只做小幅修正，不替代飞控 baseline |

## 12. 日志核对项

调试低空悬停时优先看：

| 日志项 | 作用 |
| --- | --- |
| `ATT` | 目标姿态 vs 实际姿态 |
| `CTUN` | 油门、高度、爬升率、高度控制状态 |
| `RCIN` | RC override 是否按预期进入 |
| `RCOU` | APM 对各电机的最终 PWM 输出 |
| `IMU` | 加速度、陀螺仪、振动 |
| `ERR` | failsafe、模式切换、传感器错误 |

特别关注：

- `ATT` 中 desired roll / pitch 与实际 roll / pitch 是否同向；
- `CTUN` 中高度和爬升率是否在切换模式后突变；
- `RCOU` 四电机是否长期打满或严重不均；
- `RCIN` 是否持续刷新，避免 RC override 超时或 failsafe。

## 13. 一句话结论

这版 APM 2.74b 的可靠核心能力是原生姿态 / 高度 / Loiter PID 和 AP_Motors 混控。SDK 应作为外环 residual，先通过 `RC_CHANNELS_OVERRIDE` 小幅修正 `STABILIZE` / `ALT_HOLD`，再在 GPS/home/nav 可靠后接入 APM 原生 `LOITER` 定点路径。
