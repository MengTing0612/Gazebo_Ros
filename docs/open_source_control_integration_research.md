# Open-Source Control Integration Research

调研日期：2026-06-10

本文档评估当前 Gazebo X3 + ArduCopter 2.74b SITL + Gaia SDK 项目中，
是否有成熟开源方案可以替代或减少手写 RC 外环。

## 1. 当前项目约束

当前控制链路是：

```text
Gaia SDK action10
    -> flight_bridge ActionAdapter
    -> RC_CHANNELS_OVERRIDE
    -> ArduCopter 2.74b 原生姿态 / 油门 / 电机混控链路
```

这条链路已经复用了 ArduCopter 的姿态稳定和电机混控，但位置 / 速度外环
仍主要由项目代码手写。最近 baseline-only 测试说明，高度能基本控制，
主要问题是 XY 漂移和速度阻尼不足。

本地 ArduCopter 2.74b 源码显示：

- 支持 `RC_CHANNELS_OVERRIDE`。
- 支持 `SET_MODE` 切到 `LOITER` / `GUIDED` 等旧模式。
- 支持 `MISSION_ITEM current == 2` 形式的老式 Guided waypoint。
- `LOITER` / `GUIDED` / `POSITION` 等模式要求 GPS 3D lock，否则 `set_mode`
  会回落到 `STABILIZE`。
- 当前 `ArduCopter/GCS_Mavlink.pde` 未发现现代
  `SET_POSITION_TARGET_LOCAL_NED` 或 `SET_ATTITUDE_TARGET` handler。

因此，继续在 2.74b 上用 `RC_CHANNELS_OVERRIDE` 手写 XY 外环，工程上可行，
但会把本来应由飞控承担的位置 / 速度控制转移到桥接层，调参成本高且不稳。

## 2. 成熟方案对比

### 2.1 现代 ArduPilot + ardupilot_gazebo

结论：最推荐作为主线升级方向。

成熟度：

- ArduPilot 是成熟开源飞控，现代 Copter 支持 Guided mode 里的位置、速度、
  加速度和姿态目标。
- 官方 Gazebo 插件 `ardupilot_gazebo` 支持 Gazebo Garden / Harmonic，并提供
  Iris quadcopter SITL 示例。

能解决的问题：

- 用原生 `LOITER` / `GUIDED` / `SET_POSITION_TARGET_LOCAL_NED` 接管 XY 和高度
  控制，不再手写大部分 RC 外环。
- Gaia action 可以改为小 residual：

```text
stable setpoint baseline
    + Gaia residual velocity / position correction
    -> SET_POSITION_TARGET_LOCAL_NED
    -> ArduPilot position / velocity / attitude / motor stack
```

关键接口：

- `SET_POSITION_TARGET_LOCAL_NED`
- `SET_POSITION_TARGET_GLOBAL_INT`
- `SET_ATTITUDE_TARGET`
- 外部定位可用 `ODOMETRY` / `VISION_POSITION_ESTIMATE` 等消息送入 EKF。

项目适配成本：

- 中等。需要新增现代 ArduPilot SITL/Gazebo 启动脚本，保留现有
  `flight_bridge` 的 obs / feedback / logger / visualizer。
- `MavlinkAdapter.send_velocity_cmd()` 可以真正启用，而不是现在的实验性路径。
- 需要重新做 reset / anchor / log 字段对齐。

风险：

- 和当前 2.74b 教学/源码树不完全一致。
- 若公司要求必须使用 2.74b，需要保留旧链路作对照。

参考：

- ArduPilot Guided commands:
  https://ardupilot.org/dev/docs/copter-commands-in-guided-mode.html
- ArduPilot Loiter:
  https://ardupilot.org/copter/docs/loiter-mode.html
- ArduPilot Gazebo:
  https://ardupilot.org/dev/docs/sitl-with-gazebo.html
- ArduPilot Non-GPS / ExternalNav:
  https://ardupilot.org/dev/docs/mavlink-nongps-position-estimation.html

### 2.2 PX4 + Gazebo + MAVSDK Offboard

结论：如果不强绑定 ArduPilot 2.74b，这是最干净的快速验证路线。

成熟度：

- PX4 Offboard 是成熟的外部控制模式。
- MAVSDK 提供更清晰的 C++ / Python API，Offboard 插件能自动以 20Hz 重发
  setpoint。

能解决的问题：

- 用 `set_velocity_ned()` / `set_velocity_body()` 直接给速度 setpoint。
- PX4 原生处理 position / velocity / attitude / motor 控制级联。
- Gaia action 可以自然适配成小速度 residual，而不是 RC 杆量。

推荐控制语义：

```text
baseline hover velocity = (0, 0, 0, yaw_hold)
Gaia action10 -> learned/small velocity residual
SafetyShield clamp velocity / yaw_rate
MAVSDK Offboard set_velocity_body or set_velocity_ned
```

项目适配成本：

- 中等偏高。需要引入 PX4 SITL/Gazebo 启动和 MAVSDK 依赖。
- 原有 ArduCopter 2.74b 源码链路会变成对照组，而不是主线。

风险：

- 飞控栈切换会改变参数、日志、模式语义。
- 真机若计划用 ArduPilot，则 PX4 结果只能作为控制接口验证，不能直接迁移。

参考：

- PX4 Offboard:
  https://docs.px4.io/main/en/flight_modes/offboard
- MAVSDK Offboard:
  https://mavsdk.mavlink.io/main/en/cpp/guide/offboard.html

### 2.3 ROS 2 + MAVROS / ArduPilot ROS 2 / PX4 ROS 2

结论：适合中后期接入感知、VIO、SLAM、规划；不建议作为下一步最小改动。

成熟度：

- MAVROS 是 MAVLink 到 ROS 的成熟桥，当前仓库已是 ROS 2 方向。
- ArduPilot 官方也有 ROS 2 文档和 SITL/Gazebo 入口。
- PX4 ROS 2 接口适合更完整的 autonomy stack。

能解决的问题：

- 统一接入 VIO / optical flow / SLAM / camera / lidar 等传感器。
- 更容易扩展复杂任务，而不仅是 hover。
- 可让 Gaia SDK 作为 ROS 2 node 接入。

项目适配成本：

- 高。当前项目是纯 Python bridge，不依赖 ROS。
- 加 ROS 会引入 launch、topic、tf、QoS、时间同步、bag 等额外复杂度。

推荐定位：

- 不是现在替代 RC baseline 的第一步。
- 等现代 ArduPilot/PX4 setpoint 路线稳定后，再作为传感器和任务扩展层。

参考：

- ArduPilot ROS 2:
  https://ardupilot.org/dev/docs/ros2.html
- MAVROS:
  https://github.com/mavlink/mavros

### 2.4 DroneKit-Python

结论：不推荐作为新接入主线。

优点：

- Python API 简单，历史上常用于 ArduPilot companion computer。
- 能做 telemetry、mode、mission、简单 movement 控制。

问题：

- 官方仓库明确提示项目不活跃，需要维护者。
- 最新 release 很老。
- 对现代控制接口和长期维护不如 pymavlink / MAVSDK / ROS 2。

适用场景：

- 可参考示例和概念，不建议新增依赖到主线。

参考：

- https://github.com/dronekit/dronekit-python

### 2.5 gym-pybullet-drones / Flightmare / AirSim

结论：适合训练和算法原型，不适合作为当前 SITL 控制主链路。

gym-pybullet-drones：

- 适合快速验证 RL hover、PID、action 语义、奖励函数。
- 有 Gymnasium / Stable-Baselines3 示例。
- 不直接复用 ArduCopter/PX4 的真实飞控链路。

Flightmare / Aerial Gym 类方案：

- 适合学习控制和高速仿真。
- 接入成本高，且会偏离当前 Gazebo + MAVLink 项目。

AirSim：

- 曾经成熟，支持 PX4 / ArduPilot SITL 和视觉数据。
- 但 Microsoft 原仓库已声明停止更新并转向 Project AirSim，不适合作为新主线。

参考：

- gym-pybullet-drones:
  https://github.com/learnsyslab/gym-pybullet-drones
- AirSim:
  https://github.com/microsoft/AirSim

## 3. 推荐路线

### 短期：验证 2.74b 原生 LOITER 可用性

目的不是继续扩大手写 RC baseline，而是确认当前旧固件能否进入它自己的
位置保持链路。

检查项：

```text
1. SITL 是否有 GPS 3D lock / home_is_set
2. SET_MODE LOITER 后 heartbeat 是否真的变为 LOITER
3. LOITER 下不发 RC override 是否能定点
4. LOITER 下小幅 RC stick 是否表现为位置/速度偏置
5. 若无 GPS，是否能用仿真定位注入替代
```

如果 2.74b 的 LOITER 能稳定：

```text
Gaia residual -> 小幅 RC stick bias
LOITER 原生位置/速度/姿态控制 -> motor output
```

如果 LOITER 仍然进不去或不稳，应停止在 2.74b 上继续调 XY 手写外环。

### 中期主线：迁到现代 ArduPilot + Gazebo

推荐目标架构：

```text
Gazebo / ArduPilot modern SITL
    -> MAVLink state + Gazebo truth
    -> flight_bridge obs16 + feedback
    -> Gaia SDK action10
    -> learned/small velocity residual
    -> SET_POSITION_TARGET_LOCAL_NED
    -> ArduPilot native position/velocity/attitude/motor stack
```

保留：

- `flight_bridge` obs / feedback / SDK client
- `SafetyShield`
- JSONL logger
- 三面板实时可视化
- reset/home anchor 任务定义

替换：

- `RC_CHANNELS_OVERRIDE` 主路径
- 手写 XY baseline
- 旧 2.74b Gazebo 启动脚本

### 备选主线：PX4 + MAVSDK Offboard

如果目标是尽快验证“Gaia residual 是否能改善 hover”，而不是研究旧 APM
源码，则 PX4 + MAVSDK 是更干净的实验平台：

```text
Gaia action10 -> velocity residual -> MAVSDK Offboard velocity setpoint
```

这条路线可以最快排除 RC 杆量映射错误、旧固件接口缺失和手写外环调参问题。

## 4. 当前结论

有成熟开源方案可以接入，但不建议继续把重点放在当前 2.74b 的手写
RC XY baseline 上。

推荐优先级：

```text
1. 先用当前 2.74b 做一次 LOITER/GUIDED 原生能力验证。
2. 若失败，切现代 ArduPilot + ardupilot_gazebo，接 Guided velocity/position。
3. 若目标是最快验证 SDK residual，开 PX4 + MAVSDK Offboard 对照分支。
4. ROS 2/MAVROS 放到感知、VIO、SLAM、复杂任务阶段。
5. gym-pybullet-drones 只做算法/奖励/action 语义原型，不替代飞控 SITL。
```

核心判断：

```text
Gaia 不应该继续被解释成无人机四通道杆量。
更合理的接入层是成熟飞控的 position / velocity setpoint。
```
