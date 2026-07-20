# PX4 + ROS 2 悬停仿真与迁移方案

## 1. 固定的技术基线

| 层级 | 选择 | 原因 |
| --- | --- | --- |
| 飞控 | PX4 v1.17.0 | 包含 MicoAir743v2 官方板级 target。 |
| 板端固件 | `micoair_h743-v2_default.px4` | PX4 release v1.17.0 对该板的正式产物。 |
| 仿真 | PX4 SITL + Gazebo Harmonic `gz_x500` | 同一套 PX4 估计、位置和多旋翼控制器，不伪造板载 MCU。 |
| ROS 2 | Jazzy + `px4_msgs release/1.17` | 与 Ubuntu 24.04 和 PX4 版本匹配。 |
| 中间件 | uXRCE-DDS | PX4 原生 ROS 2 接口；不在主控制链中引入 MAVROS。 |
| 控制接口 | `OffboardControlMode` + `TrajectorySetpoint` | 让 PX4 保留姿态、角速度、推力和位置闭环。 |

MicoAir743v2 的 PX4 board config 已包含 `uxrce_dds_client`、BMI088/BMI270、SPL06 和距离传感器驱动。仿真使用 `gz_x500` 的动力学，而不是模拟具体 PCB；真机迁移时 ROS 2 节点和 DDS 接口保持不变。

## 2. 本仓库新增的仿真链路

```text
Gazebo Harmonic x500
        |
PX4 SITL v1.17.0
        | uXRCE-DDS / UDP 8888
MicroXRCEAgent
        |
ROS 2 Jazzy + px4_msgs
        |
micoair_hover/offboard_hover
        |-- 原生位置悬停 target
        `-- 可选 SDK 小幅速度 residual
```

控制节点只发布：

- `/fmu/in/offboard_control_mode`：`position=true`。
- `/fmu/in/trajectory_setpoint`：固定 NED 位置目标，SDK residual 仅作为速度前馈。
- `/fmu/in/vehicle_command`：仅当 `auto_arm=true` 时，在发布一秒 setpoint 后请求 Offboard 和解锁。

PX4 v1.17 对 `VehicleLocalPosition` 与 `VehicleStatus` 使用消息版本 1，因此本节点默认订阅 `/fmu/out/vehicle_local_position_v1` 和 `/fmu/out/vehicle_status_v1`。如果后续更换 PX4 主版本，只能在确认 DDS topic version 后通过启动参数改写，不应混用无后缀话题。

在默认配置中，`auto_arm=false`、`sdk_mode=disabled`。仿真启动脚本才将 `auto_arm` 打开。SDK 线程独立于 20 Hz 控制定时器；推理超时、状态过期、NaN 或连接失败时输出会归零，PX4 继续维持原生位置 target。

## 3. 坐标系约束

PX4 本地位置与 setpoint 使用 NED，机体系使用 FRD；项目 SDK 状态使用 body-FLU：

```text
PX4 NED/FRD -- micoair_hover.frames --> SDK body-FLU
SDK body-FLU -- micoair_hover.frames --> PX4 NED setpoint
```

转换只允许出现在 `micoair_hover/frames.py`。不允许控制节点、感知节点或策略节点各自实现符号转换。特别是：NED 的 `z` 向下、FLU 的 `z` 向上；FRD 的 `y` 向右、FLU 的 `y` 向左。

## 4. 推荐控制路线与对照实验

| 路线 | 用途 | 是否作为主线 |
| --- | --- | --- |
| PX4 `POSCTL` / 原生位置悬停 | 校验 EKF、机架、电机、传感器和调参。 | 是，所有外部控制前必须通过。 |
| ROS 2 Offboard 固定 `TrajectorySetpoint` | 复现与评估自动定点悬停。 | 是，当前实现。 |
| ROS 2 Offboard + SDK velocity residual | 检查 SDK 是否降低漂移/速度/倾角。 | 是，但必须在 baseline 后，小于 0.1 m/s。 |
| PX4 ROS 2 Interface Library External Mode | 任务逻辑复杂、需要 PX4 更高层 mode 接口时的候选。 | 后续对照，不先引入。 |
| MAVSDK Offboard | 非 ROS 2 对照客户端，适合单独验证 MAVLink 指令链。 | 辅助，不与 DDS 控制端同时运行。 |
| 直接姿态/推力、direct actuator | 研究低层控制器。 | 不用于首次悬停，风险和调参成本过高。 |

不要让 MAVROS/MAVSDK 和 uXRCE-DDS 同时向同一 PX4 实例发送 Offboard setpoint。每一轮实验只有一个控制所有者。

## 5. 仿真运行顺序

首次安装需要在有 sudo 密码的终端执行：

```bash
cd /home/wuzelei/桌面/workspace
scripts/bootstrap_px4_ros2.sh
```

它安装 ROS 2 Jazzy、PX4 SITL 依赖、MicroXRCE-DDS Agent，并编译 `ros2_ws`。当前自动化会话不能输入 sudo 密码，因此不能代替这一步。

工作区源代码可以保留在中文目录中，但 Jazzy 的接口生成器要求其 CMake 产物目录使用 ASCII 路径。脚本默认将 `build`、`install` 和日志放在 `$HOME/.cache/micoair_px4_ros2`；启动脚本会自动从该位置加载，无需手动 `source ros2_ws/install/setup.bash`。两个脚本也会隔离 Conda 的 CMake、头文件和库路径，避免其旧版 Protobuf 与 Gazebo Harmonic 混用。

无界面 SITL 没有真实 GCS 和机载 `system_power` 话题。启动脚本会运行一个仅限 SITL 的 MAVLink 辅助进程：它只发送 GCS 心跳，并设置开发用供电检查断路器 `CBRK_SUPPLY_CHK=894281`。该辅助进程不发送模式、解锁或 setpoint，ROS 2 节点仍是 Offboard 控制唯一所有者。此配置绝不能迁移到 MicoAir743v2 真机。

先跑纯 PX4 baseline：

```bash
cd /home/wuzelei/桌面/workspace
SDK_MODE=disabled TARGET_ALTITUDE_M=2.0 scripts/start_px4_ros2_hover_sim.sh
```

之后才启用本地 mock SDK：

```bash
SDK_MODE=mock TARGET_ALTITUDE_M=2.0 scripts/start_px4_ros2_hover_sim.sh
```

### Gazebo 视觉演示：从明显漂移到恢复定点

下面的脚本只用于 Gazebo SITL。它在 `ARMED + OFFBOARD` 后对固定位置目标叠加一段确定性的速度前馈：先以小圆周和高度起伏造成可见漂移，再在 14 秒内按余弦曲线衰减为零。速度前馈归零后，PX4 原生位置环会将飞机拉回原定点，因此画面呈现“先不稳、后恢复悬停”的过程。

```bash
cd /home/wuzelei/桌面/workspace
scripts/start_sitl_visual_hover_recovery_demo.sh
```

这是一段**脚本化扰动恢复演示，不是 SDK 学会悬停**。默认关闭 SDK，使视觉效果可复现；若要同时保持已登录的真实 SDK 残差链路，可运行：

```bash
SDK_MODE=real scripts/start_sitl_visual_hover_recovery_demo.sh
```

演示默认参数为 `0.55 m/s` 水平前馈、`0.12 m/s` 垂直前馈、保持 8 秒、衰减 14 秒。它们只允许通过 `SITL_VISUAL_DEMO_*` 环境变量在 Gazebo 中调整，绝不能迁移到真机。

真实 SDK 仅在 baseline 与 mock 都通过后使用：

```bash
SDK_MODE=real TARGET_ALTITUDE_M=2.0 scripts/start_px4_ros2_hover_sim.sh
```

在另一个 ROS 2 终端检查：

```bash
ros2 topic hz /fmu/in/trajectory_setpoint
ros2 topic echo /fmu/out/vehicle_local_position --once
ros2 topic echo /fmu/out/vehicle_status --once
```

## 6. 悬停验收与调研指标

每个配置至少记录 60 秒，使用相同的起飞点、高度和风扰动场景。先报告 baseline，再报告 SDK 相对 baseline 的变化：

| 指标 | 初始目标 |
| --- | --- |
| `xy` 位置 RMSE / P95 | 作为定点悬停的主指标。 |
| 高度 RMSE / P95 | 与水平误差分开统计。 |
| 水平速度 P95 | 识别慢漂与振荡。 |
| roll/pitch P95 | 防止用大倾角伪造短期误差改善。 |
| Offboard 中断、EKF reset、failsafe 次数 | 必须为零，或有明确故障解释。 |
| SDK stale / safety clamp 比例 | 高比例时不接受 SDK 结果。 |

推荐实验顺序：原生 POSCTL、Offboard position baseline、Offboard+mock residual、Offboard+真实 SDK。若真实 SDK 未能同时改善位置误差和速度/倾角，保留 PX4 baseline，不提高 residual 权重。

## 7. 真机迁移门槛

仿真通过不等于允许真飞。迁移到 MicoAir743v2 前必须完成：

1. QGroundControl 刷入 `micoair_h743-v2_default.px4`，校准传感器、遥控器、电调和电源。
2. 先拆桨核对机架、执行器分配、电机编号和转向。
3. 室外使用 GPS；室内则先使光流加可靠测距或 VIO 被 EKF2 接受。仅靠 IMU/气压计不能稳定定点。
4. 选择一个实际空闲串口给 `uxrce_dds_client`，设置正确波特率和 client transport；USB 可用于初次调试。
5. 配置遥控人工接管、Offboard loss、低电压和 EKF failsafe。Offboard 丢失动作应是 Hold/Land 等受控行为，不能用“零油门”处理。
6. 真机首轮保持 `sdk_mode=disabled`、`auto_arm=false`，在系留/空旷区域手动解锁后验证原生 Position 模式。
