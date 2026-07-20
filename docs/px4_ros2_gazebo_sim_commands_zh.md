# PX4 + ROS 2 + Gazebo 仿真命令手册

本手册对应当前工作区的 PX4 v1.17、ROS 2 Jazzy、Gazebo Harmonic、uXRCE-DDS 和本地 SDK 服务。所有命令在工作区根目录执行：

```bash
cd /home/wuzelei/桌面/workspace
```

`scripts/start_px4_ros2_hover_sim.sh` 是唯一的 SITL 控制启动入口。它会启动 MicroXRCEAgent、PX4 SITL、Gazebo、仅限 SITL 的 GCS 心跳辅助和 ROS 2 悬停节点。

## 1. 首次安装和构建

仅首次执行。脚本会要求输入 `sudo` 密码，并通过镜像拉取缺失的源码与子模块：

```bash
cd /home/wuzelei/桌面/workspace
scripts/bootstrap_px4_ros2.sh
```

代码修改后，只重建 ROS 2 悬停包：

```bash
cd /home/wuzelei/桌面/workspace/ros2_ws
unset CONDA_PREFIX CONDA_DEFAULT_ENV CONDA_PROMPT_MODIFIER CONDA_SHLVL
unset PYTHONHOME PYTHONPATH
source /opt/ros/jazzy/setup.bash
colcon build --symlink-install --packages-select micoair_hover \
  --build-base "$HOME/.cache/micoair_px4_ros2/build" \
  --install-base "$HOME/.cache/micoair_px4_ros2/install"
```

快速验证控制数学和坐标系转换：

```bash
cd /home/wuzelei/桌面/workspace
/usr/bin/python3 tools/test_px4_hover_math.py
```

## 2. 启动 SDK 服务

在**终端 A**启动本地 SDK 服务。它会使用已有的网页登录信息文件自动完成授权；不在终端、日志或本文档中填写账号和密码。

```bash
cd /home/wuzelei/桌面/workspace
/usr/bin/python3 scripts/start_retina_server.py \
  --server examples/retina-server-x86_64
```

看到服务监听 `127.0.0.1:9000` 后保持该终端运行。另开终端检查端口：

```bash
ss -ltn '( sport = :9000 )'
```

若要自己在 SDK 服务终端输入网页登录信息，禁用自动填写：

```bash
cd /home/wuzelei/桌面/workspace
/usr/bin/python3 scripts/start_retina_server.py \
  --server examples/retina-server-x86_64 \
  --no-auto-login
```

## 3. 启动纯 PX4 悬停基线

在**终端 B**运行。该模式不调用 SDK，Gazebo GUI 会启动，PX4 以固定 NED 位置目标进行 Offboard 悬停：

```bash
cd /home/wuzelei/桌面/workspace
SDK_MODE=disabled TARGET_ALTITUDE_M=2.0 \
  scripts/start_px4_ros2_hover_sim.sh
```

无 GUI 的服务器模式：

```bash
cd /home/wuzelei/桌面/workspace
HEADLESS=1 SDK_MODE=disabled TARGET_ALTITUDE_M=2.0 \
  scripts/start_px4_ros2_hover_sim.sh
```

默认 GUI 模式会自动检测本机 NVIDIA EGL，并在需要时以 PRIME offload 启动 `gz sim -g`。GUI 会在自动解锁前预热 12 秒，避免渲染初始化阻断 Gazebo 的 IMU 数据流。可显式调整：

```bash
# NVIDIA 图形栈的推荐设置；预热完成后才开始自动起飞。
GAZEBO_GUI_RENDERER=nvidia GAZEBO_GUI_WARMUP_S=12 \
  SDK_MODE=disabled scripts/start_px4_ros2_hover_sim.sh

# 仅用于排查本机图形驱动问题，强制使用系统默认 OpenGL。
GAZEBO_GUI_RENDERER=system SDK_MODE=disabled \
  scripts/start_px4_ros2_hover_sim.sh
```

若日志出现 `libEGL ... driver (null)` 或 `Accel #0 fail: TIMEOUT`，不要继续调 PID。先停止会话，再以 `GAZEBO_GUI_RENDERER=nvidia` 重启；此错误表示 GUI 没有使用 NVIDIA EGL，导致 PX4 的仿真 IMU 数据中断。

一次实验只启动一个控制脚本。不要同时运行基线脚本、视觉演示脚本或其他会发布 PX4 Offboard setpoint 的节点。
启动脚本带有排他锁；已有仿真在运行时，第二次执行会直接退出并提示先停止原会话。这是预期保护行为，不会打开第二个 Gazebo 窗口。

## 4. 启动 SDK + 视觉扰动恢复演示

SDK 服务已在终端 A 正常监听时，在**终端 B**执行：

```bash
cd /home/wuzelei/桌面/workspace
SDK_MODE=real scripts/start_sitl_visual_hover_recovery_demo.sh
```

默认使用 `SDK_RESUME=false`，与 `examples/sdk_comm` 的客户端默认行为一致，避免在每次连接时加载服务端保存的会话状态。只有已确认模型必须恢复其内部会话时才显式开启：

```bash
SDK_MODE=real SDK_RESUME=true scripts/start_sitl_visual_hover_recovery_demo.sh
```

SDK 的登录和首轮推理允许最长 10 秒，与示例客户端的默认 timeout 一致。SDK worker 独立于 PX4 的 20 Hz 控制定时器，等待期间节点会安全回退为 PX4-only 悬停。若需覆盖该等待时间，单位为毫秒：

```bash
SDK_MODE=real SDK_TIMEOUT_MS=10000 scripts/start_sitl_visual_hover_recovery_demo.sh
```

启动终端默认实时显示悬停节点日志。每秒一条 `SDK output`，其中 `action` 是 SDK 返回的 10 维原始动作，`weight` 是当前残差衰减权重，`residual_body_flu_mps` 才是经过 ActionAdapter、SafetyShield 和限幅后送给 PX4 的 SDK 速度前馈。视觉演示在 22 秒后将该 residual 衰减为零；此时 raw action 仍可能变化，但不会继续影响 PX4。

若只想保留日志文件而不在启动终端显示，使用：

```bash
SDK_MODE=real SHOW_HOVER_LOGS=0 scripts/start_sitl_visual_hover_recovery_demo.sh
```

过程如下：

1. PX4 解锁并进入 Offboard。
2. SDK 以受限的速度 residual 参与 setpoint。
3. SITL 脚本叠加确定性的圆周/高度扰动。
4. 扰动保持 8 秒，接着在 14 秒内余弦衰减至零。
5. PX4 原生位置环把机体拉回固定位置目标。

这段画面是**脚本化扰动恢复演示，不是 SDK 学会悬停**。默认的视觉参数：

```bash
cd /home/wuzelei/桌面/workspace
SDK_MODE=real \
SITL_VISUAL_DEMO_HOLD_S=8 \
SITL_VISUAL_DEMO_DECAY_S=14 \
SITL_VISUAL_DEMO_HORIZONTAL_MPS=0.55 \
SITL_VISUAL_DEMO_VERTICAL_MPS=0.12 \
SITL_VISUAL_DEMO_PERIOD_S=7 \
scripts/start_sitl_visual_hover_recovery_demo.sh
```

为了只复现视觉效果、不依赖 SDK 服务：

```bash
cd /home/wuzelei/桌面/workspace
SDK_MODE=disabled scripts/start_sitl_visual_hover_recovery_demo.sh
```

## 5. 运行状态检查

在**终端 C**加载与 PX4 消息版本匹配的 ROS 环境：

```bash
unset CONDA_PREFIX CONDA_DEFAULT_ENV CONDA_PROMPT_MODIFIER CONDA_SHLVL
unset PYTHONHOME PYTHONPATH
source /opt/ros/jazzy/setup.bash
source "$HOME/.cache/micoair_px4_ros2/install/setup.bash"
```

检查 ROS 节点和话题：

```bash
ros2 node list
ros2 topic list | rg '^/fmu/'
```

检查 setpoint 发布频率，预期约为 `20 Hz`：

```bash
ros2 topic hz /fmu/in/trajectory_setpoint
```

读取 PX4 位置与飞行状态。PX4 v1.17 使用带 `_v1` 后缀的话题：

```bash
ros2 topic echo /fmu/out/vehicle_local_position_v1 --once
ros2 topic echo /fmu/out/vehicle_status_v1 --once
```

正常演示应满足：`arming_state: 2`、`nav_state: 14`、`failsafe: false`。首次刚启动的几秒可能出现本地位置数据尚未就绪的 warning，等待 EKF 与 DDS 就绪后节点会捕获悬停目标并进入 Offboard。

查看本次日志：

```bash
cd /home/wuzelei/桌面/workspace
ls -lt logs/px4_ros2_hover/
tail -f logs/px4_ros2_hover/hover_node_*.log
```

真实 SDK 接通时，节点日志会出现以下两条关键记录：

```text
SDK residual perturbation started
SDK residual attenuation complete
```

### SDK 连接诊断

先确认 SDK 服务正在监听：

```bash
ss -ltnp '( sport = :9000 )'
```

在 `SDK_MODE=real` 仿真运行后，检查 SDK TCP 连接。正常情况应只有一对 `ESTAB` 记录：一端是 `retina-server-x86_64`，另一端是 `offboard_hover`。

```bash
ss -tnp '( sport = :9000 or dport = :9000 )'
```

连续观察 10 秒，正常时 `offboard_hover` 的源端口保持不变：

```bash
for sample in {1..10}; do
  date '+%H:%M:%S'
  ss -tnp '( sport = :9000 or dport = :9000 )' | rg 'ESTAB|9000' || true
  sleep 1
done
```

服务端日志中的 `Resuming from saved state...` 表示客户端在恢复 SDK 会话；单个仿真节点出现一次是正常的。`Connection reset by peer` 表示客户端在重启、停止或监控程序退出时关闭连接，偶发一次不影响服务。

若短时间持续出现大量不同源端口的 `connected`，先停止所有重复的仿真和监控客户端，再仅保留一个 `SDK_MODE=real` 仿真。此时禁止直接删除 `/tmp/.st_*/policy_state.pkl`：`Error loading saved state ... using initial state` 表示 SDK 已自行退回初始状态，是否重置保存状态需要先备份并确认模型会话语义。

视觉演示会额外记录：

```text
SITL visual disturbance started: this is a scripted recovery demo, not SDK learning
SITL visual disturbance attenuated to zero
```

## 6. 停止仿真

在启动仿真的终端 B 按 `Ctrl-C`。启动脚本会停止 PX4 SITL、Gazebo、MicroXRCEAgent、GCS 心跳辅助和 ROS 2 节点。

如果启动终端已经关闭，先查找该次启动脚本对应的进程组：

```bash
ps -eo pid,pgid,etime,cmd | rg 'start_px4_ros2_hover_sim.sh|MicroXRCE|offboard_hover|gz sim|px4 -d'
```

确认 `PGID` 后，再停止该进程组。将 `<PGID>` 替换为上一步看到的数字：

```bash
kill -TERM -- -<PGID>
```

当前脚本启动的 Gazebo GUI 属于同一进程组，会一并退出。若此前手工执行过 `setsid gz sim -g`，它不属于启动脚本的进程组，需要单独停止：

```bash
pkill -TERM -f '^gz sim -g$'
```

SDK 服务在终端 A 中用 `Ctrl-C` 单独停止。

## 7. 严格限制

本手册中的 `start_px4_ros2_hover_sim.sh`、`start_sitl_visual_hover_recovery_demo.sh` 和 `sitl_gcs_heartbeat.py` 都只适用于 SITL。它们包含自动解锁和仿真供电检查辅助逻辑，不能连接或迁移到真机。视觉扰动参数同样绝不能用于真机。
