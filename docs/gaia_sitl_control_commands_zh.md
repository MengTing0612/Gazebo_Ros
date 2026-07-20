# Gaia SITL 仿真控制命令速查

本文整理当前仓库里 Gazebo X3 + ArduCopter 2.74b SITL + Gaia SDK + `flight_bridge` 的常用控制命令。

核心链路：

```text
Gazebo X3
  -> ArduCopter SITL
  -> arducopter_gz_bridge
  -> MAVLink tcp:127.0.0.1:5760
  -> flight_bridge.main
  -> Gaia SDK tcp:127.0.0.1:9000
  -> RC_CHANNELS_OVERRIDE
```

只启动 Gazebo/SITL 不会起飞。起飞需要 `flight_bridge.main --control rc` 连接 MAVLink、自动解锁并持续发送 RC 油门/姿态指令。

## 1. 启动 Gazebo + ArduCopter SITL

第一次运行时如果缺少 `/tmp/ArduCopter.build/ArduCopter.elf`，需要编译：

```bash
cd APM源代码原理图/ArduPlane-2.74b
GZ_BRIDGE_EXT_MOTOR_ENABLE=1 \
GZ_BRIDGE_EXT_MOTOR_TIMEOUT_MS=200 \
GZ_BRIDGE_SDK_ENABLE=0 \
BUILD=1 \
GZ_GUI=0 \
Tools/start_arducopter_gz_sitl.sh
```

后续已有 SITL 可执行文件后，用无 GUI 后台仿真：

```bash
cd APM源代码原理图/ArduPlane-2.74b
GZ_BRIDGE_EXT_MOTOR_ENABLE=1 \
GZ_BRIDGE_EXT_MOTOR_TIMEOUT_MS=200 \
GZ_BRIDGE_SDK_ENABLE=0 \
BUILD=0 \
GZ_GUI=0 \
Tools/start_arducopter_gz_sitl.sh
```

启动成功后 MAVLink 入口是：

```text
tcp:127.0.0.1:5760
```

## 2. 打开 Gazebo 可视化

如果 server 已经在后台运行，可单独打开 GUI client：

```bash
setsid env \
GZ_SIM_RESOURCE_PATH=/home/wuzelei/桌面/workspace/APM源代码原理图/ArduPlane-2.74b/Tools/gazebo_bridge/models:${GZ_SIM_RESOURCE_PATH:-} \
gz sim -g \
> APM源代码原理图/ArduPlane-2.74b/logs/arducopter_gz_sitl/gazebo_gui.log \
2>&1 < /dev/null &
```

也可以让启动脚本重启一套带 GUI 的仿真栈：

```bash
cd APM源代码原理图/ArduPlane-2.74b
GZ_BRIDGE_EXT_MOTOR_ENABLE=1 \
GZ_BRIDGE_EXT_MOTOR_TIMEOUT_MS=200 \
GZ_BRIDGE_SDK_ENABLE=0 \
BUILD=0 \
GZ_GUI=1 \
Tools/start_arducopter_gz_sitl.sh
```

## 3. 走 Gaia residual 主线

推荐入口：

```bash
scripts/start_residual_infinite_test.sh
```

后台运行并保存启动输出：

```bash
setsid scripts/start_residual_infinite_test.sh \
> logs/start_residual_infinite_live.stdout \
2>&1 < /dev/null &
```

这个脚本会做 5 件事：

1. 停掉旧的 `flight_bridge.main` 和反馈可视化。
2. 检查 `127.0.0.1:9000` 上是否已有 Gaia SDK。
3. 如果 SDK 未启动，则运行 `python3 scripts/start_retina_server.py --server examples/gaia-oceanus-v0.1.0-alpha.1-x86`。
4. 检查 Gazebo GUI 和 MAVLink 心跳；缺失时重启 Gazebo/SITL GUI 栈。
5. 启动 `flight_bridge.main --rc-control-mode residual` 和实时反馈可视化。

脚本内部的关键控制命令等价于：

```bash
python3 -m flight_bridge.main \
  --phase 1 \
  --mode sitl \
  --control rc \
  --host 127.0.0.1 \
  --port 9000 \
  --rc-control-mode residual \
  --iterations 0 \
  --target-alt 3.0 \
  --reset-on-flip \
  --reset-tilt-deg 85.0 \
  --no-reset-recovery \
  --post-reset-guard-s 0.75 \
  --reset-pose-z 0.35 \
  --reset-xy-runaway-m 20.0 \
  --reset-runaway-hold-s 2.0 \
  --log-file logs/flight_bridge_residual_anchor_hover_<timestamp>.jsonl
```

## 4. 重要：默认不会自动使用 lam020/lam300

当前 `scripts/start_residual_infinite_test.sh` 没有传 `--adapter-mode learned`，所以默认是 `front-4` residual 路径，不会自动读取这些文件：

```text
flight_bridge/gaia_adapter.yaml
flight_bridge/gaia_adapter_a0_a1_a9.yaml
flight_bridge/gaia_adapter_a0_a1_a9_lam020.yaml
flight_bridge/gaia_adapter_a0_a1_a9_lam300.yaml
```

如果要显式使用 learned adapter，需要手动加：

```bash
python3 -m flight_bridge.main \
  --phase 1 \
  --mode sitl \
  --control rc \
  --host 127.0.0.1 \
  --port 9000 \
  --rc-control-mode residual \
  --adapter-mode learned \
  --gaia-adapter-config flight_bridge/gaia_adapter_a0_a1_a9_lam020.yaml \
  --iterations 0 \
  --target-alt 3.0 \
  --reset-on-flip \
  --reset-tilt-deg 85.0 \
  --no-reset-recovery \
  --post-reset-guard-s 0.75 \
  --reset-pose-z 0.35 \
  --reset-xy-runaway-m 20.0 \
  --reset-runaway-hold-s 2.0 \
  --log-file logs/residual_learned_lam020_live.jsonl
```

`lam300` 的 `residual_lambda=3.00` 很激进，建议先确认默认 residual 或 `lam020` 不会快速触发 `xy_runaway`，再切：

```bash
--gaia-adapter-config flight_bridge/gaia_adapter_a0_a1_a9_lam300.yaml
```

## 5. baseline-only 对照线

baseline-only 不依赖 Gaia action，只验证 ArduCopter + RC baseline 是否能起飞和基本定高：

```bash
scripts/start_baseline_only_infinite_test.sh
```

手动等价命令：

```bash
python3 -m flight_bridge.main \
  --phase 1 \
  --mode sitl \
  --control rc \
  --host 127.0.0.1 \
  --port 9000 \
  --rc-control-mode baseline-only \
  --iterations 0 \
  --target-alt 3.0 \
  --reset-on-flip \
  --reset-tilt-deg 85.0 \
  --no-reset-recovery \
  --post-reset-guard-s 0.75 \
  --reset-pose-z 0.35 \
  --reset-xy-runaway-m 20.0 \
  --reset-runaway-hold-s 2.0 \
  --log-file logs/baseline_anchor_hover_live.jsonl
```

## 6. 状态检查

查看核心进程：

```bash
pgrep -af 'gz sim|ArduCopter.elf|arducopter_gz_bridge|python3 -m flight_bridge.main|tools/visualize_feedback.py'
```

预期至少看到：

```text
gz sim ...
ArduCopter.elf
arducopter_gz_bridge
python3 -m flight_bridge.main ...
python3 tools/visualize_feedback.py ...
```

查看最新日志：

```bash
ls -lt logs | sed -n '1,12p'
tail -n 80 logs/start_residual_infinite_live.stdout
```

查看最新控制帧的高度、解锁、油门和发送状态：

```bash
python3 -c "import json, pathlib; p=pathlib.Path('logs/flight_bridge_residual_anchor_hover_20260618_165600.jsonl'); lines=p.read_text().splitlines()[-8:];
for s in lines:
 d=json.loads(s); cs=d.get('copter_state') or {}; print('frame',d.get('frame'),'armed',d.get('armed'),'mode',d.get('vehicle_mode'),'alt',round(cs.get('altitude_m',-999),3),'z_err',round(cs.get('z_err',-999),3),'vz',round(cs.get('vz',-999),3),'baseline',d.get('baseline_rc'),'safe',d.get('safe_command'),'sent',d.get('sent'),'err',d.get('error'),'timeout',d.get('timeout_discard'))"
```

把上面文件名替换成当前 `logs/flight_bridge_residual_anchor_hover_*.jsonl` 的最新文件。

注意：`tcp:127.0.0.1:5760` 是 ArduCopter serial0 TCP 入口。`flight_bridge.main` 正在控制时，不建议再用单独 MAVLink 探针长连同一个端口，以免干扰控制连接。控制运行中优先看 JSONL 日志。

## 7. 停止

只停控制和可视化，保留 Gazebo/SITL：

```bash
pkill -f 'python3 -m flight_bridge.main|tools/visualize_feedback.py'
```

停止 Gazebo + ArduCopter + Gazebo bridge：

```bash
APM源代码原理图/ArduPlane-2.74b/Tools/stop_arducopter_gz_sitl.sh
```

如需停止 Gaia SDK：

```bash
pkill -f 'gaia-oceanus|scripts/start_retina_server.py'
```

## 8. 常用参数含义

```text
--phase 1
  启动真实控制链路。Phase 0 是 dry-run/观测日志。

--mode sitl
  连接 ArduCopter SITL。

--control rc
  使用 MAVLink RC_CHANNELS_OVERRIDE。当前 APM 2.74b 主路径。

--rc-control-mode residual
  baseline RC + Gaia/SDK residual。

--rc-control-mode baseline-only
  只用 deterministic baseline，对照用。

--target-alt 3.0
  悬停目标高度 3m。

--reset-on-flip
  检测翻机/跑飞后重置 Gazebo 模型。

--reset-xy-runaway-m 20.0
  水平误差超过 20m 且持续 --reset-runaway-hold-s 后触发 reset。

--no-reset-recovery
  reset 后不走完整恢复状态机，只用短 post-reset guard。

--post-reset-guard-s 0.75
  reset 后短时间压住 SDK residual，先发 baseline。
```

## 9. 敏感文件

`examples/config.json` 是 Gaia SDK 登录与设备激活缓存，里面可能含有 `login_token`、证书或设备信息。不要把它贴到公开文档、日志或提交记录里。
