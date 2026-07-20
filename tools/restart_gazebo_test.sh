#!/usr/bin/env bash
# Restart Gazebo + ArduCopter + Bridge, arm, then run flight_bridge.
# Resets X3 to origin every time.
set -euo pipefail

ROOT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
APM_DIR="${ROOT_DIR}/APM源代码原理图/ArduPlane-2.74b"
PYTHON=/home/wuzelei/anaconda3/envs/jax-gpu/bin/python

TARGET_ALT="${TARGET_ALT:-3.0}"
ITERATIONS="${ITERATIONS:-500}"
SUMMARY="${SUMMARY:-50}"
CONNECT="${CONNECT:-tcp:127.0.0.1:5760}"
USERNAME="${SDK_USERNAME:-admin}"
PASSWORD="${SDK_PASSWORD:-admin123}"

echo "=== [0] Killing old processes ==="
for proc in "ArduCopter.elf" "gz sim" "gz_bridge"; do
    ps aux | grep "$proc" | grep -v grep | awk '{print $2}' | while read pid; do kill "$pid" 2>/dev/null; done
done
sleep 3
echo "Cleaned"

echo "=== [1] Starting Gazebo Sim ==="
cd "${APM_DIR}"
GZ_SIM_RESOURCE_PATH="${APM_DIR}/Tools/gazebo_bridge/models:${GZ_SIM_RESOURCE_PATH:-}" \
  gz sim -r Tools/gazebo_bridge/worlds/arducopter_official_x3.sdf \
  > /tmp/gazebo.log 2>&1 &
sleep 8
echo "Gazebo started"

echo "=== [2] Starting ArduCopter SITL ==="
SITL_KEEP_ALIVE=1 /tmp/ArduCopter.build/ArduCopter.elf \
  > /tmp/arducopter_gz.log 2>&1 &
sleep 3
echo "ArduCopter started"

echo "=== [3] Starting GZ Bridge ==="
./Tools/gazebo_bridge/build/arducopter_gz_bridge \
  > /tmp/bridge_gz.log 2>&1 &
sleep 5
echo "Bridge started"

echo "=== [4] Waiting for baro + arm ==="
$PYTHON -c "
from pymavlink import mavutil; import time
mav = mavutil.mavlink_connection('${CONNECT}')
mav.wait_heartbeat(timeout=15)
mav.mav.request_data_stream_send(mav.target_system, mav.target_component, mavutil.mavlink.MAV_DATA_STREAM_ALL, 10, 1)
# Wait for baro calibration
deadline = time.time() + 40
while time.time() < deadline:
    msg = mav.recv_match(type='STATUSTEXT', blocking=True, timeout=1)
    if msg and 'calibration complete' in msg.text.lower(): break
# Wait for baro to finish
print('Waiting for baro calibration...')
deadline = time.time() + 40
calibrated = False
while time.time() < deadline:
    msg = mav.recv_match(type='STATUSTEXT', blocking=True, timeout=1)
    if msg and 'calibration complete' in msg.text.lower():
        calibrated = True
        print(f'  {msg.text}')
        break
    elif msg:
        print(f'  [{msg.severity}] {msg.text}')

if not calibrated:
    print('WARNING: baro calibration not detected, trying to arm anyway...')

# Try COMMAND_LONG arm first, then RC stick as fallback
armed = False
for attempt in range(3):
    # COMMAND_LONG
    mav.mav.command_long_send(mav.target_system, 250, 400, 0, 1, 0, 0, 0, 0, 0, False)
    time.sleep(2)
    hb = mav.recv_match(type='HEARTBEAT', blocking=True, timeout=2)
    if hb and (hb.base_mode & 0x80):
        armed = True
        print(f'Arm attempt {attempt+1}: COMMAND_LONG OK')
        break
    # RC stick fallback
    print(f'Arm attempt {attempt+1}: RC sticks...')
    mav.mav.rc_channels_override_send(mav.target_system, mav.target_component, 1500, 1500, 1000, 2000, 0, 0, 0, 0)
    time.sleep(3)
    hb = mav.recv_match(type='HEARTBEAT', blocking=True, timeout=2)
    if hb and (hb.base_mode & 0x80):
        armed = True
        print(f'Arm attempt {attempt+1}: RC sticks OK')
        break

if not armed:
    print('WARNING: arming failed, starting bridge anyway')

# Interlock: throttle through zero
mav.mav.rc_channels_override_send(mav.target_system, mav.target_component, 1500, 1500, 1000, 1500, 0, 0, 0, 0)
time.sleep(0.8)
mav.mav.rc_channels_override_send(mav.target_system, mav.target_component, 1500, 1500, 1450, 1500, 0, 0, 0, 0)
time.sleep(1.5)
servo = mav.recv_match(type='SERVO_OUTPUT_RAW', blocking=True, timeout=8)
gpi = mav.recv_match(type='GLOBAL_POSITION_INT', blocking=True, timeout=5)
hb = mav.recv_match(type='HEARTBEAT', blocking=True, timeout=5)
armed = (hb.base_mode & 0x80) != 0 if hb else False
spinning = False
z = '?'
if servo:
    m = [servo.servo1_raw, servo.servo2_raw, servo.servo3_raw, servo.servo4_raw]
    spinning = any(x > 1150 for x in m)
if gpi:
    z = f'{gpi.relative_alt/1000:.2f}'
print(f'Armed: {armed}  Motors spinning: {spinning}  z={z}m')
mav.close()
"

echo "=== [5] Starting flight_bridge ==="
cd "${ROOT_DIR}"
exec $PYTHON -m flight_bridge.main \
  --phase 1 --mode sitl --control rc --sdk-mode real \
  --username "${USERNAME}" --password "${PASSWORD}" \
  --iterations "${ITERATIONS}" --summary-every "${SUMMARY}" \
  --connect "${CONNECT}" \
  --config flight_bridge/config.yaml \
  --target-alt "${TARGET_ALT}"
