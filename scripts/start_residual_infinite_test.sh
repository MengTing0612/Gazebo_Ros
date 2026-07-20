#!/usr/bin/env bash
set -euo pipefail

ROOT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
cd "$ROOT_DIR"

LOG_DIR="$ROOT_DIR/logs"
mkdir -p "$LOG_DIR"

STAMP="$(date +%Y%m%d_%H%M%S)"
SDK_LOG="$LOG_DIR/gaia_oceanus_${STAMP}.log"
BRIDGE_LOG="$LOG_DIR/flight_bridge_residual_anchor_hover_${STAMP}.jsonl"

SDK_SERVER="${SDK_SERVER:-examples/gaia-oceanus-v0.1.0-alpha.1-x86}"
SDK_HOST="${SDK_HOST:-127.0.0.1}"
SDK_PORT="${SDK_PORT:-9000}"
MAVLINK_URL="${MAVLINK_URL:-tcp:127.0.0.1:5760}"
TARGET_ALT="${TARGET_ALT:-3.0}"

SITL_DIR="$ROOT_DIR/APM源代码原理图/ArduPlane-2.74b"
SITL_START="$SITL_DIR/Tools/start_arducopter_gz_sitl.sh"
SITL_STOP="$SITL_DIR/Tools/stop_arducopter_gz_sitl.sh"

echo "[1/5] Stopping old bridge / visualizer processes..."
pkill -TERM -f "python3 -m flight_bridge.main" >/dev/null 2>&1 || true
pkill -TERM -f "tools/visualize_feedback.py" >/dev/null 2>&1 || true
sleep 1
pkill -KILL -f "python3 -m flight_bridge.main" >/dev/null 2>&1 || true
pkill -KILL -f "tools/visualize_feedback.py" >/dev/null 2>&1 || true

echo "[2/5] Checking Gaia SDK at ${SDK_HOST}:${SDK_PORT}..."
if ! ss -tnl | grep -q ":${SDK_PORT} "; then
  echo "  SDK not listening; starting ${SDK_SERVER}"
  python3 scripts/start_retina_server.py \
    --server "$SDK_SERVER" \
    >"$SDK_LOG" 2>&1 &
  SDK_PID=$!
  echo "  SDK pid=${SDK_PID}, log=${SDK_LOG}"

  for _ in $(seq 1 30); do
    if ss -tnl | grep -q ":${SDK_PORT} "; then
      break
    fi
    sleep 1
  done

  if ! ss -tnl | grep -q ":${SDK_PORT} "; then
    echo "  SDK did not open ${SDK_PORT}. Check log:"
    echo "    ${SDK_LOG}"
    echo "  If login_token expired, run this in a terminal and complete login:"
    echo "    python3 scripts/start_retina_server.py --server ${SDK_SERVER}"
    exit 1
  fi
else
  echo "  SDK already listening."
fi

echo "[3/5] Checking Gazebo GUI and MAVLink heartbeat..."
HAVE_GAZEBO_GUI=0
if pgrep -f "gz sim gui" >/dev/null 2>&1; then
  HAVE_GAZEBO_GUI=1
fi

HAVE_MAVLINK=0
if python3 - "$MAVLINK_URL" <<'PY'
import sys
from pymavlink import mavutil

url = sys.argv[1]
mav = mavutil.mavlink_connection(url, robust_parsing=True)
msg = mav.recv_match(type="HEARTBEAT", blocking=True, timeout=5)
raise SystemExit(0 if msg is not None else 1)
PY
then
  HAVE_MAVLINK=1
fi

if [[ "$HAVE_GAZEBO_GUI" != "1" || "$HAVE_MAVLINK" != "1" ]]; then
  echo "  Gazebo GUI or MAVLink missing; restarting Gazebo/SITL GUI stack..."
  if [[ -x "$SITL_STOP" ]]; then
    "$SITL_STOP" >/dev/null 2>&1 || true
  fi
  SITL_BUILD="${SITL_BUILD:-0}"
  if [[ ! -x /tmp/ArduCopter.build/ArduCopter.elf ]]; then
    echo "  Missing /tmp/ArduCopter.build/ArduCopter.elf; building ArduCopter once..."
    SITL_BUILD=1
  fi
  (
    cd "$SITL_DIR"
    GZ_BRIDGE_EXT_MOTOR_ENABLE=1 \
    GZ_BRIDGE_EXT_MOTOR_TIMEOUT_MS=200 \
    GZ_BRIDGE_SDK_ENABLE=0 \
    BUILD="$SITL_BUILD" \
    GZ_GUI=1 \
    "$SITL_START"
  )
else
  echo "  Gazebo GUI and MAVLink are available."
fi

echo "[4/5] Starting residual infinite bridge..."
TARGET_ARGS=(--target-alt "$TARGET_ALT")
if [[ -n "${TARGET_X:-}" && -n "${TARGET_Y:-}" ]]; then
  TARGET_ARGS=(--target-x "$TARGET_X" --target-y "$TARGET_Y" "${TARGET_ARGS[@]}")
  echo "  fixed XY target: (${TARGET_X}, ${TARGET_Y})"
else
  echo "  hover anchor: reset/home XY"
fi
python3 -m flight_bridge.main \
  --phase 1 \
  --mode sitl \
  --control rc \
  --host "$SDK_HOST" \
  --port "$SDK_PORT" \
  --rc-control-mode residual \
  --iterations 0 \
  "${TARGET_ARGS[@]}" \
  --reset-on-flip \
  --reset-tilt-deg 85.0 \
  --no-reset-recovery \
  --post-reset-guard-s 0.75 \
  --reset-pose-z 0.35 \
  --reset-xy-runaway-m 20.0 \
  --reset-runaway-hold-s 2.0 \
  --log-file "$BRIDGE_LOG" &
BRIDGE_PID=$!
echo "  bridge pid=${BRIDGE_PID}, log=${BRIDGE_LOG}"

sleep 2
if ! kill -0 "$BRIDGE_PID" >/dev/null 2>&1; then
  echo "  bridge exited early. Last log lines:"
  tail -n 20 "$BRIDGE_LOG" 2>/dev/null || true
  exit 1
fi

echo "[5/5] Starting realtime feedback visualizer..."
MPLCONFIGDIR="${MPLCONFIGDIR:-/tmp/matplotlib-flight-bridge}" \
python3 tools/visualize_feedback.py "$BRIDGE_LOG" &
VIS_PID=$!
echo "  visualizer pid=${VIS_PID}"
echo
echo "Running."
echo "  bridge log: ${BRIDGE_LOG}"
echo "  SDK log:    ${SDK_LOG}"
echo
echo "Stop with:"
echo "  pkill -f 'python3 -m flight_bridge.main|tools/visualize_feedback.py'"

wait "$BRIDGE_PID"
