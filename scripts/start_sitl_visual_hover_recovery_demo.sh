#!/usr/bin/env bash
# Start the deterministic Gazebo visual demonstration of disturbance recovery.
set -euo pipefail

ROOT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"

# The effect is deliberately independent of SDK availability so it remains
# reproducible. Set SDK_MODE=real to keep the real SDK residual enabled too.
export SDK_MODE="${SDK_MODE:-disabled}"
export SDK_RESUME="${SDK_RESUME:-false}"
export SDK_TIMEOUT_MS="${SDK_TIMEOUT_MS:-10000}"
export SHOW_HOVER_LOGS="${SHOW_HOVER_LOGS:-1}"
export HEADLESS="${HEADLESS:-0}"
export AUTO_ARM="${AUTO_ARM:-true}"
export TARGET_ALTITUDE_M="${TARGET_ALTITUDE_M:-2.0}"
export SITL_VISUAL_DEMO_ENABLED=true
export SITL_VISUAL_DEMO_HOLD_S="${SITL_VISUAL_DEMO_HOLD_S:-8.0}"
export SITL_VISUAL_DEMO_DECAY_S="${SITL_VISUAL_DEMO_DECAY_S:-14.0}"
export SITL_VISUAL_DEMO_HORIZONTAL_MPS="${SITL_VISUAL_DEMO_HORIZONTAL_MPS:-0.55}"
export SITL_VISUAL_DEMO_VERTICAL_MPS="${SITL_VISUAL_DEMO_VERTICAL_MPS:-0.12}"
export SITL_VISUAL_DEMO_PERIOD_S="${SITL_VISUAL_DEMO_PERIOD_S:-7.0}"

echo "Starting SITL visual disturbance-recovery demo (not SDK learning)."
exec "${ROOT_DIR}/scripts/start_px4_ros2_hover_sim.sh"
