#!/usr/bin/env bash
set -euo pipefail

# Start PX4 v1.17 Gazebo SITL, the uXRCE-DDS Agent, and the ROS 2 hover node.
# The default launches the native PX4 position-hover baseline.  Set SDK_MODE to
# mock or real only after baseline metrics have been recorded.

ROOT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
PX4_DIR="${ROOT_DIR}/sim/px4/PX4-Autopilot"
ROS_WS="${ROOT_DIR}/ros2_ws"
ROS_DISTRO="${ROS_DISTRO:-jazzy}"
ROS_COLCON_ROOT="${ROS_COLCON_ROOT:-${HOME}/.cache/micoair_px4_ros2}"
ROS_INSTALL_BASE="${ROS_COLCON_ROOT}/install"
PX4_BUILD_DIR="${PX4_DIR}/build/px4_sitl_default"
SDK_MODE="${SDK_MODE:-disabled}"
SDK_RESUME="${SDK_RESUME:-false}"
SDK_TIMEOUT_MS="${SDK_TIMEOUT_MS:-10000}"
SHOW_HOVER_LOGS="${SHOW_HOVER_LOGS:-1}"
TARGET_ALTITUDE_M="${TARGET_ALTITUDE_M:-2.0}"
AUTO_ARM="${AUTO_ARM:-true}"
SDK_RESIDUAL_START_WEIGHT="${SDK_RESIDUAL_START_WEIGHT:-1.0}"
SDK_RESIDUAL_HOLD_S="${SDK_RESIDUAL_HOLD_S:-2.0}"
SDK_RESIDUAL_DECAY_S="${SDK_RESIDUAL_DECAY_S:-20.0}"
SDK_RESIDUAL_FINAL_WEIGHT="${SDK_RESIDUAL_FINAL_WEIGHT:-0.0}"
SITL_VISUAL_DEMO_ENABLED="${SITL_VISUAL_DEMO_ENABLED:-false}"
SITL_VISUAL_DEMO_HOLD_S="${SITL_VISUAL_DEMO_HOLD_S:-8.0}"
SITL_VISUAL_DEMO_DECAY_S="${SITL_VISUAL_DEMO_DECAY_S:-14.0}"
SITL_VISUAL_DEMO_HORIZONTAL_MPS="${SITL_VISUAL_DEMO_HORIZONTAL_MPS:-0.55}"
SITL_VISUAL_DEMO_VERTICAL_MPS="${SITL_VISUAL_DEMO_VERTICAL_MPS:-0.12}"
SITL_VISUAL_DEMO_PERIOD_S="${SITL_VISUAL_DEMO_PERIOD_S:-7.0}"
HEADLESS="${HEADLESS:-0}"
GAZEBO_GUI_RENDERER="${GAZEBO_GUI_RENDERER:-auto}"
GAZEBO_GUI_WARMUP_S="${GAZEBO_GUI_WARMUP_S:-12}"
LOG_DIR="${ROOT_DIR}/logs/px4_ros2_hover"
SITL_LOCK_FILE="${ROS_COLCON_ROOT}/px4_ros2_hover_sim.lock"
LOCAL_UXRCE_AGENT="${ROOT_DIR}/sim/px4/Micro-XRCE-DDS-Agent/build/MicroXRCEAgent"
SITL_GCS_HEARTBEAT="${ROOT_DIR}/scripts/sitl_gcs_heartbeat.py"
SITL_GCS_RECEIVE_PORT="${SITL_GCS_RECEIVE_PORT:-14540}"
NVIDIA_EGL_VENDOR_FILE="/usr/share/glvnd/egl_vendor.d/10_nvidia.json"

require_command() {
  if ! command -v "$1" >/dev/null 2>&1; then
    echo "Required command not found: $1" >&2
    echo "Run scripts/bootstrap_px4_ros2.sh first." >&2
    exit 1
  fi
}

use_system_environment() {
  unset CONDA_PREFIX CONDA_DEFAULT_ENV CONDA_PROMPT_MODIFIER CONDA_SHLVL
  unset PYTHONHOME PYTHONPATH
  unset CMAKE_PREFIX_PATH CMAKE_INCLUDE_PATH CMAKE_LIBRARY_PATH
  unset PKG_CONFIG_PATH LD_LIBRARY_PATH LIBRARY_PATH CPATH C_INCLUDE_PATH CPLUS_INCLUDE_PATH
  export PATH="/usr/local/sbin:/usr/local/bin:/usr/sbin:/usr/bin:/sbin:/bin:${HOME}/.local/bin"
}

if [[ ! -f "/opt/ros/${ROS_DISTRO}/setup.bash" ]]; then
  echo "ROS 2 ${ROS_DISTRO} is not installed." >&2
  echo "Run scripts/bootstrap_px4_ros2.sh first." >&2
  exit 1
fi

use_system_environment
# shellcheck disable=SC1091
set +u
source "/opt/ros/${ROS_DISTRO}/setup.bash"
set -u

require_command ros2

if ! /usr/bin/python3 -c "from pymavlink import mavutil" >/dev/null 2>&1; then
  echo "pymavlink is required for the SITL GCS heartbeat helper." >&2
  echo "Run scripts/bootstrap_px4_ros2.sh first." >&2
  exit 1
fi

if command -v MicroXRCEAgent >/dev/null 2>&1; then
  UXRCE_AGENT_BIN="$(command -v MicroXRCEAgent)"
elif [[ -x "${LOCAL_UXRCE_AGENT}" ]]; then
  UXRCE_AGENT_BIN="${LOCAL_UXRCE_AGENT}"
else
  echo "MicroXRCEAgent is not available globally or at ${LOCAL_UXRCE_AGENT}." >&2
  echo "Run scripts/bootstrap_px4_ros2.sh first." >&2
  exit 1
fi

if [[ ! -f "${ROS_INSTALL_BASE}/setup.bash" ]]; then
  echo "ROS workspace is not built: ${ROS_INSTALL_BASE}" >&2
  echo "Run scripts/bootstrap_px4_ros2.sh first." >&2
  exit 1
fi

if [[ -f "${PX4_BUILD_DIR}/CMakeCache.txt" ]] \
  && grep -q '/anaconda' "${PX4_BUILD_DIR}/CMakeCache.txt"; then
  echo "Removing PX4 SITL cache contaminated by Conda libraries." >&2
  rm -rf "${PX4_BUILD_DIR}"
fi

PX4_BIN="${PX4_BUILD_DIR}/bin/px4"
PX4_GZ_BRIDGE_DIR="${PX4_BUILD_DIR}/src/modules/simulation/gz_bridge"

build_px4_sitl() {
  if [[ ! -x "${PX4_BIN}" ]]; then
    echo "PX4 SITL has not been built yet; compiling the default SITL target." >&2
    make -C "${PX4_DIR}" px4_sitl_default
  fi

  # Build changed Gazebo bridge assets without invoking the gz_x500 run target.
  cmake --build "${PX4_BUILD_DIR}"
}

start_px4_sitl() {
  if [[ ! -d "${PX4_GZ_BRIDGE_DIR}" ]]; then
    echo "PX4 Gazebo bridge directory is missing: ${PX4_GZ_BRIDGE_DIR}" >&2
    exit 1
  fi

  cd "${PX4_GZ_BRIDGE_DIR}"
  # Keep PX4 responsible for the Gazebo server only. The launcher owns the
  # client process so it can choose the graphics driver and warm it up before
  # the vehicle is armed.
  PX4_SYS_AUTOSTART=4001 PX4_SIM_MODEL=gz_x500 HEADLESS=1 \
    "${PX4_BIN}" -d
}

resolve_gazebo_gui_renderer() {
  case "${GAZEBO_GUI_RENDERER}" in
    auto)
      if command -v nvidia-smi >/dev/null 2>&1 \
        && [[ -r "${NVIDIA_EGL_VENDOR_FILE}" ]]; then
        printf '%s\n' "nvidia"
      else
        printf '%s\n' "system"
      fi
      ;;
    nvidia|system)
      printf '%s\n' "${GAZEBO_GUI_RENDERER}"
      ;;
    *)
      echo "GAZEBO_GUI_RENDERER must be auto, nvidia, or system; got: ${GAZEBO_GUI_RENDERER}" >&2
      exit 1
      ;;
  esac
}

start_gazebo_gui_if_needed() {
  if [[ "${HEADLESS}" == "1" ]]; then
    return
  fi

  if pgrep -f 'gz sim -g' >/dev/null 2>&1; then
    echo "Gazebo GUI is already running."
    return
  fi

  require_command gz
  local renderer
  renderer="$(resolve_gazebo_gui_renderer)"

  if [[ "${renderer}" == "nvidia" ]]; then
    if [[ ! -r "${NVIDIA_EGL_VENDOR_FILE}" ]]; then
      echo "NVIDIA EGL vendor file is unavailable: ${NVIDIA_EGL_VENDOR_FILE}" >&2
      exit 1
    fi

    echo "Starting Gazebo GUI with NVIDIA EGL/PRIME offload."
    env __NV_PRIME_RENDER_OFFLOAD=1 \
      __GLX_VENDOR_LIBRARY_NAME=nvidia \
      __EGL_VENDOR_LIBRARY_FILENAMES="${NVIDIA_EGL_VENDOR_FILE}" \
      gz sim -g >"${LOG_DIR}/gazebo_gui_${STAMP}.log" 2>&1 9>&- &
  else
    echo "Starting Gazebo GUI with the system OpenGL renderer."
    gz sim -g >"${LOG_DIR}/gazebo_gui_${STAMP}.log" 2>&1 9>&- &
  fi
  PIDS+=("$!")
}

validate_launch_configuration() {
  if [[ "${HEADLESS}" != "0" && "${HEADLESS}" != "1" ]]; then
    echo "HEADLESS must be 0 (GUI) or 1 (server-only), got: ${HEADLESS}" >&2
    exit 1
  fi

  if ! [[ "${GAZEBO_GUI_WARMUP_S}" =~ ^[0-9]+$ ]]; then
    echo "GAZEBO_GUI_WARMUP_S must be a non-negative integer, got: ${GAZEBO_GUI_WARMUP_S}" >&2
    exit 1
  fi
}

mkdir -p "${LOG_DIR}" "${ROS_COLCON_ROOT}"
require_command flock
validate_launch_configuration
# Keep one PX4, DDS Agent, and Offboard publisher per workstation session.
exec 9>"${SITL_LOCK_FILE}"
if ! flock -n 9; then
  echo "PX4 ROS 2 SITL is already running from another launcher." >&2
  echo "Stop the existing launcher with Ctrl-C before starting another instance." >&2
  exit 1
fi

STAMP="$(date +%Y%m%d_%H%M%S)"
HOVER_NODE_LOG="${LOG_DIR}/hover_node_${STAMP}.log"
PIDS=()

cleanup() {
  for pid in "${PIDS[@]:-}"; do
    kill "${pid}" >/dev/null 2>&1 || true
  done
}
trap cleanup EXIT INT TERM

set +u
# shellcheck disable=SC1091
source "${ROS_INSTALL_BASE}/setup.bash"
set -u

"${UXRCE_AGENT_BIN}" udp4 -p 8888 \
  >"${LOG_DIR}/uxrce_agent_${STAMP}.log" 2>&1 9>&- &
PIDS+=("$!")

build_px4_sitl

start_px4_sitl \
  >"${LOG_DIR}/px4_sitl_${STAMP}.log" 2>&1 9>&- &
PIDS+=("$!")

sleep 5
start_gazebo_gui_if_needed
if [[ "${HEADLESS}" == "0" && "${GAZEBO_GUI_WARMUP_S}" != "0" ]]; then
  echo "Waiting ${GAZEBO_GUI_WARMUP_S}s for Gazebo GUI renderer warmup before arming."
  sleep "${GAZEBO_GUI_WARMUP_S}"
fi

"/usr/bin/python3" "${SITL_GCS_HEARTBEAT}" \
  --receive-port "${SITL_GCS_RECEIVE_PORT}" \
  >"${LOG_DIR}/sitl_gcs_heartbeat_${STAMP}.log" 2>&1 9>&- &
PIDS+=("$!")

ros2 launch micoair_hover hover_sim.launch.py \
  auto_arm:="${AUTO_ARM}" \
  sdk_mode:="${SDK_MODE}" \
  sdk_resume:="${SDK_RESUME}" \
  sdk_timeout_ms:="${SDK_TIMEOUT_MS}" \
  target_altitude_m:="${TARGET_ALTITUDE_M}" \
  sdk_residual_start_weight:="${SDK_RESIDUAL_START_WEIGHT}" \
  sdk_residual_hold_s:="${SDK_RESIDUAL_HOLD_S}" \
  sdk_residual_decay_s:="${SDK_RESIDUAL_DECAY_S}" \
  sdk_residual_final_weight:="${SDK_RESIDUAL_FINAL_WEIGHT}" \
  sitl_visual_demo_enabled:="${SITL_VISUAL_DEMO_ENABLED}" \
  sitl_visual_demo_hold_s:="${SITL_VISUAL_DEMO_HOLD_S}" \
  sitl_visual_demo_decay_s:="${SITL_VISUAL_DEMO_DECAY_S}" \
  sitl_visual_demo_horizontal_mps:="${SITL_VISUAL_DEMO_HORIZONTAL_MPS}" \
  sitl_visual_demo_vertical_mps:="${SITL_VISUAL_DEMO_VERTICAL_MPS}" \
  sitl_visual_demo_period_s:="${SITL_VISUAL_DEMO_PERIOD_S}" \
  >"${HOVER_NODE_LOG}" 2>&1 9>&- &
PIDS+=("$!")

if [[ "${SHOW_HOVER_LOGS}" == "1" ]]; then
  tail -n 0 -F "${HOVER_NODE_LOG}" 9>&- &
  PIDS+=("$!")
elif [[ "${SHOW_HOVER_LOGS}" != "0" ]]; then
  echo "SHOW_HOVER_LOGS must be 0 or 1, got: ${SHOW_HOVER_LOGS}" >&2
  exit 1
fi

echo "PX4 SITL hover started"
echo "  SDK mode: ${SDK_MODE}"
echo "  SDK resume: ${SDK_RESUME}"
echo "  SDK timeout: ${SDK_TIMEOUT_MS} ms"
echo "  hover-node logs on terminal: ${SHOW_HOVER_LOGS}"
echo "  target altitude: ${TARGET_ALTITUDE_M} m"
echo "  automatic arming: ${AUTO_ARM}"
echo "  Gazebo GUI renderer: ${GAZEBO_GUI_RENDERER}"
echo "  Gazebo GUI warmup: ${GAZEBO_GUI_WARMUP_S} s"
echo "  SDK residual: ${SDK_RESIDUAL_START_WEIGHT} for ${SDK_RESIDUAL_HOLD_S}s, then ${SDK_RESIDUAL_DECAY_S}s to ${SDK_RESIDUAL_FINAL_WEIGHT}"
echo "  SITL visual demo: ${SITL_VISUAL_DEMO_ENABLED} (${SITL_VISUAL_DEMO_HOLD_S}s hold, ${SITL_VISUAL_DEMO_DECAY_S}s decay)"
echo "  SITL GCS heartbeat port: ${SITL_GCS_RECEIVE_PORT}"
echo "  logs: ${LOG_DIR}"
echo "Press Ctrl-C to stop SITL, DDS Agent, and the ROS node."

wait "${PIDS[@]}"
