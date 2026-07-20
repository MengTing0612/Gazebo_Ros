#!/usr/bin/env bash
set -euo pipefail

# Installs the host-side PX4/ROS 2 dependencies and builds the workspace.
# This script never flashes, connects to, or configures a physical flight
# controller.  It must be run interactively because apt may ask for sudo.

ROOT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
PX4_DIR="${ROOT_DIR}/sim/px4/PX4-Autopilot"
ROS_WS="${ROOT_DIR}/ros2_ws"
ROS_DISTRO="${ROS_DISTRO:-jazzy}"
SYSTEM_PYTHON="/usr/bin/python3"
# Jazzy rosidl currently fails while registering generated IDL files when the
# CMake binary directory contains non-ASCII characters. The source workspace
# may stay under a localized desktop path; only its build artifacts move.
ROS_COLCON_ROOT="${ROS_COLCON_ROOT:-${HOME}/.cache/micoair_px4_ros2}"
ROS_BUILD_BASE="${ROS_COLCON_ROOT}/build"
ROS_INSTALL_BASE="${ROS_COLCON_ROOT}/install"
ROS_LOG_BASE="${ROS_COLCON_ROOT}/log"
GITHUB_MIRROR="${GITHUB_MIRROR:-https://gh-proxy.com/https://github.com}"
CLONE_ATTEMPTS_PER_SOURCE="${CLONE_ATTEMPTS_PER_SOURCE:-2}"

checkout_is_ready() {
  local directory="$1"
  local required_path="$2"
  git -C "${directory}" rev-parse --verify HEAD >/dev/null 2>&1 \
    && [[ -f "${directory}/${required_path}" ]]
}

clone_with_retry() {
  local url="$1"
  local branch="$2"
  local destination="$3"
  local max_attempts="$4"
  local required_path="$5"
  local attempt
  local delay_s
  local -a clone_args=(
    -c http.version=HTTP/1.1
    -c http.lowSpeedLimit=1
    -c http.lowSpeedTime=600
    clone
    --depth 1
    --branch "${branch}"
  )

  for ((attempt = 1; attempt <= max_attempts; attempt++)); do
    rm -rf "${destination}"
    echo "[download] ${url} (${branch}), attempt ${attempt}/${max_attempts}"
    if GIT_TERMINAL_PROMPT=0 git "${clone_args[@]}" "${url}" "${destination}" \
      && checkout_is_ready "${destination}" "${required_path}"; then
      return 0
    fi
    echo "[download] clone completed without required file: ${required_path}" >&2
    if ((attempt < max_attempts)); then
      delay_s=$((attempt * 10))
      echo "[download] attempt ${attempt} failed; retrying in ${delay_s}s" >&2
      sleep "${delay_s}"
    fi
  done

  echo "Unable to download ${url} after ${max_attempts} attempts." >&2
  return 1
}

update_submodules_with_mirror() {
  local directory="$1"
  local attempt
  local delay_s
  local mirror_rule="url.${GITHUB_MIRROR%/}/.insteadOf=https://github.com/"

  for attempt in 1 2 3 4 5; do
    echo "[submodules] mirror: ${GITHUB_MIRROR%/}, attempt ${attempt}/5"
    if GIT_TERMINAL_PROMPT=0 git \
      -c http.version=HTTP/1.1 \
      -c http.lowSpeedLimit=1 \
      -c http.lowSpeedTime=600 \
      -c "${mirror_rule}" \
      -C "${directory}" \
      submodule update --init --recursive --depth 1; then
      return 0
    fi
    if ((attempt < 5)); then
      delay_s=$((attempt * 10))
      echo "[submodules] attempt ${attempt} failed; retrying in ${delay_s}s" >&2
      sleep "${delay_s}"
    fi
  done

  echo "Unable to initialize PX4 submodules through the configured mirror." >&2
  return 1
}

build_local_uxrce_agent() {
  local agent_directory="$1"
  local build_directory="${agent_directory}/build"
  local git_config_file

  git_config_file="$(mktemp)"
  if [[ -f "${HOME}/.gitconfig" ]]; then
    git config --file "${git_config_file}" include.path "${HOME}/.gitconfig"
  fi
  git config --file "${git_config_file}" \
    "url.${GITHUB_MIRROR%/}/.insteadOf" \
    "https://github.com/"

  if [[ -f "${build_directory}/CMakeCache.txt" ]] \
    && ! grep -q '^CMAKE_C_COMPILER:FILEPATH=/usr/bin/gcc$' "${build_directory}/CMakeCache.txt"; then
    echo "[uxrce-agent] removing build cache created with a non-system compiler"
    rm -rf "${build_directory}"
  fi

  if ! GIT_CONFIG_GLOBAL="${git_config_file}" CC=/usr/bin/gcc CXX=/usr/bin/g++ \
    cmake -S "${agent_directory}" -B "${build_directory}"; then
    rm -f "${git_config_file}"
    return 1
  fi
  if ! GIT_CONFIG_GLOBAL="${git_config_file}" cmake --build "${build_directory}" -j4; then
    rm -f "${git_config_file}"
    return 1
  fi
  rm -f "${git_config_file}"

  if [[ ! -x "${build_directory}/MicroXRCEAgent" ]]; then
    echo "MicroXRCEAgent build completed without its expected executable." >&2
    return 1
  fi
}

use_system_ros_python() {
  unset CONDA_PREFIX CONDA_DEFAULT_ENV CONDA_PROMPT_MODIFIER CONDA_SHLVL
  unset PYTHONHOME PYTHONPATH
  unset CMAKE_PREFIX_PATH CMAKE_INCLUDE_PATH CMAKE_LIBRARY_PATH
  unset PKG_CONFIG_PATH LD_LIBRARY_PATH LIBRARY_PATH CPATH C_INCLUDE_PATH CPLUS_INCLUDE_PATH
  export PATH="/usr/local/sbin:/usr/local/bin:/usr/sbin:/usr/bin:/sbin:/bin:${HOME}/.local/bin"
}

clone_github_repository() {
  local repository="$1"
  local branch="$2"
  local destination="$3"
  local required_path="$4"
  if [[ -z "${GITHUB_MIRROR}" ]]; then
    echo "GITHUB_MIRROR must be a non-empty GitHub mirror prefix." >&2
    return 1
  fi

  echo "[download] mirror: ${GITHUB_MIRROR%/}"
  clone_with_retry \
    "${GITHUB_MIRROR%/}/${repository}.git" \
    "${branch}" \
    "${destination}" \
    "${CLONE_ATTEMPTS_PER_SOURCE}" \
    "${required_path}"
}

if [[ "${EUID}" == "0" ]]; then
  echo "Run this script as the normal desktop user, not root." >&2
  exit 1
fi

if [[ ! -f /etc/os-release ]] || ! grep -q 'VERSION_ID="24.04"' /etc/os-release; then
  echo "This setup is pinned to Ubuntu 24.04 + ROS 2 Jazzy." >&2
  exit 1
fi
if [[ ! -x "${SYSTEM_PYTHON}" ]]; then
  echo "ROS 2 Jazzy requires the Ubuntu system Python at ${SYSTEM_PYTHON}." >&2
  exit 1
fi

use_system_ros_python

sudo apt-get update
sudo apt-get install -y \
  git python3-pip python3-venv python3-colcon-common-extensions \
  python3-rosdep python3-vcstool python3-argcomplete \
  ros-jazzy-ros-base ros-jazzy-rmw-cyclonedds-cpp \
  build-essential cmake ninja-build \
  libasio-dev libfastcdr-dev libfastrtps-dev libtinyxml2-dev

if ! checkout_is_ready "${PX4_DIR}" "Tools/setup/ubuntu.sh"; then
  mkdir -p "$(dirname "${PX4_DIR}")"
  clone_github_repository \
    PX4/PX4-Autopilot \
    v1.17.0 \
    "${PX4_DIR}" \
    Tools/setup/ubuntu.sh
fi
update_submodules_with_mirror "${PX4_DIR}"

if ! checkout_is_ready "${ROS_WS}/src/px4_msgs" "package.xml"; then
  mkdir -p "${ROS_WS}/src"
  clone_github_repository \
    PX4/px4_msgs \
    release/1.17 \
    "${ROS_WS}/src/px4_msgs" \
    package.xml
fi

"${PX4_DIR}/Tools/setup/ubuntu.sh" --no-nuttx

if ! command -v MicroXRCEAgent >/dev/null 2>&1; then
  AGENT_DIR="${ROOT_DIR}/sim/px4/Micro-XRCE-DDS-Agent"
  if ! checkout_is_ready "${AGENT_DIR}" "CMakeLists.txt"; then
    clone_github_repository \
      eProsima/Micro-XRCE-DDS-Agent \
      master \
      "${AGENT_DIR}" \
      CMakeLists.txt
  fi
  update_submodules_with_mirror "${AGENT_DIR}"
  build_local_uxrce_agent "${AGENT_DIR}"
fi

"${SYSTEM_PYTHON}" -m pip install --user --break-system-packages -e "${ROOT_DIR}"

use_system_ros_python
# shellcheck disable=SC1091
set +u
source "/opt/ros/${ROS_DISTRO}/setup.bash"
set -u
cd "${ROS_WS}"
rm -rf "${ROS_BUILD_BASE}" "${ROS_INSTALL_BASE}" "${ROS_LOG_BASE}"
mkdir -p "${ROS_COLCON_ROOT}"
colcon --log-base "${ROS_LOG_BASE}" build \
  --build-base "${ROS_BUILD_BASE}" \
  --install-base "${ROS_INSTALL_BASE}" \
  --symlink-install \
  --cmake-args \
  -DPython3_EXECUTABLE="${SYSTEM_PYTHON}" \
  -DPYTHON_EXECUTABLE="${SYSTEM_PYTHON}"

cat <<EOF

Environment is ready. Start the SITL hover baseline with:
  ${ROOT_DIR}/scripts/start_px4_ros2_hover_sim.sh
EOF
