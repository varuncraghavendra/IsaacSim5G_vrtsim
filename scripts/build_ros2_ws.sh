#!/usr/bin/env bash
# Build the ROS 2 workspace for IsaacSim5G.
#
# Usage:
#   bash scripts/build_ros2_ws.sh              # incremental build
#   bash scripts/build_ros2_ws.sh --clean      # nuke build/install/log first
#   bash scripts/build_ros2_ws.sh --debug      # CMake Debug build
#
# Assumes ROS 2 Humble is installed at /opt/ros/humble.

set -eo pipefail

REPO_ROOT="$(cd "$(dirname "$0")/.." && pwd)"
WS_DIR="${REPO_ROOT}/ros2_ws"
ROS_SETUP="${ROS_SETUP:-/opt/ros/humble/setup.bash}"

CLEAN=0
BUILD_TYPE="Release"
for arg in "$@"; do
    case "$arg" in
        --clean) CLEAN=1 ;;
        --debug) BUILD_TYPE="Debug" ;;
        -h|--help)
            sed -n '2,9p' "$0"
            exit 0
            ;;
        *) echo "unknown argument: $arg" >&2; exit 2 ;;
    esac
done

if [[ ! -f "${ROS_SETUP}" ]]; then
    echo "ROS 2 setup not found at ${ROS_SETUP}" >&2
    echo "set ROS_SETUP=/path/to/setup.bash if installed elsewhere" >&2
    exit 1
fi

# `set -u` interacts badly with the ROS setup script; toggle it while sourcing.
set +u
# shellcheck disable=SC1090
source "${ROS_SETUP}"
set -u

cd "${WS_DIR}"

if [[ "${CLEAN}" -eq 1 ]]; then
    echo "[build] cleaning build/ install/ log/"
    rm -rf build install log
fi

echo "[build] colcon build (${BUILD_TYPE}) in ${WS_DIR}"
colcon build \
    --packages-select network_aware_msgs network_aware_broker \
    --cmake-args "-DCMAKE_BUILD_TYPE=${BUILD_TYPE}" \
    --event-handlers console_cohesion+ \
    --symlink-install

echo "[build] done. To use the workspace:"
echo "    source ${WS_DIR}/install/setup.bash"
