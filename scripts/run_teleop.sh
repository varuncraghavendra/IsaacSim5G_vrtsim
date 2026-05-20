#!/usr/bin/env bash
# Direct-mode launcher for the IsaacSim5G dashboard.
#
# Usage:
#   bash scripts/run_teleop.sh                 # nova_carter (default)
#   bash scripts/run_teleop.sh nova_carter
#   bash scripts/run_teleop.sh iris_drone
#
# Override individual knobs via env:
#   CAMERA_FPS_LIMIT=10 bash scripts/run_teleop.sh nova_carter
#   TELEOP_PORT=8081     bash scripts/run_teleop.sh
#
# The dashboard auto-discovers camera topics on the ROS graph AND accepts
# HTTP-pushed frames at POST /api/camera_push. The Isaac-side script
# isaac/scripts/push_camera_http.py is the most reliable way to get camera
# frames into the browser; it bypasses ROS 2 from Isaac entirely.

set -euo pipefail

REPO_ROOT="$(cd "$(dirname "$0")/.." && pwd)"
ROS_SETUP="${ROS_SETUP:-/opt/ros/humble/setup.bash}"
LOG_DIR="${REPO_ROOT}/.run_logs"
mkdir -p "${LOG_DIR}"

PLATFORM="${1:-nova_carter}"
shift || true   # consume the platform arg if present
export PLATFORM

if [[ ! -f "${ROS_SETUP}" ]]; then
    echo "ROS 2 setup not found at ${ROS_SETUP}" >&2
    exit 1
fi

set +u
# shellcheck disable=SC1090
source "${ROS_SETUP}"
set -u

PY="python3"
if [[ -x "${REPO_ROOT}/teleop_app/.venv/bin/python" ]]; then
    PY="${REPO_ROOT}/teleop_app/.venv/bin/python"
fi

export TELEOP_MODE="${TELEOP_MODE:-direct}"
export CAMERA_FPS_LIMIT="${CAMERA_FPS_LIMIT:-15}"
export TELEOP_PORT="${TELEOP_PORT:-8080}"

echo "[run] platform = ${PLATFORM}"
echo "[run] mode     = ${TELEOP_MODE}"
echo "[run] port     = ${TELEOP_PORT}"
echo "[run] dashboard: http://localhost:${TELEOP_PORT}"
echo "[run] log file : ${LOG_DIR}/teleop_app.log"
echo

exec "${PY}" "${REPO_ROOT}/teleop_app/app.py" --platform "${PLATFORM}" "$@" \
    2>&1 | tee "${LOG_DIR}/teleop_app.log"
