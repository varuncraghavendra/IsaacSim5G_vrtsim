#!/usr/bin/env bash
# Full host-side demo for the IsaacSim5G dashboard.
#
# Starts three processes:
#   1. The teleop dashboard (FastAPI + rclpy) on http://localhost:8080
#   2. publish_demo_frames.py — synthetic LEFT + RIGHT front_hawk camera
#      streams as real ROS 2 topics
#   3. publish_demo_odom.py   — synthetic /odom + /chassis/odom on a
#      slow Lissajous trajectory
#
# Lets you exercise the dashboard end-to-end with no Isaac Sim involvement:
# two camera panels stream, pose readout updates, drive commands publish
# /cmd_vel like usual.
#
# Press Ctrl-C to stop all three. Logs land in .run_logs/.
set -euo pipefail

REPO_ROOT="$(cd "$(dirname "$0")/.." && pwd)"
ROS_SETUP="${ROS_SETUP:-/opt/ros/humble/setup.bash}"
LOG_DIR="${REPO_ROOT}/.run_logs"
mkdir -p "${LOG_DIR}"

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

# Re-use a running dashboard if there is one; otherwise start it.
if pgrep -f "teleop_app/app.py" >/dev/null; then
    APP_PID=$(pgrep -f "teleop_app/app.py" | head -1)
    echo "[demo] dashboard already running (PID=${APP_PID})"
    APP_STARTED_BY_US=0
else
    "${PY}" "${REPO_ROOT}/teleop_app/app.py" --platform nova_carter \
        >"${LOG_DIR}/teleop_app.log" 2>&1 &
    APP_PID=$!
    APP_STARTED_BY_US=1
    echo "[demo] dashboard PID=${APP_PID} (log: ${LOG_DIR}/teleop_app.log)"
fi

sleep 2

"${PY}" "${REPO_ROOT}/scripts/publish_demo_frames.py" "$@" \
    >"${LOG_DIR}/demo_frames.log" 2>&1 &
FRAMES_PID=$!
echo "[demo] frames PID=${FRAMES_PID} (log: ${LOG_DIR}/demo_frames.log)"

"${PY}" "${REPO_ROOT}/scripts/publish_demo_odom.py" \
    >"${LOG_DIR}/demo_odom.log" 2>&1 &
ODOM_PID=$!
echo "[demo] odom   PID=${ODOM_PID} (log: ${LOG_DIR}/demo_odom.log)"

echo
echo "[demo] open  http://localhost:8080  in a browser."
echo "[demo] Ctrl-C here to stop the demo publishers."
echo "       (the dashboard is left running unless we started it)"
echo

cleanup() {
    echo
    echo "[demo] shutting down demo publishers"
    for pid in ${ODOM_PID} ${FRAMES_PID}; do
        kill "${pid}" 2>/dev/null || true
    done
    if [[ "${APP_STARTED_BY_US}" -eq 1 ]]; then
        echo "[demo] also stopping dashboard PID=${APP_PID}"
        kill "${APP_PID}" 2>/dev/null || true
    fi
    sleep 0.5
    for pid in ${ODOM_PID} ${FRAMES_PID} ${APP_PID}; do
        kill -9 "${pid}" 2>/dev/null || true
    done
}
trap cleanup EXIT INT TERM

wait
