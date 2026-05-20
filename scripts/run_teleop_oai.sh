#!/usr/bin/env bash
# OAI-mode launcher: full broker stack + Isaac bridge + dashboard.
#
# Use this when you bring OpenAirInterface online and want every Twist to
# ride through the network-aware broker (simulated 5G latency / jitter / PER).
#
# In this mode:
#   * The dashboard runs in OAI mode -- it sends commands as UDP JSON
#     datagrams to 127.0.0.1:16000 instead of publishing /cmd_vel directly.
#   * The broker stack (4 ROS 2 nodes) listens on the UDP ingress and
#     republishes Twists on /network/cmd_vel.
#   * The Isaac bridge relays /network/cmd_vel -> /cmd_vel at 30 Hz with a
#     500 ms watchdog.
#
# Isaac Sim must already be running with the Nova Carter scene loaded.
#
# Press Ctrl-C to stop everything.

set -euo pipefail

REPO_ROOT="$(cd "$(dirname "$0")/.." && pwd)"
ROS_SETUP="${ROS_SETUP:-/opt/ros/humble/setup.bash}"
LOG_DIR="${REPO_ROOT}/.run_logs"
mkdir -p "${LOG_DIR}"

if [[ ! -f "${ROS_SETUP}" ]]; then
    echo "ROS 2 setup not found at ${ROS_SETUP}" >&2
    exit 1
fi
if [[ ! -f "${REPO_ROOT}/ros2_ws/install/setup.bash" ]]; then
    echo "ros2_ws not built. Run: bash scripts/build_ros2_ws.sh" >&2
    exit 1
fi

set +u
# shellcheck disable=SC1090
source "${ROS_SETUP}"
# shellcheck disable=SC1090
source "${REPO_ROOT}/ros2_ws/install/setup.bash"
set -u

PY="python3"
if [[ -x "${REPO_ROOT}/teleop_app/.venv/bin/python" ]]; then
    PY="${REPO_ROOT}/teleop_app/.venv/bin/python"
fi

export TELEOP_MODE=oai
export CAMERA_TOPIC="${CAMERA_TOPIC:-/front_stereo_camera/left/image_raw}"
export ODOM_TOPIC="${ODOM_TOPIC:-/odom}"

# 1. Broker
ros2 launch network_aware_broker network_aware_stack.launch.py \
    >"${LOG_DIR}/broker.log" 2>&1 &
BROKER_PID=$!
echo "[run] broker PID=${BROKER_PID} (log: ${LOG_DIR}/broker.log)"

# 2. Isaac bridge
"${PY}" "${REPO_ROOT}/isaac/nova_carter_bridge.py" \
    --cmd-in-topic /network/cmd_vel \
    --cmd-out-topic /cmd_vel \
    --publish-rate-hz 30 \
    --watchdog-ms 500 \
    --step-rate-hz 50 \
    >"${LOG_DIR}/bridge.log" 2>&1 &
BRIDGE_PID=$!
echo "[run] bridge PID=${BRIDGE_PID} (log: ${LOG_DIR}/bridge.log)"

# 3. Dashboard (OAI mode -- sends UDP, doesn't publish /cmd_vel)
"${PY}" "${REPO_ROOT}/teleop_app/app.py" \
    >"${LOG_DIR}/teleop_app.log" 2>&1 &
APP_PID=$!
echo "[run] dashboard PID=${APP_PID} (log: ${LOG_DIR}/teleop_app.log)"

echo
echo "[run] dashboard: http://localhost:8080"
echo "[run] Ctrl-C here to stop everything."

cleanup() {
    echo
    echo "[run] shutting down children…"
    for pid in ${APP_PID} ${BRIDGE_PID} ${BROKER_PID}; do
        if kill -0 "${pid}" 2>/dev/null; then kill "${pid}" 2>/dev/null || true; fi
    done
    sleep 1
    for pid in ${APP_PID} ${BRIDGE_PID} ${BROKER_PID}; do
        if kill -0 "${pid}" 2>/dev/null; then kill -9 "${pid}" 2>/dev/null || true; fi
    done
}
trap cleanup EXIT INT TERM
wait
