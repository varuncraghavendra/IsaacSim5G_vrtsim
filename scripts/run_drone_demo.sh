#!/usr/bin/env bash
# Drone demo launcher.
#
# Brings up:
#   - teleop_app/drone_app.py    (FastAPI + virtual joystick + MAVLink to PX4)
#   - sim_isaac_trajectory.py    (--trajectory isaac, proxies Isaac pose into 5556)
#
# PREREQS you start yourself (in order):
#   1. Isaac Sim with city_drone_fpv.py loaded and PLAY pressed.
#      (PX4 SITL is auto-launched by Pegasus; MAVLink is on UDP 14550/14540.)
#   2. In Isaac:  Window → Script Editor → File → Open
#         isaac/drone_telemetry.py
#      then Ctrl-Enter. That gives you:
#        - JPEG viewport frames → UDP 17500 (the dashboard listens here)
#        - drone pose → ZMQ tcp://127.0.0.1:5570 (sim_isaac_trajectory.py reads)
#
# Then, when you want the OAI channel pipeline in the loop:
#   3. main_paths_los_model_measurment_position.py
#   4. OAI gNB + UE (or sim_l1.py replay)
#   5. live_plot_position.py
#
# All of those subscribe to tcp://127.0.0.1:5556 — which is now driven by
# the real drone pose from Isaac, courtesy of the proxy below.
#
# Environment knobs:
#   TELEOP_PORT       HTTP port for the dashboard (default 8080).
#   MAVLINK_URL       pymavlink connection string (default
#                     udpin:127.0.0.1:14540 — the PX4 SITL companion link).
#   POSE_SUB_ADDR     where drone_telemetry.py is publishing the raw pose
#                     stream (default tcp://127.0.0.1:5570).
#   PUB_ADDR          where sim_isaac_trajectory.py should re-publish
#                     MSG_FORMAT for the channel bridge
#                     (default tcp://127.0.0.1:5556).

set -euo pipefail
REPO_ROOT="$(cd "$(dirname "$0")/.." && pwd)"
LOG_DIR="${REPO_ROOT}/.run_logs"
mkdir -p "${LOG_DIR}"

TELEOP_PORT="${TELEOP_PORT:-8080}"
MAVLINK_URL="${MAVLINK_URL:-udpin:127.0.0.1:14540}"
POSE_SUB_ADDR="${POSE_SUB_ADDR:-tcp://127.0.0.1:5570}"
PUB_ADDR="${PUB_ADDR:-tcp://127.0.0.1:5556}"
CAMERA_UDP_PORT="${CAMERA_UDP_PORT:-17500}"

PY="python3"
if [[ -x "${REPO_ROOT}/teleop_app/.venv/bin/python" ]]; then
    PY="${REPO_ROOT}/teleop_app/.venv/bin/python"
fi

_pids=()
_launch() {
    local label="$1"; shift
    "$@" >"${LOG_DIR}/${label}.log" 2>&1 &
    local pid=$!
    _pids+=("${pid}")
    echo "[drone-demo] ${label} PID=${pid} (log: ${LOG_DIR}/${label}.log)"
}

echo "[drone-demo] dashboard on http://localhost:${TELEOP_PORT}"
echo "[drone-demo] MAVLink: ${MAVLINK_URL}"
echo "[drone-demo] pose proxy: ${POSE_SUB_ADDR}  ->  ${PUB_ADDR}"
echo

_launch drone_app \
    env TELEOP_PORT="${TELEOP_PORT}" MAVLINK_URL="${MAVLINK_URL}" \
        POSE_SUB_ADDR="${POSE_SUB_ADDR}" CAMERA_UDP_PORT="${CAMERA_UDP_PORT}" \
    "${PY}" "${REPO_ROOT}/teleop_app/drone_app.py"

_launch isaac_pose_proxy \
    "${PY}" "${REPO_ROOT}/sim_isaac_trajectory.py" \
        --trajectory isaac \
        --isaac-src "${POSE_SUB_ADDR}" \
        --addr "${PUB_ADDR}"

cleanup() {
    echo
    echo "[drone-demo] shutting down"
    for pid in "${_pids[@]}"; do kill "${pid}" 2>/dev/null || true; done
    sleep 0.5
    for pid in "${_pids[@]}"; do kill -9 "${pid}" 2>/dev/null || true; done
}
trap cleanup EXIT INT TERM

echo
echo "[drone-demo] open http://localhost:${TELEOP_PORT} in your browser"
echo "[drone-demo] In Isaac, Script Editor: isaac/drone_telemetry.py  Ctrl-Enter"
echo "[drone-demo] Channel pipeline (separate terms):"
echo "[drone-demo]   python main_paths_los_model_measurment_position.py --gnb-x 0 --gnb-y 0 --gnb-z 3"
echo "[drone-demo]   python live_plot_position.py"
echo "[drone-demo] Ctrl-C to stop everything started by this launcher."
wait
