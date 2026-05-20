#!/usr/bin/env bash
# OAI-real launcher: dashboard + UE-side bridge.
#
# This launches the two pieces of the IsaacSim5G stack that live on the
# host (operator side) and on the robot side respectively. The rest --
# OpenAirInterface gNB / UE / 5G core and Isaac Sim -- you start manually.
#
# Topology assumed by the defaults:
#
#   Operator host (this script)              Robot/UE host (this script)
#   ┌───────────────────────────────┐       ┌──────────────────────────┐
#   │ teleop_app dashboard          │       │ ue_teleop_bridge.py      │
#   │   --mode oai-real             │       │   listens on :16000      │
#   │   downlink → UE_IP:16000 ─────┼──5G──►│   publishes /cmd_vel     │
#   │   uplink listener :17500  ◄───┼──5G───┤   sends pose + camera    │
#   │   serves http://:8080         │       │   to OPERATOR_IP:17500   │
#   └───────────────────────────────┘       └──────────────────────────┘
#
# Single-machine rfsim is fine; set UE_IP=127.0.0.1 (and OPERATOR_IP=127.0.0.1
# from the UE bridge side). The 5G data plane still goes through the UPF /
# gNB / rfsim pipe -- you just don't need multiple hosts to test it.
#
# Configurable via env:
#   UE_IP           — UE's 5G IP (default 12.1.1.2)
#   UE_PORT         — UE-side UDP listener port (default 16000)
#   OPERATOR_IP     — UE writes uplink here (default 127.0.0.1)
#   OPERATOR_PORT   — dashboard listens here for uplink (default 17500)
#   PLATFORM        — nova_carter / iris_drone (default nova_carter)
#   TELEOP_PORT     — dashboard HTTP port (default 8080)
#   START_BRIDGE    — if =0, do NOT start ue_teleop_bridge.py here (e.g.
#                     when the bridge runs on a different machine)
#
# You still have to manually start the rest:
#   - OAI gNB
#   - OAI UE
#   - 5G core (open5gs or oai-cn5g)
#   - Isaac Sim with Nova_Carter scene

set -euo pipefail
REPO_ROOT="$(cd "$(dirname "$0")/.." && pwd)"
ROS_SETUP="${ROS_SETUP:-/opt/ros/humble/setup.bash}"
LOG_DIR="${REPO_ROOT}/.run_logs"
mkdir -p "${LOG_DIR}"

UE_IP="${UE_IP:-12.1.1.2}"
UE_PORT="${UE_PORT:-16000}"
OPERATOR_IP="${OPERATOR_IP:-127.0.0.1}"
OPERATOR_PORT="${OPERATOR_PORT:-17500}"
PLATFORM="${PLATFORM:-nova_carter}"
TELEOP_PORT="${TELEOP_PORT:-8080}"
START_BRIDGE="${START_BRIDGE:-1}"

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

echo "[oai] manual prerequisites (not started by this script):"
echo "        - 5G core (open5gs or oai-cn5g)"
echo "        - OAI gNB    (./nr-softmodem ... --rfsim ...)"
echo "        - OAI UE     (./nr-uesoftmodem ... --rfsim ...)"
echo "        - Isaac Sim with Nova_Carter scene"
echo "      configs live in oai_rfsim_rtf/fixed_rtf_rfsim-develop/conf/"
echo
echo "[oai] dashboard (oai-real)  TELEOP_PORT=${TELEOP_PORT}"
echo "[oai]   downlink target     ${UE_IP}:${UE_PORT}"
echo "[oai]   uplink listener     0.0.0.0:${OPERATOR_PORT}"

"${PY}" "${REPO_ROOT}/teleop_app/app.py" \
    --platform "${PLATFORM}" \
    --mode oai-real \
    --port "${TELEOP_PORT}" \
    --udp-host "${UE_IP}" --udp-port "${UE_PORT}" \
    --operator-udp-host 0.0.0.0 --operator-udp-port "${OPERATOR_PORT}" \
    >"${LOG_DIR}/teleop_app.log" 2>&1 &
APP_PID=$!
echo "[oai] dashboard PID=${APP_PID} (log: ${LOG_DIR}/teleop_app.log)"

BRIDGE_PID=""
if [[ "${START_BRIDGE}" -eq 1 ]]; then
    echo
    echo "[oai] UE bridge              listens on 0.0.0.0:${UE_PORT}"
    echo "[oai]   uplink target        ${OPERATOR_IP}:${OPERATOR_PORT}"
    "${PY}" "${REPO_ROOT}/oai_bridge/ue_teleop_bridge.py" \
        --listen-host 0.0.0.0 --listen-port "${UE_PORT}" \
        --operator-host "${OPERATOR_IP}" --operator-port "${OPERATOR_PORT}" \
        >"${LOG_DIR}/ue_bridge.log" 2>&1 &
    BRIDGE_PID=$!
    echo "[oai] UE bridge PID=${BRIDGE_PID} (log: ${LOG_DIR}/ue_bridge.log)"
fi

echo
echo "[oai] open http://localhost:${TELEOP_PORT}  in a browser."
echo "[oai] Ctrl-C to stop the dashboard + bridge."

cleanup() {
    echo
    echo "[oai] shutting down"
    [[ -n "${BRIDGE_PID}" ]] && kill "${BRIDGE_PID}" 2>/dev/null || true
    kill "${APP_PID}" 2>/dev/null || true
    sleep 0.5
    [[ -n "${BRIDGE_PID}" ]] && kill -9 "${BRIDGE_PID}" 2>/dev/null || true
    kill -9 "${APP_PID}" 2>/dev/null || true
}
trap cleanup EXIT INT TERM
wait
