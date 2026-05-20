#!/usr/bin/env bash
# OAI-real + network-aware broker chain.
#
# Wires the real OAI 5G radio AND the C++ broker stack into a single
# control path. Use this when you want the radio in the loop *and* the
# broker doing post-radio pacing / watchdog / shaping.
#
# Topology:
#
#   browser ─ws─► teleop_app/app.py (mode=oai-real)
#                    │  UDP JSON
#                    │  → UE_IP:16000  (over OAI 5G radio)
#                    ▼
#               oaitun_ue1 (UE TUN)
#                    │
#                    ▼
#               oai_bridge/ue_teleop_bridge.py
#                    │  forwards raw UDP → 127.0.0.1:16100
#                    │  (does NOT publish /cmd_vel here)
#                    ▼
#               broker stack (ros2 launch network_aware_stack.launch.py
#                              with BROKER_UDP_PORT=16100):
#                  ue_ingress_node :16100 → /network/teleop_command
#                  network_aware_broker  → /network/cmd_vel
#                  metrics_ingress_node :16001
#                  rtf_clock_node        → /clock
#                    │
#                    ▼
#               isaac/nova_carter_bridge.py
#                    │  /network/cmd_vel → /cmd_vel  (30 Hz, 500 ms watchdog)
#                    ▼
#               Isaac Sim Nova Carter
#
# Uplink (camera/odom → dashboard → browser) still rides through the UE
# bridge → dashboard UDP listener exactly as in mode=oai-real. The broker
# only touches the downlink command path.
#
# Manual prerequisites (not started here):
#   * 5G core (open5gs / oai-cn5g / free5GC)
#   * OAI gNB (./nr-softmodem ... --rfsim ...)
#   * OAI UE  (./nr-uesoftmodem ... --rfsim ...)  — creates oaitun_ue1
#   * Isaac Sim with the Nova_Carter_ROS scene loaded and playing
#
# Configurable via env:
#   UE_IP           — UE's 5G IP (default 10.0.0.4 to match the shipped conf)
#   UE_PORT         — UE-side UDP listener (default 16000)
#   OPERATOR_IP     — UE writes uplink here (default 127.0.0.1)
#   OPERATOR_PORT   — dashboard uplink listener (default 17500)
#   BROKER_UDP_PORT — broker's UE ingress UDP port (default 16100)
#   PLATFORM        — nova_carter / iris_drone (default nova_carter)
#   TELEOP_PORT     — dashboard HTTP port (default 8080)

set -euo pipefail
REPO_ROOT="$(cd "$(dirname "$0")/.." && pwd)"
ROS_SETUP="${ROS_SETUP:-/opt/ros/humble/setup.bash}"
LOG_DIR="${REPO_ROOT}/.run_logs"
mkdir -p "${LOG_DIR}"

UE_IP="${UE_IP:-10.0.0.4}"
UE_PORT="${UE_PORT:-16000}"
OPERATOR_IP="${OPERATOR_IP:-127.0.0.1}"
OPERATOR_PORT="${OPERATOR_PORT:-17500}"
BROKER_UDP_PORT="${BROKER_UDP_PORT:-16100}"
CAMERA_UPLINK_LISTEN_PORT="${CAMERA_UPLINK_LISTEN_PORT:-17600}"
PLATFORM="${PLATFORM:-nova_carter}"
TELEOP_PORT="${TELEOP_PORT:-8080}"

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

echo "[chain] manual prerequisites (not started here):"
echo "          - 5G core (open5gs / oai-cn5g)"
echo "          - OAI gNB / OAI UE  (creates oaitun_ue1)"
echo "          - Isaac Sim with Nova_Carter scene playing"
echo

echo "[chain] dashboard (oai-real)  TELEOP_PORT=${TELEOP_PORT}"
echo "[chain]   downlink target     ${UE_IP}:${UE_PORT}"
echo "[chain]   uplink listener     0.0.0.0:${OPERATOR_PORT}"
"${PY}" "${REPO_ROOT}/teleop_app/app.py" \
    --platform "${PLATFORM}" \
    --mode oai-real \
    --port "${TELEOP_PORT}" \
    --udp-host "${UE_IP}" --udp-port "${UE_PORT}" \
    --operator-udp-host 0.0.0.0 --operator-udp-port "${OPERATOR_PORT}" \
    >"${LOG_DIR}/teleop_app.log" 2>&1 &
APP_PID=$!
echo "[chain] dashboard PID=${APP_PID} (log: ${LOG_DIR}/teleop_app.log)"

echo
echo "[chain] UE bridge              listens :${UE_PORT}, forward → 127.0.0.1:${BROKER_UDP_PORT}"
echo "[chain]   uplink target        ${OPERATOR_IP}:${OPERATOR_PORT}"
echo "[chain]   /cmd_vel publish     suppressed (broker chain publishes downstream)"
"${PY}" "${REPO_ROOT}/oai_bridge/ue_teleop_bridge.py" \
    --listen-host 0.0.0.0 --listen-port "${UE_PORT}" \
    --operator-host "${OPERATOR_IP}" --operator-port "${OPERATOR_PORT}" \
    --forward-udp-host 127.0.0.1 --forward-udp-port "${BROKER_UDP_PORT}" \
    --no-publish-cmd-vel \
    --camera-uplink-listen-port "${CAMERA_UPLINK_LISTEN_PORT}" \
    >"${LOG_DIR}/ue_bridge.log" 2>&1 &
BRIDGE_PID=$!
echo "[chain] UE bridge PID=${BRIDGE_PID} (log: ${LOG_DIR}/ue_bridge.log)"

echo
echo "[chain] broker stack           ue_ingress on UDP :${BROKER_UDP_PORT}"
BROKER_UDP_PORT="${BROKER_UDP_PORT}" \
    ros2 launch network_aware_broker network_aware_stack.launch.py \
    >"${LOG_DIR}/broker.log" 2>&1 &
BROKER_PID=$!
echo "[chain] broker PID=${BROKER_PID} (log: ${LOG_DIR}/broker.log)"

echo
echo "[chain] nova_carter_bridge     /network/cmd_vel → /cmd_vel"
"${PY}" "${REPO_ROOT}/isaac/nova_carter_bridge.py" \
    --cmd-in-topic /network/cmd_vel \
    --cmd-out-topic /cmd_vel \
    --publish-rate-hz 30 \
    --watchdog-ms 500 \
    --step-rate-hz 50 \
    >"${LOG_DIR}/nova_carter_bridge.log" 2>&1 &
NC_PID=$!
echo "[chain] nova_carter_bridge PID=${NC_PID} (log: ${LOG_DIR}/nova_carter_bridge.log)"

echo
echo "[chain] open http://localhost:${TELEOP_PORT}  in a browser."
echo "[chain] Ctrl-C to stop everything started here."

cleanup() {
    echo
    echo "[chain] shutting down"
    for pid in ${NC_PID} ${BROKER_PID} ${BRIDGE_PID} ${APP_PID}; do
        kill "${pid}" 2>/dev/null || true
    done
    sleep 0.5
    for pid in ${NC_PID} ${BROKER_PID} ${BRIDGE_PID} ${APP_PID}; do
        kill -9 "${pid}" 2>/dev/null || true
    done
}
trap cleanup EXIT INT TERM
wait
