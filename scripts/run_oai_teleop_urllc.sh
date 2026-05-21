#!/usr/bin/env bash
# OAI-real + URLLC + broker chain + real UL via netns.
#
# This is the architecturally-correct teleop mode: command DL rides PDSCH
# (via ext-dn → UPF → gNB → UE) AND camera/pose UL rides PUSCH (via the
# robot-ns proxy → oaitun_ue1 → UE → gNB → UPF → operator). All UDP flows
# are DSCP-marked so a properly-configured 5G QoS rule can place them on
# URLLC / interactive-video 5QI flows.
#
# Topology:
#
#   ┌───────────────── default ns (operator side) ───────────────────┐
#   │                                                                │
#   │  browser ─ws─► teleop_app/app.py  (mode=oai-real, DSCP_EF)    │
#   │                    │                                            │
#   │                    │  UDP  ──► 192.168.70.135:16000  (ext-dn)  │
#   │                    │       (DL path, unchanged from prior mode)│
#   │                    ▼                                            │
#   │                   ext-dn forwarder ──► UPF ─GTP-U─► gNB ────────┼──► rfsim AIR (PDSCH)
#   │                                                                  │
#   │                                              ┌───────────────────┘
#   │                                              ▼
#   │  ue_teleop_bridge.py     ◄── DL post-veth ── robot_netns_proxy  ◄── oaitun_ue1
#   │   listen 192.168.244.1:16005                  (DL leg: TUN→veth)     (in robot-ns)
#   │   /cmd_vel suppressed                         (UL leg: veth→TUN)
#   │   pose/odom UL → 192.168.244.2:17600 ─────────────►              │
#   │   isaac/viewport_to_udp.py ─── ISAAC_UPLINK_HOST=192.168.244.2 ─►│
#   │                                                                  │
#   │  broker stack  → /network/cmd_vel ─► nova_carter_bridge ─► /cmd_vel
#   │                                                                  │
#   │  Isaac Sim  ──► /chassis/odom ─► plotter_publisher.py ─► /plotter
#   │                                                                  │
#   └──────────────────────────────────────────────────────────────────┘
#                                              ▲
#                                              │ UL via PUSCH (the real fix!)
#                                              │ proxy → oaitun_ue1 → gNB → UPF
#                                              │ → operator host (HOST_BRIDGE_IP:17500)
#                                              │
#                                       same machine — but now packets
#                                       genuinely cross the radio in UL
#
# Prereqs (you start these, in order):
#   1. 5G core   (docker compose -f /path/to/oai-cn5g/docker-compose.yaml up -d)
#   2. OAI gNB   (./nr-softmodem ... --rfsim ...)
#   3. OAI UE    (./nr-uesoftmodem ... --rfsim ...)  — creates oaitun_ue1
#   4. Isaac Sim (Nova_Carter_ROS scene playing)
#
# This script then runs:
#   - scripts/setup_5g_data_plane.sh up         (ext-dn DL forwarder)
#   - scripts/setup_robot_netns.sh up           (creates robot-ns + veth + moves oaitun_ue1)
#   - oai_bridge/robot_netns_proxy.py           (inside robot-ns)
#   - teleop_app/app.py                         (default ns; DSCP_EF on cmds)
#   - oai_bridge/ue_teleop_bridge.py            (default ns; listens on veth)
#   - network_aware_broker stack                (post-radio shaping)
#   - isaac/nova_carter_bridge.py               (/network/cmd_vel → /cmd_vel)
#   - isaac/plotter_publisher.py                (/chassis/odom → /plotter)
#
# In Isaac's Script Editor, you still run viewport_to_udp.py — but now with
# ISAAC_UPLINK_HOST=192.168.244.2 in the environment that launches Isaac.

set -euo pipefail
REPO_ROOT="$(cd "$(dirname "$0")/.." && pwd)"
ROS_SETUP="${ROS_SETUP:-/opt/ros/humble/setup.bash}"
LOG_DIR="${REPO_ROOT}/.run_logs"
mkdir -p "${LOG_DIR}"

# ---- Config ---------------------------------------------------------------
# Operator side
OPERATOR_PORT="${OPERATOR_PORT:-17500}"
TELEOP_PORT="${TELEOP_PORT:-8080}"
PLATFORM="${PLATFORM:-nova_carter}"

# Downlink ingress (ext-dn container IP, unchanged)
EXT_DN_IP="${EXT_DN_IP:-192.168.70.135}"
EXT_DN_PORT="${EXT_DN_PORT:-16000}"

# robot-ns + veth
ROBOT_NS="${ROBOT_NS:-robot-ns}"
VETH_HOST_IP="${VETH_HOST_IP:-192.168.244.1}"
VETH_NS_IP="${VETH_NS_IP:-192.168.244.2}"
DL_LISTEN_PORT="${DL_LISTEN_PORT:-16000}"      # UE TUN side (inside robot-ns)
DL_FORWARD_PORT="${DL_FORWARD_PORT:-16005}"    # bridge listens here in default ns
UL_LISTEN_PORT="${UL_LISTEN_PORT:-17600}"      # proxy UL ingress (veth-ns side)

# Broker
BROKER_UDP_PORT="${BROKER_UDP_PORT:-16100}"

# Host bridge IP — the operator's address as seen from inside the 5G data
# network. Default is the oai-cn5g bridge gateway. The dashboard's uplink
# listener (bound to 0.0.0.0:OPERATOR_PORT) picks up traffic arriving from
# the UPF on this address.
HOST_BRIDGE_IP="${HOST_BRIDGE_IP:-192.168.70.1}"

if [[ ${EUID:-$(id -u)} -ne 0 ]]; then
    echo "this launcher needs root for netns and ext-dn forwarder setup." >&2
    echo "Try: sudo -E bash $0" >&2
    exit 1
fi

if [[ ! -f "${ROS_SETUP}" ]]; then
    echo "ROS 2 setup not found at ${ROS_SETUP}" >&2
    exit 1
fi
if [[ ! -f "${REPO_ROOT}/ros2_ws/install/setup.bash" ]]; then
    echo "ros2_ws not built. Run: bash scripts/build_ros2_ws.sh" >&2
    exit 1
fi

# ---- 1. Bring up ext-dn forwarders (DL leg) -------------------------------
echo "[urllc] ext-dn forwarders"
bash "${REPO_ROOT}/scripts/setup_5g_data_plane.sh" up

# ---- 2. Bring up the robot netns ------------------------------------------
echo
echo "[urllc] robot-ns"
bash "${REPO_ROOT}/scripts/setup_robot_netns.sh" up

# Resolve the UE TUN IP inside robot-ns now so the proxy can bind it.
UE_TUN_IP=$(ip netns exec "${ROBOT_NS}" \
    ip -4 -o addr show dev oaitun_ue1 2>/dev/null \
    | awk '{print $4}' | cut -d/ -f1 | head -n1 || true)
if [[ -z "${UE_TUN_IP}" ]]; then
    echo "[urllc][err] could not read oaitun_ue1 IP inside ${ROBOT_NS}" >&2
    exit 1
fi
echo "[urllc] UE TUN IP (inside ${ROBOT_NS}) = ${UE_TUN_IP}"

# ---- 3. Source ROS / workspace --------------------------------------------
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

# Run a process and stash its PID. Args:  $1=label, $2..=command
_pids=()
_launch() {
    local label="$1"; shift
    "$@" >"${LOG_DIR}/${label}.log" 2>&1 &
    local pid=$!
    _pids+=("${pid}")
    echo "[urllc] ${label} PID=${pid} (log: ${LOG_DIR}/${label}.log)"
}

# ---- 4. robot_netns_proxy inside robot-ns ---------------------------------
echo
echo "[urllc] robot_netns_proxy in ${ROBOT_NS}"
ip netns exec "${ROBOT_NS}" \
    "${PY}" "${REPO_ROOT}/oai_bridge/robot_netns_proxy.py" \
        --ue-tun-ip "${UE_TUN_IP}" \
        --veth-ns-ip "${VETH_NS_IP}" \
        --veth-host-ip "${VETH_HOST_IP}" \
        --dl-listen-port "${DL_LISTEN_PORT}" \
        --dl-forward-port "${DL_FORWARD_PORT}" \
        --ul-listen-port "${UL_LISTEN_PORT}" \
        --operator-host "${HOST_BRIDGE_IP}" \
        --operator-port "${OPERATOR_PORT}" \
        >"${LOG_DIR}/robot_netns_proxy.log" 2>&1 &
_pids+=("$!")
echo "[urllc] robot_netns_proxy PID=$! (log: ${LOG_DIR}/robot_netns_proxy.log)"

# ---- 5. Dashboard (operator side, DSCP_EF on commands) --------------------
echo
echo "[urllc] dashboard"
_launch teleop_app \
    "${PY}" "${REPO_ROOT}/teleop_app/app.py" \
        --platform "${PLATFORM}" \
        --mode oai-real \
        --port "${TELEOP_PORT}" \
        --udp-host "${EXT_DN_IP}" --udp-port "${EXT_DN_PORT}" \
        --operator-udp-host 0.0.0.0 --operator-udp-port "${OPERATOR_PORT}"

# ---- 6. UE bridge (listens on default-ns veth for DL post-veth) -----------
# In URLLC mode the bridge:
#   - listens on VETH_HOST_IP:DL_FORWARD_PORT for DL packets the proxy
#     relayed across the veth (these arrived via real PDSCH)
#   - forwards them to the broker on 127.0.0.1:BROKER_UDP_PORT
#   - sends pose/odom UL to the proxy at VETH_NS_IP:UL_LISTEN_PORT, which
#     ships them via PUSCH to the operator
#   - does NOT publish /cmd_vel (broker chain owns that)
#   - does NOT ingest camera over UDP (camera goes Isaac→proxy directly)
echo
echo "[urllc] ue_teleop_bridge"
_launch ue_bridge \
    "${PY}" "${REPO_ROOT}/oai_bridge/ue_teleop_bridge.py" \
        --listen-host "${VETH_HOST_IP}" --listen-port "${DL_FORWARD_PORT}" \
        --operator-host "${VETH_NS_IP}" --operator-port "${UL_LISTEN_PORT}" \
        --forward-udp-host 127.0.0.1 --forward-udp-port "${BROKER_UDP_PORT}" \
        --no-publish-cmd-vel \
        --camera-uplink-listen-port 0

# ---- 7. Broker stack ------------------------------------------------------
echo
echo "[urllc] broker stack (BROKER_UDP_PORT=${BROKER_UDP_PORT})"
_launch broker \
    env BROKER_UDP_PORT="${BROKER_UDP_PORT}" \
        ros2 launch network_aware_broker network_aware_stack.launch.py

# ---- 8. nova_carter_bridge -------------------------------------------------
echo
echo "[urllc] nova_carter_bridge"
_launch nova_carter_bridge \
    "${PY}" "${REPO_ROOT}/isaac/nova_carter_bridge.py" \
        --cmd-in-topic /network/cmd_vel \
        --cmd-out-topic /cmd_vel \
        --publish-rate-hz 30 \
        --watchdog-ms 500 \
        --step-rate-hz 50

# ---- 9. plotter_publisher --------------------------------------------------
echo
echo "[urllc] plotter_publisher (/chassis/odom -> /plotter)"
_launch plotter \
    "${PY}" "${REPO_ROOT}/isaac/plotter_publisher.py" \
        --odom-topic /chassis/odom \
        --out-topic /plotter \
        --publish-rate-hz 30

echo
echo "[urllc] up. open http://localhost:${TELEOP_PORT}"
echo "[urllc] In Isaac's Script Editor open isaac/viewport_to_udp.py and Ctrl-Enter."
echo "[urllc] (Make sure ISAAC_UPLINK_HOST=${VETH_NS_IP} is in the env that launched Isaac,"
echo "[urllc]  or edit the constant in that file before running.)"
echo "[urllc] Ctrl-C to stop everything started by this launcher."

cleanup() {
    echo
    echo "[urllc] shutting down"
    for pid in "${_pids[@]}"; do
        kill "${pid}" 2>/dev/null || true
    done
    sleep 0.5
    for pid in "${_pids[@]}"; do
        kill -9 "${pid}" 2>/dev/null || true
    done
    bash "${REPO_ROOT}/scripts/setup_robot_netns.sh" down || true
    bash "${REPO_ROOT}/scripts/setup_5g_data_plane.sh" down || true
}
trap cleanup EXIT INT TERM
wait
