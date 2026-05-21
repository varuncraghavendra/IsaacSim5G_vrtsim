#!/usr/bin/env bash
# End-to-end launcher for the drone teleop pipeline, OAI-gated.
#
# Brings up (in order):
#   - ext-dn UDP forwarders            (setup_5g_data_plane.sh up)
#   - robot-ns + veth + oaitun_ue1     (setup_robot_netns.sh up)
#   - robot_netns_proxy.py             (inside robot-ns)
#   - drone_ue_bridge.py               (default ns; speaks MAVLink to PX4)
#   - drone_app.py                     (operator dashboard, :8080)
#   - viewport_screen_capture.py       (operator host; FPV screen capture)
#
# Refuses to launch when:
#   - oaitun_ue1 is missing (no UE attach => no DL crosses the radio)
#   - the dashboard requirements aren't installed
#
# That refusal IS the "no OAI in the loop => drone is silent" guarantee.
# This launcher does not provide any fallback that talks MAVLink directly.
#
# Prereqs you start before invoking this script:
#   1. 5G core stack    (docker compose -f /path/to/oai-cn5g up -d)
#   2. OAI gNB          (./nr-softmodem --rfsim ...   or vrtsim equivalent)
#   3. OAI UE           (./nr-uesoftmodem --rfsim ...)  — creates oaitun_ue1
#   4. Isaac Sim        (Pegasus Drone Sim Extension + city scene playing —
#                        PX4 SITL is auto-launched by Pegasus, MAVLink on 14540)
#
# Then:
#   sudo -E bash scripts/run_drone_oai.sh
#
# Open http://localhost:8080  (or whatever TELEOP_PORT is set to).
#
# In Isaac, you only need to ADD a second viewport — no scripts, no
# extension changes. The host-side screen capture handles the rest.

set -euo pipefail
REPO_ROOT="$(cd "$(dirname "$0")/.." && pwd)"
LOG_DIR="${REPO_ROOT}/.run_logs"
mkdir -p "${LOG_DIR}"

# ---- Config ---------------------------------------------------------------
# Operator side
TELEOP_PORT="${TELEOP_PORT:-8080}"
OPERATOR_UDP_PORT="${OPERATOR_UDP_PORT:-17500}"

# 5G data plane ingress (ext-dn container IP in oai-cn5g)
EXT_DN_IP="${EXT_DN_IP:-192.168.70.135}"
EXT_DN_PORT="${EXT_DN_PORT:-16000}"

# Camera UL ingress on ext-dn (relays into the UE side)
FWD_CAM_LISTEN_PORT="${FWD_CAM_LISTEN_PORT:-17600}"
FWD_CAM_UE_PORT="${FWD_CAM_UE_PORT:-17600}"

# robot-ns + veth
ROBOT_NS="${ROBOT_NS:-robot-ns}"
VETH_HOST_IP_PLAIN="192.168.244.1"
VETH_NS_IP_PLAIN="192.168.244.2"
DL_LISTEN_PORT="${DL_LISTEN_PORT:-16000}"       # UE TUN side (inside robot-ns)
DL_FORWARD_PORT="${DL_FORWARD_PORT:-16005}"     # bridge listens here in default ns
UL_LISTEN_PORT="${UL_LISTEN_PORT:-17600}"       # proxy UL ingress (veth-ns side)

# Host bridge IP — operator's address as seen from inside the 5G data network.
HOST_BRIDGE_IP="${HOST_BRIDGE_IP:-192.168.70.1}"

# PX4 / MAVLink
MAVLINK_URL="${MAVLINK_URL:-udpin:127.0.0.1:14540}"

# Channel-bridge republish (set GT_DISABLE=1 to skip)
GT_ZMQ_ADDR="${GT_ZMQ_ADDR:-tcp://127.0.0.1:5556}"

# Screen capture
CAPTURE_WINDOW="${CAPTURE_WINDOW:-Isaac Sim}"
CAPTURE_REGION="${CAPTURE_REGION:-}"
CAPTURE_FPS="${CAPTURE_FPS:-8}"

# ---- sanity ---------------------------------------------------------------
if [[ ${EUID:-$(id -u)} -ne 0 ]]; then
    echo "[drone-oai] must run as root (sudo). Try: sudo -E bash $0" >&2
    exit 1
fi

if [[ ! -d "${REPO_ROOT}/teleop_app/.venv" ]]; then
    echo "[drone-oai] teleop_app/.venv missing — run:" >&2
    echo "    python3 -m venv teleop_app/.venv" >&2
    echo "    teleop_app/.venv/bin/pip install -r teleop_app/requirements.txt" >&2
    exit 1
fi

if ! ip link show oaitun_ue1 >/dev/null 2>&1; then
    if ! ip netns list | awk '{print $1}' | grep -qx "${ROBOT_NS}" \
            || ! ip netns exec "${ROBOT_NS}" ip link show oaitun_ue1 >/dev/null 2>&1; then
        echo "[drone-oai] oaitun_ue1 not found in default ns or in ${ROBOT_NS}." >&2
        echo "           Bring up OAI gNB + UE first (--rfsim or vrtsim mode)." >&2
        echo "           This launcher REFUSES to fall back to localhost MAVLink." >&2
        exit 2
    fi
    echo "[drone-oai] oaitun_ue1 already moved to ${ROBOT_NS} — reusing existing netns"
    NETNS_PREEXISTS=1
else
    NETNS_PREEXISTS=0
fi

PY="${REPO_ROOT}/teleop_app/.venv/bin/python"
if [[ ! -x "${PY}" ]]; then
    echo "[drone-oai] ${PY} not executable" >&2
    exit 1
fi

# Verify pymavlink + mss are installed.
"${PY}" - <<'PY'
import importlib, sys
missing = [m for m in ("pymavlink", "mss", "PIL", "zmq", "fastapi", "uvicorn")
           if importlib.util.find_spec(m) is None]
if missing:
    print(f"[drone-oai] missing python deps: {missing}", file=sys.stderr)
    sys.exit(3)
PY

# ---- 1. ext-dn forwarders -------------------------------------------------
echo "[drone-oai] ext-dn forwarders"
bash "${REPO_ROOT}/scripts/setup_5g_data_plane.sh" up || {
    echo "[drone-oai] WARNING: setup_5g_data_plane.sh failed — DL/UL may not flow." >&2
}

# ---- 2. robot-ns ----------------------------------------------------------
if [[ "${NETNS_PREEXISTS}" -eq 0 ]]; then
    echo "[drone-oai] robot-ns"
    bash "${REPO_ROOT}/scripts/setup_robot_netns.sh" up
fi

UE_TUN_IP=$(ip netns exec "${ROBOT_NS}" \
    ip -4 -o addr show dev oaitun_ue1 2>/dev/null \
    | awk '{print $4}' | cut -d/ -f1 | head -n1 || true)
if [[ -z "${UE_TUN_IP}" ]]; then
    echo "[drone-oai][err] UE TUN IP not found inside ${ROBOT_NS}" >&2
    exit 4
fi
echo "[drone-oai] UE TUN IP (in ${ROBOT_NS}) = ${UE_TUN_IP}"

_pids=()
_launch() {
    local label="$1"; shift
    "$@" >"${LOG_DIR}/${label}.log" 2>&1 &
    local pid=$!
    _pids+=("${pid}")
    echo "[drone-oai] ${label} PID=${pid} (log: ${LOG_DIR}/${label}.log)"
}

# ---- 3. robot_netns_proxy in robot-ns -------------------------------------
echo "[drone-oai] robot_netns_proxy in ${ROBOT_NS}"
ip netns exec "${ROBOT_NS}" \
    "${PY}" "${REPO_ROOT}/oai_bridge/robot_netns_proxy.py" \
        --ue-tun-ip "${UE_TUN_IP}" \
        --veth-ns-ip "${VETH_NS_IP_PLAIN}" \
        --veth-host-ip "${VETH_HOST_IP_PLAIN}" \
        --dl-listen-port "${DL_LISTEN_PORT}" \
        --dl-forward-port "${DL_FORWARD_PORT}" \
        --ul-listen-port "${UL_LISTEN_PORT}" \
        --operator-host "${HOST_BRIDGE_IP}" \
        --operator-port "${OPERATOR_UDP_PORT}" \
        >"${LOG_DIR}/robot_netns_proxy.log" 2>&1 &
_pids+=("$!")
echo "[drone-oai] robot_netns_proxy PID=$! (log: ${LOG_DIR}/robot_netns_proxy.log)"

# ---- 4. drone_ue_bridge ---------------------------------------------------
echo "[drone-oai] drone_ue_bridge"
_launch drone_ue_bridge \
    "${PY}" "${REPO_ROOT}/oai_bridge/drone_ue_bridge.py" \
        --listen-host "${VETH_HOST_IP_PLAIN}" \
        --listen-port "${DL_FORWARD_PORT}" \
        --ul-host "${VETH_NS_IP_PLAIN}" \
        --ul-port "${UL_LISTEN_PORT}" \
        --mavlink-url "${MAVLINK_URL}"

# ---- 5. operator dashboard ------------------------------------------------
echo "[drone-oai] dashboard on http://localhost:${TELEOP_PORT}"
_launch drone_app \
    env TELEOP_PORT="${TELEOP_PORT}" \
        EXT_DN_IP="${EXT_DN_IP}" \
        EXT_DN_PORT="${EXT_DN_PORT}" \
        OPERATOR_UDP_PORT="${OPERATOR_UDP_PORT}" \
        GT_ZMQ_ADDR="${GT_ZMQ_ADDR}" \
    "${PY}" "${REPO_ROOT}/teleop_app/drone_app.py"

# ---- 6. screen capture (operator host, no Isaac scripts) ------------------
# Run as the invoking user so we keep access to their X session. SUDO_USER
# is set by sudo; fall back to "varun" only as a hint if SUDO_USER is empty.
CAPTURE_USER="${SUDO_USER:-${USER:-varun}}"
echo "[drone-oai] viewport_screen_capture (as user ${CAPTURE_USER})"
sudo -u "${CAPTURE_USER}" \
    env DISPLAY="${DISPLAY:-:0}" XAUTHORITY="${XAUTHORITY:-/home/${CAPTURE_USER}/.Xauthority}" \
        CAPTURE_TARGET_HOST="${VETH_NS_IP_PLAIN}" \
        CAPTURE_TARGET_PORT="${UL_LISTEN_PORT}" \
        CAPTURE_WINDOW="${CAPTURE_WINDOW}" \
        CAPTURE_REGION="${CAPTURE_REGION}" \
        CAPTURE_FPS="${CAPTURE_FPS}" \
    "${PY}" "${REPO_ROOT}/isaac/viewport_screen_capture.py" \
        >"${LOG_DIR}/viewport_capture.log" 2>&1 &
_pids+=("$!")
echo "[drone-oai] viewport_screen_capture PID=$! (log: ${LOG_DIR}/viewport_capture.log)"

cleanup() {
    echo
    echo "[drone-oai] shutting down"
    for pid in "${_pids[@]}"; do kill "${pid}" 2>/dev/null || true; done
    sleep 0.5
    for pid in "${_pids[@]}"; do kill -9 "${pid}" 2>/dev/null || true; done
    if [[ "${NETNS_PREEXISTS}" -eq 0 ]]; then
        bash "${REPO_ROOT}/scripts/setup_robot_netns.sh" down || true
    fi
    bash "${REPO_ROOT}/scripts/setup_5g_data_plane.sh" down || true
}
trap cleanup EXIT INT TERM

echo
echo "[drone-oai] up. Open http://localhost:${TELEOP_PORT}"
echo "[drone-oai] In Isaac, add a second viewport pointed at the FPV camera."
echo "[drone-oai]   (No scripts to run. Screen capture grabs the Isaac window.)"
echo "[drone-oai] Ctrl-C to stop everything started by this launcher."
wait
