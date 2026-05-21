#!/usr/bin/env bash
# Build a "robot" network namespace that owns oaitun_ue1, so any process
# inside it that sends to the operator network ACTUALLY rides the 5G
# uplink (PUSCH) instead of being short-circuited through Linux loopback.
#
# Topology after `up`:
#
#   default ns                            robot-ns
#   ──────────                            ────────
#                                         oaitun_ue1   ← moved here, real UE TUN
#                                              │
#                                              ├── route to OPERATOR_NET via this dev
#                                              │   (this is the leg that crosses
#                                              │    the 5G radio in UL direction)
#                                              │
#   ve-rob-host ◄────── veth pair ────► ve-rob-ns
#   192.168.244.1/24                     192.168.244.2/24
#       │                                     │
#       └─ ue_teleop_bridge listens here       └─ robot_netns_proxy runs here:
#          (DL post-veth: 16005)                  - DL leg:  oaitun_ue1:16000  → 192.168.244.1:16005
#          (also: Isaac, dashboard,                - UL leg:  0.0.0.0:17600    → OPERATOR_HOST_IP:17500
#           broker, nova_carter,                                              (via oaitun_ue1, src=UE IP)
#           plotter all here)
#
# This makes the UL path traverse:
#
#   viewport / pose sender (default ns)
#     → 192.168.244.2:17600  (proxy in robot-ns)
#     → oaitun_ue1 OUTBOUND  (OAI UE process picks it up via TUN read)
#     → PUSCH → gNB → UPF → N6 → host bridge → dashboard
#
# Prerequisites (you start these yourself, in order):
#   1. 5G core (oai-cn5g compose)
#   2. OAI gNB (--rfsim)
#   3. OAI UE  (--rfsim)  — this is what *creates* oaitun_ue1
#   4. *Then* run this script:  sudo bash scripts/setup_robot_netns.sh up
#
# Tear down with:  sudo bash scripts/setup_robot_netns.sh down
#
# Env overrides:
#   ROBOT_NS         netns name                       (default robot-ns)
#   VETH_HOST        host-side veth name              (default ve-rob-host)
#   VETH_NS          ns-side veth name                (default ve-rob-ns)
#   VETH_HOST_IP     host-side veth IP/mask           (default 192.168.244.1/24)
#   VETH_NS_IP       ns-side veth IP/mask             (default 192.168.244.2/24)
#   UE_TUN           UE TUN device name               (default oaitun_ue1)
#   OPERATOR_NET     network the UE reaches via 5G    (default 192.168.70.0/24)

set -u

ROBOT_NS="${ROBOT_NS:-robot-ns}"
VETH_HOST="${VETH_HOST:-ve-rob-host}"
VETH_NS="${VETH_NS:-ve-rob-ns}"
VETH_HOST_IP="${VETH_HOST_IP:-192.168.244.1/24}"
VETH_NS_IP="${VETH_NS_IP:-192.168.244.2/24}"
UE_TUN="${UE_TUN:-oaitun_ue1}"
OPERATOR_NET="${OPERATOR_NET:-192.168.70.0/24}"

log()  { printf '[robot-ns] %s\n' "$*"; }
warn() { printf '[robot-ns][WARN] %s\n' "$*" >&2; }
die()  { printf '[robot-ns][ERR] %s\n' "$*" >&2; exit 1; }

if [[ ${EUID:-$(id -u)} -ne 0 ]]; then
    die "must run as root (sudo). Try: sudo bash $0 $*"
fi

cmd="${1:-status}"

case "$cmd" in
up)
    if ! ip link show "${UE_TUN}" >/dev/null 2>&1; then
        die "${UE_TUN} not found in default ns. Did the OAI UE attach? Bring it up first."
    fi

    # Read the UE IP that OAI assigned to oaitun_ue1 BEFORE we move the device
    # — once it's in robot-ns, this lookup from default ns won't see it.
    UE_IP_FULL=$(ip -4 -o addr show dev "${UE_TUN}" | awk '{print $4}' | head -n1 || true)
    if [[ -z "${UE_IP_FULL}" ]]; then
        die "${UE_TUN} has no IPv4 address yet. Wait for UE attach to complete."
    fi
    UE_IP="${UE_IP_FULL%/*}"
    UE_MASK="${UE_IP_FULL#*/}"
    log "UE TUN ${UE_TUN} has ${UE_IP_FULL}; will preserve through netns move"

    if ip netns list | awk '{print $1}' | grep -qx "${ROBOT_NS}"; then
        log "netns ${ROBOT_NS} already exists; tearing down first"
        "$0" down >/dev/null 2>&1 || true
        # Allow time for cleanup
        sleep 0.2
    fi

    log "creating netns ${ROBOT_NS}"
    ip netns add "${ROBOT_NS}" || die "failed to create netns"

    log "creating veth pair ${VETH_HOST} <-> ${VETH_NS}"
    ip link add "${VETH_HOST}" type veth peer name "${VETH_NS}" \
        || die "failed to create veth"
    ip link set "${VETH_NS}" netns "${ROBOT_NS}"
    ip addr add "${VETH_HOST_IP}" dev "${VETH_HOST}"
    ip link set "${VETH_HOST}" up
    ip netns exec "${ROBOT_NS}" ip addr add "${VETH_NS_IP}" dev "${VETH_NS}"
    ip netns exec "${ROBOT_NS}" ip link set "${VETH_NS}" up
    ip netns exec "${ROBOT_NS}" ip link set lo up

    log "moving ${UE_TUN} into ${ROBOT_NS}"
    ip link set "${UE_TUN}" netns "${ROBOT_NS}" \
        || die "failed to move ${UE_TUN} into ${ROBOT_NS}"

    # Re-apply IP and bring it up inside the ns. The OAI UE process keeps
    # its TUN FD open across the move, so packet I/O continues to work.
    ip netns exec "${ROBOT_NS}" ip addr add "${UE_IP}/${UE_MASK}" dev "${UE_TUN}" \
        || warn "could not re-add ${UE_IP}/${UE_MASK} (already present?)"
    ip netns exec "${ROBOT_NS}" ip link set "${UE_TUN}" up
    ip netns exec "${ROBOT_NS}" ip link set "${UE_TUN}" mtu 1400 || true

    # Route OPERATOR_NET via the UE TUN. Anything we send to the docker
    # bridge from inside robot-ns now egresses oaitun_ue1, which means
    # the OAI UE process reads it from the TUN FD and ships it on PUSCH.
    log "routing ${OPERATOR_NET} via ${UE_TUN} inside ${ROBOT_NS}"
    ip netns exec "${ROBOT_NS}" ip route add "${OPERATOR_NET}" dev "${UE_TUN}" \
        || warn "route add ${OPERATOR_NET} dev ${UE_TUN} failed (already present?)"

    # Also make ${UE_TUN} the default route inside the ns, so packets to
    # arbitrary destinations attempted from robot-ns go up the radio.
    ip netns exec "${ROBOT_NS}" ip route add default dev "${UE_TUN}" \
        || true

    log "ready."
    log "  default ns veth IP : ${VETH_HOST_IP%/*}   (bridge listens here, DL post-veth)"
    log "  robot-ns  veth IP  : ${VETH_NS_IP%/*}    (proxy listens here, UL ingress)"
    log "  UE TUN IP in ns    : ${UE_IP}            (proxy DL ingress, UL egress)"
    ;;

down)
    if ip netns list | awk '{print $1}' | grep -qx "${ROBOT_NS}"; then
        # Move the TUN back to default ns if it's still here, so OAI UE
        # can continue to talk on it after teardown.
        if ip netns exec "${ROBOT_NS}" ip link show "${UE_TUN}" >/dev/null 2>&1; then
            log "moving ${UE_TUN} back to default ns"
            ip netns exec "${ROBOT_NS}" ip link set "${UE_TUN}" netns 1 \
                || warn "failed to move ${UE_TUN} back"
        fi
        log "deleting netns ${ROBOT_NS}"
        ip netns del "${ROBOT_NS}" || true
    fi
    if ip link show "${VETH_HOST}" >/dev/null 2>&1; then
        ip link del "${VETH_HOST}" || true
    fi
    log "torn down."
    ;;

status)
    if ip netns list | awk '{print $1}' | grep -qx "${ROBOT_NS}"; then
        log "netns ${ROBOT_NS} present"
        ip netns exec "${ROBOT_NS}" ip -4 -o addr show
        echo
        ip netns exec "${ROBOT_NS}" ip route
    else
        log "netns ${ROBOT_NS} not present"
    fi
    ;;

exec)
    shift
    if ! ip netns list | awk '{print $1}' | grep -qx "${ROBOT_NS}"; then
        die "netns ${ROBOT_NS} doesn't exist; run '$0 up' first"
    fi
    exec ip netns exec "${ROBOT_NS}" "$@"
    ;;

*)
    die "usage: sudo bash $0 {up|down|status|exec <cmd...>}"
    ;;
esac
