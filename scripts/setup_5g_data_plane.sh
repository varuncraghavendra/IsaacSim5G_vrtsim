#!/usr/bin/env bash
# Force IsaacSim5G downlink traffic to actually traverse the OAI rfsim
# radio instead of short-circuiting through Linux loopback.
#
# Approach:
#   The dashboard sends UDP to the oai-ext-dn container at
#   192.168.70.135:16000 instead of directly to 10.0.0.4. A tiny Python
#   UDP forwarder running inside ext-dn relays each packet to the UE's
#   pool address (10.0.0.4:16000). From inside ext-dn that destination
#   resolves via its static route "10.0.0.0/16 via 192.168.70.134"
#   (the oai-upf N6 interface), so:
#
#       dashboard (host)
#          │ UDP, dst=192.168.70.135:16000
#          ▼
#       docker oai-cn5g bridge
#          │
#          ▼
#       oai-ext-dn  (Python UDP fwd)
#          │ UDP, dst=10.0.0.4:16000
#          ▼
#       oai-upf  (GTP-U encaps, N3 → gNB)
#          │
#          ▼
#       gNB → rfsim air → UE → oaitun_ue1
#
# The "ext-dn → 10.0.0.4" segment is the leg that crosses the radio.
#
# Usage:
#   sudo bash scripts/setup_5g_data_plane.sh up
#   sudo bash scripts/setup_5g_data_plane.sh down
#   sudo bash scripts/setup_5g_data_plane.sh status
#   sudo bash scripts/setup_5g_data_plane.sh probe
#
# Trace what's happening:
#   sudo bash -x scripts/setup_5g_data_plane.sh up
#
# Environment overrides:
#   FWD_LISTEN_PORT   Forwarder listen port in ext-dn       (default 16000)
#   FWD_UE_IP         UE pool address forwarded to          (default 10.0.0.4)
#   FWD_UE_PORT       UE-side port                          (default 16000)
#   FWD_CONTAINER     Container that hosts the forwarder    (default oai-ext-dn)
#   UE_POOL           UE pool (for stale-rule cleanup)      (default 10.0.0.0/24)
#   UPF_GATEWAY       UPF N6 IP (for stale-rule cleanup)    (default 192.168.70.134)

# NOTE: deliberately NOT using `set -e` here — earlier we hit a silent
# failure where one bad command killed the script with no output. We
# log every step so the user can see exactly what happened.

set -u

FWD_LISTEN_PORT="${FWD_LISTEN_PORT:-16000}"
# Auto-detect the UE pool address from oaitun_ue1 if not explicitly set —
# the SMF can hand out 10.0.0.2, 10.0.0.4, etc. depending on which UE
# attaches first and which IMSI is provisioned. Hard-coding 10.0.0.4 silently
# drops every command into a black hole on systems where the UE got a
# different address.
if [[ -z "${FWD_UE_IP:-}" ]]; then
    FWD_UE_IP="$(ip -4 addr show oaitun_ue1 2>/dev/null | awk '/inet /{print $2}' | cut -d/ -f1 | head -n1)"
    if [[ -z "${FWD_UE_IP}" ]]; then
        FWD_UE_IP="10.0.0.4"
        printf '[5g][WARN] could not read oaitun_ue1; falling back to FWD_UE_IP=%s\n' "${FWD_UE_IP}" >&2
    else
        printf '[5g] auto-detected FWD_UE_IP=%s from oaitun_ue1\n' "${FWD_UE_IP}"
    fi
fi
FWD_UE_PORT="${FWD_UE_PORT:-16000}"
FWD_CAM_LISTEN_PORT="${FWD_CAM_LISTEN_PORT:-17600}"
FWD_CAM_UE_PORT="${FWD_CAM_UE_PORT:-17600}"
FWD_CONTAINER="${FWD_CONTAINER:-oai-ext-dn}"
UE_POOL="${UE_POOL:-10.0.0.0/24}"
UPF_GATEWAY="${UPF_GATEWAY:-192.168.70.134}"

log()  { printf '[5g] %s\n' "$*"; }
warn() { printf '[5g][WARN] %s\n' "$*" >&2; }
die()  { printf '[5g][ERR] %s\n' "$*" >&2; exit 1; }

if [[ ${EUID:-$(id -u)} -ne 0 ]]; then
    die "must be run as root (sudo). Try: sudo bash $0 $*"
fi

cleanup_stale_iprule() {
    log "cleaning up any stale ip rule / route from previous attempts"
    ip rule del to "${UE_POOL}" lookup 100 pref 50  2>/dev/null || true
    ip route del "${UE_POOL}" via "${UPF_GATEWAY}" table 100 2>/dev/null || true
}

container_ip() {
    docker inspect -f '{{range .NetworkSettings.Networks}}{{.IPAddress}}{{end}}' \
        "${FWD_CONTAINER}" 2>/dev/null
}

write_forwarder_script() {
    local tmpfile
    tmpfile=$(mktemp /tmp/udp_5g_forwarder.XXXXXX.py)
    cat >"${tmpfile}" <<'PYEOF'
import os, socket, time, sys

LISTEN_PORT = int(os.environ["FWD_LISTEN_PORT"])
UE_IP       = os.environ["FWD_UE_IP"]
UE_PORT     = int(os.environ["FWD_UE_PORT"])

rx = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
rx.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
rx.bind(("0.0.0.0", LISTEN_PORT))
tx = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)

print(f"[fwd] listening 0.0.0.0:{LISTEN_PORT}, forwarding -> {UE_IP}:{UE_PORT}",
      flush=True)

count = 0
last_log = time.monotonic()
while True:
    try:
        data, addr = rx.recvfrom(65536)
    except OSError as e:
        print(f"[fwd] recv error: {e}", flush=True)
        time.sleep(0.5)
        continue
    try:
        tx.sendto(data, (UE_IP, UE_PORT))
        count += 1
    except OSError as e:
        print(f"[fwd] send error: {e}", flush=True)
    now = time.monotonic()
    if now - last_log >= 5.0:
        print(f"[fwd] forwarded {count} packets (last src {addr})", flush=True)
        last_log = now
PYEOF
    echo "${tmpfile}"
}

write_probe_script() {
    local tmpfile
    tmpfile=$(mktemp /tmp/udp_probe_fwd.XXXXXX.py)
    cat >"${tmpfile}" <<'PYEOF'
import socket, time, os, sys
UE_IP = os.environ.get("PROBE_UE_IP", "10.0.0.4")
rx = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
rx.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
rx.bind(("0.0.0.0", 19998))
tx = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
rx.settimeout(6)
end = time.monotonic() + 6
n = 0
while time.monotonic() < end:
    try:
        data, _ = rx.recvfrom(65536)
    except socket.timeout:
        break
    tx.sendto(data, (UE_IP, 19999))
    n += 1
print(f"[probe-fwd] relayed {n} packets to {UE_IP}:19999", flush=True)
PYEOF
    echo "${tmpfile}"
}

cmd="${1:-status}"
log "cmd=${cmd}  FWD_CONTAINER=${FWD_CONTAINER}  FWD_LISTEN_PORT=${FWD_LISTEN_PORT}  FWD_UE_IP=${FWD_UE_IP}  FWD_UE_PORT=${FWD_UE_PORT}"

case "$cmd" in
up)
    cleanup_stale_iprule

    if ! docker ps --format '{{.Names}}' | grep -qx "${FWD_CONTAINER}"; then
        die "container ${FWD_CONTAINER} is not running"
    fi
    EXT_IP=$(container_ip)
    [[ -z "${EXT_IP}" ]] && die "could not determine IP for ${FWD_CONTAINER}"
    log "ext-dn container ${FWD_CONTAINER} is at ${EXT_IP}"

    log "stopping any previous forwarders in ${FWD_CONTAINER}"
    docker exec "${FWD_CONTAINER}" sh -c 'pkill -f udp_5g_forwarder.py 2>/dev/null; true' \
        || warn "pkill returned non-zero (may be benign)"

    log "writing forwarder script onto host then docker-cp into container"
    FWDFILE=$(write_forwarder_script)
    docker cp "${FWDFILE}" "${FWD_CONTAINER}:/tmp/udp_5g_forwarder.py" \
        || die "docker cp failed"
    rm -f "${FWDFILE}"

    log "verifying file landed inside container"
    if ! docker exec "${FWD_CONTAINER}" test -s /tmp/udp_5g_forwarder.py; then
        die "/tmp/udp_5g_forwarder.py is missing or empty inside ${FWD_CONTAINER}"
    fi
    docker exec "${FWD_CONTAINER}" wc -l /tmp/udp_5g_forwarder.py

    log "starting DOWNLINK forwarder (port ${FWD_LISTEN_PORT} -> ${FWD_UE_IP}:${FWD_UE_PORT})"
    docker exec -d \
        -e "FWD_LISTEN_PORT=${FWD_LISTEN_PORT}" \
        -e "FWD_UE_IP=${FWD_UE_IP}" \
        -e "FWD_UE_PORT=${FWD_UE_PORT}" \
        "${FWD_CONTAINER}" \
        sh -c "python3 /tmp/udp_5g_forwarder.py > /tmp/udp_5g_forwarder.log 2>&1"

    log "starting CAMERA forwarder (port ${FWD_CAM_LISTEN_PORT} -> ${FWD_UE_IP}:${FWD_CAM_UE_PORT})"
    docker exec -d \
        -e "FWD_LISTEN_PORT=${FWD_CAM_LISTEN_PORT}" \
        -e "FWD_UE_IP=${FWD_UE_IP}" \
        -e "FWD_UE_PORT=${FWD_CAM_UE_PORT}" \
        "${FWD_CONTAINER}" \
        sh -c "python3 /tmp/udp_5g_forwarder.py > /tmp/udp_5g_forwarder_cam.log 2>&1"

    sleep 0.7

    log "forwarder processes:"
    docker exec "${FWD_CONTAINER}" sh -c 'ps -ef | grep udp_5g_forwarder.py | grep -v grep' \
        || warn "forwarders NOT running. Tail of logs:"
    docker exec "${FWD_CONTAINER}" sh -c 'tail -n 5 /tmp/udp_5g_forwarder.log 2>/dev/null'      || true
    docker exec "${FWD_CONTAINER}" sh -c 'tail -n 5 /tmp/udp_5g_forwarder_cam.log 2>/dev/null'  || true

    log "done."
    log "  DOWNLINK (commands): dashboard -> ${EXT_IP}:${FWD_LISTEN_PORT} -> radio -> UE :${FWD_UE_PORT}"
    log "  UPLINK   (camera):   Isaac     -> ${EXT_IP}:${FWD_CAM_LISTEN_PORT} -> radio -> UE :${FWD_CAM_UE_PORT}"
    ;;

down)
    cleanup_stale_iprule
    if docker ps --format '{{.Names}}' | grep -qx "${FWD_CONTAINER}"; then
        docker exec "${FWD_CONTAINER}" sh -c 'pkill -f udp_5g_forwarder.py; true' || true
        docker exec "${FWD_CONTAINER}" sh -c 'pkill -f udp_probe_fwd.py; true' || true
        log "forwarder stopped"
    else
        warn "container ${FWD_CONTAINER} not running, nothing to stop"
    fi
    ;;

status)
    log "stale ip rule / route (should be empty after 'down'):"
    ip rule show | grep -E "lookup 100|${UE_POOL}" || echo "    (none)"
    ip route show table 100 2>/dev/null || echo "    (table 100 empty)"
    echo
    if docker ps --format '{{.Names}}' | grep -qx "${FWD_CONTAINER}"; then
        log "forwarder processes inside ${FWD_CONTAINER}:"
        docker exec "${FWD_CONTAINER}" sh -c 'ps -ef | grep udp_5g_forwarder.py | grep -v grep' \
            || echo "    (no forwarder running)"
        echo
        log "tail of /tmp/udp_5g_forwarder.log:"
        docker exec "${FWD_CONTAINER}" sh -c 'tail -n 10 /tmp/udp_5g_forwarder.log 2>/dev/null' \
            || echo "    (no log)"
        echo
        log "container IP: $(container_ip)"
    else
        warn "container ${FWD_CONTAINER} not running"
    fi
    ;;

probe)
    if ! docker ps --format '{{.Names}}' | grep -qx "${FWD_CONTAINER}"; then
        die "container ${FWD_CONTAINER} is not running"
    fi
    EXT_IP=$(container_ip)
    [[ -z "${EXT_IP}" ]] && die "could not determine IP for ${FWD_CONTAINER}"

    log "writing probe forwarder script into ${FWD_CONTAINER} (port 19998 -> 10.0.0.4:19999)"
    PROBEFILE=$(write_probe_script)
    docker cp "${PROBEFILE}" "${FWD_CONTAINER}:/tmp/udp_probe_fwd.py" \
        || die "docker cp probe failed"
    rm -f "${PROBEFILE}"

    log "starting probe forwarder (detached, ~6s lifetime)"
    docker exec "${FWD_CONTAINER}" sh -c 'pkill -f udp_probe_fwd.py; true' || true
    docker exec -d -e "PROBE_UE_IP=${FWD_UE_IP}" "${FWD_CONTAINER}" \
        sh -c "python3 /tmp/udp_probe_fwd.py > /tmp/udp_probe_fwd.log 2>&1"

    log "starting tcpdump on oaitun_ue1 (8s)"
    timeout 8 tcpdump -i oaitun_ue1 -n -c 16 \
        "udp and dst host ${FWD_UE_IP} and dst port 19999" \
        > /tmp/oaitun_probe.txt 2>&1 &
    DPID=$!
    sleep 1

    log "sending 10 UDP packets host -> ${EXT_IP}:19998 (probe ingress)"
    python3 -c "
import socket, time
s = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
for i in range(10):
    s.sendto(b'probe-%d' % i, ('${EXT_IP}', 19998))
    time.sleep(0.1)
print('[probe] sent 10 packets to ${EXT_IP}:19998', flush=True)
"
    wait $DPID 2>/dev/null || true

    echo
    log "--- tcpdump on oaitun_ue1 ---"
    cat /tmp/oaitun_probe.txt
    echo
    log "--- probe forwarder log inside ${FWD_CONTAINER} ---"
    docker exec "${FWD_CONTAINER}" sh -c 'cat /tmp/udp_probe_fwd.log 2>/dev/null' || true
    echo
    log "verdict: if you see ~10 lines of '> 10.0.0.4.19999' under tcpdump,"
    log "         the downlink genuinely crosses the rfsim radio."
    ;;
*)
    die "usage: sudo bash $0 {up|down|status|probe}"
    ;;
esac
