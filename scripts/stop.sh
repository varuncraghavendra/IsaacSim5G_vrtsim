#!/usr/bin/env bash
# Stop any IsaacSim5G processes left running by the demo launchers.
# Also tears down robot-ns and ext-dn forwarders if either is up.

set -u
REPO_ROOT="$(cd "$(dirname "$0")/.." && pwd)"

echo "stopping IsaacSim5G processes…"
for pat in \
    "ros2 launch network_aware_broker" \
    "network_aware_broker/lib/network_aware_broker" \
    "nova_carter_bridge.py" \
    "dynamic_obstacle_bridge.py" \
    "plotter_publisher.py" \
    "teleop_app/app.py" \
    "ue_teleop_bridge.py" \
    "robot_netns_proxy.py"
do
    pkill -f "${pat}" 2>/dev/null || true
done
sleep 1
remaining=$(ps aux | grep -E "(nova_carter_bridge|plotter_publisher|teleop_app/app|network_aware_broker|broker_main|ue_ingress|metrics_ingress|rtf_clock|robot_netns_proxy)" | grep -v grep || true)
if [[ -n "${remaining}" ]]; then
    echo "still running, escalating to SIGKILL:"
    echo "${remaining}"
    pkill -9 -f "ros2 launch network_aware_broker" 2>/dev/null || true
    pkill -9 -f "network_aware_broker/lib/network_aware_broker" 2>/dev/null || true
    pkill -9 -f "nova_carter_bridge.py" 2>/dev/null || true
    pkill -9 -f "plotter_publisher.py" 2>/dev/null || true
    pkill -9 -f "teleop_app/app.py" 2>/dev/null || true
    pkill -9 -f "ue_teleop_bridge.py" 2>/dev/null || true
    pkill -9 -f "robot_netns_proxy.py" 2>/dev/null || true
fi

# Tear down robot-ns / ext-dn forwarders if they're up. These need root —
# we attempt with sudo only if not already root so a non-root stop.sh still
# kills user processes.
if ip netns list 2>/dev/null | awk '{print $1}' | grep -qx "robot-ns"; then
    echo "tearing down robot-ns…"
    if [[ ${EUID:-$(id -u)} -eq 0 ]]; then
        bash "${REPO_ROOT}/scripts/setup_robot_netns.sh" down || true
    else
        sudo -n bash "${REPO_ROOT}/scripts/setup_robot_netns.sh" down 2>/dev/null \
            || echo "  (skipped, need sudo: 'sudo bash scripts/setup_robot_netns.sh down')"
    fi
fi
if docker ps --format '{{.Names}}' 2>/dev/null | grep -qx "oai-ext-dn"; then
    if docker exec oai-ext-dn pgrep -f udp_5g_forwarder.py >/dev/null 2>&1; then
        echo "stopping ext-dn forwarders…"
        if [[ ${EUID:-$(id -u)} -eq 0 ]]; then
            bash "${REPO_ROOT}/scripts/setup_5g_data_plane.sh" down || true
        else
            sudo -n bash "${REPO_ROOT}/scripts/setup_5g_data_plane.sh" down 2>/dev/null \
                || echo "  (skipped, need sudo: 'sudo bash scripts/setup_5g_data_plane.sh down')"
        fi
    fi
fi
echo "done."
