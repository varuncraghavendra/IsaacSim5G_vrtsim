#!/usr/bin/env bash
# Stop any IsaacSim5G processes left running by the demo launchers.

set -u
echo "stopping IsaacSim5G processes…"
for pat in \
    "ros2 launch network_aware_broker" \
    "network_aware_broker/lib/network_aware_broker" \
    "nova_carter_bridge.py" \
    "dynamic_obstacle_bridge.py" \
    "teleop_app/app.py" \
    "ue_teleop_bridge.py"
do
    pkill -f "${pat}" 2>/dev/null || true
done
sleep 1
remaining=$(ps aux | grep -E "(nova_carter_bridge|teleop_app/app|network_aware_broker|broker_main|ue_ingress|metrics_ingress|rtf_clock)" | grep -v grep || true)
if [[ -n "${remaining}" ]]; then
    echo "still running, escalating to SIGKILL:"
    echo "${remaining}"
    pkill -9 -f "ros2 launch network_aware_broker" 2>/dev/null || true
    pkill -9 -f "network_aware_broker/lib/network_aware_broker" 2>/dev/null || true
    pkill -9 -f "nova_carter_bridge.py" 2>/dev/null || true
    pkill -9 -f "teleop_app/app.py" 2>/dev/null || true
    pkill -9 -f "ue_teleop_bridge.py" 2>/dev/null || true
fi
echo "done."
