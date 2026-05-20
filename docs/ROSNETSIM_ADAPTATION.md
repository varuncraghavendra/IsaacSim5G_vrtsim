# ROS-NetSim adaptation

ROS-NetSim contributes three ideas that map well into this repository:

1. **Transparent ROS integration**
   - Robot software should continue to speak ROS topics.
   - This repo preserves `/cmd_vel`, `/odom`, `/clock`, and telemetry topics instead of forcing robot code to call a network API.

2. **Tunable fidelity / complexity**
   - ROS-NetSim describes a framework that is transparent to the ROS target application, agnostic to simulator choice, and tunable in fidelity and complexity.
   - Here that tunability appears as broker parameters for cadence, gating, queue depth, replay logging, and ingress path selection.

3. **Shared synchronism updates**
   - ROS-NetSim's example pushes synchronism updates containing node positions and path details.
   - Here the equivalent is:
     - `/network/clock_state`
     - `/network/robot_telemetry`
     - optional obstacle / LOS updates through metrics ingress

## Why not copy ROS-NetSim directly?

ROS-NetSim is ROS1-era infrastructure and its examples rely on an older coordinator layout and network simulator plumbing. This repository keeps the useful concepts but re-targets them to:

- ROS 2 Humble
- OAI RFSim
- Isaac Sim / Nova Carter
- RTF-aware timing
