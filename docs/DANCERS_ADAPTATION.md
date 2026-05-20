# DANCERS adaptation and improvements

## What was adopted

1. **Decoupled time-stepped synchronization**
   - DANCERS models `Δ = N_phy * δ_phy = N_net * δ_net`.
   - In this repository, the same idea is expressed through broker parameters:
     - `physics_step_ms`
     - `network_step_ms`
     - `publish_rate_hz`
   - The adaptive scheduler uses these values to release commands only when both timing and gating constraints are satisfied.

2. **Coordinator / connector separation**
   - DANCERS uses a coordinator plus simulator connectors.
   - Here, the ROS 2 broker library is the coordinator, while:
     - `ue_ingress_node` is the network-side connector
     - `nova_carter_bridge.py` is the physics-side connector
     - `dynamic_obstacle_bridge.py` is the channel / environment connector

3. **Minimal synchronization payloads**
   - DANCERS stresses keeping synchronization messages compact.
   - Here, ingress packets are small JSON payloads, normalized immediately into ROS 2 messages.

4. **Virtualization mindset**
   - DANCERS supports network virtualization using namespaces and TAP/TUN concepts.
   - This repo includes OAI-oriented UDP bridge scripts and notes to plug into a real namespace/TAP path later.

## Limitations in the paper and how v4 addresses them

### Limitation: static obstacles shared only at initialization
**Paper note:** DANCERS keeps obstacle scenes static during a run to reduce synchronization payloads.

**v4 improvement:** `isaac/dynamic_obstacle_bridge.py` hot-reloads an obstacle JSON file and recomputes LOS/path-loss penalties at runtime. This makes obstacle updates possible without rebuilding the simulation.

### Limitation: virtualization mode is hard to prove reproducible
**Paper note:** correctness is difficult to prove in virtualization mode.

**v4 improvement:** the broker can write deterministic CSV event logs, and `integration/replay_csv.py` replays the exact command / metrics timing for regression testing.

### Limitation: single network simulator focus
**Paper note:** DANCERS mainly ships with ns-3 and Wi-Fi examples.

**v4 improvement:** this repository is organized around OAI RFSim / 5G-facing ingress and metric updates instead of ns-3-specific APIs.

### Limitation: no RTF-aware adaptive control release
**Paper note:** the paper defines synchronization but does not target Isaac Sim + configurable RTF + OAI timing.

**v4 improvement:** release deadlines scale with measured latency, jitter, PER, and current RTF.
