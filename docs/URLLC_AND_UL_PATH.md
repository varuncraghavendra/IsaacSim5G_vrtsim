# Real uplink path + URLLC QoS (mode=oai-urllc)

This document explains the architectural fixes added to make IsaacSim5G
theoretically correct for 5G teleoperation. It assumes you've already
read the main README and understand the original `mode=oai-real` setup.

## What was broken in the original setup

1. **Camera "uplink" wasn't actually UL.** `viewport_to_udp.py` runs
   inside Isaac on the operator host and sent frames to ext-dn — that
   pushed the packet onto the **downlink** (PDSCH), bounced it off the
   UE TUN, then looped it back to the dashboard. The 5G NR UL scheduler
   (PUSCH, SR/BSR, UL grant timing) never touched the camera flow.

2. **No QoS markings.** All traffic was best-effort eMBB. URLLC features
   (5QI 82/83 mini-slot scheduling, AF interactive video) were unused
   even though they're the entire point of running over 5G NR.

3. **No isolation of the "robot" side.** Anything on the operator host
   could short-circuit the radio via Linux's local routing table.

This mode (`oai-urllc`) fixes (1) and (2) and adds the netns isolation
that makes (1) physically real instead of "logically real".

## Topology

```
default ns (operator)                      robot-ns (robot)
─────────────────────                      ─────────────────
                                           oaitun_ue1
                                              ▲ ▼
                                              │ │   ┌── DL leg: TUN → veth
                                              │ │   │     proxy reads on
                                              │ │   │     oaitun_ue1:16000
                                              │ │   │
ve-rob-host ◄──── veth pair ────► ve-rob-ns ──┘ │   ▼
192.168.244.1                     192.168.244.2 │
    │                                  │       │
    │                                  ▲       │
    │  DL post-veth ─┐                 │       │
    │  16005         │                 │       │
    ▼                │                 │       │
ue_teleop_bridge ────┘                 │       │
    │ forwards →                       │       │
    │ broker (127.0.0.1:16100)         │       │
    ▼                                  │       │
network_aware_broker                   │       │
    │                                  │       │
    ▼                                  │       │
nova_carter_bridge → /cmd_vel          │       │
    │                                  │       │
    ▼                                  │       │
Isaac Sim ── /chassis/odom ─►          │       │
    │              │                   │       │
    │              ▼                   │       │
    │         plotter_publisher        │       │
    │              │                   │       │
    │              ▼                   │       │
    │           /plotter               │       │
    │                                  │       │
    └─ viewport_to_udp ────────────────►       │
       (env ISAAC_UPLINK_HOST=192.168.244.2)   │
       DSCP_AF41                               │
                                               │
                                       robot_netns_proxy
                                       ├── DL leg (above)
                                       └── UL leg: veth → oaitun_ue1
                                           reads on 192.168.244.2:17600
                                           writes to OPERATOR_HOST:17500
                                           via oaitun_ue1  ← this is the
                                                              real PUSCH leg
```

## DSCP / 5QI mapping

| Flow                       | DSCP    | Hex  | Suggested 5QI | 5QI characteristics                                |
|----------------------------|---------|------|---------------|----------------------------------------------------|
| Operator → robot commands  | EF      | 0xb8 | 82 or 83      | delay-critical GBR, ≤10 ms PDB, URLLC              |
| Camera / pose UL           | AF41    | 0x88 | 7 or 65       | non-GBR interactive video, ≤100 ms PDB             |
| Everything else            | (none)  | 0x00 | 9             | non-GBR default                                    |

DSCP markings are applied in:

- `teleop_app/app.py` — the command UDP socket
- `oai_bridge/ue_teleop_bridge.py` — the pose UL socket
- `isaac/viewport_to_udp.py` — the camera capture socket
- `oai_bridge/robot_netns_proxy.py` — both relay sockets, so the marking
  is re-applied even if it was stripped by an intermediate hop.

For the markings to actually map to QoS flows, your 5G core / gNB needs
QoS rules on the PDU session. OAI's UPF supports DSCP-based QoS rule
matching; the relevant block in `oai_rfsim_rtf/.../conf/gnb.conf` looks
roughly like:

```yaml
QoS_Flows:
  - 5QI: 82          # URLLC, commands
    DSCP_uplink: 46  # AF41=46 if you instead want the UL camera here
    DSCP_downlink: 46
  - 5QI: 65          # mission-critical interactive
    DSCP_uplink: 34
    DSCP_downlink: 34
```

Without these rules the DSCP marks are still preserved end-to-end and any
midbox between the UPF and your operator host can use them. With them,
the gNB can actually schedule the URLLC flow with mini-slot priority.

## Why netns

`SO_BINDTODEVICE` and policy routing both *can* force traffic out
`oaitun_ue1`, but they don't survive Linux's local-routing short-circuit
reliably. Putting the egress process in a netns whose only route to
`OPERATOR_NET` is via `oaitun_ue1` makes the path unambiguous: the
kernel has no other choice but to write the IP packet to the UE TUN,
which means the OAI UE process reads it from the TUN FD and ships it on
PUSCH.

This is the same isolation pattern OAI's own integration tests use.

## Trade-offs

- **DDS discovery** stays entirely in default ns because the bridge,
  broker, nova_carter, plotter, and Isaac are all there. Only the dumb
  UDP proxy lives in robot-ns. No CycloneDDS unicast peer fiddling
  required.
- **One extra hop** on each direction (proxy ↔ veth). At loopback /
  veth speeds this is ≪ 1 ms — negligible compared to rfsim latency.
- **Requires root** to create the netns and move the TUN. The launcher
  uses `sudo -E` so your environment carries through.

## Plotter topic

`/plotter` is published by `isaac/plotter_publisher.py` at 30 Hz from
the Isaac side (default ns, before the radio). It carries:

- `sim_time` (builtin_interfaces/Time) — from `/clock`, falling back to
  the odom header stamp
- `rtf` (float64) — rolling slope of `d(sim_time)/d(wall_time)` over
  the last 60 `/clock` samples
- `x, y, z` (float64) — position from `/chassis/odom`
- `vx, vy, vz` (float64) — body-frame linear velocity from `/chassis/odom`
- `heading` (float64) — yaw, derived from the odom quaternion

If you want the post-radio version of these (i.e. what the operator
sees after the 5G UL), subscribe to the bridge's existing pose UDP
envelopes on the dashboard side instead — the bridge encodes the same
fields (minus `rtf`) in `KIND_POSE` IST5 datagrams.

## Running

```bash
# 1-3 you start manually: 5G core, OAI gNB, OAI UE, Isaac.
# 4. URLLC launcher (needs root for netns + ext-dn forwarder).
sudo -E bash scripts/run_oai_teleop_urllc.sh

# 5. In Isaac Script Editor:
#     - ensure Isaac was launched with ISAAC_UPLINK_HOST=192.168.244.2 in env
#       (or edit isaac/viewport_to_udp.py TARGET_HOST constant)
#     - File → Open isaac/viewport_to_udp.py → Ctrl-Enter
```

Verify the path is real UL:

```bash
# tcpdump on the operator-side host bridge interface — you should see
# UDP from oaitun_ue1's IP arriving here, NOT from 127.0.0.1:
sudo tcpdump -n -i any -c 20 'udp and dst port 17500'

# expected:  IP <UE_IP>.<random> > <HOST_BRIDGE_IP>.17500  ...
# NOT:       IP 127.0.0.1.<...> > 127.0.0.1.17500
```

And:

```bash
# ros2 topic, /plotter should be ticking:
source /opt/ros/humble/setup.bash
source ros2_ws/install/setup.bash
ros2 topic hz /plotter
ros2 topic echo --once /plotter
```
