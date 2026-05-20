# oai_bridge — IsaacSim5G ↔ OAI 5G data plane

This folder contains the host-side and UE-side pieces that connect the
IsaacSim5G dashboard to a real OpenAirInterface 5G stack running in
RFsim mode. The full OAI build + configs are kept separately under
`../oai_rfsim_rtf/fixed_rtf_rfsim-develop/`.

## Architecture

```
Browser ──ws──► dashboard (mode=oai-real)
                  │ UDP JSON to UE_IP:UE_PORT
                  ▼
              5G core UPF / SPGW-U
                  │ GTP-U
                  ▼
                  gNB
                  │ rfsim 5G air interface
                  ▼
                  UE  ── tun(oaitun_ue1) ↦ 12.1.1.2:UE_PORT
                       │
                       ▼
               ue_teleop_bridge.py   ← this folder
                       │ rclpy
                       ▼
                  /cmd_vel
                       │
                       ▼
               Isaac Sim Nova Carter

Reverse path (camera + odom):
   Isaac → /odom + /front_hawk/*/image_raw
       └─► ue_teleop_bridge subscribes (rclpy)
           └─► UDP envelopes to OPERATOR_IP:OPERATOR_PORT
                  │ over the 5G uplink
                  ▼
              dashboard listener → WebSocket → browser
```

## Files

| File | Purpose |
|------|---------|
| `ue_teleop_bridge.py` | The "robot side" daemon. Listens for downlink UDP commands and republishes them as ROS 2 `geometry_msgs/Twist` on `/cmd_vel`. Also subscribes to ROS 2 odom + camera topics and sends them via UDP back through the 5G uplink. |
| `protocol.py` | Shared wire format for the uplink envelopes (magic `0x49535435 = "IST5"`, plus JSON header + binary payload). Imported by both the bridge and the dashboard. |

## Run

There are four manually-started pieces (5G core, gNB, UE, Isaac), and
two pieces driven by `scripts/run_oai_teleop.sh` (dashboard +
ue_teleop_bridge). Quick start, single machine:

```bash
# Terminal A — 5G core (your favourite implementation; example open5gs)
sudo systemctl start open5gs-amfd  open5gs-smfd  open5gs-upfd  open5gs-nrfd ...

# Terminal B — OAI gNB (in the OAI build directory)
cd /path/to/oai_rfsim_rtf/fixed_rtf_rfsim-develop/openairinterface5g/cmake_targets/ran_build/build
sudo ./nr-softmodem \
    -O ../../conf/gnb.conf \
    --gNBs.[0].min_rxtxtime 6 \
    --rfsim --rfsimulator.[0].serveraddr server

# Terminal C — OAI UE
sudo ./nr-uesoftmodem \
    -r 106 --numerology 1 --band 78 -C 3619200000 --ssb 516 \
    --rfsim \
    -O ../../conf/ue.conf

# Terminal D — Isaac Sim with the Nova Carter ROS scene (start manually)

# Terminal E — dashboard + UE bridge
cd /path/to/IsaacSim5G-master
UE_IP=12.1.1.2 OPERATOR_IP=127.0.0.1 bash scripts/run_oai_teleop.sh
```

Then open <http://localhost:8080>. Hold W/A/S/D in the browser — the
commands ride the 5G data plane, get decoded by `ue_teleop_bridge.py`
on the UE side, and become `/cmd_vel` Twists for Isaac. The robot's
camera frames and odometry come back the same way.

## Environment knobs

| Variable | Default | Used by |
|----------|---------|---------|
| `UE_IP` | `12.1.1.2` | dashboard downlink target |
| `UE_PORT` | `16000` | both ends |
| `OPERATOR_IP` | `127.0.0.1` | bridge uplink target |
| `OPERATOR_PORT` | `17500` | both ends |
| `CAMERA_TOPICS` | `/Nova_Carter_ROS/front_hawk/left/image_raw,/Nova_Carter_ROS/front_hawk/right/image_raw` | bridge subscribes |
| `ODOM_TOPICS` | `/odom,/chassis/odom` | bridge subscribes |
| `CAMERA_FPS_LIMIT` | `10` | bridge per-topic rate cap |
| `CAMERA_JPEG_QUALITY` | `60` | bridge JPEG re-encode quality (raw frames) |
| `PLATFORM` | `nova_carter` | dashboard UI |

## Notes

* The downlink datagrams are plain JSON, identical to what the
  dashboard's original simulated-broker `oai` mode already produced —
  so the bridge accepts both formats.
* The uplink envelopes carry a small magic header (`IST5`) so the
  dashboard listener can frame them correctly when several
  datagrams arrive interleaved.
* Camera frames are sent as JPEGs (compressed by default, or
  re-encoded from raw on the bridge side). Stick to a reasonable
  `CAMERA_FPS_LIMIT` and small resolution — rfsim's effective uplink
  bandwidth is modest, and large fragmented datagrams will get
  dropped.
* If you want to use the OAI VRTsim path with channel modelling, see
  the time_bridge folder under `oai_rfsim_rtf/fixed_rtf_rfsim-develop/`.
  That's orthogonal to this data-plane bridge.
