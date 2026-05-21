# IsaacSim5G (vrtsim drone teleop)

Browser-based teleoperation for an Isaac Sim **Iris quadcopter** in the
NVIDIA City demo scene, where every byte the operator sends to the drone
— commands DL, FPV camera UL, and pose telemetry UL — actually crosses
an OpenAirInterface 5G air interface (rfsim or vrtsim). If OAI isn't in
the loop, the drone is silent. If the UE TUN isn't up, the launcher
refuses to start. No localhost MAVLink fallback exists anywhere.

The original Nova Carter pipeline is still here for reference (see the
"Nova Carter" sections further down) — the drone path is purpose-built
on top of the same URLLC/netns plumbing.

## TL;DR — drone, OAI-only, no Isaac scripts

```
Browser  ── WebSocket joystick ──►  teleop_app/drone_app.py  (:8080)
                                      │  UDP JSON
                                      ▼
                            EXT_DN_IP:16000 (oai-ext-dn)
                                      │
                                      ▼
                            UPF → gNB → AIR (rfsim/vrtsim)
                                      │
                                      ▼
                            UE → oaitun_ue1   (inside robot-ns)
                                      │
                                      ▼
                            robot_netns_proxy ─veth─►
                            oai_bridge/drone_ue_bridge.py
                                      │  MAVLink MANUAL_CONTROL
                                      ▼
                            PX4 SITL (Pegasus auto-launch in Isaac)
                                      │
                                      ▼
                            Iris in city  ◄── camera + pose UL ──►
                                                                    │
   isaac/viewport_screen_capture.py (host-side mss, no Isaac script)│
   PX4 LOCAL_POSITION_NED + ATTITUDE                                │
                                                                    │
                              ── IST5 envelopes ── via OAI UL ──────┘
                                                                    │
                                                                    ▼
                              teleop_app/drone_app.py UL listener (:17500)
                                       │
                                       ├─► browser camera + HUD via WebSocket
                                       └─► ZMQ PUB tcp://127.0.0.1:5556
                                            (MSG_FORMAT for live_plot_position.py
                                             / main_paths_los_model_*.py — same wire
                                              format the channel bridge already speaks)
```

In Isaac you do exactly one thing: **add a second viewport** that frames
the FPV camera. No Script Editor, no OmniGraph, no extension changes.

## Components added in this rewrite

| File | Role |
|------|------|
| `teleop_app/drone_app.py` | Operator dashboard. Twin virtual-joystick UI. Sends DL JSON to ext-dn only. Listens UL on UDP 17500. Republishes pose as `MSG_FORMAT` on `tcp://5556`. No pymavlink. |
| `teleop_app/static/drone.{html,js,css}` | Twin-stick browser UI + camera panel + telemetry HUD. |
| `oai_bridge/drone_ue_bridge.py` | UE-side bridge in the default ns. Listens on the veth for DL after the radio; translates JSON to MAVLink for PX4 (`udpin:127.0.0.1:14540`). Reads PX4 NED pose, converts to ENU, ships IST5 pose envelopes via the robot-ns proxy. |
| `isaac/viewport_screen_capture.py` | Host-side X11/mss capture of the Isaac second viewport. xdotool window auto-detect or explicit `CAPTURE_REGION`. JPEG+IST5 over UDP to the robot-ns ingress so the camera also rides OAI UL. |
| `scripts/run_drone_oai.sh` | End-to-end launcher. Brings up ext-dn forwarders, robot-ns + veth, proxy, bridge, dashboard, capture. Refuses to start when `oaitun_ue1` is missing. |
| `sim_isaac_trajectory.py` (modified) | Adds `--trajectory isaac` mode that proxies pose from the (optional) Isaac sidecar into `MSG_FORMAT`. Synthetic trajectories untouched. |
| `isaac/drone_telemetry.py` | OPTIONAL Isaac Script Editor sidecar (merged viewport + ZMQ pose). Kept for environments where the Script Editor is available. The OAI-gated path above does NOT require it. |

## Prereqs you bring up before launching the demo

1. **5G core** (`docker compose -f /path/to/oai-cn5g/docker-compose.yaml up -d`).
2. **OAI gNB** in rfsim or vrtsim mode:
   ```bash
   sudo ./nr-softmodem \
       -O /path/to/gnb.conf \
       --gNBs.[0].min_rxtxtime 6 \
       --rfsim --rfsimulator.[0].serveraddr server
   ```
   (For vrtsim, use `gnb_v.conf` + the `time_bridge` companions; the
   data-plane path the dashboard sees is unchanged.)
3. **OAI UE** — this is what creates `oaitun_ue1`:
   ```bash
   sudo ./nr-uesoftmodem -r 106 --numerology 1 --band 78 -C 3619200000 \
       --ssb 516 --rfsim -O /path/to/ue.conf
   ```
4. **Isaac Sim** with the Pegasus Drone Simulator Extension running
   `city_drone_fpv.py` (or any scene that spawns Iris + PX4 SITL).
   In Isaac, add a second viewport showing the FPV camera.

## Launching the drone pipeline

```bash
# one-time: build the dashboard venv
python3 -m venv teleop_app/.venv
teleop_app/.venv/bin/pip install -r teleop_app/requirements.txt

# on every run, after the four prereqs above are up
sudo -E TELEOP_PORT=8080 bash scripts/run_drone_oai.sh
# open http://localhost:8080
```

Logs land in `.run_logs/{drone_app,drone_ue_bridge,robot_netns_proxy,viewport_capture}.log`.
Ctrl-C the launcher to tear everything down (ext-dn forwarders + netns + bridge + dashboard + capture).

## Safety guarantee

Three things make "no OAI = no drone response" mechanical, not a policy:

1. `drone_app.py` only ever sends DL bytes to `EXT_DN_IP:EXT_DN_PORT`.
   There is no code path that talks to `127.0.0.1:14*` directly.
2. `drone_ue_bridge.py` only listens on the veth address that the
   robot-ns proxy uses. With no UE TUN there's no proxy egress, so the
   bridge never receives anything, so PX4 never receives a
   `MANUAL_CONTROL`.
3. `run_drone_oai.sh` refuses to start if `oaitun_ue1` is missing in
   default ns *and* in `robot-ns`.

You can verify (with OAI down) by hitting `http://localhost:8080/healthz`
while jiggling the joystick: `downlink_sent` will tick up but
`poses_rx` / `frames_rx` stay at zero — the dashboard is alive, the
drone is silent, exactly as required.

---

## Original Nova Carter pipeline (kept for reference)

> **For the architecturally-correct setup (real PUSCH UL + URLLC QoS +
> `/plotter`)**, see `docs/URLLC_AND_UL_PATH.md` and use
> `scripts/run_oai_teleop_urllc.sh`. Earlier modes (`oai-real`,
> `oai-real + chain + ext-dn`) only exercise the DL direction of the
> radio — the camera "uplink" path on those modes is physically PDSCH
> looped back through the UE TUN, not true PUSCH. The URLLC mode fixes
> that with a robot-side netns.

## Drone demo — fly the Iris in `city_drone_fpv.py` from the browser

If your starting point is the city + Iris quadcopter scene (the one
launched by `city_drone_fpv.py`), the channel pipeline now sources its
"ground truth" from the real drone instead of `sim_isaac_trajectory.py`'s
synthetic patterns. You fly with a virtual joystick in the browser;
pose feedback (x, y, z, vx, vy, vz, heading) is read out of Isaac and
re-published in the existing `MSG_FORMAT` so the OAI L1 / channel /
plotter stack runs unchanged.

```
Browser (twin virtual joystick + FPV)
  │  WebSocket joystick state @ 30 Hz
  ▼
teleop_app/drone_app.py
  │  MAVLink MANUAL_CONTROL @ 50 Hz (pymavlink)
  ▼
PX4 SITL (auto-launched by Pegasus inside Isaac)
  ▼
Iris quad in city_drone_fpv scene

isaac/drone_telemetry.py  (in Isaac's Script Editor)
  │  ZMQ PUB  tcp://127.0.0.1:5570  (raw pose JSON @ ~100 Hz)
  ▼
sim_isaac_trajectory.py --trajectory isaac
  │  ZMQ PUB  tcp://127.0.0.1:5556  (MSG_FORMAT "<ddddddddd">)
  ▼
main_paths_los_model_measurment_position.py
  ▼
OAI gNB / sim_l1.py  →  live_plot_position.py

isaac/drone_telemetry.py also captures the FPV viewport and ships it
as IST5 UDP datagrams to the dashboard (default 127.0.0.1:17500). The
dashboard listens, decodes, and forwards each JPEG to the browser via
the same WebSocket — same wire path the Nova Carter dashboard already
uses, no extra plumbing.
```

Run order (after Isaac is already playing the city scene):

```bash
cd "$REPO/IsaacSim5G-master"

# A. Make sure pymavlink + pyzmq are in the dashboard venv (one-time).
teleop_app/.venv/bin/pip install -r teleop_app/requirements.txt

# B. Dashboard + Isaac→channel-bridge pose proxy.
bash scripts/run_drone_demo.sh
#  logs land in .run_logs/{drone_app,isaac_pose_proxy}.log
#  open http://localhost:8080 in your browser

# C. Inside Isaac, Window → Script Editor → File → Open
#         isaac/drone_telemetry.py
#    Ctrl-Enter to start the viewport-capture + pose-publisher loop.

# D. Channel pipeline (each in its own terminal — same as the existing
#     /tmp/ground_truth.csv flow used by live_plot_position.py):
python main_paths_los_model_measurment_position.py --gnb-x 0 --gnb-y 0 --gnb-z 3
python ../oai_rfsim_rtf/fixed_rtf_rfsim-develop/time_bridge/live_plot_position.py \
    --gnb-x 0 --gnb-y 0
# plus your OAI gNB/UE (or sim_l1.py if you don't want the radio in this run)
```

To stop the dashboard + proxy, Ctrl-C the `run_drone_demo.sh` terminal.
In Isaac's Script Editor, call `STOP_DRONE_TELEMETRY()` to halt the
capture + pose loops.

UI quick reference (browser keyboard maps to the joystick buttons too):

| Control | Action |
|---------|--------|
| Left stick (vertical) | throttle (down = idle, up = full) |
| Left stick (horizontal) | yaw |
| Right stick (vertical) | pitch (forward / back) |
| Right stick (horizontal) | roll |
| Space | E-stop (zero sticks while held) |
| Z / X | ARM / DISARM |
| T / L | TAKEOFF / LAND |
| ARM/DISARM/TAKEOFF/LAND/POSCTL/ALTCTL/STAB/OFFBOARD | clickable buttons |

Each MANUAL_CONTROL goes to PX4 SITL at 50 Hz. If you want to keep
QGroundControl connected on UDP 14550 at the same time, leave the
default MAVLink URL alone — the dashboard binds the *companion* port
14540 and PX4 multiplexes both endpoints.

To skip the proxy and have `drone_telemetry.py` publish `MSG_FORMAT`
directly on `tcp://127.0.0.1:5556`, set in the env that launches Isaac:

```bash
export ISAAC_POSE_DIRECT_ADDR=tcp://127.0.0.1:5556
```

In that mode you don't run `sim_isaac_trajectory.py` at all — Isaac
publishes the channel-bridge stream itself.

---

## Quickstart (fresh clone → driving in the browser)

This is the abbreviated, copy-pasteable recipe. The full step-by-step
explanation lives in the sections below; read them if anything fails.

```bash
# 1. clone (assumes the outer wrapper layout)
git clone <your-repo-url> IsaacSim5G
cd IsaacSim5G                               # contains IsaacSim5G-master/ + oai_rfsim_rtf/
export REPO="$PWD"

# 2. dashboard venv (FastAPI, uvicorn, Pillow, OpenCV, etc.)
cd "$REPO/IsaacSim5G-master"
python3 -m venv teleop_app/.venv
teleop_app/.venv/bin/pip install --upgrade pip
teleop_app/.venv/bin/pip install -r teleop_app/requirements.txt

# 3. build the C++ broker workspace (network-aware shaping + ingress nodes)
bash scripts/build_ros2_ws.sh

# 4. build OAI gNB/UE if not already built
cd "$REPO/oai_rfsim_rtf/fixed_rtf_rfsim-develop/openairinterface5g/cmake_targets"
./build_oai -w SIMU --gNB --nrUE --ninja -C
```

Then, on every run, bring up the stack in this order (each item lives in
its own terminal except the launcher in step 6, which fans out four
processes for you):

```bash
# A. 5G core (oai-cn5g compose stack — installed separately)
docker compose -f /path/to/oai-cn5g/docker-compose.yaml up -d
# wait until amf/smf/upf/ext-dn report (healthy)

# B. OAI gNB
cd "$REPO/oai_rfsim_rtf/fixed_rtf_rfsim-develop/openairinterface5g/cmake_targets/ran_build/build"
sudo ./nr-softmodem \
    -O "$REPO/oai_rfsim_rtf/fixed_rtf_rfsim-develop/conf/gnb.conf" \
    --gNBs.[0].min_rxtxtime 6 \
    --rfsim --rfsimulator.[0].serveraddr server

# C. OAI UE  (creates oaitun_ue1 — note its IP, but you no longer need to pass it)
cd "$REPO/oai_rfsim_rtf/fixed_rtf_rfsim-develop/openairinterface5g/cmake_targets/ran_build/build"
sudo ./nr-uesoftmodem -r 106 --numerology 1 --band 78 -C 3619200000 --ssb 516 \
    --rfsim -O "$REPO/oai_rfsim_rtf/fixed_rtf_rfsim-develop/conf/ue.conf"

# D. start Isaac Sim, load the Nova_Carter_ROS scene, press Play.

# E. ext-dn UDP forwarders (auto-detect UE IP from oaitun_ue1)
cd "$REPO/IsaacSim5G-master"
sudo bash scripts/setup_5g_data_plane.sh up
#  expected:  [5g] auto-detected FWD_UE_IP=<your-ue-ip> from oaitun_ue1
#             [5g] starting DOWNLINK forwarder (port 16000 -> <ue>:16000)
#             [5g] starting CAMERA   forwarder (port 17600 -> <ue>:17600)

# F. dashboard + UE bridge + broker + nova_carter_bridge (the broker-chain launcher)
UE_IP=192.168.70.135 \
OPERATOR_IP=127.0.0.1 \
BROKER_UDP_PORT=16100 \
CAMERA_UPLINK_LISTEN_PORT=17600 \
PLATFORM=nova_carter \
TELEOP_PORT=8080 \
CAMERA_FPS_LIMIT=30 \
    bash scripts/run_oai_teleop_broker.sh
#  logs land in .run_logs/{teleop_app,ue_bridge,broker,nova_carter_bridge}.log

# G. inside Isaac:  Window → Script Editor → File → Open
#       $REPO/IsaacSim5G-master/isaac/viewport_to_udp.py
#    Ctrl-Enter to start capturing Viewport 2 into the 5G uplink.

# H. open http://localhost:8080 in a browser; drive with W/A/S/D, Q/E, Space.
```

To shut down:

```bash
bash scripts/stop.sh                              # dashboard + bridge + broker + nova_carter_bridge
sudo bash scripts/setup_5g_data_plane.sh down     # ext-dn forwarders
# In Isaac's Script Editor:  STOP_VIEWPORT_CAPTURE()
# Then Ctrl-C the gNB / UE terminals and  docker compose down  for the core.
```

The rest of this README explains each piece in detail.

## End-to-end pipeline (recommended: real-radio + ext-dn + viewport capture)

```
Browser
  │  WebSocket (W/S=fwd/back, A/D=strafe, Q/E=rotate)
  ▼
teleop_app/app.py (mode=oai-real, --udp-host 192.168.70.135)
  │  UDP JSON → ext-dn ingress
  ▼
oai-ext-dn container  (Python UDP forwarder, port 16000 → 10.0.0.4:16000)
  │
  ▼
oai-upf   (GTP-U encap, N3 → gNB)
  │
  ▼
OAI gNB  (./nr-softmodem --rfsim ...)
  │  rfsim 5G AIR  ◄── REAL RADIO CROSSING
  ▼
OAI UE   (./nr-uesoftmodem --rfsim ...) → oaitun_ue1 (10.0.0.4:16000)
  │
  ▼
ue_teleop_bridge.py  (default ns, :16000)
  │  forwards UDP → 127.0.0.1:16100, --no-publish-cmd-vel
  ▼
network_aware_broker stack  → /network/cmd_vel
  │
  ▼
nova_carter_bridge.py  /network/cmd_vel → /cmd_vel @ 30 Hz, 500 ms watchdog
  │
  ▼
Isaac Sim Nova Carter (mecanum) — physics steps the robot

UPLINK (camera): isaac/viewport_to_udp.py captures Isaac's "Viewport 2",
  JPEG-encodes, sends UDP IST5 → 192.168.70.135:17600 → ext-dn forwarder →
  UPF → gNB → rfsim AIR (radio in the reverse direction) → UE → bridge :17600
  → 127.0.0.1:17500 → dashboard → browser camera panel.

UPLINK (odom): /chassis/odom subscribed by the bridge, packed as IST5,
  sent to the dashboard uplink listener (loopback in single-host setup).
```

Two simpler variants (`mode=oai-real` straight, and `mode=oai-real` +
broker chain without ext-dn) are documented further down — useful for
debugging individual layers, but neither exercises the actual rfsim
radio in a single-host setup.

---

## Prerequisites

Already installed and working on the host:

- **Ubuntu 22.04** (or compatible) — required for ROS 2 Humble.
- **ROS 2 Humble** at `/opt/ros/humble`. Override with `ROS_SETUP=...` if elsewhere.
- **Python 3.10+** with `venv`.
- **OpenAirInterface 5G** — gNB and UE binaries built (see step 4 below if not).
- **A 5G core** (open5gs, oai-cn5g, free5GC — any will do).
- **Isaac Sim 2023.1+** with the Nova Carter ROS scene available.
- **Build tools**: `cmake`, `ninja-build`, `colcon-common-extensions`, `git`.

---

## Step-by-step setup from a fresh clone

### 1. Clone the repository

```bash
git clone <your-repo-url> IsaacSim5G
cd IsaacSim5G
export REPO="$PWD"          # used in later commands
```

The repo expects this layout (already present):

```
$REPO/
├── IsaacSim5G-master/                  # dashboard, bridge, scripts, ros2_ws
└── oai_rfsim_rtf/fixed_rtf_rfsim-develop/   # OAI source + conf files
```

If your top-level directory is just `IsaacSim5G-master` (no outer wrapper),
adjust `$REPO` accordingly. The dashboard scripts assume their parent is
the repo root.

### 2. Create the dashboard Python venv

The dashboard uses FastAPI / uvicorn / OpenCV. `rclpy` is *not* installed
into the venv — it comes from the system ROS 2 install.

```bash
cd "$REPO/IsaacSim5G-master"
python3 -m venv teleop_app/.venv
teleop_app/.venv/bin/pip install --upgrade pip
teleop_app/.venv/bin/pip install -r teleop_app/requirements.txt
```

The launcher auto-detects `teleop_app/.venv/bin/python` and uses it.

### 3. Build the ROS 2 workspace *(needed for `mode=oai` and for the broker-chain mode)*

This builds the C++ network-aware broker stack. You can **skip this step**
if you're only running plain `mode=oai-real` (radio only, no broker).
Build it if you'll run either:

- **`mode=oai`** — broker simulates the 5G, no radio in the path; or
- **`mode=oai-real` + broker chain** — radio carries the command and the
  broker shapes/paces what comes out (see "Running with the broker
  chained after the radio" below).

```bash
cd "$REPO/IsaacSim5G-master"
bash scripts/build_ros2_ws.sh        # wraps colcon build
```

Pass `--clean` to nuke `build/ install/ log/` first, or `--debug` for a
debug build. Output ends up in `ros2_ws/install/`; source it with
`source ros2_ws/install/setup.bash` before `ros2 launch`.

### 4. *(Optional)* Build OAI if not already built

Skip if `nr-softmodem` and `nr-uesoftmodem` already exist under
`oai_rfsim_rtf/fixed_rtf_rfsim-develop/openairinterface5g/cmake_targets/ran_build/build/`.

```bash
cd "$REPO/oai_rfsim_rtf/fixed_rtf_rfsim-develop/openairinterface5g/cmake_targets"
./build_oai -w SIMU --gNB --nrUE --ninja -C
```

OAI configs the rest of this README references:

```
oai_rfsim_rtf/fixed_rtf_rfsim-develop/conf/
├── gnb.conf       # gNB config (use with --rfsim)
├── gnb_v.conf     # gNB config for VRTsim mode (optional)
└── ue.conf        # UE config (already contains rfsimulator.rtf knob)
```

---

## Running the pipeline from scratch (`mode=oai-real`)

Open six terminals. Each block stays in its own terminal — don't background.

### Terminal A — 5G core

Bring up your core however you normally do (this README is agnostic).
Examples:

```bash
# open5gs
sudo systemctl start open5gs-mmed open5gs-amfd open5gs-smfd \
                    open5gs-upfd open5gs-nrfd open5gs-ausfd \
                    open5gs-udmd open5gs-udrd open5gs-pcfd open5gs-bsfd
```

The UE subscriber's IMSI/key/opc must already be provisioned in the
core's subscriber DB and must match `ue.conf`.

### Terminal B — OAI gNB

```bash
export REPO="/path/to/IsaacSim5G"
cd "$REPO/oai_rfsim_rtf/fixed_rtf_rfsim-develop/openairinterface5g/cmake_targets/ran_build/build"
sudo ./nr-softmodem \
    -O "$REPO/oai_rfsim_rtf/fixed_rtf_rfsim-develop/conf/gnb.conf" \
    --gNBs.[0].min_rxtxtime 6 \
    --rfsim --rfsimulator.[0].serveraddr server
```

### Terminal C — OAI UE

```bash
export REPO="/path/to/IsaacSim5G"
cd "$REPO/oai_rfsim_rtf/fixed_rtf_rfsim-develop/openairinterface5g/cmake_targets/ran_build/build"
sudo ./nr-uesoftmodem \
    -r 106 --numerology 1 --band 78 -C 3619200000 --ssb 516 \
    --rfsim \
    -O "$REPO/oai_rfsim_rtf/fixed_rtf_rfsim-develop/conf/ue.conf"
```

Once attach succeeds, the UE creates the `oaitun_ue1` interface.

### Terminal D — discover the UE IP

```bash
ip -4 addr show oaitun_ue1
# inet 10.0.0.4/24 scope global oaitun_ue1
```

Note the address — you'll pass it as `UE_IP=` in Terminal F. The exact
value depends on your core's UE pool config (open5gs default is
`10.45.0.x`; the conf shipped here yields `10.0.0.4`).

### Terminal E — Isaac Sim

Launch Isaac, open the **Nova_Carter_ROS** scene (the one with a
`/cmd_vel` subscriber and camera/odom publishers), and press Play. Verify
from another shell:

```bash
source /opt/ros/humble/setup.bash
ros2 topic list | grep -E "cmd_vel|odom|hawk"
```

`/cmd_vel`, `/chassis/odom`, and ideally
`/Nova_Carter_ROS/front_hawk/{left,right}/image_raw` should be listed. If
the camera topics are missing the dashboard will still drive — only the
video panels will be empty.

### Terminal F — dashboard + UE bridge

Easiest path is the bundled launcher (starts both in one terminal, logs
to `.run_logs/`):

```bash
cd "$REPO/IsaacSim5G-master"
UE_IP=10.0.0.4 OPERATOR_IP=127.0.0.1 PLATFORM=nova_carter TELEOP_PORT=8080 \
    bash scripts/run_oai_teleop.sh
```

Replace `UE_IP` with whatever Terminal D reported. `OPERATOR_IP` stays
`127.0.0.1` for single-machine RFsim.

#### Split-terminal alternative

If you'd rather see dashboard and bridge logs separately, run each in
its own terminal instead of the launcher:

```bash
# Terminal F1 — dashboard (operator side)
cd "$REPO/IsaacSim5G-master"
source /opt/ros/humble/setup.bash
./teleop_app/.venv/bin/python teleop_app/app.py \
    --platform nova_carter \
    --mode oai-real \
    --port 8080 \
    --udp-host 10.0.0.4 --udp-port 16000 \
    --operator-udp-host 0.0.0.0 --operator-udp-port 17500
```

```bash
# Terminal F2 — UE-side bridge
cd "$REPO/IsaacSim5G-master"
source /opt/ros/humble/setup.bash
./teleop_app/.venv/bin/python oai_bridge/ue_teleop_bridge.py \
    --listen-host 0.0.0.0 --listen-port 16000 \
    --operator-host 127.0.0.1 --operator-port 17500
```

Both terminals need ROS sourced — both processes use `rclpy`.

### Drive

Open <http://localhost:8080> and steer with **W / A / S / D**.

You should see in `.run_logs/ue_bridge.log` (or the bridge terminal):

```
stats  downlink: cmds=NNN bad=0 bytes=...  uplink: pose=NNN camera=...
```

`cmds` increments as you press keys (downlink flowing through 5G); `pose`
increments on its own (uplink flowing back from Isaac).

---

## Running with the broker chained after the radio

Chains the C++ `network_aware_broker` stack *downstream* of the OAI 5G
crossing — so the radio still carries every command, **and** the broker
applies post-radio pacing, watchdog, and shaping on top. Useful when you
want to study broker-side behavior (deterministic replay, RTF clock,
adaptive command release) on top of real radio dynamics rather than
instead of them.

### Chain pipeline

```
Browser
  │  WebSocket
  ▼
teleop_app/app.py  (mode=oai-real)
  │  UDP JSON → UE_IP:16000   (over OAI 5G radio)
  ▼
oaitun_ue1 (UE TUN)
  │
  ▼
oai_bridge/ue_teleop_bridge.py
  │  forwards raw UDP → 127.0.0.1:16100      (NEW: --forward-udp-host/port)
  │  does NOT publish /cmd_vel                (NEW: --no-publish-cmd-vel)
  ▼
network_aware_broker stack
  │  ue_ingress_node :16100 → /network/teleop_command
  │  network_aware_broker    → /network/cmd_vel  (100 Hz, 600 ms hold)
  │  metrics_ingress_node :16001
  │  rtf_clock_node         → /clock (replay only; idle in live mode)
  ▼
isaac/nova_carter_bridge.py
  │  /network/cmd_vel → /cmd_vel  (30 Hz, 500 ms watchdog)
  ▼
Isaac Sim Nova Carter
```

The uplink (camera + odom back to the dashboard) still flows directly
through the UE bridge as in plain `oai-real` — the broker only touches
the downlink command path.

### Why the broker is on a different UDP port

The broker's `ue_ingress_node` defaults to UDP :16000 in `mode=oai` —
the same port the UE bridge already owns in `oai-real`. To stack them,
the broker is launched with `BROKER_UDP_PORT=16100` (env var, picked up
by `network_aware_stack.launch.py`).

### Launch

Same six terminals as plain `oai-real` (core, gNB, UE, UE-IP lookup,
Isaac) plus one — Terminal F is swapped for the chain launcher:

```bash
cd "$REPO/IsaacSim5G-master"
UE_IP=10.0.0.4 OPERATOR_IP=127.0.0.1 BROKER_UDP_PORT=16100 \
PLATFORM=nova_carter TELEOP_PORT=8080 \
    bash scripts/run_oai_teleop_broker.sh
```

This starts four processes in one terminal (logs in `.run_logs/`):

| Log | Process |
|-----|---------|
| `.run_logs/teleop_app.log` | dashboard (mode=oai-real, downlink → UE_IP:16000) |
| `.run_logs/ue_bridge.log` | UE bridge with `--forward-udp-host 127.0.0.1 --forward-udp-port 16100 --no-publish-cmd-vel` |
| `.run_logs/broker.log` | `ros2 launch network_aware_broker network_aware_stack.launch.py` (with broker on :16100) |
| `.run_logs/nova_carter_bridge.log` | `/network/cmd_vel` → `/cmd_vel` relay (30 Hz, 500 ms watchdog) |

After launch, `ros2 node list` should show seven nodes:
`/isaacsim5g_teleop`, `/ue_teleop_bridge`, `/ue_ingress_node`,
`/metrics_ingress_node`, `/network_aware_broker`, `/rtf_clock_node`,
`/nova_carter_bridge`.

### Verifying the chain end-to-end

Inject a synthetic command on the operator side and watch it ride the
radio, hit the bridge's forward path, pass through the broker, and
emerge on `/cmd_vel`:

```bash
source /opt/ros/humble/setup.bash
( ros2 topic echo --once /cmd_vel & ) ; sleep 0.5
python3 -c "
import socket, json, time
s = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
for _ in range(20):
    s.sendto(json.dumps({'linear_x':0.4,'angular_z':0.2}).encode(),
             ('10.0.0.4', 16000))
    time.sleep(0.02)
"
```

Within ~1 s you should see `linear.x: 0.4, angular.z: 0.2` echoed on
`/cmd_vel`, and `forward: cmds=N` in `.run_logs/ue_bridge.log` should
tick up (not all 20 will arrive — RFsim drops a few; that's expected).

### Topic contracts in chain mode

| Topic | Publisher | Subscriber |
|-------|-----------|------------|
| `/network/teleop_command` (`TeleopCommand`) | `ue_ingress_node` | `network_aware_broker` |
| `/network/cmd_vel` (`Twist`) | `network_aware_broker` | `nova_carter_bridge` |
| `/cmd_vel` (`Twist`) | `nova_carter_bridge` (+ dashboard's defensive pub) | Isaac articulation graph (+ dashboard HUD) |

Keeping `/cmd_vel` and `/network/cmd_vel` distinct is what prevents the
historical self-loop bug — there is still exactly one *active*
publisher on `/cmd_vel`: `nova_carter_bridge`.

### When to use which mode

- Plain `mode=oai-real` (`scripts/run_oai_teleop.sh`) — when you just
  want the cleanest measurement of "what does the radio do to my
  Twist". Fewest moving parts.
- Chain mode (`scripts/run_oai_teleop_broker.sh`) — when you want both
  the radio in the loop *and* the broker doing pacing/watchdog/shaping
  on top of it.
- `mode=oai` (`scripts/run_teleop_oai.sh`) — when the radio isn't
  available or you want a pure simulated-5G control study.

---

## Running with **real 5G radio crossing** + Isaac Viewport capture (recommended)

This is the most complete setup: the teleop downlink (commands) actually
traverses the rfsim air interface, and the camera uplink (Isaac
Viewport 2 frames) crosses the radio in the opposite direction. The
network-aware broker is chained downstream of the radio for pacing /
watchdog / shaping. A single dashboard panel renders the Viewport 2
feed.

### Why this exists

In single-machine OAI rfsim, Linux's local routing table short-circuits
any traffic to the UE's TUN address (`10.0.0.4`) through loopback — the
"5G downlink" never actually touches the radio. To force a real radio
crossing without netns surgery, we route every UDP flow through the
`oai-ext-dn` data-network container, which already has the right route
(`10.0.0.0/16 via 192.168.70.134`) to push traffic into the UPF / N3 /
gNB / rfsim air.

### Full pipeline

```
Browser
  │ WebSocket
  ▼
teleop_app/app.py  (mode=oai-real, --udp-host 192.168.70.135)
  │ UDP JSON commands
  │
  │  ┌────────────────── DOWNLINK ─────────────────┐
  │  ▼                                              │
  │ oai-ext-dn container 192.168.70.135:16000      │
  │   UDP forwarder (port 16000 → 10.0.0.4:16000)  │
  │  │                                              │
  │  ▼                                              │
  │ oai-upf  GTP-U N3                              │
  │  │                                              │
  │  ▼                                              │
  │ gNB → rfsim AIR → UE → oaitun_ue1  ◄── REAL RADIO
  │  │                                              │
  │  ▼                                              │
  │ ue_teleop_bridge   (default ns, :16000)         │
  │   forwards UDP → 127.0.0.1:16100                │
  │   --no-publish-cmd-vel                          │
  │  │                                              │
  │  ▼                                              │
  │ network_aware_broker  → /network/cmd_vel        │
  │  │                                              │
  │  ▼                                              │
  │ nova_carter_bridge   /network/cmd_vel → /cmd_vel│
  │  │                                              │
  │  ▼                                              │
  │ Isaac Sim Nova Carter (mecanum)                 │
  │  ▲                                              │
  │  │                                              │
  │  │  ┌─────────────── UPLINK (camera) ──────────┐│
  │  │  │                                          ││
  │  │  └─ isaac/viewport_to_udp.py                ││
  │  │       (Script Editor, in Isaac's Python)    ││
  │  │       captures "Viewport 2" → JPEG IST5     ││
  │  │       sends UDP → 192.168.70.135:17600      ││
  │  │            │                                  ││
  │  │            ▼                                  ││
  │  │       oai-ext-dn forwarder (17600 → UE)     ││
  │  │            │                                  ││
  │  │            ▼                                  ││
  │  │       UPF → gNB → rfsim AIR → UE  ◄── REAL RADIO
  │  │            │                                  ││
  │  │            ▼                                  ││
  │  │       ue_teleop_bridge :17600 (relay)       ││
  │  │            │ UDP IST5 → 127.0.0.1:17500     ││
  │  │            ▼                                  ││
  │  │       dashboard uplink listener             ││
  │  │            │                                  ││
  │  │            ▼                                  ││
  └──┴──── browser camera panel "Isaac Viewport 2" ──┘
```

### One-time setup

Steps 1–4 from "Step-by-step setup from a fresh clone" above (clone,
venv, build ros2_ws, build OAI). The `ros2_ws` build *is* required for
this mode — it provides the broker stack.

### Recurring launch

You'll need seven things, in order. The first three are your existing
processes; this section's scripts handle the rest.

| # | Where | Who starts it | What it is |
|---|-------|---------------|-------------|
| 1 | docker | you (one-time `docker compose up -d`) | oai-cn5g core: amf/smf/upf/nrf/ausf/udm/udr/mysql/ext-dn |
| 2 | terminal | you | OAI gNB  (`nr-softmodem ... --rfsim ...`) |
| 3 | terminal | you | OAI UE   (`nr-uesoftmodem ... --rfsim ...`) — creates `oaitun_ue1` |
| 4 | terminal | you | `sudo bash scripts/setup_5g_data_plane.sh up` — starts the ext-dn forwarders |
| 5 | Isaac Sim | you (Window → Script Editor → Open `isaac/viewport_to_udp.py` → Ctrl-Enter) | viewport capture loop |
| 6 | terminal | you | `bash scripts/run_oai_teleop_broker.sh` — dashboard + bridge + broker + nova_carter_bridge |
| 7 | browser | you | open <http://localhost:8080> and drive |

Detailed commands for steps 4 and 6:

```bash
# Step 4 — ext-dn UDP forwarders (downlink :16000 and camera uplink :17600)
cd /path/to/IsaacSim5G/IsaacSim5G-master
sudo bash scripts/setup_5g_data_plane.sh up
```

Expected output ends with:

```
[5g] DOWNLINK (commands): dashboard -> 192.168.70.135:16000 -> radio -> UE :16000
[5g] UPLINK   (camera):   Isaac     -> 192.168.70.135:17600 -> radio -> UE :17600
```

Step 5 — in Isaac (you only do this once Isaac is playing the Nova
Carter scene with a viewport named **"Viewport 2"** showing the camera
you want streamed):

```
Window -> Script Editor -> File -> Open
   /path/to/IsaacSim5G/IsaacSim5G-master/isaac/viewport_to_udp.py
Ctrl-Enter to run
```

Isaac's console should print:

```
[vp2] using viewport 'Viewport 2'
[vp2] capture -> 192.168.70.135:17600 as /isaac/viewport2, ~8 fps
[vp2] sent=N errors=0
```

Falls back to the active viewport if "Viewport 2" doesn't exist. Stop
later with `STOP_VIEWPORT_CAPTURE()` in the Script Editor.

```bash
# Step 6 — the local chain pipeline
cd /path/to/IsaacSim5G/IsaacSim5G-master
UE_IP=192.168.70.135 \
OPERATOR_IP=127.0.0.1 \
BROKER_UDP_PORT=16100 \
CAMERA_UPLINK_LISTEN_PORT=17600 \
PLATFORM=nova_carter \
TELEOP_PORT=8080 \
CAMERA_FPS_LIMIT=30 \
    bash scripts/run_oai_teleop_broker.sh
```

`UE_IP=192.168.70.135` is the key change vs. the earlier "broker
chained" recipe — the dashboard now sends downlink to the ext-dn
container instead of directly to `10.0.0.4`. The ext-dn forwarder
relays it through the UPF/radio to the UE.

### Verifying the chain end-to-end

```bash
# downlink: send a probe through ext-dn, watch the bridge log
python3 -c "
import socket, json, time
s = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
for _ in range(12):
    s.sendto(json.dumps({'linear_x':0.3,'angular_z':0.1}).encode(),
             ('192.168.70.135', 16000))
    time.sleep(0.05)
"
tail -f .run_logs/ue_bridge.log
# expect: forward: cmds=N bytes=M ticking up. Means the packets crossed
# the radio (ext-dn → UPF → gNB → UE → bridge).

# camera uplink: nothing local to inject — just press Ctrl-Enter on the
# Isaac script and watch the bridge log:
#     stats ... uplink: ... camera=N ticking up
# Means Isaac's frames are being relayed via the bridge's :17600
# listener, after crossing the radio in the other direction.
```

### Keyboard controls (mecanum)

| Key | Action |
|-----|--------|
| W / S | linear_x: forward / reverse |
| A / D | linear_y: **strafe left / right** |
| Q / E | angular_z: rotate left / right |
| Space | E-stop (toggle) |

A/D produce strafe (`linear_y`) on the Nova Carter, not yaw — the
robot is mecanum and the dashboard's ground-platform keymap was
rewired to match. Q/E produce in-place rotation.

### Stop everything

```bash
cd /path/to/IsaacSim5G/IsaacSim5G-master
bash scripts/stop.sh                            # dashboard + bridge + broker + nova_carter_bridge
sudo bash scripts/setup_5g_data_plane.sh down   # ext-dn forwarders
# In Isaac Script Editor: STOP_VIEWPORT_CAPTURE()
```

`stop.sh` does not touch OAI gNB / UE / core / Isaac — you started
those, you stop them.

### Architecture notes

- **Radio crossing is via ext-dn, not netns.** A purer "uplink through
  5G" would put the dashboard in a network namespace whose only
  ingress is via the UPF. That's a bigger change with DDS-discovery
  implications and isn't done here. The current path uses ext-dn as
  the data-network ingress, which is the standard 5G data-plane
  pattern (external host → UPF → radio → UE).
- **The "5G downlink" probe earlier (sending UDP directly to
  10.0.0.4)** was short-circuited to loopback by Linux's local table.
  This recipe avoids that by routing through ext-dn.
- **Single panel by default.** `platforms.py` ships one
  Viewport-2-bound camera panel. Switch via the panel dropdown if you
  want to test ROS-side camera topics instead.

---

## Verifying the pipeline

Quick sanity checks if something looks wrong:

```bash
# OAI processes alive?
ps -ef | grep -E "nr-soft|nr-uesoft" | grep -v grep

# UE tunnel up?
ip -4 addr show oaitun_ue1

# Dashboard / bridge listeners up?
ss -tlnp | grep -E ":8080|:17500"
ss -ulnp | grep -E ":16000|:17500"

# Isaac side actually publishing?
source /opt/ros/humble/setup.bash
ros2 topic hz /chassis/odom
```

If `cmds=0` after pressing keys, the dashboard isn't reaching the UE
tunnel — re-check `UE_IP` matches `oaitun_ue1`'s address.

If `pose=0`, the bridge isn't seeing Isaac's odom — check
`ROS_DOMAIN_ID` matches between Isaac and your shells (default 0).

---

## Modes at a glance

| mode | command path | launcher | who do you need? |
|------|--------------|----------|-------------------|
| `direct` | dashboard publishes `/cmd_vel` itself | `scripts/run_teleop.sh` | Isaac only |
| `oai` | UDP → C++ broker (`network_aware_broker`) → ROS | `scripts/run_teleop_oai.sh` | Isaac + broker (simulated 5G) |
| `oai-real` | UDP → UE TUN (loopback in single-host RFsim, NOT actual radio) → `/cmd_vel` | `scripts/run_oai_teleop.sh` | Isaac + OAI gNB + UE + 5G core |
| `oai-real` + chain | radio-bypassed (same caveat) → bridge **forwards** UDP → broker → `nova_carter_bridge` → `/cmd_vel` | `scripts/run_oai_teleop_broker.sh` | Isaac + OAI gNB + UE + 5G core + broker stack |
| **`oai-real` + chain + ext-dn (recommended)** | UDP → **ext-dn → UPF → gNB → rfsim AIR → UE** → bridge → broker → `/cmd_vel`; Isaac viewport → ext-dn → radio → UE → bridge → dashboard | `scripts/setup_5g_data_plane.sh` + `scripts/run_oai_teleop_broker.sh` + `isaac/viewport_to_udp.py` | Isaac + OAI gNB + UE + 5G core + broker + ext-dn forwarder + Isaac viewport capture |
| **`oai-urllc` (theoretically-correct UL + URLLC QoS)** | Commands: same as above (PDSCH via ext-dn). **Camera + pose UL: Isaac → robot-ns proxy → oaitun_ue1 → PUSCH → gNB → UPF → operator** (the real UL, not loopback). All flows DSCP-marked for 5QI mapping. New `/plotter` topic emits `sim_time, rtf, x/y/z, vx/vy/vz, heading`. | `scripts/run_oai_teleop_urllc.sh` (calls `setup_5g_data_plane.sh` + `setup_robot_netns.sh` internally) | Same as above + root for netns + `isaac/plotter_publisher.py` |

Both broker-using modes (`oai` and the chain) launch the `ros2_ws`
broker stack via:

```bash
source /opt/ros/humble/setup.bash
source ros2_ws/install/setup.bash
ros2 launch network_aware_broker network_aware_stack.launch.py
# chain mode overrides the broker's ingress port:
BROKER_UDP_PORT=16100 ros2 launch network_aware_broker network_aware_stack.launch.py
```

Plain `mode=oai-real` runs `app.py` and `ue_teleop_bridge.py` as plain
`rclpy` Python scripts — no `ros2 launch` involved.

---

## Architecture

```
┌──────────────────── operator host ─────────────────────┐
│                                                        │
│   browser ── ws ──►  teleop_app/app.py                 │
│                       (oai-real mode)                  │
│                          │                             │
│                          │ UDP JSON   ┌─────────────┐  │
│                          └──────────►│  5G core /  │  │
│                       ◄──────────────│  UPF        │  │
│                        UDP IST5      └─────┬───────┘  │
│                                            │          │
└────────────────────────────────────────────┼──────────┘
                                             │
                                       gNB↔UE rfsim
                                             │
┌──────────────────── UE / robot host ───────┼──────────┐
│                                            │          │
│  oai_bridge/ue_teleop_bridge.py            │          │
│    │                              ◄────────┘          │
│    │ rclpy publish /cmd_vel                           │
│    │ rclpy subscribe                                  │
│    │     /odom, /chassis/odom                         │
│    │     /Nova_Carter_ROS/front_hawk/{left,right}     │
│    ▼                                                  │
│  Isaac Sim Nova Carter                                │
│                                                       │
└───────────────────────────────────────────────────────┘
```

In single-machine RFsim the "operator host" and "UE host" boxes are the
same machine. The dashboard sends UDP to the UE's 5G IP and Linux routes
it through the OAI data plane.

---

## Repository layout

The repo is expected to sit on disk like this (the dashboard scripts assume
their parent is the **outer** wrapper that also contains `oai_rfsim_rtf/`):

```
IsaacSim5G/
├── IsaacSim5G-master/                  # this README's home; dashboard + bridge + scripts + ros2_ws
└── oai_rfsim_rtf/
    └── fixed_rtf_rfsim-develop/        # OAI source tree + conf files (gNB / UE)
```

Inside `IsaacSim5G-master/`:

```
IsaacSim5G-master/
├── README.md                           # this file
├── summary.txt                         # one-pager: running processes + what crosses 5G
├── teleop_app/                         # FastAPI + rclpy operator dashboard
│   ├── app.py                          # WebSocket ↔ UDP ↔ ROS; modes: direct / oai / oai-real
│   ├── platforms.py                    # per-platform key bindings + camera panel routing
│   ├── requirements.txt                # uvicorn, fastapi, opencv-python, pillow, websockets, …
│   ├── .venv/                          # auto-created by step 2 of setup (gitignored)
│   └── static/
│       ├── index.html                  # dashboard markup
│       ├── dashboard.css
│       └── dashboard.js                # WebSocket client, key handling, frame rendering
│
├── oai_bridge/                         # UE-side UDP ↔ ROS bridge (rclpy)
│   ├── ue_teleop_bridge.py             # listens on UE TUN, forwards / publishes
│   ├── protocol.py                     # shared IST5 wire format (downlink + uplink envelopes)
│   └── README.md
│
├── isaac/                              # Isaac-side helper scripts (run in Isaac's Python)
│   ├── viewport_to_udp.py              # Viewport 2 → JPEG → IST5 UDP capture loop
│   ├── nova_carter_bridge.py           # /network/cmd_vel → /cmd_vel (30 Hz, 500 ms watchdog)
│   ├── enable_camera_graph.py          # forces the ROS camera publishers on
│   ├── fix_and_start_camera.py         # one-shot fixer for missing image publishers
│   ├── dynamic_obstacle_bridge.py      # spawns obstacles via ROS service
│   └── README.md
│
├── ros2_ws/                            # C++ network-aware broker workspace
│   ├── src/
│   │   ├── network_aware_msgs/         # ROS 2 message package
│   │   │   ├── msg/{TeleopCommand,NetworkMetrics,RobotTelemetry,ClockState}.msg
│   │   │   ├── CMakeLists.txt
│   │   │   └── package.xml
│   │   └── network_aware_broker/       # nodes + launch + params
│   │       ├── src/
│   │       │   ├── ue_ingress_node.cpp     # UDP :BROKER_UDP_PORT → /network/teleop_command
│   │       │   ├── metrics_ingress_node.cpp# UDP :16001 → /network/metrics
│   │       │   ├── broker_main.cpp / broker_node.cpp
│   │       │   ├── metrics_estimator.cpp / telemetry_fuser.cpp / pose_cache.cpp
│   │       │   ├── time_broker.cpp / rtf_clock_node.cpp
│   │       ├── launch/network_aware_stack.launch.py
│   │       ├── config/broker_params.yaml
│   │       ├── CMakeLists.txt
│   │       └── package.xml
│   ├── build/  install/  log/                  # colcon outputs (gitignored)
│
├── integration/                        # simulated-broker glue (mode=oai, no radio)
│   ├── oai_udp_bridge.py
│   ├── replay_csv.py
│   └── sample_metrics_sender.py
│
├── scripts/
│   ├── build_ros2_ws.sh                # colcon build wrapper (--clean / --debug)
│   ├── run_teleop.sh                   # mode=direct (Isaac only, no radio, no broker)
│   ├── run_drone.sh                    # iris_drone (PX4 offboard) variant
│   ├── run_teleop_oai.sh               # mode=oai — simulated 5G via broker (no radio)
│   ├── run_oai_teleop.sh               # mode=oai-real — radio only, no broker
│   ├── run_oai_teleop_broker.sh        # mode=oai-real + broker chain (the recommended path)
│   ├── setup_5g_data_plane.sh          # ext-dn UDP forwarders for the radio-crossing recipe
│   ├── stop.sh                         # kills dashboard / bridge / broker / nova_carter_bridge
│   ├── publish_demo_frames.py          # diagnostic: synthesize fake camera frames
│   ├── publish_demo_odom.py            # diagnostic: synthesize fake odom
│   ├── isaac_driver.py                 # synthetic /cmd_vel driver for headless smoke tests
│   ├── run_demo.sh                     # end-to-end demo runner
│   └── diagnose_ros.py                 # quick ROS topic + node sanity check
│
├── config/
│   └── obstacles.example.json
│
├── docs/                               # design / architecture notes
│   ├── ARCHITECTURE.md
│   ├── BUILD_AND_RUN.md
│   ├── DANCERS_ADAPTATION.md
│   ├── ROSNETSIM_ADAPTATION.md
│   └── VALIDATION.md
│
└── .run_logs/                          # auto-created by launchers — per-process stdout/stderr
```

And in the sibling `oai_rfsim_rtf/fixed_rtf_rfsim-develop/`:

```
fixed_rtf_rfsim-develop/
├── README.md
├── conf/
│   ├── gnb.conf                        # gNB config (use with --rfsim)
│   ├── gnb_v.conf                      # VRTsim variant (optional)
│   └── ue.conf                         # UE config (carries rfsimulator.rtf knob)
├── openairinterface5g/                 # OAI source tree
│   └── cmake_targets/ran_build/build/  # built binaries land here:
│       ├── nr-softmodem                # OAI gNB
│       └── nr-uesoftmodem              # OAI UE (creates oaitun_ue1)
└── time_bridge/                        # auxiliary RTF time-bridge utility
```

The 5G core itself (`oai-cn5g`: amf / smf / upf / nrf / ausf / udm / udr /
mysql / ext-dn / ims) lives outside this repo as a separate Docker compose
deployment — install it however you like (the official `oai-cn5g-fed`
compose stack works out of the box).

---

## Configuration knobs (`mode=oai-real`)

| env / flag | default | meaning |
|------------|---------|---------|
| `UE_IP` / `--udp-host` | `12.1.1.2` (script default); shipped conf yields `10.0.0.4` | UE's 5G IP allocated by the core's UPF |
| `UE_PORT` / `--udp-port` | `16000` | UE-side UDP listener |
| `OPERATOR_IP` (bridge) | `127.0.0.1` | dashboard's external IP (uplink target) |
| `OPERATOR_PORT` / `--operator-udp-port` | `17500` | dashboard uplink listener |
| `PLATFORM` | `nova_carter` | also selects camera panel bindings |
| `TELEOP_PORT` | `8080` | dashboard HTTP / WebSocket port |
| `CAMERA_TOPICS` (bridge) | front_hawk left+right | which ROS topics the bridge forwards |
| `ODOM_TOPICS` (bridge) | `/odom,/chassis/odom` | which odom topics the bridge forwards |
| `CAMERA_FPS_LIMIT` (bridge) | `10` | per-topic uplink rate cap |
| `ROS_SETUP` | `/opt/ros/humble/setup.bash` | path to ROS 2 setup if not in default location |

The `UE_IP` default in the launcher is `12.1.1.2`, but your actual UE IP
depends on the 5G core / UPF config — always cross-check with
`ip -4 addr show oaitun_ue1`.

### Extra knobs for the broker-chain mode

Additional flags/env used by `scripts/run_oai_teleop_broker.sh` on top
of the table above:

| env / flag | default | meaning |
|------------|---------|---------|
| `BROKER_UDP_PORT` | `16100` (chain launcher); `16000` (broker launch file fallback) | UDP port the broker's `ue_ingress_node` binds — must differ from `UE_PORT` so it doesn't collide with the UE bridge |
| `--forward-udp-host` / `FORWARD_UDP_HOST` (bridge) | unset | If set, UE bridge re-emits each downlink datagram to this host. Used to feed the broker post-radio |
| `--forward-udp-port` / `FORWARD_UDP_PORT` (bridge) | `0` (disabled) | Companion port for `--forward-udp-host` |
| `--no-publish-cmd-vel` / `NO_PUBLISH_CMD_VEL=1` (bridge) | off | Suppress the UE bridge's `/cmd_vel` publisher (chain mode sets this so `nova_carter_bridge` is the only publisher) |
| `--camera-uplink-listen-port` / `CAMERA_UPLINK_LISTEN_PORT` (bridge) | `0` (disabled); `17600` in `run_oai_teleop_broker.sh` | UDP port the bridge ingests already-IST5-framed camera packets that arrived over the radio. Relayed to the dashboard uplink as-is |

### Extra knobs for the ext-dn / real-radio variant

Used by `scripts/setup_5g_data_plane.sh` and the recommended recipe:

| env / flag | default | meaning |
|------------|---------|---------|
| `FWD_LISTEN_PORT` | `16000` | Downlink UDP port the forwarder binds inside `oai-ext-dn` |
| `FWD_UE_IP` | **auto-detected from `oaitun_ue1`**; falls back to `10.0.0.4` if the TUN can't be read | UE pool address the forwarder relays to. The script reads `oaitun_ue1`'s address at launch so it tracks whatever the SMF actually allocated (e.g. `10.0.0.2` vs. `10.0.0.4`). Override only if you have a reason to. |
| `FWD_UE_PORT` | `16000` | UE-side port |
| `FWD_CAM_LISTEN_PORT` | `17600` | Camera-uplink UDP port the forwarder binds inside `oai-ext-dn` |
| `FWD_CAM_UE_PORT` | `17600` | UE-side camera-uplink port (matches `CAMERA_UPLINK_LISTEN_PORT`) |
| `FWD_CONTAINER` | `oai-ext-dn` | Docker container that hosts the forwarders (must have python3 and a route to the UE pool via the UPF) |
| `UE_IP` (launcher) | `12.1.1.2` script default; **set to `192.168.70.135` for the ext-dn recipe** | Where the dashboard sends downlink |

### Isaac Viewport 2 capture knobs

Edit these at the top of `isaac/viewport_to_udp.py` before running it
in Isaac's Script Editor:

| constant | default | meaning |
|----------|---------|---------|
| `TARGET_VIEWPORT_NAME` | `"Viewport 2"` | Window name to capture; falls back to active viewport if missing |
| `TARGET_HOST` | `192.168.70.135` | oai-ext-dn ingress IP (the camera forwarder listens here) |
| `TARGET_PORT` | `17600` | Must match `FWD_CAM_LISTEN_PORT` and `CAMERA_UPLINK_LISTEN_PORT` |
| `TOPIC` | `/isaac/viewport2` | Synthetic topic name used by the dashboard's panel routing |
| `FPS` | `8` | Capture rate (file-based; ~10 fps is the practical ceiling) |
| `JPEG_QUALITY` | `60` | Starting PIL JPEG quality. The encoder steps down (in increments of 10, floor 20) until the frame fits under `MAX_UDP_PAYLOAD`. |
| `MAX_FRAME_DIM` | `960` | Longest-side pixel cap applied before JPEG encode. Keeps the encoded body well under the UDP datagram ceiling. |
| `MAX_UDP_PAYLOAD` | `60000` | Hard cap on the JPEG body (bytes). Frames that can't be squeezed under this are dropped rather than triggering `OSError: [Errno 90] Message too long`. |
| `FILE_READY_TIMEOUT` | `0.5` | Max seconds to wait for `capture_viewport_to_file` to finish writing the PNG (the render thread writes async). |
| `FILE_READY_POLL` | `0.015` | Stat poll interval while waiting for the PNG to stabilise. |

---

## Stop everything

```bash
bash scripts/stop.sh                            # dashboard + bridge + broker + nova_carter_bridge
sudo bash scripts/setup_5g_data_plane.sh down   # ext-dn UDP forwarders (only if you ran 'up')
```

In Isaac's Script Editor:

```
STOP_VIEWPORT_CAPTURE()
```

`stop.sh` and `setup_5g_data_plane.sh down` leave OAI gNB / UE / core
and Isaac itself alone — you started those, you stop them.

## License

MIT.
