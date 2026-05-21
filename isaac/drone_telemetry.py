# Isaac Sim drone telemetry sidecar.
#
# Runs INSIDE Isaac Sim's Script Editor. Does two jobs in one script so
# you only have to Ctrl-Enter once:
#
#   1. Captures the FPV viewport ("Viewport 2" or active) at ~8 FPS,
#      JPEG-encodes it, wraps it in the IST5 envelope used by the
#      teleop dashboard, and ships it via UDP to the dashboard.
#
#   2. Reads the drone body's world transform every render step,
#      computes velocity by finite difference, derives heading (yaw)
#      from the rotation matrix, and publishes a ZMQ message in the
#      same struct layout that sim_isaac_trajectory.py produces:
#
#          MSG_FORMAT = "<ddddddddd"
#          (sim_time, rtf, x, y, z, vx, vy, vz, heading)
#
#      That makes this script a drop-in replacement for the "dummy
#      trajectory" source used by main_paths_los_model_measurment_position.py
#      — every downstream consumer (channel bridge, live plotter, OAI
#      L1) sees the same wire format, but the numbers are now coming
#      from the real Iris drone in the running city scene.
#
# How to run inside Isaac:
#
#   Window → Script Editor → File → Open this file.
#   Ctrl-Enter.
#
# To stop the loops:
#
#   STOP_DRONE_TELEMETRY()   in the Script Editor.
#
# Environment knobs (read once when the script is Ctrl-Entered):
#
#   ISAAC_UPLINK_HOST    UDP dest for the viewport JPEG. Default
#                        127.0.0.1 for the local dashboard; set to the
#                        oai-ext-dn ingress (192.168.70.135) or the
#                        robot-ns proxy IP (192.168.244.2) to push the
#                        camera through the 5G radio.
#   ISAAC_UPLINK_PORT    UDP dest port (default 17500 — the dashboard's
#                        operator_udp_port).
#   ISAAC_POSE_PUB_ADDR  ZMQ PUB bind address for the drone pose
#                        (default tcp://127.0.0.1:5570). Read by
#                        sim_isaac_trajectory.py --trajectory isaac.
#   ISAAC_POSE_RATE_HZ   Rate the pose loop wakes up to read the
#                        drone transform (default 100 Hz).

import os, io, time, json, math, struct, socket, threading, traceback

# ---- Config ----------------------------------------------------------------
TARGET_VIEWPORT_NAME = "Viewport 2"     # falls back to active viewport if missing
TARGET_HOST          = os.environ.get("ISAAC_UPLINK_HOST", "127.0.0.1")
TARGET_PORT          = int(os.environ.get("ISAAC_UPLINK_PORT", "17500"))
TOPIC                = "/isaac/viewport2"
FPS                  = 8
JPEG_QUALITY         = 60
TMP_PNG              = "/dev/shm/isaac_vp2.png"
MAX_FRAME_DIM        = 960
MAX_UDP_PAYLOAD      = 60000
FILE_READY_TIMEOUT   = 0.5
FILE_READY_POLL      = 0.015
DSCP_AF41            = 0x88

DRONE_BODY_PRIM      = os.environ.get("ISAAC_DRONE_BODY_PRIM", "/World/quadrotor/body")
POSE_PUB_ADDR        = os.environ.get("ISAAC_POSE_PUB_ADDR", "tcp://127.0.0.1:5570")
POSE_RATE_HZ         = float(os.environ.get("ISAAC_POSE_RATE_HZ", "100"))
# Optional second PUB that publishes pre-packed MSG_FORMAT directly on the
# port main_paths_los_model_measurment_position.py listens to. When set,
# sim_isaac_trajectory.py is not required in the pipeline — the channel
# bridge can subscribe to this address directly. Leave empty to keep the
# proxy in the loop.
POSE_DIRECT_ADDR     = os.environ.get("ISAAC_POSE_DIRECT_ADDR", "")
# Filter time constant on velocity finite differences (seconds). Drone
# body transform jitters between physics ticks; a small LPF keeps the
# numbers usable as ground truth without latency.
VEL_LPF_TAU_S        = float(os.environ.get("ISAAC_VEL_LPF_TAU", "0.05"))
# ---------------------------------------------------------------------------

# IST5 wire format (mirrors oai_bridge/protocol.py).
UPLINK_MAGIC   = 0x49535435  # 'IST5'
KIND_CAMERA    = 2
UPLINK_HDR_FMT = "<IBBHI"

# Ground-truth pose wire format (mirrors sim_isaac_trajectory.py).
POSE_MSG_FORMAT = "<ddddddddd"


def _pack_ist5(kind, header_dict, body):
    hdr = json.dumps(header_dict, separators=(",", ":")).encode("utf-8")
    return (struct.pack(UPLINK_HDR_FMT, UPLINK_MAGIC, kind, 0, len(hdr), len(body))
            + hdr + body)


# ============================================================================
# Viewport capture loop (lifted from isaac/viewport_to_udp.py).
# ============================================================================
def _wait_for_png(png_path, timeout=FILE_READY_TIMEOUT, poll=FILE_READY_POLL):
    deadline = time.monotonic() + timeout
    last_size = -1
    stable = 0
    while time.monotonic() < deadline:
        try:
            st = os.stat(png_path)
        except FileNotFoundError:
            time.sleep(poll); continue
        if st.st_size > 0 and st.st_size == last_size:
            stable += 1
            if stable >= 2:
                return True
        else:
            stable = 0
            last_size = st.st_size
        time.sleep(poll)
    return False


def _encode_jpeg_under_limit(png_path, base_quality, max_dim, max_bytes):
    from PIL import Image
    with open(png_path, "rb") as f:
        png_bytes = f.read()
    img = Image.open(io.BytesIO(png_bytes))
    img.load()
    img = img.convert("RGB")
    w, h = img.size
    if max(w, h) > max_dim:
        scale = max_dim / float(max(w, h))
        img = img.resize((max(1, int(w * scale)), max(1, int(h * scale))),
                         Image.BILINEAR)
        w, h = img.size
    quality = base_quality
    while True:
        buf = io.BytesIO()
        img.save(buf, format="JPEG", quality=quality, optimize=False)
        data = buf.getvalue()
        if len(data) <= max_bytes or quality <= 20:
            return data, w, h, len(data) <= max_bytes
        quality -= 10


def _get_viewport():
    from omni.kit.viewport.utility import (
        get_viewport_from_window_name, get_active_viewport,
    )
    try:
        vp = get_viewport_from_window_name(TARGET_VIEWPORT_NAME)
        if vp is not None:
            print(f"[telemetry] viewport: '{TARGET_VIEWPORT_NAME}'")
            return vp
    except Exception:
        pass
    vp = get_active_viewport()
    print(f"[telemetry] viewport: '{TARGET_VIEWPORT_NAME}' not found; active fallback")
    return vp


class _ViewportLoop:
    def __init__(self):
        self._stop = threading.Event()
        self._thread = None
        self._sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
        try:
            self._sock.setsockopt(socket.IPPROTO_IP, socket.IP_TOS, DSCP_AF41)
        except OSError:
            pass
        self._vp = None
        self.frames_sent = 0
        self.errors = 0

    def start(self):
        if self._thread is not None and self._thread.is_alive():
            return
        self._vp = _get_viewport()
        if self._vp is None:
            print("[telemetry] no viewport; camera disabled")
            return
        self._stop.clear()
        self._thread = threading.Thread(target=self._run, name="vp-capture", daemon=True)
        self._thread.start()
        print(f"[telemetry] camera -> {TARGET_HOST}:{TARGET_PORT} as {TOPIC}, ~{FPS} fps")

    def stop(self):
        self._stop.set()

    def _run(self):
        import asyncio
        try:
            asyncio.get_event_loop()
        except RuntimeError:
            asyncio.set_event_loop(asyncio.new_event_loop())
        from omni.kit.viewport.utility import capture_viewport_to_file
        period = 1.0 / max(1.0, float(FPS))
        last_log = time.monotonic()
        while not self._stop.is_set():
            t0 = time.monotonic()
            try:
                try: os.unlink(TMP_PNG)
                except FileNotFoundError: pass
                capture_viewport_to_file(self._vp, TMP_PNG)
                if not _wait_for_png(TMP_PNG):
                    self.errors += 1
                    continue
                body, w, h, ok = _encode_jpeg_under_limit(
                    TMP_PNG, JPEG_QUALITY, MAX_FRAME_DIM, MAX_UDP_PAYLOAD)
                if not ok:
                    self.errors += 1
                    continue
                header = {
                    "topic":  TOPIC,
                    "base":   TOPIC,
                    "stamp":  time.time(),
                    "width":  int(w),
                    "height": int(h),
                    "fmt":    "jpeg",
                }
                pkt = _pack_ist5(KIND_CAMERA, header, body)
                self._sock.sendto(pkt, (TARGET_HOST, TARGET_PORT))
                self.frames_sent += 1
            except Exception:
                self.errors += 1
                if self.errors <= 5:
                    traceback.print_exc()
            now = time.monotonic()
            if now - last_log >= 5.0:
                print(f"[telemetry] camera sent={self.frames_sent} err={self.errors}")
                last_log = now
            sleep_for = period - (time.monotonic() - t0)
            if sleep_for > 0:
                time.sleep(sleep_for)
        print("[telemetry] camera loop stopped")


# ============================================================================
# Pose publisher loop.
# ============================================================================
def _yaw_from_matrix(m):
    """USD's ComputeLocalToWorldTransform returns a row-vector Matrix4d
    where rows 0..2 are the world-frame basis vectors of the body's
    local X, Y, Z axes (last row is translation).
    Heading is the yaw of the body's local +X axis projected onto the
    world XY plane."""
    fx, fy = float(m[0][0]), float(m[0][1])
    return math.atan2(fy, fx)


class _PoseLoop:
    """Reads the drone body's world transform at POSE_RATE_HZ and
    publishes a sim_isaac_trajectory-compatible message via ZMQ."""
    def __init__(self):
        self._stop = threading.Event()
        self._thread = None
        self._proxy_pub = None    # PUB on POSE_PUB_ADDR (raw, for the proxy)
        self._direct_pub = None   # optional PUB in MSG_FORMAT
        self._ctx = None
        self.samples_sent = 0
        self.errors = 0

    def start(self):
        try:
            import zmq
        except ImportError:
            print("[telemetry] pyzmq NOT installed in Isaac's Python; pose disabled.\n"
                  "            install: /home/varun/isaacsim/python.sh -m pip install pyzmq")
            return
        self._ctx = zmq.Context.instance()
        try:
            self._proxy_pub = self._ctx.socket(zmq.PUB)
            self._proxy_pub.bind(POSE_PUB_ADDR)
        except Exception as exc:
            print(f"[telemetry] could not bind {POSE_PUB_ADDR}: {exc}")
            return
        if POSE_DIRECT_ADDR:
            try:
                self._direct_pub = self._ctx.socket(zmq.PUB)
                self._direct_pub.bind(POSE_DIRECT_ADDR)
                print(f"[telemetry] pose direct -> {POSE_DIRECT_ADDR} (MSG_FORMAT)")
            except Exception as exc:
                print(f"[telemetry] direct bind {POSE_DIRECT_ADDR} failed: {exc}")
                self._direct_pub = None
        print(f"[telemetry] pose proxy -> {POSE_PUB_ADDR} (raw JSON) @ {POSE_RATE_HZ:.0f} Hz")

        self._stop.clear()
        self._thread = threading.Thread(target=self._run, name="pose-publisher", daemon=True)
        self._thread.start()

    def stop(self):
        self._stop.set()

    def _run(self):
        # Lazy imports — only valid inside Isaac's Python.
        from pxr import Usd, UsdGeom
        import omni.usd

        stage = omni.usd.get_context().get_stage()
        if stage is None:
            print("[telemetry] no USD stage; pose loop exiting")
            return

        body = stage.GetPrimAtPath(DRONE_BODY_PRIM)
        if not body or not body.IsValid():
            print(f"[telemetry] {DRONE_BODY_PRIM} not found; pose loop exiting")
            return

        # State for finite-difference velocity + LPF.
        last_t = None
        last_x = last_y = last_z = None
        f_vx = f_vy = f_vz = 0.0
        sim_t0 = time.monotonic()
        last_log = sim_t0
        period = 1.0 / max(1.0, POSE_RATE_HZ)
        rtf_window = []   # (sim_time, wall_time) for slope

        while not self._stop.is_set():
            wall_t = time.monotonic()
            try:
                world = UsdGeom.Xformable(body).ComputeLocalToWorldTransform(
                    Usd.TimeCode.Default())
                tr = world.ExtractTranslation()
                x, y, z = float(tr[0]), float(tr[1]), float(tr[2])
                heading = _yaw_from_matrix(world)

                if last_t is None:
                    vx = vy = vz = 0.0
                else:
                    dt = max(1e-3, wall_t - last_t)
                    raw_vx = (x - last_x) / dt
                    raw_vy = (y - last_y) / dt
                    raw_vz = (z - last_z) / dt
                    # Exponential moving average toward the raw FD value.
                    a = dt / max(dt, VEL_LPF_TAU_S)
                    f_vx += a * (raw_vx - f_vx)
                    f_vy += a * (raw_vy - f_vy)
                    f_vz += a * (raw_vz - f_vz)
                    vx, vy, vz = f_vx, f_vy, f_vz
                last_t = wall_t
                last_x, last_y, last_z = x, y, z

                sim_time = wall_t - sim_t0
                rtf_window.append((sim_time, wall_t))
                if len(rtf_window) > 200:
                    rtf_window.pop(0)
                if len(rtf_window) >= 2:
                    ds = rtf_window[-1][0] - rtf_window[0][0]
                    dw = rtf_window[-1][1] - rtf_window[0][1]
                    rtf = (ds / dw) if dw > 0 else 1.0
                else:
                    rtf = 1.0

                payload = {
                    "sim_time": sim_time,
                    "rtf": rtf,
                    "x": x, "y": y, "z": z,
                    "vx": vx, "vy": vy, "vz": vz,
                    "heading": heading,
                }
                # Proxy channel — JSON, sim_isaac_trajectory.py decodes.
                if self._proxy_pub is not None:
                    self._proxy_pub.send_json(payload)
                # Direct channel — already in the channel-bridge wire format.
                if self._direct_pub is not None:
                    self._direct_pub.send(struct.pack(
                        POSE_MSG_FORMAT,
                        sim_time, rtf, x, y, z, vx, vy, vz, heading))
                self.samples_sent += 1
            except Exception:
                self.errors += 1
                if self.errors <= 5:
                    traceback.print_exc()

            if wall_t - last_log >= 5.0:
                print(f"[telemetry] pose sent={self.samples_sent} "
                      f"err={self.errors} pos=({x:.2f},{y:.2f},{z:.2f}) "
                      f"v=({f_vx:.2f},{f_vy:.2f},{f_vz:.2f}) "
                      f"hdg={math.degrees(heading):.1f}°")
                last_log = wall_t

            sleep_for = period - (time.monotonic() - wall_t)
            if sleep_for > 0:
                time.sleep(sleep_for)
        try:
            if self._proxy_pub is not None: self._proxy_pub.close(0)
            if self._direct_pub is not None: self._direct_pub.close(0)
        except Exception:
            pass
        print("[telemetry] pose loop stopped")


# ============================================================================
# Public entry points (Script Editor re-runs this file idempotently).
# ============================================================================
_RUNNING = globals().get("_RUNNING", None)
if _RUNNING is not None:
    try:
        _RUNNING["vp"].stop()
        _RUNNING["pose"].stop()
    except Exception:
        pass

_vp_loop = _ViewportLoop()
_pose_loop = _PoseLoop()
_vp_loop.start()
_pose_loop.start()
_RUNNING = {"vp": _vp_loop, "pose": _pose_loop}


def STOP_DRONE_TELEMETRY():
    """Call from the Script Editor to stop both loops."""
    global _RUNNING
    if _RUNNING is not None:
        try: _RUNNING["vp"].stop()
        except Exception: pass
        try: _RUNNING["pose"].stop()
        except Exception: pass
        print("[telemetry] stop requested")
