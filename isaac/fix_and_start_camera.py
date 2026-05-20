# Fix replicator orchestrator dtype spam + start dual front_hawk camera
# publishers (LEFT and RIGHT) as real ROS 2 topics.
#
# Approach:
#   1. Quiet the Replicator orchestrator (which spams dtype warnings every
#      tick on some Isaac builds).
#   2. Locate the left + right front_hawk Camera prims by walking the USD
#      stage.
#   3. Read each camera with omni.isaac.sensor.Camera (Isaac's Python 3.11).
#   4. Spawn ONE helper subprocess running system Python 3.10 with the real
#      rclpy from /opt/ros/humble. The helper opens N pairs of TCP sockets
#      (one raw + one compressed per camera) and publishes the bytes it
#      receives on the configured ROS 2 topics.
#
# This sidesteps the rclpy Python-version-mismatch issue completely: Isaac
# uses its bundled Python for capture, the helper uses the system Python
# for ROS 2.
#
# After running, both panels in the dashboard will fill within ~2 s
# because they subscribe to the two ROS 2 topics this script publishes.
#
# How to use:
#   1. Make sure the dashboard is running: bash scripts/run_teleop.sh
#   2. In Isaac Sim, press Play.
#   3. Window -> Script Editor -> File -> Open this file.
#   4. Ctrl-Enter to Run.
#
# To stop: call STOP_CAMERAS() in the Script Editor and run again.

# ---- Config -----------------------------------------------------------------
# One (ros_topic, prim-path-fragments) pair per camera. The fragments are
# lowercase substrings the Camera prim's path must contain. Edit if your
# scene has different camera prim names.
SOURCES = [
    ("/Nova_Carter_ROS/front_hawk/left/image_raw",  ["front_hawk", "left"]),
    ("/Nova_Carter_ROS/front_hawk/right/image_raw", ["front_hawk", "right"]),
]
CAMERA_HZ    = 15.0
RESOLUTION   = (640, 480)
JPEG_QUALITY = 70
ROS_PYTHON   = "/usr/bin/python3"            # system python with rclpy
ROS_SETUP    = "/opt/ros/humble/setup.bash"
HELPER_PORT_BASE = 49300                     # raw_port_i = base + 2*i
# ----------------------------------------------------------------------------

import os
import random
import socket
import struct
import subprocess
import sys
import tempfile
import threading
import time

import carb
import numpy as np
import omni.usd


# ===========================================================================
# Step 1: quiet the Replicator orchestrator (silences dtype warning spam).
# ===========================================================================
def _quiet_replicator():
    print("[fix] quieting replicator orchestrator")
    try:
        import omni.replicator.core as rep
        try:
            rep.orchestrator.stop()
            print("[fix]   orchestrator stopped")
        except Exception as exc:
            print(f"[fix]   orchestrator.stop() raised: {exc}")
    except ImportError:
        print("[fix]   replicator not loaded; nothing to quiet")
    try:
        import omni.replicator.core.scripts.orchestrator as _orch
        if hasattr(_orch, "_orchestrator"):
            o = _orch._orchestrator
            for attr in ("_update_sub", "_timeline_sub"):
                if hasattr(o, attr):
                    setattr(o, attr, None)
                    print(f"[fix]   orchestrator {attr} cleared")
    except Exception as exc:
        print(f"[fix]   patch attempt raised: {exc}")
    try:
        carb.settings.get_settings().set(
            "/omni/replicator/backend/writeThreads", 0)
        print("[fix]   replicator writeThreads = 0")
    except Exception as exc:
        print(f"[fix]   settings patch raised: {exc}")


# ===========================================================================
# Camera prim auto-detection
# ===========================================================================
def _list_camera_prims():
    stage = omni.usd.get_context().get_stage()
    return [str(p.GetPath()) for p in stage.TraverseAll() if p.GetTypeName() == "Camera"]


def _find_camera_prim(fragments):
    """Return the first Camera prim whose path contains every lowercase
    fragment in ``fragments``."""
    needles = [f.lower() for f in fragments]
    for path in _list_camera_prims():
        p = path.lower()
        if all(n in p for n in needles):
            return path
    return None


def _is_playing():
    try:
        import omni.timeline
        return bool(omni.timeline.get_timeline_interface().is_playing())
    except Exception:
        return False


# ===========================================================================
# Frame capture (Isaac Camera with viewport fallback)
# ===========================================================================
def _make_capture(prim_path):
    """Return a callable get_frame() -> RGB uint8 ndarray or None."""
    try:
        from omni.isaac.sensor import Camera
        cam = Camera(prim_path=prim_path, resolution=RESOLUTION, frequency=CAMERA_HZ)
        cam.initialize()
        print(f"[fix]   {prim_path}: using omni.isaac.sensor.Camera")
        def get_frame():
            try:
                rgba = cam.get_rgba()
            except Exception:
                return None
            if rgba is None:
                return None
            arr = np.asarray(rgba)
            if arr.size == 0 or arr.ndim < 3:
                return None
            if arr.dtype != np.uint8:
                arr = (np.clip(arr * 255.0, 0, 255).astype(np.uint8)
                       if arr.dtype.kind == "f" else arr.astype(np.uint8))
            return np.ascontiguousarray(arr[..., :3])
        return get_frame
    except Exception as exc:
        print(f"[fix]   {prim_path}: Isaac Camera failed ({exc}); "
              f"falling back to viewport capture")

    def get_frame_viewport():
        try:
            import omni.kit.viewport.utility as vp_util
            vp = vp_util.get_active_viewport()
            if vp is None:
                return None
            vp.set_active_camera(prim_path)
            tex = vp.get_texture_resource()
            if tex is None:
                return None
            arr = np.array(tex)
            if arr.size == 0:
                return None
            if arr.dtype != np.uint8:
                arr = np.clip(arr * 255, 0, 255).astype(np.uint8)
            if arr.ndim == 3 and arr.shape[2] == 4:
                arr = arr[..., :3]
            return arr
        except Exception:
            return None
    return get_frame_viewport


# ===========================================================================
# JPEG encode
# ===========================================================================
def _encode_jpeg(rgb):
    try:
        import cv2
        bgr = cv2.cvtColor(rgb, cv2.COLOR_RGB2BGR)
        ok, buf = cv2.imencode(".jpg", bgr,
                               [int(cv2.IMWRITE_JPEG_QUALITY), JPEG_QUALITY])
        return buf.tobytes() if ok else None
    except Exception:
        try:
            from PIL import Image as PILImage
            import io
            buf = io.BytesIO()
            PILImage.fromarray(rgb).save(buf, format="JPEG", quality=JPEG_QUALITY)
            return buf.getvalue()
        except Exception:
            return None


# ===========================================================================
# Helper subprocess source. Accepts N (topic, raw_port, cmp_port) triples
# as positional args; opens a TCP listener on each port and publishes
# whatever it receives on the corresponding ROS 2 topic.
# ===========================================================================
_HELPER_SRC = r'''
import sys, socket, struct, threading
import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy
from sensor_msgs.msg import Image as RosImage, CompressedImage

# Args: triples of (topic, raw_port, cmp_port).
triples = []
i = 1
while i + 2 < len(sys.argv) + 1 and i + 2 <= len(sys.argv):
    topic = sys.argv[i]
    raw_port = int(sys.argv[i + 1])
    cmp_port = int(sys.argv[i + 2])
    triples.append((topic, raw_port, cmp_port))
    i += 3

rclpy.init()
node = Node("isaac_hawk_helper")
qos = QoSProfile(reliability=ReliabilityPolicy.BEST_EFFORT,
                 history=HistoryPolicy.KEEP_LAST, depth=5)

publishers = {}
for topic, _rp, _cp in triples:
    publishers[topic] = {
        "raw": node.create_publisher(RosImage, topic, qos),
        "cmp": node.create_publisher(CompressedImage, topic + "/compressed", qos),
        "frame_id": topic.strip("/").replace("/", "_"),
    }
    print(f"[helper] publishing {topic} (+ /compressed)", flush=True)

HDR = "<QqqII"
HDR_SIZE = struct.calcsize(HDR)

def recv_exact(conn, n):
    buf = b""
    while len(buf) < n:
        d = conn.recv(n - len(buf))
        if not d: raise ConnectionResetError
        buf += d
    return buf

def handle(conn, topic, is_raw):
    pubs = publishers[topic]
    while True:
        try:
            hdr = recv_exact(conn, HDR_SIZE)
        except Exception:
            return
        n, ts_sec, ts_nsec, h, w = struct.unpack(HDR, hdr)
        try:
            data = recv_exact(conn, n)
        except Exception:
            return
        stamp = node.get_clock().now().to_msg()
        if ts_sec > 0:
            stamp.sec = ts_sec; stamp.nanosec = ts_nsec
        if is_raw:
            m = RosImage()
            m.header.stamp = stamp; m.header.frame_id = pubs["frame_id"]
            m.height = h; m.width = w
            m.encoding = "rgb8"; m.is_bigendian = 0
            m.step = w * 3; m.data = data
            pubs["raw"].publish(m)
        else:
            m = CompressedImage()
            m.header.stamp = stamp; m.header.frame_id = pubs["frame_id"]
            m.format = "jpeg"; m.data = data
            pubs["cmp"].publish(m)

def accept(port, topic, is_raw):
    s = socket.socket()
    s.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
    s.bind(("127.0.0.1", port))
    s.listen(1)
    while True:
        conn, _ = s.accept()
        threading.Thread(target=handle, args=(conn, topic, is_raw), daemon=True).start()

for topic, rp, cp in triples:
    threading.Thread(target=accept, args=(rp, topic, True),  daemon=True).start()
    threading.Thread(target=accept, args=(cp, topic, False), daemon=True).start()

print("[helper] ready " + " ".join(f"{t}@{rp}/{cp}" for t,rp,cp in triples), flush=True)
while rclpy.ok():
    rclpy.spin_once(node, timeout_sec=0.1)
'''

HDR_FMT  = "<QqqII"
HDR_SIZE = struct.calcsize(HDR_FMT)


def _send_raw(sock, rgb, ts_sec, ts_nsec):
    data = rgb.tobytes()
    h, w = rgb.shape[:2]
    sock.sendall(struct.pack(HDR_FMT, len(data), ts_sec, ts_nsec, h, w) + data)


def _send_cmp(sock, jpg, ts_sec, ts_nsec):
    sock.sendall(struct.pack(HDR_FMT, len(jpg), ts_sec, ts_nsec, 0, 0) + jpg)


# ===========================================================================
# Lifecycle
# ===========================================================================
if "_hawk_state" not in globals():
    _hawk_state = {"running": False, "stop": None, "shutdown": None}


def STOP_CAMERAS():  # noqa: N802
    st = _hawk_state
    if not st["running"]:
        print("[fix] not running")
        return
    if st["stop"]:
        st["stop"].set()
    if st["shutdown"]:
        try: st["shutdown"]()
        except Exception: pass
    st["running"] = False
    print("[fix] camera publishers stopped")


def _wait_helper_ready(proc, timeout=15.0):
    """Drain stdout until the helper prints '[helper] ready' or dies."""
    deadline = time.time() + timeout
    while time.time() < deadline:
        line = proc.stdout.readline()
        if not line:
            if proc.poll() is not None:
                return False
            continue
        text = line.decode(errors="replace").rstrip()
        print(f"  helper: {text}")
        if "[helper] ready" in text:
            return True
    return False


def start_cameras():
    st = _hawk_state
    if st["running"]:
        print("[fix] already running -- call STOP_CAMERAS() first")
        return

    print("=" * 60)
    print(f"[fix] simulation playing? {_is_playing()}")
    cams = _list_camera_prims()
    print(f"[fix] {len(cams)} Camera prim(s) in stage:")
    for c in cams:
        print(f"      {c}")

    # Resolve every SOURCES entry to a real prim path.
    resolved = []
    for topic, fragments in SOURCES:
        prim = _find_camera_prim(fragments)
        if prim is None:
            print(f"[fix] {topic}: no Camera matches {fragments}; skipping")
            continue
        print(f"[fix] {topic}  <-  {prim}")
        resolved.append((topic, fragments, prim))
    if not resolved:
        print("[fix] no usable cameras; aborting")
        return

    # Allocate ports + write helper source.
    base = HELPER_PORT_BASE + 4 * random.randint(0, 50)
    triples = []
    for i, (topic, _frag, _prim) in enumerate(resolved):
        raw_port = base + 2 * i
        cmp_port = base + 2 * i + 1
        triples.append((topic, raw_port, cmp_port))

    tmp = tempfile.NamedTemporaryFile(
        mode="w", suffix=".py", delete=False, prefix="hawk_helper_")
    tmp.write(_HELPER_SRC)
    tmp.close()

    args = [tmp.name]
    for topic, rp, cp in triples:
        args += [topic, str(rp), str(cp)]
    env = os.environ.copy()
    env["PYTHONPATH"] = ""
    cmd = f"source {ROS_SETUP} && {ROS_PYTHON} " + " ".join(args)
    print(f"[fix] launching helper: {cmd}")
    proc = subprocess.Popen(
        ["bash", "-c", cmd], env=env,
        stdout=subprocess.PIPE, stderr=subprocess.STDOUT,
    )
    if not _wait_helper_ready(proc):
        proc.terminate()
        print("[fix] ERROR: helper failed to start; check ROS_PYTHON/ROS_SETUP")
        return

    # Drain stdout in background so the helper doesn't block on a full pipe.
    def _drain():
        try:
            for raw in iter(proc.stdout.readline, b""):
                line = raw.decode(errors="replace").rstrip()
                if line:
                    print(f"  helper: {line}")
        except Exception:
            pass
    threading.Thread(target=_drain, name="helper_drain", daemon=True).start()

    # Connect Isaac-side client sockets for every (topic, raw, cmp) triple.
    socks = {}
    for topic, rp, cp in triples:
        raw_s = socket.socket()
        cmp_s = socket.socket()
        connected = False
        for _ in range(30):
            try:
                raw_s.connect(("127.0.0.1", rp))
                cmp_s.connect(("127.0.0.1", cp))
                connected = True
                break
            except ConnectionRefusedError:
                time.sleep(0.2)
        if not connected:
            print(f"[fix] could not connect to helper sockets for {topic}; aborting")
            proc.terminate()
            return
        socks[topic] = (raw_s, cmp_s)

    # Per-camera capture thread.
    stop_event = threading.Event()
    period = 1.0 / max(1.0, CAMERA_HZ)

    def capture_thread(topic, prim, get_frame, raw_s, cmp_s):
        sent = 0
        empty = 0
        last_tick = 0.0
        last_err = ""
        while not stop_event.is_set():
            now = time.time()
            if now - last_tick < period:
                time.sleep(0.005); continue
            last_tick = now
            rgb = get_frame()
            if rgb is None:
                empty += 1
                if empty in (5, 30, 150):
                    print(f"[fix] {topic}: no frames yet ({empty} ticks)")
                continue
            empty = 0
            ns = time.time_ns()
            ts_sec, ts_nsec = ns // 1_000_000_000, ns % 1_000_000_000
            try:
                _send_raw(raw_s, rgb, ts_sec, ts_nsec)
            except Exception as exc:
                if str(exc) != last_err:
                    print(f"[fix] {topic}: raw send error: {exc}")
                    last_err = str(exc)
                break
            jpg = _encode_jpeg(rgb)
            if jpg:
                try:
                    _send_cmp(cmp_s, jpg, ts_sec, ts_nsec)
                except Exception as exc:
                    if str(exc) != last_err:
                        print(f"[fix] {topic}: cmp send error: {exc}")
                        last_err = str(exc)
                    break
            sent += 1
            if sent in (1, 10, 60) or sent % 300 == 0:
                print(f"[fix] {topic}: {sent} frames sent")
        print(f"[fix] {topic}: capture loop exited")

    threads = []
    for topic, _frag, prim in resolved:
        get_frame = _make_capture(prim)
        raw_s, cmp_s = socks[topic]
        t = threading.Thread(
            target=capture_thread, args=(topic, prim, get_frame, raw_s, cmp_s),
            name=f"hawk_cap:{topic}", daemon=True,
        )
        t.start()
        threads.append(t)

    def shutdown():
        for raw_s, cmp_s in socks.values():
            try: raw_s.close()
            except Exception: pass
            try: cmp_s.close()
            except Exception: pass
        try: proc.terminate()
        except Exception: pass
        try: os.unlink(tmp.name)
        except Exception: pass

    _hawk_state.update({"running": True, "stop": stop_event,
                        "shutdown": shutdown, "threads": threads,
                        "proc": proc})
    print(f"[fix] camera publishers running for {len(resolved)} camera(s)")
    print(f"[fix] verify:")
    print(f"      source {ROS_SETUP}")
    print(f"      ros2 topic hz /Nova_Carter_ROS/front_hawk/left/image_raw/compressed")
    print(f"[fix] to stop: STOP_CAMERAS()")


_quiet_replicator()
start_cameras()
