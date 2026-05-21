# Isaac Sim "Viewport 2" -> UDP capture for the IsaacSim5G teleop pipeline.
#
# Captures the contents of Isaac's "Viewport 2" (or the active viewport
# if "Viewport 2" doesn't exist), JPEG-encodes the frame, wraps it in an
# IST5 envelope and sends it as a UDP datagram. The packet is routed via
# the oai-ext-dn forwarder (192.168.70.135:17600), which sends it across
# the rfsim 5G radio to the UE TUN where ue_teleop_bridge.py picks it up
# and relays it to the dashboard.
#
# Why this script lives inside Isaac:
#   The omni.kit.viewport.utility APIs only exist inside Isaac's Python
#   process. Frames have to be acquired here, then handed off over a
#   plain UDP socket to the rest of the pipeline.
#
# How to run:
#   1. Make sure scripts/setup_5g_data_plane.sh up has been run on the
#      host (so the ext-dn forwarders are listening on UDP 16000 and
#      17600).
#   2. Make sure the chain pipeline is running:
#         bash scripts/run_oai_teleop_broker.sh
#      (its UE bridge listens for camera frames on UDP 17600 in default ns).
#   3. In Isaac Sim:  Window -> Script Editor -> File -> Open this file.
#   4. Ctrl-Enter to run.
#
# To stop:  call STOP_VIEWPORT_CAPTURE() in the Script Editor.

import os, io, time, json, struct, socket, threading, traceback

# ---- Config ----------------------------------------------------------------
# TARGET_HOST is read from env when present so the same script can target
# either the legacy ext-dn ingress (192.168.70.135 — the "loopback DL"
# camera path) or the new robot-ns proxy that traverses the real 5G UL
# (default 192.168.244.2, the robot-ns side of the veth created by
# scripts/setup_robot_netns.sh). Set ISAAC_UPLINK_HOST in the env that
# launches Isaac to flip between them.
TARGET_VIEWPORT_NAME = "Viewport 2"    # falls back to active viewport if missing
TARGET_HOST          = os.environ.get("ISAAC_UPLINK_HOST", "192.168.244.2")
TARGET_PORT          = int(os.environ.get("ISAAC_UPLINK_PORT", "17600"))
TOPIC                = "/isaac/viewport2"
FPS                  = 8                # be conservative: capture is heavyish
JPEG_QUALITY         = 60
TMP_PNG              = "/dev/shm/isaac_vp2.png"   # tmpfs -> no disk hits
MAX_FRAME_DIM        = 960              # downscale longest side; keeps body well under UDP MTU
MAX_UDP_PAYLOAD      = 60000            # guard against >64K UDP "Message too long" (Errno 90)
FILE_READY_TIMEOUT   = 0.5              # max seconds to wait for capture_viewport_to_file output
FILE_READY_POLL      = 0.015
# DSCP_AF41 = 0x88; marks the camera UL flow as interactive video so a
# downstream 5G QoS rule mapping DSCP→5QI can place it on the right
# QoS flow (5QI-7/65, AF interactive video).
DSCP_AF41            = 0x88
# ---------------------------------------------------------------------------

# IST5 wire format (mirrors oai_bridge/protocol.py / ue_teleop_bridge.py).
UPLINK_MAGIC   = 0x49535435  # 'IST5'
KIND_CAMERA    = 2
UPLINK_HDR_FMT = "<IBBHI"

def _pack(kind, header_dict, body):
    hdr = json.dumps(header_dict, separators=(",", ":")).encode("utf-8")
    return (struct.pack(UPLINK_HDR_FMT, UPLINK_MAGIC, kind, 0, len(hdr), len(body))
            + hdr + body)


def _png_dims(png_bytes):
    # PNG signature is 8 bytes, then a 13-byte IHDR with width/height as
    # big-endian uint32 at offsets 16 and 20.
    try:
        if png_bytes[:8] != b"\x89PNG\r\n\x1a\n":
            return 0, 0
        w = int.from_bytes(png_bytes[16:20], "big")
        h = int.from_bytes(png_bytes[20:24], "big")
        return w, h
    except Exception:
        return 0, 0


def _wait_for_png(png_path, timeout=FILE_READY_TIMEOUT, poll=FILE_READY_POLL):
    """Block until the PNG written by capture_viewport_to_file is fully on
    disk. The render thread writes it asynchronously, so reading at a fixed
    delay produces truncated files. We poll until (a) the file exists,
    (b) its size is non-zero, and (c) the size is unchanged across two
    consecutive polls (writer is done flushing)."""
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
    """Read PNG, downscale longest-side to max_dim, JPEG-encode at
    base_quality. If the result is still over max_bytes, step quality
    down until it fits (or we hit quality=20). Returns (body, w, h, ok)
    where ok=False means we couldn't squeeze it under max_bytes — caller
    should drop the frame rather than blowing the UDP MTU."""
    from PIL import Image
    with open(png_path, "rb") as f:
        png_bytes = f.read()
    img = Image.open(io.BytesIO(png_bytes))
    img.load()  # force full decode here so truncation surfaces as a clear error
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
            print(f"[vp2] using viewport '{TARGET_VIEWPORT_NAME}'")
            return vp
    except Exception:
        pass
    vp = get_active_viewport()
    print(f"[vp2] '{TARGET_VIEWPORT_NAME}' not found, falling back to active viewport")
    return vp


class _CaptureLoop:
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
            print("[vp2] already running")
            return
        self._vp = _get_viewport()
        if self._vp is None:
            print("[vp2] no viewport available")
            return
        self._stop.clear()
        self._thread = threading.Thread(target=self._run, name="vp2-capture", daemon=True)
        self._thread.start()
        print(f"[vp2] capture -> {TARGET_HOST}:{TARGET_PORT} as {TOPIC}, ~{FPS} fps")

    def stop(self):
        self._stop.set()

    def _run(self):
        # capture_viewport_to_file() internally constructs an asyncio.Future,
        # which on Python 3.10+ raises if the calling thread has no event
        # loop bound. We never await this Future — Isaac's render pipeline
        # writes the file on the main thread — so a private loop here is
        # safe and only exists to satisfy the Future constructor.
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
                # Delete any previous frame so _wait_for_png distinguishes
                # "writer is finishing" from "stale leftover".
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
                    if self.errors <= 5:
                        print(f"[vp2] frame too large after compression ({len(body)} B); dropping")
                    continue

                header = {
                    "topic":  TOPIC,
                    "base":   TOPIC,
                    "stamp":  time.time(),
                    "width":  int(w),
                    "height": int(h),
                    "fmt":    "jpeg",
                }
                pkt = _pack(KIND_CAMERA, header, body)
                self._sock.sendto(pkt, (TARGET_HOST, TARGET_PORT))
                self.frames_sent += 1
            except Exception as e:
                self.errors += 1
                if self.errors <= 5:
                    traceback.print_exc()
            now = time.monotonic()
            if now - last_log >= 5.0:
                print(f"[vp2] sent={self.frames_sent} errors={self.errors}")
                last_log = now
            sleep_for = period - (time.monotonic() - t0)
            if sleep_for > 0:
                time.sleep(sleep_for)
        print("[vp2] capture loop stopped")


# Singleton handle so the Script Editor can re-run this file idempotently.
_RUNNING = globals().get("_RUNNING", None)
if _RUNNING is not None:
    try:
        _RUNNING.stop()
    except Exception:
        pass
_RUNNING = _CaptureLoop()
_RUNNING.start()


def STOP_VIEWPORT_CAPTURE():
    """Call from the Script Editor to stop the capture loop."""
    global _RUNNING
    if _RUNNING is not None:
        _RUNNING.stop()
        print("[vp2] stop requested")
