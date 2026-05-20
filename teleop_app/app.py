"""
IsaacSim5G teleop dashboard backend.

Single FastAPI process that speaks ROS 2 via rclpy and serves a browser
dashboard on :8080.

Camera frames reach the browser via TWO independent paths so something
always works:

1. **ROS 2** -- rclpy node subscribes to a list of candidate camera topics
   (Image + CompressedImage). Each frame is decoded with cv2 (no
   cv_bridge -- it's broken under NumPy 2.x) and pushed to every connected
   WebSocket.

2. **HTTP push** -- ``POST /api/camera_push`` accepts a JPEG body and
   feeds it to the same WebSocket pipe with a synthetic topic name like
   ``http://<source>``. Use this when Isaac's ROS 2 bridge is uncooperative:
   the Isaac-side script (``isaac/scripts/push_camera_http.py``) reads the
   camera with Isaac's Python API and POSTs the JPEG -- no rclpy involved
   from inside Isaac at all.

Platform selection: ``--platform nova_carter`` (default) or
``--platform iris_drone`` picks the topic set, command publisher, and UI
personality. See ``teleop_app/platforms.py``.

The watchdog / hold-to-drive control logic is unchanged: continuous
command publish at 30 Hz while a key is held, zero-twist on release,
500 ms rclpy-side watchdog, 300 ms WS-side watchdog.
"""

from __future__ import annotations

import argparse
import asyncio
import base64
import json
import os
import socket
import sys
import threading
import time
from dataclasses import dataclass, field
from pathlib import Path
from typing import Callable, Optional

import numpy as np
import cv2
import uvicorn
from fastapi import FastAPI, Request, WebSocket, WebSocketDisconnect
from fastapi.responses import HTMLResponse, JSONResponse
from fastapi.staticfiles import StaticFiles

import rclpy
from nav_msgs.msg import Odometry
from rclpy.executors import SingleThreadedExecutor
from rclpy.node import Node
from rclpy.qos import HistoryPolicy, QoSProfile, ReliabilityPolicy
from sensor_msgs.msg import CompressedImage, Image

from platforms import (
    ALL_PLATFORMS, PlatformSpec, build_publishers, get_platform,
)

# Shared uplink wire format with the UE-side bridge.
sys.path.insert(0, str(Path(__file__).resolve().parent.parent / "oai_bridge"))
from protocol import (  # noqa: E402
    KIND_POSE, KIND_CAMERA, pack_uplink, unpack_uplink,
)

APP_DIR = Path(__file__).resolve().parent


# ---------------------------------------------------------------------------
# Config
# ---------------------------------------------------------------------------
@dataclass
class Config:
    platform: PlatformSpec
    # mode:
    #   "direct"    — publish /cmd_vel via rclpy + subscribe ROS topics
    #                 directly (single-machine, no broker, no OAI)
    #   "oai"       — send commands as UDP JSON to the network-aware
    #                 broker stack on localhost (simulated 5G)
    #   "oai-real"  — send commands as UDP JSON to the UE IP allocated
    #                 by OpenAirInterface's 5G core. The UE-side
    #                 oai_bridge/ue_teleop_bridge.py decodes them and
    #                 publishes /cmd_vel inside Isaac. Camera + odom
    #                 come back via the same OAI link as binary uplink
    #                 envelopes on operator_udp_port.
    mode: str = "direct"
    udp_host: str = "127.0.0.1"           # downlink target (broker or UE IP)
    udp_port: int = 16000
    operator_udp_host: str = "0.0.0.0"    # uplink listener bind (oai-real)
    operator_udp_port: int = 17500

    camera_fps_limit: float = 15.0     # per-topic cap
    jpeg_quality: int = 70
    cmd_publish_rate_hz: float = 30.0
    watchdog_ms: float = 500.0
    server_watchdog_ms: float = 300.0
    http_port: int = 8080

    @classmethod
    def from_args_and_env(cls) -> "Config":
        parser = argparse.ArgumentParser(
            description="IsaacSim5G teleop dashboard backend.",
        )
        parser.add_argument(
            "--platform", default=os.environ.get("PLATFORM", "nova_carter"),
            choices=list(ALL_PLATFORMS),
            help="Robot platform configuration (default: nova_carter).",
        )
        parser.add_argument("--mode", choices=["direct", "oai", "oai-real"],
                            default=os.environ.get("TELEOP_MODE", "direct"))
        parser.add_argument("--port", type=int,
                            default=int(os.environ.get("TELEOP_PORT", "8080")))
        parser.add_argument("--udp-host",
                            default=os.environ.get("TELEOP_UDP_HOST", "127.0.0.1"),
                            help="downlink UDP target. In oai mode this is the "
                                 "broker ingress (default 127.0.0.1). In "
                                 "oai-real mode this is the UE's 5G IP, e.g. "
                                 "12.1.1.2.")
        parser.add_argument("--udp-port", type=int,
                            default=int(os.environ.get("TELEOP_UDP_PORT", "16000")))
        parser.add_argument("--operator-udp-host",
                            default=os.environ.get("OPERATOR_UDP_HOST", "0.0.0.0"),
                            help="bind address for the oai-real uplink "
                                 "listener (camera + odom from the UE).")
        parser.add_argument("--operator-udp-port", type=int,
                            default=int(os.environ.get("OPERATOR_UDP_PORT", "17500")))
        parser.add_argument("--camera-fps-limit", type=float,
                            default=float(os.environ.get("CAMERA_FPS_LIMIT", "15")))
        parser.add_argument("--jpeg-quality", type=int,
                            default=int(os.environ.get("CAMERA_JPEG_QUALITY", "70")))
        args = parser.parse_args()

        plat = get_platform(args.platform)
        cfg = cls(
            platform=plat,
            mode=args.mode,
            udp_host=args.udp_host,
            udp_port=args.udp_port,
            operator_udp_host=args.operator_udp_host,
            operator_udp_port=args.operator_udp_port,
            camera_fps_limit=args.camera_fps_limit,
            jpeg_quality=args.jpeg_quality,
            http_port=args.port,
        )
        return cfg


# ---------------------------------------------------------------------------
# Image decoding without cv_bridge.
# ---------------------------------------------------------------------------
def decode_raw_image(msg: Image) -> Optional[np.ndarray]:
    """Decode sensor_msgs/Image to BGR numpy without cv_bridge."""
    h, w = msg.height, msg.width
    if h == 0 or w == 0 or not msg.data:
        return None
    enc = msg.encoding.lower()
    raw_bytes = bytes(msg.data)
    expected_uint8_rgb = h * w * 3
    expected_uint8_rgba = h * w * 4
    expected_float_rgba = h * w * 4 * 4
    try:
        if enc in ("rgb8", "bgr8", "8uc3"):
            if len(raw_bytes) < expected_uint8_rgb:
                return None
            arr = np.frombuffer(raw_bytes, dtype=np.uint8, count=expected_uint8_rgb)
            img = arr.reshape((h, w, 3))
            if enc == "rgb8":
                img = cv2.cvtColor(img, cv2.COLOR_RGB2BGR)
            return img
        if enc in ("rgba8", "bgra8", "8uc4"):
            if len(raw_bytes) >= expected_uint8_rgba:
                arr = np.frombuffer(raw_bytes, dtype=np.uint8, count=expected_uint8_rgba)
                img = arr.reshape((h, w, 4))
                if enc == "rgba8":
                    return cv2.cvtColor(img, cv2.COLOR_RGBA2BGR)
                return cv2.cvtColor(img, cv2.COLOR_BGRA2BGR)
            # Isaac Sim sometimes still labels float32 RGBA frames as "rgba8"
            # during warm-up. Detect by buffer size and convert.
            if len(raw_bytes) >= expected_float_rgba:
                arr = np.frombuffer(raw_bytes, dtype=np.float32,
                                    count=h * w * 4).reshape((h, w, 4))
                img = np.clip(arr[..., :3] * 255.0, 0, 255).astype(np.uint8)
                return cv2.cvtColor(img, cv2.COLOR_RGB2BGR)
            return None
        if enc in ("32fc4",):
            if len(raw_bytes) < expected_float_rgba:
                return None
            arr = np.frombuffer(raw_bytes, dtype=np.float32,
                                count=h * w * 4).reshape((h, w, 4))
            img = np.clip(arr[..., :3] * 255.0, 0, 255).astype(np.uint8)
            return cv2.cvtColor(img, cv2.COLOR_RGB2BGR)
        if enc in ("mono8", "8uc1"):
            arr = np.frombuffer(raw_bytes, dtype=np.uint8, count=h * w)
            return cv2.cvtColor(arr.reshape((h, w)), cv2.COLOR_GRAY2BGR)
        if enc == "16uc1":
            arr = np.frombuffer(raw_bytes, dtype=np.uint16, count=h * w).reshape((h, w))
            mono8 = cv2.normalize(arr, None, 0, 255, cv2.NORM_MINMAX).astype(np.uint8)
            return cv2.cvtColor(mono8, cv2.COLOR_GRAY2BGR)
    except ValueError:
        return None
    return None


def encode_jpeg(img: np.ndarray, quality: int) -> Optional[bytes]:
    if img is None:
        return None
    ok, enc = cv2.imencode(".jpg", img, [int(cv2.IMWRITE_JPEG_QUALITY), quality])
    return enc.tobytes() if ok else None


def jpeg_dimensions(data: bytes) -> tuple[int, int]:
    """Read (w, h) from a JPEG SOF marker without decoding pixels."""
    if len(data) < 4 or data[0] != 0xFF or data[1] != 0xD8:
        return (0, 0)
    i, n = 2, len(data)
    while i + 9 < n:
        if data[i] != 0xFF:
            return (0, 0)
        marker = data[i + 1]
        i += 2
        if 0xC0 <= marker <= 0xCF and marker not in (0xC4, 0xC8, 0xCC):
            if i + 7 > n:
                return (0, 0)
            h = (data[i + 3] << 8) | data[i + 4]
            w = (data[i + 5] << 8) | data[i + 6]
            return (w, h)
        if marker in (0xD0, 0xD1, 0xD2, 0xD3, 0xD4, 0xD5, 0xD6, 0xD7, 0xD8, 0xD9):
            continue
        if i + 1 >= n:
            return (0, 0)
        seg_len = (data[i] << 8) | data[i + 1]
        i += seg_len
    return (0, 0)


# ---------------------------------------------------------------------------
@dataclass
class FeedbackChannel:
    queue: asyncio.Queue
    loop: asyncio.AbstractEventLoop


class TeleopNode(Node):
    def __init__(self, cfg: Config):
        super().__init__("isaacsim5g_teleop")
        self.cfg = cfg
        self.platform = cfg.platform

        self._channels: list[FeedbackChannel] = []
        self._channels_lock = threading.Lock()
        self._cv_lock = threading.Lock()

        # Per-topic counters, recency, and rate-limit timestamps. We cap
        # each topic INDEPENDENTLY at camera_fps_limit so the browser can
        # show multiple panels side-by-side without one starving the other.
        self._frame_counts: dict[str, int] = {}
        self._last_frame_at: dict[str, float] = {}
        self._last_emit_at: dict[str, float] = {}
        self._frame_counts_lock = threading.Lock()
        self._min_camera_period = 1.0 / max(0.1, cfg.camera_fps_limit)

        # Cmd watchdog
        self._last_intent: dict = {}
        self._last_intent_t = 0.0
        self._estop_latched = False
        self._stopped_already = False
        self._cmd_seq = 0

        # UDP socket used by oai + oai-real to send downlink commands.
        self._udp_sock: Optional[socket.socket] = None
        if cfg.mode in ("oai", "oai-real"):
            self._udp_sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)

        # In oai-real mode, also open a UDP listener for the UE-side uplink
        # (camera + odom envelopes from oai_bridge/ue_teleop_bridge.py).
        self._uplink_rx_sock: Optional[socket.socket] = None
        self._uplink_thread: Optional[threading.Thread] = None
        self._uplink_stop = threading.Event()

        # Platform-aware command + event publishers (used in direct mode).
        self._publish_command, self._publish_event = build_publishers(
            self.platform, self)

        # Sensor QoS for cameras + odometry.
        sensor_qos = QoSProfile(
            reliability=ReliabilityPolicy.BEST_EFFORT,
            history=HistoryPolicy.KEEP_LAST,
            depth=5,
        )
        self._sensor_qos = sensor_qos
        self._cam_subs: list[tuple[str, str, object]] = []

        if cfg.mode in ("direct", "oai"):
            # Subscribe to every candidate + every panel default so frames
            # show up the moment a publisher appears anywhere.
            bases: set[str] = set(self.platform.camera_topics)
            for _label, topic in self.platform.camera_panels:
                bases.add(topic)
            for base in sorted(bases):
                self._subscribe_camera_pair(base)
            for odom_topic in self.platform.odom_topics:
                self.create_subscription(
                    Odometry, odom_topic, self._on_odom, sensor_qos)
        else:
            # oai-real: feedback arrives over UDP, not ROS 2.
            self._start_uplink_listener()

        # Fixed-rate command publish + watchdog.
        self.create_timer(
            1.0 / max(1.0, cfg.cmd_publish_rate_hz),
            self._on_publish_tick,
        )
        # Topic-scan every 2 s; in oai-real mode this just snapshots the
        # http:// + UDP virtual entries for the dashboard's topic table.
        self.create_timer(2.0, self._on_topic_scan)

        self.get_logger().info(
            f"teleop node up | platform={self.platform.name} mode={cfg.mode} "
            f"cmd_vel={self.platform.cmd_vel_topic} "
            f"cameras={list(self.platform.camera_topics)} "
            f"odom={list(self.platform.odom_topics)}"
        )
        if cfg.mode == "oai-real":
            self.get_logger().info(
                f"oai-real | downlink -> {cfg.udp_host}:{cfg.udp_port}  "
                f"uplink listener on {cfg.operator_udp_host}:{cfg.operator_udp_port}")

    # ----- camera subscription management ---------------------------------
    def _subscribe_camera_pair(self, base: str):
        """Subscribe to ``base`` and ``base + '/compressed'`` with per-topic
        callback closures so each callback knows its source topic."""
        already = {(b, v) for (b, v, _) in self._cam_subs}
        for variant, msg_type in (("raw", Image), ("compressed", CompressedImage)):
            full_topic = base if variant == "raw" else base + "/compressed"
            if (base, variant) in already:
                continue
            try:
                if variant == "raw":
                    cb = (lambda msg, _base=base: self._on_raw_image(msg, _base))
                else:
                    cb = (lambda msg, _base=base: self._on_compressed_image(msg, _base))
                sub = self.create_subscription(msg_type, full_topic, cb, self._sensor_qos)
                self._cam_subs.append((base, variant, sub))
            except Exception as exc:
                self.get_logger().warn(f"could not subscribe {full_topic}: {exc}")

    def add_camera_topic(self, base: str):
        existing = {b for (b, _, _) in self._cam_subs}
        if not base or base in existing:
            return
        self._subscribe_camera_pair(base)
        self.get_logger().info(f"added camera topic {base}")

    def ensure_camera_topic(self, base: str):
        """Make sure ``base`` has a live subscription. Multi-panel UI uses
        this when the user picks a topic for a panel."""
        if not base:
            return
        existing = {b for (b, _, _) in self._cam_subs}
        if base not in existing:
            self.add_camera_topic(base)

    # ----- channel queues -------------------------------------------------
    def register_channel(self, ch: FeedbackChannel):
        with self._channels_lock:
            self._channels.append(ch)

    def unregister_channel(self, ch: FeedbackChannel):
        with self._channels_lock:
            if ch in self._channels:
                self._channels.remove(ch)

    def _push(self, envelope: dict):
        with self._channels_lock:
            channels = list(self._channels)
        for ch in channels:
            try:
                ch.loop.call_soon_threadsafe(self._try_put_nowait, ch.queue, envelope)
            except RuntimeError:
                pass

    @staticmethod
    def _try_put_nowait(q: asyncio.Queue, item):
        try:
            q.put_nowait(item)
        except asyncio.QueueFull:
            try:
                _ = q.get_nowait()
                q.put_nowait(item)
            except Exception:
                pass

    # ----- camera callbacks ----------------------------------------------
    def _bump_count(self, base: str):
        now = time.monotonic()
        with self._frame_counts_lock:
            self._frame_counts[base] = self._frame_counts.get(base, 0) + 1
            self._last_frame_at[base] = now

    def _emit_jpeg(self, jpeg_bytes: bytes, w: int, h: int, source_topic: str):
        """Emit a camera envelope for *every* subscribed topic. Per-topic
        rate limiting means each panel gets its own steady stream.

        ``source_topic`` may end in ``/compressed``. We strip that to get
        the canonical ``base`` topic the browser uses to route the frame
        to the correct panel. The full topic name is still sent in the
        envelope for diagnostics."""
        base = (source_topic[:-len("/compressed")]
                if source_topic.endswith("/compressed") else source_topic)
        self._bump_count(base)
        now = time.time()
        last = self._last_emit_at.get(base, 0.0)
        if now - last < self._min_camera_period:
            return
        self._last_emit_at[base] = now
        # Envelope carries both the canonical base topic and the raw
        # source. The browser routes on ``base`` (which is what panel
        # selectors store).
        self._push({
            "type": "camera",
            "stamp": now,
            "topic": source_topic,
            "base": base,
            "width": int(w), "height": int(h),
            "data_b64": base64.b64encode(jpeg_bytes).decode("ascii"),
        })

    def _on_compressed_image(self, msg: CompressedImage, base: str):
        data = bytes(msg.data)
        if not data:
            return
        topic_name = base + "/compressed"
        fmt = (msg.format or "").lower()
        if "jpeg" in fmt or "jpg" in fmt or fmt == "":
            w, h = jpeg_dimensions(data)
            self._emit_jpeg(data, w, h, topic_name)
            return
        with self._cv_lock:
            arr = np.frombuffer(data, dtype=np.uint8)
            img = cv2.imdecode(arr, cv2.IMREAD_COLOR)
            if img is None:
                return
            enc = encode_jpeg(img, self.cfg.jpeg_quality)
            if enc:
                self._emit_jpeg(enc, img.shape[1], img.shape[0], topic_name)

    def _on_raw_image(self, msg: Image, base: str):
        with self._cv_lock:
            img = decode_raw_image(msg)
            if img is None:
                self.get_logger().warn(
                    f"could not decode encoding='{msg.encoding}' "
                    f"size={msg.width}x{msg.height}",
                    throttle_duration_sec=5.0,
                )
                return
            enc = encode_jpeg(img, self.cfg.jpeg_quality)
            if enc:
                self._emit_jpeg(enc, img.shape[1], img.shape[0], base)

    def ingest_http_frame(self, jpeg_bytes: bytes, source: str):
        """Called by the FastAPI /api/camera_push endpoint."""
        topic = source if source.startswith("http") or source.startswith("/") else f"http://{source}"
        # Pre-register the topic so it shows up in the dashboard table
        # even before the second frame.
        with self._frame_counts_lock:
            if topic not in self._frame_counts:
                self._frame_counts[topic] = 0
        w, h = jpeg_dimensions(jpeg_bytes)
        self._emit_jpeg(jpeg_bytes, w, h, topic)

    # ----- oai-real uplink listener -------------------------------------
    def _start_uplink_listener(self):
        """Open UDP socket and spawn a thread that demultiplexes
        IST5-framed pose / camera envelopes from the UE bridge."""
        sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
        sock.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
        try:
            # Larger recv buffer for camera frames.
            sock.setsockopt(socket.SOL_SOCKET, socket.SO_RCVBUF, 4 * 1024 * 1024)
        except OSError:
            pass
        sock.bind((self.cfg.operator_udp_host, self.cfg.operator_udp_port))
        sock.settimeout(0.5)
        self._uplink_rx_sock = sock

        def loop():
            while not self._uplink_stop.is_set():
                try:
                    data, _addr = sock.recvfrom(65536)
                except socket.timeout:
                    continue
                except OSError as exc:
                    self.get_logger().warn(f"uplink recv error: {exc}")
                    continue
                parsed = unpack_uplink(data)
                if parsed is None:
                    continue
                kind, header, body = parsed
                if kind == KIND_POSE:
                    env = {
                        "type": "pose",
                        "stamp": float(header.get("stamp", time.time())),
                        "frame_id": header.get("frame_id", "odom"),
                        "x": float(header.get("x", 0.0)),
                        "y": float(header.get("y", 0.0)),
                        "z": float(header.get("z", 0.0)),
                        "yaw": float(header.get("yaw", 0.0)),
                        "linear_x": float(header.get("linear_x", 0.0)),
                        "linear_y": float(header.get("linear_y", 0.0)),
                        "linear_z": float(header.get("linear_z", 0.0)),
                        "angular_z": float(header.get("angular_z", 0.0)),
                    }
                    self._push(env)
                elif kind == KIND_CAMERA:
                    base = header.get("base") or header.get("topic", "")
                    if base and base.endswith("/compressed"):
                        base = base[:-len("/compressed")]
                    w = int(header.get("width", 0))
                    h = int(header.get("height", 0))
                    if (w == 0 or h == 0) and body[:2] == b"\xff\xd8":
                        w, h = jpeg_dimensions(body)
                    if base:
                        self._emit_jpeg(body, w, h,
                                        header.get("topic", base))

        t = threading.Thread(target=loop, name="oai-uplink-rx", daemon=True)
        t.start()
        self._uplink_thread = t

    def shutdown_uplink(self):
        self._uplink_stop.set()
        try:
            if self._uplink_rx_sock is not None:
                self._uplink_rx_sock.close()
        except Exception:
            pass

    def _on_odom(self, msg: Odometry):
        import math
        p = msg.pose.pose.position
        o = msg.pose.pose.orientation
        t = msg.twist.twist
        siny_cosp = 2.0 * (o.w * o.z + o.x * o.y)
        cosy_cosp = 1.0 - 2.0 * (o.y * o.y + o.z * o.z)
        yaw = math.atan2(siny_cosp, cosy_cosp)
        self._push({
            "type": "pose",
            "stamp": time.time(),
            "frame_id": msg.header.frame_id,
            "x": float(p.x), "y": float(p.y), "z": float(p.z),
            "yaw": yaw,
            "linear_x": float(t.linear.x),
            "linear_y": float(t.linear.y),
            "linear_z": float(t.linear.z),
            "angular_z": float(t.angular.z),
        })

    # ----- command path --------------------------------------------------
    def submit_command(self, intent: dict) -> dict:
        self._cmd_seq = (self._cmd_seq + 1) & 0xFFFFFFFF
        payload = dict(intent)
        payload.update({
            "seq": self._cmd_seq,
            "stamp_ms": int(time.time() * 1000),
        })
        if self.cfg.mode in ("oai", "oai-real") and self._udp_sock is not None:
            try:
                self._udp_sock.sendto(
                    json.dumps(payload).encode("utf-8"),
                    (self.cfg.udp_host, self.cfg.udp_port),
                )
            except OSError as exc:
                self.get_logger().warn(f"udp send failed: {exc}")
        self._last_intent = intent
        self._last_intent_t = time.monotonic()
        self._estop_latched = bool(intent.get("emergency_stop", False))
        self._stopped_already = False
        return payload

    def submit_event(self, name: str):
        self.get_logger().info(f"event: {name}")
        try:
            self._publish_event(name)
        except Exception as exc:
            self.get_logger().error(f"event publish failed: {exc}")

    def _on_publish_tick(self):
        if self.cfg.mode != "direct":
            return
        age = time.monotonic() - self._last_intent_t if self._last_intent_t else float("inf")
        if self._estop_latched:
            self._publish_command({"emergency_stop": True})
            return
        if age <= self.cfg.watchdog_ms / 1000.0:
            self._publish_command(self._last_intent)
            self._stopped_already = False
            return
        if not self._stopped_already:
            self.get_logger().warn(
                f"watchdog: no cmd for {age*1000:.0f} ms, sending zero",
                throttle_duration_sec=2.0,
            )
            self._publish_command({"emergency_stop": False})
            self._stopped_already = True

    # ----- topic scan -----------------------------------------------------
    def _on_topic_scan(self):
        # In oai-real mode we don't subscribe to ROS topics directly; the
        # camera + odom data arrives over UDP and lives under the synthetic
        # topic names sent by the UE bridge.
        if self.cfg.mode != "oai-real":
            try:
                tnt = self.get_topic_names_and_types()
            except Exception:
                tnt = []
            # Subscribe to any newly-discovered image topic.
            for name, types in tnt:
                if "sensor_msgs/msg/Image" in types or "sensor_msgs/msg/CompressedImage" in types:
                    base = (name[:-len("/compressed")]
                            if name.endswith("/compressed") else name)
                    if base not in {b for (b, _, _) in self._cam_subs}:
                        self.add_camera_topic(base)

        # Compose the full live table.
        cams: list[dict] = []
        bases = {b for (b, _, _) in self._cam_subs}
        # Also include HTTP push + oai-real uplink sources we've seen.
        # Their canonical topic names look like ROS topic paths so they
        # match the panel bindings ("/Nova_Carter_ROS/..."), or the
        # http://source-name convention for the camera_push endpoint.
        with self._frame_counts_lock:
            for k in self._frame_counts.keys():
                bases.add(k)
        now = time.monotonic()
        for base in sorted(bases):
            if base.startswith("http://") or self.cfg.mode == "oai-real":
                # In oai-real mode the dashboard isn't a ROS subscriber for
                # those topics -- frames arrive over UDP from the UE bridge
                # -- so publisher counts aren't meaningful here.
                pubs = 0
            else:
                try:
                    pubs = self.count_publishers(base) + self.count_publishers(base + "/compressed")
                except Exception:
                    pubs = 0
            with self._frame_counts_lock:
                frames = self._frame_counts.get(base, 0)
                last_t = self._last_frame_at.get(base, 0.0)
            age_s = (now - last_t) if last_t else None
            cams.append({
                "topic": base,
                "publishers": pubs,
                "frames_received": frames,
                "last_frame_age_s": age_s,
            })
        self._push({"type": "topics", "cameras": cams})


# ---------------------------------------------------------------------------
# ROS 2 executor thread
# ---------------------------------------------------------------------------
class RosThread:
    def __init__(self, cfg: Config):
        self.cfg = cfg
        self.node: Optional[TeleopNode] = None
        self.executor: Optional[SingleThreadedExecutor] = None
        self._thread: Optional[threading.Thread] = None
        self._stop = threading.Event()
        self._ready = threading.Event()

    def start(self):
        self._thread = threading.Thread(target=self._run, daemon=True, name="rclpy-exec")
        self._thread.start()
        self._ready.wait(timeout=5.0)

    def stop(self):
        self._stop.set()
        if self.executor is not None:
            try:
                self.executor.shutdown()
            except Exception:
                pass

    def _run(self):
        rclpy.init(args=None)
        self.node = TeleopNode(self.cfg)
        self.executor = SingleThreadedExecutor()
        self.executor.add_node(self.node)
        self._ready.set()
        try:
            while not self._stop.is_set():
                self.executor.spin_once(timeout_sec=0.1)
        finally:
            try:
                # Final zero command on shutdown (direct mode only).
                if self.cfg.mode == "direct":
                    self.node._publish_command({"emergency_stop": False})
            except Exception:
                pass
            try:
                self.node.shutdown_uplink()
            except Exception:
                pass
            try:
                self.node.destroy_node()
            except Exception:
                pass
            try:
                rclpy.shutdown()
            except Exception:
                pass


# ---------------------------------------------------------------------------
# FastAPI app
# ---------------------------------------------------------------------------
cfg = Config.from_args_and_env()
ros = RosThread(cfg)

app = FastAPI(title=f"IsaacSim5G Teleop ({cfg.platform.display})")
app.mount("/static", StaticFiles(directory=str(APP_DIR / "static")), name="static")


@app.on_event("startup")
async def _startup():
    ros.start()


@app.on_event("shutdown")
async def _shutdown():
    ros.stop()


@app.get("/")
async def index():
    return HTMLResponse((APP_DIR / "static" / "index.html").read_text())


@app.get("/healthz")
async def healthz():
    return {
        "ok": True,
        "platform": cfg.platform.name,
        "platform_display": cfg.platform.display,
        "platform_kind": cfg.platform.kind,
        "mode": cfg.mode,
        "camera_topics": list(cfg.platform.camera_topics),
        "camera_panels": [
            {"label": label, "topic": topic}
            for (label, topic) in cfg.platform.camera_panels
        ],
        "odom_topics": list(cfg.platform.odom_topics),
        "cmd_vel_topic": cfg.platform.cmd_vel_topic,
        "downlink_target": f"{cfg.udp_host}:{cfg.udp_port}",
        "uplink_listener": (
            f"{cfg.operator_udp_host}:{cfg.operator_udp_port}"
            if cfg.mode == "oai-real" else None
        ),
    }


@app.get("/api/platform")
async def api_platform():
    """Static description of the current platform for the UI to render."""
    p = cfg.platform
    return {
        "name": p.name,
        "display": p.display,
        "kind": p.kind,
        "has_throttle": p.has_throttle,
        "has_arm": p.has_arm,
        "control_help": p.control_help,
        "linear_max_mps": p.linear_max_mps,
        "angular_max_rps": p.angular_max_rps,
        "vertical_max_mps": p.vertical_max_mps,
    }


@app.get("/api/topics")
async def list_topics():
    if ros.node is None:
        return JSONResponse({"cameras": []})
    bases = {b for (b, _, _) in ros.node._cam_subs}
    try:
        tnt = ros.node.get_topic_names_and_types()
    except Exception:
        tnt = []
    for name, types in tnt:
        if "sensor_msgs/msg/Image" in types or "sensor_msgs/msg/CompressedImage" in types:
            base = name[:-len("/compressed")] if name.endswith("/compressed") else name
            bases.add(base)
    with ros.node._frame_counts_lock:
        bases.update({k for k in ros.node._frame_counts.keys() if k.startswith("http://")})
    now = time.monotonic()
    cams: list[dict] = []
    for base in sorted(bases):
        if base.startswith("http://"):
            pubs = 0
        else:
            try:
                pubs = ros.node.count_publishers(base) + ros.node.count_publishers(base + "/compressed")
            except Exception:
                pubs = 0
        with ros.node._frame_counts_lock:
            frames = ros.node._frame_counts.get(base, 0)
            last_t = ros.node._last_frame_at.get(base, 0.0)
        age_s = (now - last_t) if last_t else None
        cams.append({
            "topic": base, "publishers": pubs,
            "frames_received": frames, "last_frame_age_s": age_s,
        })
    return {"cameras": cams}


@app.post("/api/camera_push")
async def camera_push(request: Request):
    """
    Accept a JPEG body POST and inject it as a camera frame.

    Use this when Isaac Sim's ROS 2 publishers are uncooperative: an Isaac-
    side script (see ``isaac/scripts/push_camera_http.py``) reads the
    camera with Isaac's Python API and POSTs the JPEG bytes here. We pipe
    them straight to the WebSocket camera channel.

    Headers/params:
        ``X-Camera-Source``: free-form name shown in the dashboard
                              (default: ``isaac_push``)
        ``X-Camera-Width``, ``X-Camera-Height``: optional, default = parsed
                              from JPEG header.
    Body: raw JPEG bytes.
    """
    body = await request.body()
    if not body:
        return JSONResponse({"ok": False, "reason": "empty_body"}, status_code=400)
    if ros.node is None:
        return JSONResponse({"ok": False, "reason": "ros_not_ready"}, status_code=503)
    source = request.headers.get("X-Camera-Source", "isaac_push")
    ros.node.ingest_http_frame(body, source)
    return {"ok": True, "bytes": len(body), "source": source}


@app.websocket("/ws")
async def ws_endpoint(websocket: WebSocket):
    await websocket.accept()
    loop = asyncio.get_running_loop()
    ch = FeedbackChannel(queue=asyncio.Queue(maxsize=8), loop=loop)
    if ros.node is not None:
        ros.node.register_channel(ch)

    await websocket.send_text(json.dumps({
        "type": "config",
        "platform": {
            "name": cfg.platform.name,
            "display": cfg.platform.display,
            "kind": cfg.platform.kind,
            "has_throttle": cfg.platform.has_throttle,
            "has_arm": cfg.platform.has_arm,
            "control_help": cfg.platform.control_help,
            "linear_max_mps": cfg.platform.linear_max_mps,
            "angular_max_rps": cfg.platform.angular_max_rps,
            "vertical_max_mps": cfg.platform.vertical_max_mps,
        },
        "mode": cfg.mode,
        "camera_topics": list(cfg.platform.camera_topics),
        "camera_panels": [
            {"label": label, "topic": topic}
            for (label, topic) in cfg.platform.camera_panels
        ],
        "odom_topics": list(cfg.platform.odom_topics),
        "cmd_vel_topic": cfg.platform.cmd_vel_topic,
        "watchdog_ms": cfg.watchdog_ms,
        "server_watchdog_ms": cfg.server_watchdog_ms,
    }))

    last_cmd_ms = int(time.time() * 1000)
    stop_already_sent = False

    async def pump():
        try:
            while True:
                env = await ch.queue.get()
                await websocket.send_text(json.dumps(env))
        except (WebSocketDisconnect, RuntimeError):
            return
        except Exception:
            return

    async def watchdog():
        nonlocal stop_already_sent
        try:
            while True:
                await asyncio.sleep(cfg.server_watchdog_ms / 1000.0 / 2.0)
                gap_ms = int(time.time() * 1000) - last_cmd_ms
                if gap_ms > cfg.server_watchdog_ms and not stop_already_sent:
                    if ros.node is not None:
                        ros.node.submit_command({"emergency_stop": False})
                    stop_already_sent = True
        except asyncio.CancelledError:
            return

    pump_task = asyncio.create_task(pump())
    dog_task = asyncio.create_task(watchdog())

    try:
        while True:
            data = await websocket.receive_text()
            try:
                msg = json.loads(data)
            except json.JSONDecodeError:
                await websocket.send_text(json.dumps({"type": "error", "reason": "bad_json"}))
                continue

            msg_type = msg.get("type", "cmd")
            # The browser asks the backend to make sure a topic is being
            # subscribed to (multi-panel workflow). No "active" concept --
            # the browser routes camera envelopes to panels by topic name.
            if msg_type in ("ensure_camera", "set_camera", "add_camera"):
                topic = str(msg.get("topic", "")).strip()
                if topic and ros.node is not None:
                    ros.node.ensure_camera_topic(topic)
                continue
            if msg_type == "event":
                name = str(msg.get("name", "")).strip()
                if name and ros.node is not None:
                    ros.node.submit_event(name)
                continue

            # Default: a teleop command
            intent = {
                "linear_x": float(msg.get("linear_x", 0.0)),
                "linear_y": float(msg.get("linear_y", 0.0)),
                "linear_z": float(msg.get("linear_z", 0.0)),
                "angular_z": float(msg.get("angular_z", 0.0)),
                "emergency_stop": bool(msg.get("emergency_stop", False)),
                "source": str(msg.get("source", "dashboard")),
            }
            payload = (ros.node.submit_command(intent) if ros.node is not None else {})
            last_cmd_ms = int(time.time() * 1000)
            if intent["linear_x"] or intent["linear_y"] or intent["linear_z"] \
               or intent["angular_z"] or intent["emergency_stop"]:
                stop_already_sent = False
            await websocket.send_text(json.dumps({"type": "cmd_ack", "payload": payload}))
    except WebSocketDisconnect:
        return
    finally:
        try:
            if ros.node is not None:
                ros.node.submit_command({"emergency_stop": False})
        except Exception:
            pass
        pump_task.cancel()
        dog_task.cancel()
        if ros.node is not None:
            ros.node.unregister_channel(ch)


def main():
    uvicorn.run(app, host="0.0.0.0", port=cfg.http_port, log_level="info")


if __name__ == "__main__":
    main()
