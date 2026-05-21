#!/usr/bin/env python3
"""
UE-side teleop bridge for the OAI 5G data plane.

This process is "the robot". In the deployment described in
``oai_rfsim_rtf/fixed_rtf_rfsim-develop/README.md``, OpenAirInterface
runs a gNB and a soft-UE communicating over rfsim. The UE attaches to a
5G core (open5gs / oai-cn5g) and is allocated an IP on its TUN
interface (e.g. ``12.1.1.2`` on ``oaitun_ue1``).

End-to-end flow this bridge participates in:

    Browser ─ ws ─► dashboard (mode=oai-real)
                       │ UDP JSON to UE_IP:UE_PORT
                       ▼
                  5G core UPF / SPGW-U
                       │ GTP-U
                       ▼
                       gNB
                       │ rfsim 5G air interface
                       ▼
                       UE  ← TUN ↦ 12.1.1.2:UE_PORT
                            │
                            ▼
                  ue_teleop_bridge.py  (this file)
                            │ rclpy
                            ▼
                       /cmd_vel
                            │
                            ▼
                  Isaac Sim Nova Carter

Reverse path (camera + odom from robot to operator HUD):

    Isaac Sim → /odom, /front_hawk/*/image_raw
                  │ rclpy subscriptions
                  ▼
              this bridge
                  │ UDP datagrams to OPERATOR_IP:OPERATOR_PORT
                  ▼
                  UE (TUN egress)
                  │ rfsim 5G air interface
                  ▼
                  gNB → UPF → operator host
                  │
                  ▼
            dashboard oai-real listener → WebSocket → browser

The wire format for the uplink is a compact binary header followed by a
JSON metadata blob and a binary payload:

    uint32   magic = 0x49535435 ("IST5", little-endian)
    uint8    kind  (1 = pose, 2 = camera)
    uint8    reserved
    uint16   header_len    (bytes of JSON metadata)
    uint32   body_len      (bytes of payload)
    bytes    header_json
    bytes    payload

This keeps each datagram self-describing so the dashboard listener can
parse without prior coordination, and it fits within a typical 1500-byte
MTU for pose messages (camera frames are usually larger -- if your link
has a small MTU, run with --camera-fragment).
"""
from __future__ import annotations

import argparse
import json
import math
import os
import signal
import socket
import struct
import sys
import threading
import time
from dataclasses import dataclass, field
from typing import Optional

import numpy as np

import rclpy
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry
from rclpy.executors import SingleThreadedExecutor
from rclpy.node import Node
from rclpy.qos import HistoryPolicy, QoSProfile, ReliabilityPolicy
from sensor_msgs.msg import CompressedImage, Image

try:
    import cv2
    _HAVE_CV2 = True
except Exception:
    _HAVE_CV2 = False


# ---------------------------------------------------------------------------
# Wire format
# ---------------------------------------------------------------------------
UPLINK_MAGIC = 0x49535435           # 'IST5' little-endian
KIND_POSE = 1
KIND_CAMERA = 2
UPLINK_HDR = "<IBBHI"
UPLINK_HDR_SIZE = struct.calcsize(UPLINK_HDR)


def pack_uplink(kind: int, header: dict, body: bytes) -> bytes:
    hdr_bytes = json.dumps(header, separators=(",", ":")).encode("utf-8")
    return struct.pack(
        UPLINK_HDR,
        UPLINK_MAGIC, int(kind), 0,
        len(hdr_bytes), len(body),
    ) + hdr_bytes + body


def unpack_uplink(buf: bytes):
    """Reverse of pack_uplink. Returns (kind, header_dict, body_bytes) or None."""
    if len(buf) < UPLINK_HDR_SIZE:
        return None
    magic, kind, _r, hlen, blen = struct.unpack(UPLINK_HDR, buf[:UPLINK_HDR_SIZE])
    if magic != UPLINK_MAGIC:
        return None
    if len(buf) < UPLINK_HDR_SIZE + hlen + blen:
        return None
    try:
        header = json.loads(buf[UPLINK_HDR_SIZE:UPLINK_HDR_SIZE + hlen])
    except json.JSONDecodeError:
        return None
    body = buf[UPLINK_HDR_SIZE + hlen: UPLINK_HDR_SIZE + hlen + blen]
    return int(kind), header, body


# ---------------------------------------------------------------------------
@dataclass
class Stats:
    cmds_rx: int = 0
    cmds_rx_bad: int = 0
    bytes_rx: int = 0
    cmds_fwd: int = 0
    bytes_fwd: int = 0
    pose_tx: int = 0
    camera_tx: int = 0
    bytes_tx: int = 0
    last_cmd_t: float = 0.0


# ---------------------------------------------------------------------------
class UeBridge(Node):
    def __init__(
        self, *,
        listen_host: str, listen_port: int,
        operator_host: str, operator_port: int,
        cmd_vel_topic: str,
        odom_topics: list[str],
        camera_topics: list[str],
        camera_fps_limit: float,
        jpeg_quality: int,
        watchdog_ms: float,
        cmd_publish_rate_hz: float,
        forward_addr: Optional[tuple] = None,
        publish_cmd_vel: bool = True,
        camera_uplink_listen_port: int = 0,
    ):
        super().__init__("ue_teleop_bridge")
        self.stats = Stats()
        self.operator_addr = (operator_host, operator_port)
        self._watchdog_s = max(0.05, watchdog_ms / 1000.0)
        self._jpeg_quality = jpeg_quality
        self._forward_addr = forward_addr
        self._publish_cmd_vel = publish_cmd_vel

        # --- Downlink: UDP -> /cmd_vel (and/or UDP forward) ------------
        self._last_intent: dict = {}
        self._last_intent_t: float = 0.0
        self._estop_latched: bool = False
        self._stopped_already: bool = False

        self.cmd_pub = None
        if self._publish_cmd_vel:
            self.cmd_pub = self.create_publisher(Twist, cmd_vel_topic, 20)
            self.create_timer(
                1.0 / max(1.0, cmd_publish_rate_hz),
                self._on_publish_tick,
            )

        self._fwd_sock = None
        if self._forward_addr is not None:
            self._fwd_sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)

        self._rx_sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
        self._rx_sock.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
        self._rx_sock.bind((listen_host, listen_port))
        self._rx_sock.settimeout(0.5)
        sink = cmd_vel_topic if self._publish_cmd_vel else "(no /cmd_vel publish)"
        if self._forward_addr is not None:
            sink = f"{sink}, forward UDP -> {self._forward_addr[0]}:{self._forward_addr[1]}"
        self.get_logger().info(
            f"downlink: UDP {listen_host}:{listen_port} -> {sink}")

        self._stop = threading.Event()
        self._rx_thread = threading.Thread(
            target=self._rx_loop, name="udp-rx", daemon=True)
        self._rx_thread.start()

        # --- Uplink: ROS subs -> UDP -----------------------------------
        # IP_TOS=0x88 (DSCP_AF41) marks the pose/odom + relayed camera
        # uplinks as interactive video so the 5G QoS rules can route them
        # onto a non-default QoS flow (5QI 7/65) when the PDU session has
        # DSCP→5QI rules wired up. Falls back silently on hosts where the
        # setsockopt isn't permitted.
        self._tx_sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
        try:
            self._tx_sock.setsockopt(socket.IPPROTO_IP, socket.IP_TOS, 0x88)
        except OSError:
            pass
        self.get_logger().info(
            f"uplink target: {operator_host}:{operator_port}")

        sensor_qos = QoSProfile(
            reliability=ReliabilityPolicy.BEST_EFFORT,
            history=HistoryPolicy.KEEP_LAST,
            depth=5,
        )

        for topic in odom_topics:
            self.create_subscription(Odometry, topic, self._on_odom, sensor_qos)
            self.get_logger().info(f"  uplink odom subscribed: {topic}")

        self._cam_period = 1.0 / max(0.1, camera_fps_limit)
        self._last_cam_emit_at: dict[str, float] = {}
        for base in camera_topics:
            self._subscribe_camera_pair(base, sensor_qos)

        # --- Optional UDP camera uplink ingest (frames coming in over the
        # 5G radio path from an Isaac-side capture script). Packets here
        # are expected to already be IST5 envelopes (kind=camera). We
        # forward them to the operator as-is, no decode needed.
        self._cam_rx_sock = None
        if camera_uplink_listen_port and camera_uplink_listen_port > 0:
            self._cam_rx_sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
            self._cam_rx_sock.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
            self._cam_rx_sock.bind(("0.0.0.0", camera_uplink_listen_port))
            self._cam_rx_sock.settimeout(0.5)
            self.get_logger().info(
                f"camera uplink: UDP 0.0.0.0:{camera_uplink_listen_port} "
                f"-> operator {self.operator_addr[0]}:{self.operator_addr[1]}")
            self._cam_rx_thread = threading.Thread(
                target=self._camera_rx_loop, name="udp-cam-rx", daemon=True)
            self._cam_rx_thread.start()

        self.create_timer(5.0, self._log_stats)

    # ----- downlink (UDP -> ROS) -----------------------------------------
    def _rx_loop(self):
        while not self._stop.is_set():
            try:
                data, addr = self._rx_sock.recvfrom(65536)
            except socket.timeout:
                continue
            except OSError as exc:
                self.get_logger().warn(f"rx error: {exc}")
                continue
            self.stats.bytes_rx += len(data)
            self._handle_downlink(data, addr)

    def _handle_downlink(self, data: bytes, addr):
        if self._forward_addr is not None and self._fwd_sock is not None:
            try:
                self._fwd_sock.sendto(data, self._forward_addr)
                self.stats.cmds_fwd += 1
                self.stats.bytes_fwd += len(data)
            except OSError as exc:
                self.get_logger().warn(
                    f"forward failed: {exc}", throttle_duration_sec=2.0)

        if not self._publish_cmd_vel:
            return

        # Two accepted encodings: plain JSON (dashboard "oai" mode payload),
        # or the same binary envelope we use for uplinks (kind=command).
        # We always try JSON first because the existing dashboard already
        # speaks it.
        intent: Optional[dict] = None
        try:
            msg = json.loads(data.decode("utf-8"))
            if isinstance(msg, dict):
                intent = msg
        except (UnicodeDecodeError, json.JSONDecodeError):
            pass

        if intent is None:
            unpacked = unpack_uplink(data)
            if unpacked and unpacked[0] == KIND_POSE:
                intent = unpacked[1]

        if intent is None:
            self.stats.cmds_rx_bad += 1
            return

        try:
            lx = float(intent.get("linear_x", 0.0))
            ly = float(intent.get("linear_y", 0.0))
            lz = float(intent.get("linear_z", 0.0))
            az = float(intent.get("angular_z", 0.0))
            estop = bool(intent.get("emergency_stop", False))
        except (TypeError, ValueError):
            self.stats.cmds_rx_bad += 1
            return

        self._last_intent = {"linear_x": lx, "linear_y": ly,
                             "linear_z": lz, "angular_z": az}
        self._last_intent_t = time.monotonic()
        self._estop_latched = estop
        self._stopped_already = False
        self.stats.cmds_rx += 1
        self.stats.last_cmd_t = time.time()

    def _on_publish_tick(self):
        out = Twist()
        if self._estop_latched:
            self.cmd_pub.publish(out)
            return
        age = (time.monotonic() - self._last_intent_t
               if self._last_intent_t else float("inf"))
        if age <= self._watchdog_s and self._last_intent:
            out.linear.x = self._last_intent.get("linear_x", 0.0)
            out.linear.y = self._last_intent.get("linear_y", 0.0)
            out.linear.z = self._last_intent.get("linear_z", 0.0)
            out.angular.z = self._last_intent.get("angular_z", 0.0)
            self.cmd_pub.publish(out)
            self._stopped_already = False
            return
        if not self._stopped_already:
            self.get_logger().warn(
                f"downlink watchdog: no cmd for {age*1000:.0f} ms; "
                f"publishing zero Twist",
                throttle_duration_sec=2.0)
            self.cmd_pub.publish(out)
            self._stopped_already = True

    # ----- uplink (ROS -> UDP) -------------------------------------------
    def _on_odom(self, msg: Odometry):
        p = msg.pose.pose.position
        o = msg.pose.pose.orientation
        t = msg.twist.twist
        siny_cosp = 2.0 * (o.w * o.z + o.x * o.y)
        cosy_cosp = 1.0 - 2.0 * (o.y * o.y + o.z * o.z)
        yaw = math.atan2(siny_cosp, cosy_cosp)
        header = {
            "stamp": time.time(),
            "frame_id": msg.header.frame_id,
            "x": float(p.x), "y": float(p.y), "z": float(p.z),
            "yaw": yaw,
            "linear_x": float(t.linear.x),
            "linear_y": float(t.linear.y),
            "linear_z": float(t.linear.z),
            "angular_z": float(t.angular.z),
        }
        packet = pack_uplink(KIND_POSE, header, b"")
        try:
            self._tx_sock.sendto(packet, self.operator_addr)
            self.stats.pose_tx += 1
            self.stats.bytes_tx += len(packet)
        except OSError as exc:
            self.get_logger().warn(
                f"pose uplink failed: {exc}", throttle_duration_sec=2.0)

    def _subscribe_camera_pair(self, base: str, qos: QoSProfile):
        self.create_subscription(
            CompressedImage, base + "/compressed",
            lambda m, b=base: self._on_compressed(m, b), qos,
        )
        self.create_subscription(
            Image, base,
            lambda m, b=base: self._on_raw(m, b), qos,
        )
        self.get_logger().info(f"  uplink camera subscribed: {base}(+/compressed)")

    def _maybe_emit_camera(self, base: str, jpeg: bytes, w: int, h: int,
                           variant: str):
        now = time.monotonic()
        if now - self._last_cam_emit_at.get(base, 0.0) < self._cam_period:
            return
        self._last_cam_emit_at[base] = now
        header = {
            "topic": base + ("/compressed" if variant == "compressed" else ""),
            "base": base,
            "stamp": time.time(),
            "width": int(w), "height": int(h),
        }
        packet = pack_uplink(KIND_CAMERA, header, jpeg)
        try:
            self._tx_sock.sendto(packet, self.operator_addr)
            self.stats.camera_tx += 1
            self.stats.bytes_tx += len(packet)
        except OSError as exc:
            self.get_logger().warn(
                f"camera uplink failed (len={len(packet)}): {exc}",
                throttle_duration_sec=2.0)

    def _on_compressed(self, msg: CompressedImage, base: str):
        data = bytes(msg.data)
        if not data:
            return
        fmt = (msg.format or "").lower()
        if "jpeg" in fmt or "jpg" in fmt or fmt == "":
            self._maybe_emit_camera(base, data, 0, 0, "compressed")
            return
        if not _HAVE_CV2:
            return
        arr = np.frombuffer(data, dtype=np.uint8)
        img = cv2.imdecode(arr, cv2.IMREAD_COLOR)
        if img is None:
            return
        ok, enc = cv2.imencode(
            ".jpg", img, [int(cv2.IMWRITE_JPEG_QUALITY), self._jpeg_quality])
        if ok:
            self._maybe_emit_camera(base, enc.tobytes(),
                                    img.shape[1], img.shape[0], "compressed")

    def _on_raw(self, msg: Image, base: str):
        if not _HAVE_CV2:
            return
        h, w = msg.height, msg.width
        if h == 0 or w == 0 or not msg.data:
            return
        enc = msg.encoding.lower()
        raw = bytes(msg.data)
        try:
            if enc in ("rgb8", "8uc3"):
                img = np.frombuffer(raw, dtype=np.uint8, count=h*w*3).reshape((h, w, 3))
                if enc == "rgb8":
                    img = cv2.cvtColor(img, cv2.COLOR_RGB2BGR)
            elif enc == "bgr8":
                img = np.frombuffer(raw, dtype=np.uint8, count=h*w*3).reshape((h, w, 3))
            elif enc in ("rgba8", "8uc4"):
                img4 = np.frombuffer(raw, dtype=np.uint8, count=h*w*4).reshape((h, w, 4))
                img = cv2.cvtColor(img4, cv2.COLOR_RGBA2BGR)
            elif enc == "bgra8":
                img4 = np.frombuffer(raw, dtype=np.uint8, count=h*w*4).reshape((h, w, 4))
                img = cv2.cvtColor(img4, cv2.COLOR_BGRA2BGR)
            else:
                return
        except ValueError:
            return
        ok, encoded = cv2.imencode(
            ".jpg", img, [int(cv2.IMWRITE_JPEG_QUALITY), self._jpeg_quality])
        if ok:
            self._maybe_emit_camera(base, encoded.tobytes(),
                                    img.shape[1], img.shape[0], "raw")

    # ----- diagnostics ---------------------------------------------------
    def _camera_rx_loop(self):
        """Receive IST5 camera frames over UDP (from the 5G radio path)
        and relay them to the operator as-is."""
        while not self._stop.is_set():
            try:
                data, _addr = self._cam_rx_sock.recvfrom(65536)
            except socket.timeout:
                continue
            except OSError as exc:
                self.get_logger().warn(f"camera-rx error: {exc}")
                continue
            try:
                self._tx_sock.sendto(data, self.operator_addr)
                self.stats.camera_tx += 1
                self.stats.bytes_tx += len(data)
            except OSError as exc:
                self.get_logger().warn(
                    f"camera relay failed: {exc}", throttle_duration_sec=2.0)

    def _log_stats(self):
        s = self.stats
        fwd_part = (f"  forward: cmds={s.cmds_fwd} bytes={s.bytes_fwd}"
                    if self._forward_addr is not None else "")
        self.get_logger().info(
            f"stats  downlink: cmds={s.cmds_rx} bad={s.cmds_rx_bad} bytes={s.bytes_rx}"
            f"{fwd_part}  "
            f"uplink: pose={s.pose_tx} camera={s.camera_tx} bytes={s.bytes_tx}")

    def shutdown(self):
        self._stop.set()
        try: self._rx_sock.close()
        except Exception: pass
        try: self._tx_sock.close()
        except Exception: pass
        if self._fwd_sock is not None:
            try: self._fwd_sock.close()
            except Exception: pass
        if self._cam_rx_sock is not None:
            try: self._cam_rx_sock.close()
            except Exception: pass


# ---------------------------------------------------------------------------
def csv(s: str) -> list[str]:
    return [t.strip() for t in s.split(",") if t.strip()]


def main() -> int:
    ap = argparse.ArgumentParser(description=__doc__.split("\n\n", 1)[0])
    ap.add_argument("--listen-host", default=os.environ.get("UE_LISTEN_HOST", "0.0.0.0"),
                    help="UDP listener bind address (UE side)")
    ap.add_argument("--listen-port", type=int,
                    default=int(os.environ.get("UE_LISTEN_PORT", "16000")))
    ap.add_argument("--operator-host", default=os.environ.get("OPERATOR_HOST", "127.0.0.1"),
                    help="dashboard's external IP (uplink target)")
    ap.add_argument("--operator-port", type=int,
                    default=int(os.environ.get("OPERATOR_PORT", "17500")))
    ap.add_argument("--cmd-vel-topic", default=os.environ.get("CMD_VEL_TOPIC", "/cmd_vel"))
    ap.add_argument("--odom-topics",
                    default=os.environ.get("ODOM_TOPICS", "/odom,/chassis/odom"),
                    type=csv)
    ap.add_argument("--camera-topics",
                    default=os.environ.get(
                        "CAMERA_TOPICS",
                        # front_hawk pair: what the Nova Carter USD declares
                        # if the OmniGraph camera publishers are active.
                        "/Nova_Carter_ROS/front_hawk/left/image_raw,"
                        "/Nova_Carter_ROS/front_hawk/right/image_raw,"
                        # front_stereo_camera: alternate camera prim that
                        # publishes by default in some Nova Carter scenes
                        # even when front_hawk is silent.
                        "/front_stereo_camera/left/image_raw,"
                        "/front_stereo_camera/right/image_raw"),
                    type=csv)
    ap.add_argument("--camera-fps-limit", type=float,
                    default=float(os.environ.get("CAMERA_FPS_LIMIT", "10")))
    ap.add_argument("--jpeg-quality", type=int,
                    default=int(os.environ.get("CAMERA_JPEG_QUALITY", "60")))
    ap.add_argument("--cmd-publish-rate-hz", type=float, default=30.0)
    ap.add_argument("--watchdog-ms", type=float, default=500.0)
    ap.add_argument("--forward-udp-host",
                    default=os.environ.get("FORWARD_UDP_HOST", ""),
                    help="If set, raw downlink UDP is forwarded to this host. "
                         "Used to chain the network-aware broker after the radio.")
    ap.add_argument("--forward-udp-port", type=int,
                    default=int(os.environ.get("FORWARD_UDP_PORT", "0")))
    ap.add_argument("--no-publish-cmd-vel", action="store_true",
                    default=os.environ.get("NO_PUBLISH_CMD_VEL", "") == "1",
                    help="Suppress the bridge's /cmd_vel publisher (use when "
                         "a downstream node publishes /cmd_vel from the "
                         "forwarded UDP stream).")
    ap.add_argument("--camera-uplink-listen-port", type=int,
                    default=int(os.environ.get("CAMERA_UPLINK_LISTEN_PORT", "0")),
                    help="UDP port to ingest already-IST5-framed camera "
                         "packets that arrived over the 5G radio. 0 disables.")
    args = ap.parse_args()

    forward_addr = None
    if args.forward_udp_host and args.forward_udp_port > 0:
        forward_addr = (args.forward_udp_host, args.forward_udp_port)

    rclpy.init(args=None)
    node = UeBridge(
        listen_host=args.listen_host, listen_port=args.listen_port,
        operator_host=args.operator_host, operator_port=args.operator_port,
        cmd_vel_topic=args.cmd_vel_topic,
        odom_topics=args.odom_topics,
        camera_topics=args.camera_topics,
        camera_fps_limit=args.camera_fps_limit,
        jpeg_quality=args.jpeg_quality,
        watchdog_ms=args.watchdog_ms,
        cmd_publish_rate_hz=args.cmd_publish_rate_hz,
        forward_addr=forward_addr,
        publish_cmd_vel=not args.no_publish_cmd_vel,
        camera_uplink_listen_port=args.camera_uplink_listen_port,
    )

    def _stop(*_):
        node.get_logger().info("shutting down")
        node.shutdown()
        rclpy.shutdown()
    signal.signal(signal.SIGINT, _stop)
    signal.signal(signal.SIGTERM, _stop)

    executor = SingleThreadedExecutor()
    executor.add_node(node)
    try:
        executor.spin()
    except KeyboardInterrupt:
        pass
    finally:
        node.shutdown()
        try: node.destroy_node()
        except Exception: pass
        try: rclpy.shutdown()
        except Exception: pass
    return 0


if __name__ == "__main__":
    sys.exit(main())
