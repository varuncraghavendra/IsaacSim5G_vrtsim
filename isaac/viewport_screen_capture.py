#!/usr/bin/env python3
"""
Host-side screen capture of the Isaac Sim FPV viewport.

This script runs on the operator host — **outside** Isaac Sim. It does
not require Script Editor access and does not need any Python code to be
loaded into Isaac. Use this when you've added a second viewport to Isaac
manually (the only thing you touch inside Isaac).

Pipeline:

    [your screen]
        │ X11 capture of a Isaac window region (mss)
        ▼
    JPEG encode (PIL)
        │
        ▼
    IST5 envelope (KIND_CAMERA)
        │ UDP datagram
        ▼
    robot-ns veth ingress (192.168.244.2:17600 by default)
        │ robot_netns_proxy relays to oaitun_ue1
        ▼
    5G UL (PUSCH) -> gNB -> UPF
        │
        ▼
    operator dashboard (drone_app.py) UL listener (UDP :17500)
        │
        ▼
    browser camera panel

Region selection (in priority order):

    1. ``CAPTURE_REGION="x,y,w,h"`` env var — pixel rectangle of the
       desktop you want captured.

    2. ``CAPTURE_WINDOW`` env var — xdotool window-name substring to
       look up. Defaults to "Isaac Sim" so the entire Kit window is
       captured. The Iris FPV is then whatever you've parked Viewport 2
       on inside Isaac.

    3. Full primary monitor.

You can also pass ``--region`` / ``--window`` on the command line.

Dependencies:
    pip install mss pillow
    xdotool   (for --window auto-detect; not required for --region)
"""
from __future__ import annotations

import argparse
import io
import json
import os
import shutil
import socket
import struct
import subprocess
import sys
import time
from dataclasses import dataclass
from typing import Optional

try:
    from mss import mss
except ImportError as exc:
    print("[capture] ERROR: 'mss' not installed (pip install mss)", file=sys.stderr)
    raise

try:
    from PIL import Image
except ImportError as exc:
    print("[capture] ERROR: 'Pillow' not installed (pip install pillow)", file=sys.stderr)
    raise


# Mirrors oai_bridge/protocol.py and isaac/drone_telemetry.py.
UPLINK_MAGIC   = 0x49535435   # 'IST5'
KIND_CAMERA    = 2
UPLINK_HDR_FMT = "<IBBHI"

DSCP_AF41 = 0x88


def pack_ist5(kind: int, header: dict, body: bytes) -> bytes:
    hdr = json.dumps(header, separators=(",", ":")).encode("utf-8")
    return (struct.pack(UPLINK_HDR_FMT, UPLINK_MAGIC, kind, 0, len(hdr), len(body))
            + hdr + body)


# ---------------------------------------------------------------------------
@dataclass
class Region:
    left: int
    top: int
    width: int
    height: int

    def as_mss_dict(self) -> dict:
        return {"left": self.left, "top": self.top,
                "width": self.width, "height": self.height}

    def __str__(self) -> str:
        return f"({self.left},{self.top}) {self.width}x{self.height}"


def parse_region_arg(s: str) -> Region:
    """Parse 'x,y,w,h' into a Region."""
    parts = [p.strip() for p in s.split(",")]
    if len(parts) != 4:
        raise ValueError(f"region must be 'x,y,w,h', got: {s!r}")
    x, y, w, h = (int(p) for p in parts)
    if w <= 0 or h <= 0:
        raise ValueError(f"region dims must be positive, got w={w} h={h}")
    return Region(x, y, w, h)


def find_window_region(name_substring: str) -> Optional[Region]:
    """Use xdotool to find a window whose title contains ``name_substring``
    and return its on-screen rectangle. Returns None if xdotool isn't
    installed or no matching window exists."""
    if shutil.which("xdotool") is None:
        return None
    try:
        out = subprocess.check_output(
            ["xdotool", "search", "--name", name_substring],
            stderr=subprocess.DEVNULL).decode().strip().split()
    except subprocess.CalledProcessError:
        return None
    if not out:
        return None
    best: Optional[Region] = None
    for wid in out:
        try:
            geom = subprocess.check_output(
                ["xdotool", "getwindowgeometry", "--shell", wid],
                stderr=subprocess.DEVNULL).decode()
        except subprocess.CalledProcessError:
            continue
        d = {}
        for line in geom.splitlines():
            if "=" not in line: continue
            k, v = line.split("=", 1)
            try:
                d[k.strip()] = int(v.strip())
            except ValueError:
                pass
        try:
            r = Region(d["X"], d["Y"], d["WIDTH"], d["HEIGHT"])
        except KeyError:
            continue
        # Pick the biggest matching window (the main Kit frame).
        if best is None or (r.width * r.height) > (best.width * best.height):
            best = r
    return best


def default_monitor_region() -> Region:
    with mss() as sct:
        mon = sct.monitors[1]   # 0 is "all monitors", 1 is the primary.
    return Region(mon["left"], mon["top"], mon["width"], mon["height"])


# ---------------------------------------------------------------------------
def main() -> int:
    ap = argparse.ArgumentParser(description=__doc__.split("\n\n", 1)[0])
    ap.add_argument("--target-host",
                    default=os.environ.get("CAPTURE_TARGET_HOST",
                                           "192.168.244.2"),
                    help="IST5 UDP destination (robot-ns veth ingress by "
                         "default; the proxy ships it over OAI UL).")
    ap.add_argument("--target-port", type=int,
                    default=int(os.environ.get("CAPTURE_TARGET_PORT", "17600")))
    ap.add_argument("--topic",
                    default=os.environ.get("CAPTURE_TOPIC",
                                           "/isaac/viewport2"))
    ap.add_argument("--fps", type=float,
                    default=float(os.environ.get("CAPTURE_FPS", "8")))
    ap.add_argument("--jpeg-quality", type=int,
                    default=int(os.environ.get("CAPTURE_JPEG_QUALITY", "60")))
    ap.add_argument("--max-dim", type=int,
                    default=int(os.environ.get("CAPTURE_MAX_DIM", "960")),
                    help="Longest-side pixel cap before JPEG encode.")
    ap.add_argument("--max-payload", type=int,
                    default=int(os.environ.get("CAPTURE_MAX_PAYLOAD", "60000")),
                    help="Hard cap on the JPEG body to stay under UDP MTU.")
    ap.add_argument("--region",
                    default=os.environ.get("CAPTURE_REGION", ""),
                    help="Explicit 'x,y,w,h' on the desktop.")
    ap.add_argument("--window",
                    default=os.environ.get("CAPTURE_WINDOW", "Isaac Sim"),
                    help="Window-title substring for xdotool auto-detect.")
    args = ap.parse_args()

    # Resolve capture region.
    region: Optional[Region] = None
    if args.region:
        region = parse_region_arg(args.region)
        print(f"[capture] explicit region: {region}")
    if region is None and args.window:
        region = find_window_region(args.window)
        if region is not None:
            print(f"[capture] window '{args.window}' geometry: {region}")
    if region is None:
        region = default_monitor_region()
        print(f"[capture] full primary monitor: {region}")

    sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
    try:
        sock.setsockopt(socket.IPPROTO_IP, socket.IP_TOS, DSCP_AF41)
    except OSError:
        pass
    addr = (args.target_host, args.target_port)
    print(f"[capture] UDP target: {addr[0]}:{addr[1]}  topic={args.topic}  "
          f"fps={args.fps}")

    period = 1.0 / max(1.0, args.fps)
    next_t = time.monotonic()
    frames_sent = 0
    errors = 0
    last_log = time.monotonic()

    with mss() as sct:
        mss_region = region.as_mss_dict()
        while True:
            t0 = time.monotonic()
            try:
                shot = sct.grab(mss_region)
                # mss returns BGRA bytes; Pillow handles that directly.
                img = Image.frombytes("RGB", shot.size,
                                      shot.rgb)   # already converted RGB
                w, h = img.size
                if max(w, h) > args.max_dim:
                    scale = args.max_dim / float(max(w, h))
                    img = img.resize(
                        (max(1, int(w * scale)), max(1, int(h * scale))),
                        Image.BILINEAR)
                    w, h = img.size

                quality = args.jpeg_quality
                while True:
                    buf = io.BytesIO()
                    img.save(buf, format="JPEG", quality=quality, optimize=False)
                    body = buf.getvalue()
                    if len(body) <= args.max_payload or quality <= 20:
                        break
                    quality -= 10
                if len(body) > args.max_payload:
                    errors += 1
                    if errors <= 5:
                        print(f"[capture] frame too large ({len(body)} B); drop")
                else:
                    header = {
                        "topic":  args.topic,
                        "base":   args.topic,
                        "stamp":  time.time(),
                        "width":  int(w),
                        "height": int(h),
                        "fmt":    "jpeg",
                    }
                    pkt = pack_ist5(KIND_CAMERA, header, body)
                    sock.sendto(pkt, addr)
                    frames_sent += 1
            except Exception as exc:
                errors += 1
                if errors <= 5:
                    print(f"[capture] grab/send error: {exc}")

            now = time.monotonic()
            if now - last_log >= 5.0:
                print(f"[capture] sent={frames_sent} err={errors} "
                      f"region={region}")
                last_log = now

            next_t += period
            sleep_for = next_t - time.monotonic()
            if sleep_for > 0:
                time.sleep(sleep_for)
            else:
                next_t = time.monotonic()


if __name__ == "__main__":
    try:
        sys.exit(main() or 0)
    except KeyboardInterrupt:
        sys.exit(0)
