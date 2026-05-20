#!/usr/bin/env python3
"""
ROS 2 diagnostic for IsaacSim5G.

Prints a structured report of every topic visible from this host, including
publisher count, subscriber count, and (for image topics) measured rate.
Useful when the dashboard says "0 publishers" and you want to know whether
Isaac Sim is publishing anywhere at all under a different name.

Usage:
    source /opt/ros/humble/setup.bash
    python3 scripts/diagnose_ros.py
    python3 scripts/diagnose_ros.py --probe-rate /front_hawk/left/image_raw

The --probe-rate option subscribes to a topic for 3 s and reports the
measured Hz (helpful when ros2 topic hz hangs on lazy publishers).
"""

from __future__ import annotations

import argparse
import sys
import time
from typing import Optional

import rclpy
from rclpy.node import Node
from rclpy.qos import HistoryPolicy, QoSProfile, ReliabilityPolicy


IMAGE_TYPES = ("sensor_msgs/msg/Image", "sensor_msgs/msg/CompressedImage")

# Topic names commonly emitted by Isaac Sim's Nova Carter / Hawk camera graphs.
# We surface them in the report even if 0 pub/0 sub so the user sees the
# expected names side-by-side with what Isaac is actually doing.
NOVA_CARTER_CANDIDATES = [
    "/front_hawk/left/image_raw",
    "/front_hawk/right/image_raw",
    "/front_hawk/left/image_raw/compressed",
    "/front_hawk/right/image_raw/compressed",
    "/front_stereo_camera/left/image_raw",
    "/front_stereo_camera/right/image_raw",
    "/front_stereo_camera/left/image_raw/compressed",
    "/front_stereo_camera/right/image_raw/compressed",
    "/back_stereo_camera/left/image_raw",
    "/left_stereo_camera/left/image_raw",
    "/right_stereo_camera/left/image_raw",
    "/rgb",
    "/rgb_left",
    "/Nova_Carter_ROS/front_hawk/left/image_raw",
]


def fmt_row(name: str, type_str: str, pubs: int, subs: int, note: str = "") -> str:
    pubs_s = f"\033[32m{pubs}\033[0m" if pubs > 0 else f"\033[31m{pubs}\033[0m"
    subs_s = f"{subs}"
    return f"  {pubs_s:>4} pub  {subs_s:>4} sub   {type_str:<36}  {name}{note}"


def discover(node: Node) -> dict[str, dict]:
    """Return topic_name -> {types, publishers, subscribers}."""
    out: dict[str, dict] = {}
    for name, types in node.get_topic_names_and_types():
        out[name] = {
            "types": types,
            "publishers": node.count_publishers(name),
            "subscribers": node.count_subscribers(name),
        }
    return out


def probe_rate(node: Node, topic: str, duration: float) -> tuple[int, Optional[str]]:
    """Subscribe to ``topic`` for ``duration`` seconds and count messages."""
    from sensor_msgs.msg import CompressedImage, Image

    # Discover its type
    tnt = dict(node.get_topic_names_and_types())
    types = tnt.get(topic, [])
    if not types:
        return (-1, f"topic {topic} not visible to ROS daemon")
    msg_type = None
    if "sensor_msgs/msg/CompressedImage" in types:
        msg_type = CompressedImage
    elif "sensor_msgs/msg/Image" in types:
        msg_type = Image
    else:
        return (-1, f"topic {topic} type {types[0]} not supported by probe")

    count = 0
    qos = QoSProfile(
        reliability=ReliabilityPolicy.BEST_EFFORT,
        history=HistoryPolicy.KEEP_LAST,
        depth=5,
    )

    def cb(_msg):
        nonlocal count
        count += 1

    sub = node.create_subscription(msg_type, topic, cb, qos)
    end = time.monotonic() + duration
    while time.monotonic() < end:
        rclpy.spin_once(node, timeout_sec=0.1)
    node.destroy_subscription(sub)
    return (count, None)


def main() -> int:
    parser = argparse.ArgumentParser(description="IsaacSim5G ROS 2 diagnostic.")
    parser.add_argument("--probe-rate", metavar="TOPIC",
                        help="Subscribe to TOPIC for 3 s and report observed rate.")
    parser.add_argument("--probe-duration", type=float, default=3.0)
    parser.add_argument("--no-color", action="store_true")
    args = parser.parse_args()

    if args.no_color:
        # neutralise ANSI codes
        global fmt_row
        _orig = fmt_row
        def _nc(*a, **kw):
            line = _orig(*a, **kw)
            return line.replace("\033[32m", "").replace("\033[31m", "").replace("\033[0m", "")
        fmt_row = _nc  # type: ignore

    rclpy.init(args=None)
    node = Node("isaacsim5g_diagnostic")
    try:
        # Sleep briefly so DDS discovery has time to populate.
        end = time.monotonic() + 1.5
        while time.monotonic() < end:
            rclpy.spin_once(node, timeout_sec=0.1)

        topics = discover(node)

        print(f"\n=== {len(topics)} topics visible to ROS daemon ===\n")
        # All topics
        for name in sorted(topics):
            t = topics[name]
            type_str = t["types"][0] if t["types"] else "?"
            print(fmt_row(name, type_str, t["publishers"], t["subscribers"]))

        # Image topics specifically
        image_topics = {n: t for n, t in topics.items()
                        if any(x in t["types"] for x in IMAGE_TYPES)}
        print(f"\n=== {len(image_topics)} image-typed topics ===\n")
        if not image_topics:
            print("  (none) -- Isaac Sim is not currently publishing any "
                  "Image or CompressedImage topic.")
        for name, t in sorted(image_topics.items()):
            type_str = t["types"][0]
            note = ""
            if t["publishers"] == 0:
                note = "    \033[33m[topic exists but no publisher]\033[0m" if not args.no_color \
                       else "    [topic exists but no publisher]"
            print(fmt_row(name, type_str, t["publishers"], t["subscribers"], note))

        # Nova Carter candidates
        print(f"\n=== {len(NOVA_CARTER_CANDIDATES)} expected Nova Carter camera names ===\n")
        for name in NOVA_CARTER_CANDIDATES:
            t = topics.get(name)
            if t is None:
                star = ""
                print(fmt_row(name, "(not registered)", 0, 0, star))
            else:
                type_str = t["types"][0] if t["types"] else "?"
                print(fmt_row(name, type_str, t["publishers"], t["subscribers"]))

        if args.probe_rate:
            print(f"\n=== probing rate on {args.probe_rate} for {args.probe_duration}s ===\n")
            count, err = probe_rate(node, args.probe_rate, args.probe_duration)
            if err:
                print(f"  error: {err}")
            else:
                rate = count / args.probe_duration
                ok = count > 0
                marker = ("\033[32m✓\033[0m" if ok else "\033[31m✗\033[0m") \
                         if not args.no_color else ("OK" if ok else "FAIL")
                print(f"  {marker}  received {count} messages in {args.probe_duration}s "
                      f"-> {rate:.2f} Hz")

        print()
        return 0
    finally:
        try:
            node.destroy_node()
        except Exception:
            pass
        try:
            rclpy.shutdown()
        except Exception:
            pass


if __name__ == "__main__":
    sys.exit(main())
