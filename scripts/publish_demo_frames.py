#!/usr/bin/env python3
"""
Host-side ROS 2 camera publisher for the IsaacSim5G dashboard.

Mirrors what ``isaac/fix_and_start_camera.py`` does from inside Isaac --
publishes synthetic frames on the same two topics the dashboard's
Front-left and Front-right panels are bound to:

    /Nova_Carter_ROS/front_hawk/left/image_raw   (+ /compressed)
    /Nova_Carter_ROS/front_hawk/right/image_raw  (+ /compressed)

Both raw sensor_msgs/Image and CompressedImage flavours are published, at
``--hz`` frames per second.

Usage:
    source /opt/ros/humble/setup.bash
    python3 scripts/publish_demo_frames.py
    python3 scripts/publish_demo_frames.py --hz 20 --resolution 800x600
"""
from __future__ import annotations

import argparse
import math
import signal
import sys
import time
from typing import List, Tuple

import numpy as np
import cv2

import rclpy
from rclpy.node import Node
from rclpy.qos import HistoryPolicy, QoSProfile, ReliabilityPolicy
from sensor_msgs.msg import CompressedImage, Image


TOPICS: List[Tuple[str, str, Tuple[int, int, int]]] = [
    # (ros_topic, label, gradient_tint)
    ("/Nova_Carter_ROS/front_hawk/left/image_raw",  "LEFT",  ( 50, 200, 240)),
    ("/Nova_Carter_ROS/front_hawk/right/image_raw", "RIGHT", (240, 130,  80)),
]


def synth_frame(label: str, tint: Tuple[int, int, int],
                resolution: Tuple[int, int], frame_n: int) -> np.ndarray:
    """Return an RGB uint8 frame with a moving sphere on a tinted gradient."""
    w, h = resolution
    img = np.zeros((h, w, 3), dtype=np.uint8)
    # tinted vertical gradient
    for y in range(h):
        s = y / h
        img[y, :] = (int(s * tint[0]), int(s * tint[1]), int(s * tint[2]))
    t = frame_n * 0.05
    cx = int(w / 2 + (w * 0.3) * math.sin(t))
    cy = int(h / 2 + (h * 0.2) * math.cos(t * 1.7))
    r = max(20, int(min(w, h) * 0.1))
    cv2.circle(img, (cx, cy), r, (250, 250, 250), -1)
    cv2.circle(img, (cx, cy), r, (40, 40, 40), 3)
    cv2.putText(img, label, (12, 38), cv2.FONT_HERSHEY_SIMPLEX,
                1.0, (255, 255, 255), 2)
    cv2.putText(img, f"demo  frame {frame_n}", (12, 68),
                cv2.FONT_HERSHEY_SIMPLEX, 0.55, (220, 255, 220), 2)
    # cv2 default is BGR; convert so the rgb8 msg carries true RGB.
    return cv2.cvtColor(img, cv2.COLOR_BGR2RGB)


class DemoCameraPublisher(Node):
    def __init__(self, resolution: Tuple[int, int], hz: float, jpeg_quality: int):
        super().__init__("demo_camera_publisher")
        self.resolution = resolution
        self.hz = hz
        self.jpeg_quality = jpeg_quality
        qos = QoSProfile(
            reliability=ReliabilityPolicy.BEST_EFFORT,
            history=HistoryPolicy.KEEP_LAST,
            depth=5,
        )
        self.streams = []
        for topic, label, tint in TOPICS:
            raw_pub = self.create_publisher(Image, topic, qos)
            cmp_pub = self.create_publisher(CompressedImage, topic + "/compressed", qos)
            self.streams.append({
                "topic": topic, "label": label, "tint": tint,
                "raw_pub": raw_pub, "cmp_pub": cmp_pub,
                "frame_id": topic.strip("/").replace("/", "_"),
                "n": 0,
            })
            self.get_logger().info(
                f"publishing {topic} (+ /compressed) as {label}")

        period = 1.0 / max(0.1, hz)
        self.create_timer(period, self._tick)

    def _tick(self):
        stamp = self.get_clock().now().to_msg()
        for s in self.streams:
            rgb = synth_frame(s["label"], s["tint"], self.resolution, s["n"])
            h, w = rgb.shape[:2]

            raw = Image()
            raw.header.stamp = stamp
            raw.header.frame_id = s["frame_id"]
            raw.height = h
            raw.width = w
            raw.encoding = "rgb8"
            raw.is_bigendian = 0
            raw.step = w * 3
            raw.data = rgb.tobytes()
            s["raw_pub"].publish(raw)

            bgr = cv2.cvtColor(rgb, cv2.COLOR_RGB2BGR)
            ok, enc = cv2.imencode(
                ".jpg", bgr,
                [int(cv2.IMWRITE_JPEG_QUALITY), self.jpeg_quality],
            )
            if ok:
                cmsg = CompressedImage()
                cmsg.header.stamp = stamp
                cmsg.header.frame_id = s["frame_id"]
                cmsg.format = "jpeg"
                cmsg.data = enc.tobytes()
                s["cmp_pub"].publish(cmsg)

            s["n"] += 1
            if s["n"] in (1, 15, 60) or s["n"] % 300 == 0:
                self.get_logger().info(
                    f"{s['topic']}: {s['n']} frames published")


def main() -> int:
    ap = argparse.ArgumentParser()
    ap.add_argument("--hz", type=float, default=15.0)
    ap.add_argument("--resolution", default="640x480",
                    help="WxH (default 640x480)")
    ap.add_argument("--jpeg-quality", type=int, default=70)
    args = ap.parse_args()
    w, h = (int(x) for x in args.resolution.lower().split("x", 1))

    rclpy.init(args=None)
    node = DemoCameraPublisher((w, h), args.hz, args.jpeg_quality)

    def _stop(*_):
        node.get_logger().info("shutting down")
        rclpy.shutdown()
    signal.signal(signal.SIGINT, _stop)
    signal.signal(signal.SIGTERM, _stop)

    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        try: node.destroy_node()
        except Exception: pass
    return 0


if __name__ == "__main__":
    sys.exit(main())
