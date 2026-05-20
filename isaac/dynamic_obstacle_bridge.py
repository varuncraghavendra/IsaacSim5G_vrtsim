#!/usr/bin/env python3
"""
Dynamic obstacle bridge.

Watches a JSON file describing line-segment obstacles between the robot and a
fixed gNB (5G base-station) position. Each time /odom updates, the node:

  1.  Recomputes line-of-sight by testing every robot-gNB segment against
      every obstacle segment.
  2.  Synthesises path loss, SNR, latency, jitter and PER samples that
      reflect LOS vs NLOS.
  3.  Sends the resulting NetworkMetrics-shaped JSON to the metrics-ingress
      UDP port so the broker can react to it.

The obstacles file is hot-reloaded whenever its mtime changes, so an external
process (or a human editing the JSON) can move walls around in real time.

Example obstacles file (see config/obstacles_example.json):

    {
      "segments": [
        {"x1":  1.0, "y1": -2.0, "x2":  1.0, "y2": 2.0},
        {"x1": -3.0, "y1":  1.5, "x2": -1.0, "y2": 1.5}
      ]
    }
"""

import argparse
import json
import math
import os
import socket

import rclpy
from nav_msgs.msg import Odometry
from rclpy.node import Node


def _ccw(ax, ay, bx, by, cx, cy):
    return (cy - ay) * (bx - ax) > (by - ay) * (cx - ax)


def _segments_intersect(a, b, c, d):
    return (
        _ccw(*a, *c, *d) != _ccw(*b, *c, *d)
        and _ccw(*a, *b, *c) != _ccw(*a, *b, *d)
    )


class DynamicObstacleBridge(Node):
    def __init__(
        self,
        obstacles_file: str,
        metrics_host: str,
        metrics_port: int,
        gnb_x: float,
        gnb_y: float,
    ):
        super().__init__("dynamic_obstacle_bridge")
        self.obstacles_file = obstacles_file
        self.metrics_host = metrics_host
        self.metrics_port = metrics_port
        self.gnb_x = gnb_x
        self.gnb_y = gnb_y
        self.last_reload = 0.0
        self.segments: list = []
        self._reload_if_needed(force=True)

        self.sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
        self.sub = self.create_subscription(Odometry, "/odom", self._on_odom, 20)

        self.get_logger().info(
            f"Dynamic obstacle bridge: obstacles={obstacles_file} "
            f"gNB=({gnb_x:.2f},{gnb_y:.2f}) metrics_target={metrics_host}:{metrics_port}"
        )

    def _reload_if_needed(self, force: bool = False):
        if not os.path.exists(self.obstacles_file):
            self.segments = []
            return
        mtime = os.path.getmtime(self.obstacles_file)
        if not (force or mtime > self.last_reload):
            return
        try:
            with open(self.obstacles_file, "r", encoding="utf-8") as fh:
                data = json.load(fh)
        except (OSError, json.JSONDecodeError) as exc:
            self.get_logger().error(f"obstacle reload failed: {exc}")
            return
        self.segments = data.get("segments", [])
        self.last_reload = mtime
        self.get_logger().info(
            f"reloaded {len(self.segments)} obstacle segments from {self.obstacles_file}"
        )

    def _on_odom(self, msg: Odometry):
        self._reload_if_needed()
        rx = float(msg.pose.pose.position.x)
        ry = float(msg.pose.pose.position.y)
        robot = (rx, ry)
        gnb = (self.gnb_x, self.gnb_y)

        los = True
        for seg in self.segments:
            try:
                p1 = (float(seg["x1"]), float(seg["y1"]))
                p2 = (float(seg["x2"]), float(seg["y2"]))
            except (KeyError, TypeError, ValueError):
                continue
            if _segments_intersect(robot, gnb, p1, p2):
                los = False
                break

        dist = math.hypot(rx - self.gnb_x, ry - self.gnb_y)
        base_loss = 38.0 + 20.0 * math.log10(max(dist, 0.5))
        penalty = 0.0 if los else 18.0
        snr = max(-5.0, 35.0 - (base_loss + penalty))

        payload = {
            "one_way_latency_ms": 12.0 if los else 28.0,
            "round_trip_latency_ms": 24.0 if los else 56.0,
            "jitter_ms": 3.0 if los else 8.0,
            "packet_error_rate": 0.01 if los else 0.08,
            "snr_db": snr,
            "rtf": 0.95,
            "los": los,
            "path_loss_db": base_loss + penalty,
            "source": "dynamic_obstacle_bridge",
        }
        self.sock.sendto(
            json.dumps(payload).encode("utf-8"),
            (self.metrics_host, self.metrics_port),
        )


def main():
    parser = argparse.ArgumentParser(description="Dynamic obstacle / LOS metrics bridge.")
    parser.add_argument("--obstacles-file", required=True)
    parser.add_argument("--metrics-host", default="127.0.0.1")
    parser.add_argument("--metrics-port", type=int, default=16001)
    parser.add_argument("--gnb-x", type=float, default=0.0)
    parser.add_argument("--gnb-y", type=float, default=0.0)
    args = parser.parse_args()

    rclpy.init()
    node = DynamicObstacleBridge(
        args.obstacles_file,
        args.metrics_host,
        args.metrics_port,
        args.gnb_x,
        args.gnb_y,
    )
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
