#!/usr/bin/env python3
"""
Host-side synthetic odometry publisher.

Publishes ``nav_msgs/Odometry`` on ``/odom`` (also ``/chassis/odom`` since
the dashboard listens to both) at ``--hz`` Hz. The simulated robot drives a
slow Lissajous path so the dashboard's x / y / yaw / speed panels update
visibly.

This is meant to be a stand-in for whatever Isaac Sim or your real robot
would normally publish. Run it on the host so the dashboard has live pose
data while you iterate on other things.

Usage:
    source /opt/ros/humble/setup.bash
    python3 scripts/publish_demo_odom.py
    python3 scripts/publish_demo_odom.py --hz 30 --radius 2.0 --speed 0.4
"""
from __future__ import annotations

import argparse
import math
import signal
import sys
import time

import rclpy
from rclpy.node import Node
from rclpy.qos import HistoryPolicy, QoSProfile, ReliabilityPolicy
from nav_msgs.msg import Odometry


def yaw_to_quat(yaw: float):
    return (0.0, 0.0, math.sin(yaw / 2.0), math.cos(yaw / 2.0))


class DemoOdomPublisher(Node):
    def __init__(self, hz: float, radius: float, speed: float,
                 topics=("/odom", "/chassis/odom"),
                 frame_id="odom", child_frame_id="base_link"):
        super().__init__("demo_odom_publisher")
        self.hz = hz
        self.radius = radius
        self.speed = speed
        self.frame_id = frame_id
        self.child_frame_id = child_frame_id
        qos = QoSProfile(
            reliability=ReliabilityPolicy.BEST_EFFORT,
            history=HistoryPolicy.KEEP_LAST,
            depth=5,
        )
        self.pubs = [self.create_publisher(Odometry, t, qos) for t in topics]
        for t in topics:
            self.get_logger().info(f"publishing nav_msgs/Odometry on {t}")
        self.start_t = time.monotonic()
        self._prev_pose = None
        self.create_timer(1.0 / max(0.1, hz), self._tick)
        self._n = 0

    def _tick(self):
        t = time.monotonic() - self.start_t

        # Lissajous: x = R sin(speed t), y = R sin(2 speed t)/2.
        omega = self.speed
        x = self.radius * math.sin(omega * t)
        y = (self.radius / 2.0) * math.sin(2.0 * omega * t)

        # Tangent vector for yaw.
        dx = self.radius * omega * math.cos(omega * t)
        dy = self.radius * omega * math.cos(2.0 * omega * t)
        yaw = math.atan2(dy, dx)

        # Linear x in the body frame.
        speed_mag = math.hypot(dx, dy)
        if self._prev_pose is None:
            yaw_rate = 0.0
            self._prev_pose = (x, y, yaw, t)
        else:
            _, _, prev_yaw, prev_t = self._prev_pose
            dt = max(1e-3, t - prev_t)
            # unwrap yaw
            dy_yaw = math.atan2(math.sin(yaw - prev_yaw), math.cos(yaw - prev_yaw))
            yaw_rate = dy_yaw / dt
            self._prev_pose = (x, y, yaw, t)

        msg = Odometry()
        stamp = self.get_clock().now().to_msg()
        msg.header.stamp = stamp
        msg.header.frame_id = self.frame_id
        msg.child_frame_id = self.child_frame_id
        msg.pose.pose.position.x = x
        msg.pose.pose.position.y = y
        msg.pose.pose.position.z = 0.0
        qx, qy, qz, qw = yaw_to_quat(yaw)
        msg.pose.pose.orientation.x = qx
        msg.pose.pose.orientation.y = qy
        msg.pose.pose.orientation.z = qz
        msg.pose.pose.orientation.w = qw
        msg.twist.twist.linear.x = speed_mag
        msg.twist.twist.linear.y = 0.0
        msg.twist.twist.linear.z = 0.0
        msg.twist.twist.angular.z = yaw_rate

        for p in self.pubs:
            p.publish(msg)

        self._n += 1
        if self._n in (1, 30, 100) or self._n % 300 == 0:
            self.get_logger().info(
                f"odom n={self._n}  x={x:+.2f} y={y:+.2f} "
                f"yaw={math.degrees(yaw):+.1f}deg speed={speed_mag:.2f} m/s")


def main() -> int:
    ap = argparse.ArgumentParser()
    ap.add_argument("--hz", type=float, default=30.0)
    ap.add_argument("--radius", type=float, default=1.5,
                    help="path radius in metres")
    ap.add_argument("--speed", type=float, default=0.3,
                    help="angular speed of the parameterisation (rad/s)")
    args = ap.parse_args()

    rclpy.init(args=None)
    node = DemoOdomPublisher(args.hz, args.radius, args.speed)

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
