#!/usr/bin/env python3
"""
Nova Carter <-> ROS 2 bridge.

Sits between the network-aware broker (publishes ``/network/cmd_vel``) and the
Isaac Sim Nova Carter articulation (subscribes to ``/cmd_vel`` via its Twist
subscriber action graph). Three jobs:

1.  RELAY    -- subscribe to ``--cmd-in-topic`` and republish on
                ``--cmd-out-topic`` at ``--publish-rate-hz``. The default
                pair is ``/network/cmd_vel`` -> ``/cmd_vel`` so that there
                is exactly one publisher on Isaac's input topic.

2.  WATCHDOG -- if no command has been received within ``--watchdog-ms``,
                publish a single zero Twist on the out topic. Guarantees the
                robot stops when the operator releases every key, even if a
                "stop" packet is lost upstream.

3.  STEP TIC -- emit ``/network/step_complete`` (std_msgs/Bool) at
                ``--step-rate-hz`` so the broker's barrier knows the
                downstream is alive.

No Python-side kinematics. Pose comes back on /odom directly from the Isaac
Odometry publisher graph.

If ``--cmd-in-topic`` and ``--cmd-out-topic`` are identical the bridge
detects the loop and switches to *watchdog-only mode*: it just listens and
emits a zero on timeout, never republishing (because that would feed back
into its own subscription and confuse the broker).
"""

import argparse
import time

import rclpy
from geometry_msgs.msg import Twist
from rclpy.node import Node
from rclpy.qos import HistoryPolicy, QoSProfile, ReliabilityPolicy
from std_msgs.msg import Bool


class NovaCarterBridge(Node):
    def __init__(
        self,
        robot_prim: str,
        cmd_in_topic: str,
        cmd_out_topic: str,
        step_topic: str,
        step_rate_hz: float,
        watchdog_ms: float,
        publish_rate_hz: float,
    ):
        super().__init__("nova_carter_bridge")
        self.robot_prim = robot_prim
        self.watchdog_s = max(0.0, watchdog_ms) / 1000.0
        self._self_loop = (cmd_in_topic == cmd_out_topic)

        reliable_qos = QoSProfile(
            reliability=ReliabilityPolicy.RELIABLE,
            history=HistoryPolicy.KEEP_LAST,
            depth=20,
        )

        self._last_cmd: Twist = Twist()
        self._last_cmd_t: float = 0.0
        self._stopped_already: bool = False

        self.cmd_sub = self.create_subscription(
            Twist, cmd_in_topic, self._on_cmd, reliable_qos,
        )
        self.cmd_pub = self.create_publisher(Twist, cmd_out_topic, reliable_qos)

        self.step_pub = self.create_publisher(Bool, step_topic, 20)
        self.create_timer(1.0 / max(1.0, float(step_rate_hz)), self._on_step_tick)

        # In normal (distinct-topic) mode we run a fixed-rate republisher so
        # Isaac's Twist subscriber always has a fresh value. In self-loop
        # mode we only run the watchdog tick.
        self._publish_period = 1.0 / max(1.0, float(publish_rate_hz))
        self.create_timer(self._publish_period, self._on_publish_tick)

        mode = "watchdog-only (self-loop detected)" if self._self_loop else "relay+watchdog"
        self.get_logger().info(
            f"Nova Carter bridge ready [{mode}]: in={cmd_in_topic} out={cmd_out_topic} "
            f"publish_rate={publish_rate_hz} Hz watchdog={watchdog_ms} ms "
            f"step={step_topic}@{step_rate_hz}Hz prim={robot_prim}"
        )

    # ----- callbacks -------------------------------------------------------
    def _on_cmd(self, msg: Twist):
        self._last_cmd = msg
        self._last_cmd_t = self._now()
        self._stopped_already = False

    def _on_publish_tick(self):
        age = self._now() - self._last_cmd_t if self._last_cmd_t else float("inf")

        if age <= self.watchdog_s:
            if not self._self_loop:
                self.cmd_pub.publish(self._last_cmd)
            return

        # Watchdog: emit a single explicit stop, then go quiet until a fresh
        # command arrives. Logging is throttled to avoid filling the journal.
        if not self._stopped_already:
            self.get_logger().warn(
                f"Watchdog: no /cmd_vel for {age*1000:.0f} ms, emitting zero twist.",
                throttle_duration_sec=2.0,
            )
            self.cmd_pub.publish(Twist())
            self._stopped_already = True

    def _on_step_tick(self):
        msg = Bool()
        msg.data = True
        self.step_pub.publish(msg)

    @staticmethod
    def _now() -> float:
        return time.monotonic()


def main():
    parser = argparse.ArgumentParser(description="Nova Carter cmd_vel relay + watchdog.")
    parser.add_argument("--robot-prim", default="/World/Nova_Carter")
    parser.add_argument("--cmd-in-topic", default="/network/cmd_vel",
                        help="Topic the broker publishes commands on.")
    parser.add_argument("--cmd-out-topic", default="/cmd_vel",
                        help="Topic Isaac Sim's articulation graph subscribes to.")
    parser.add_argument("--step-topic", default="/network/step_complete")
    parser.add_argument("--step-rate-hz", type=float, default=50.0)
    parser.add_argument("--publish-rate-hz", type=float, default=30.0,
                        help="Rate at which the cached Twist is re-emitted to Isaac.")
    parser.add_argument("--watchdog-ms", type=float, default=500.0,
                        help="If no fresh command arrives within this window the "
                             "bridge emits a single zero twist.")
    args = parser.parse_args()

    rclpy.init()
    node = NovaCarterBridge(
        robot_prim=args.robot_prim,
        cmd_in_topic=args.cmd_in_topic,
        cmd_out_topic=args.cmd_out_topic,
        step_topic=args.step_topic,
        step_rate_hz=args.step_rate_hz,
        watchdog_ms=args.watchdog_ms,
        publish_rate_hz=args.publish_rate_hz,
    )
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        try:
            node.cmd_pub.publish(Twist())
        except Exception:
            pass
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
