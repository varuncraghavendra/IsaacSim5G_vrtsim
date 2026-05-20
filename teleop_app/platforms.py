"""
Platform definitions for the IsaacSim5G teleop dashboard.

A *platform* bundles every robot-specific decision in one place:

* the ROS 2 topics we subscribe to (camera, odom)
* how we drive it (Twist on /cmd_vel for Nova Carter, PX4 trajectory
  setpoints for the Iris quad, etc.)
* what controls the UI exposes (ground robot vs. quadcopter)
* the human-readable name shown in the dashboard

The dashboard's rest of the code is platform-agnostic: it asks the platform
how to publish a command (``platform.build_command_publishers(node)``) and
the platform returns a callable that takes the operator's intent
(linear_x, linear_y, linear_z, yaw, throttle, plus drone-specific extras)
and produces the right ROS 2 messages.

Two platforms are defined here:

* ``nova_carter``: standard ground-robot mecanum drive. Publishes Twist on
  /cmd_vel. Subscribes to /odom and a list of candidate camera topics
  (front_hawk variants).

* ``iris_drone``: Iris quadcopter via PX4 ROS 2 bridge (``micrortps_agent``
  or the modern ``px4_msgs`` bridge). Publishes
  geometry_msgs/TwistStamped on a configurable offboard-velocity topic
  (default ``/offboard_velocity_cmd`` -- adapt to your bridge's input). It
  also exposes ARM/DISARM/TAKEOFF/LAND commands via VehicleCommand on
  ``/fmu/in/vehicle_command``. Subscribes to ``/fmu/out/vehicle_odometry``
  for pose and the configured camera topic.

Pick a platform via ``--platform`` flag or the ``PLATFORM`` env var.
Default is ``nova_carter``.
"""

from __future__ import annotations

import math
from dataclasses import dataclass, field
from typing import Callable, Optional


# ---------------------------------------------------------------------------
@dataclass
class PlatformSpec:
    """Static configuration for a robot platform."""

    # ID and UI label
    name: str                          # e.g. "nova_carter"
    display: str                       # e.g. "Nova Carter (mecanum)"

    # ROS 2 wiring
    cmd_vel_topic: str
    odom_topics: tuple[str, ...]       # primary first, fallbacks after
    camera_topics: tuple[str, ...]     # all topics the node subscribes to

    # The browser renders one panel per entry of ``camera_panels``. Each
    # entry is (label, default_topic) -- the default topic must be one of
    # ``camera_topics`` (or any topic auto-discovered later). The user can
    # change a panel's topic at runtime through the panel's dropdown.
    camera_panels: tuple[tuple[str, str], ...] = ()

    # UI personality
    kind: str = "ground"               # "ground" | "drone"
    control_help: str = ""             # one-line text shown in dashboard
    has_throttle: bool = False
    has_arm: bool = False

    # Ranges for the dashboard sliders
    linear_max_mps: float = 1.5
    angular_max_rps: float = 2.5
    vertical_max_mps: float = 1.0      # drone only


NOVA_CARTER = PlatformSpec(
    name="nova_carter",
    display="Nova Carter (mecanum)",
    cmd_vel_topic="/cmd_vel",
    odom_topics=("/odom", "/chassis/odom"),
    camera_topics=(
        # Isaac-side Viewport 2 capture script (isaac/viewport_to_udp.py)
        # publishes frames under this synthetic topic name.
        "/isaac/viewport2",
        # Legacy ROS-side camera topics — kept in the dropdown so the
        # user can still pick them if their Isaac scene publishes them.
        "/Nova_Carter_ROS/front_hawk/left/image_raw",
        "/Nova_Carter_ROS/front_hawk/right/image_raw",
        "/front_stereo_camera/left/image_raw",
        "/front_stereo_camera/right/image_raw",
    ),
    # Single panel: Isaac Viewport 2 capture. The viewport->UDP script
    # in isaac/viewport_to_udp.py packs IST5 frames tagged with this
    # synthetic topic name, so the dashboard's topic-routing matches
    # without UI work.
    camera_panels=(
        ("Isaac Viewport 2", "/isaac/viewport2"),
    ),
    kind="ground",
    control_help=(
        "W/S forward·reverse · A/D strafe left·right · "
        "Q/E rotate left·right · Space E-stop"
    ),
    has_throttle=False,
    has_arm=False,
    linear_max_mps=1.5,
    angular_max_rps=2.5,
)


IRIS_DRONE = PlatformSpec(
    name="iris_drone",
    display="Iris quadcopter (PX4 offboard)",
    # The PX4 ROS 2 bridge accepts TrajectorySetpoint / offboard control
    # primitives on /fmu/in/*. For simple operator teleop we publish a
    # body-frame velocity command on this topic and let a small offboard
    # controller node convert it to TrajectorySetpoint -- decoupling the
    # dashboard from the exact PX4 message zoo. If you prefer to publish
    # TrajectorySetpoint directly, change this topic and edit the publish
    # callback in build_command_publishers below.
    cmd_vel_topic="/offboard_velocity_cmd",
    odom_topics=("/fmu/out/vehicle_odometry", "/iris/odom"),
    camera_topics=(
        # Standard Isaac Sim camera attached to the Iris quad. Adjust to
        # whatever your USD names; the dashboard auto-discovers any topic
        # of Image/CompressedImage type at runtime.
        "/iris/standard_camera/image_raw",
        "/iris/camera/image_raw",
        "/standard_camera/image_raw",
        "/iris/front_camera/image_raw",
    ),
    # Single-camera FPV for the drone by default. Add a second entry here
    # if your Iris USD has a downward / side camera you want side-by-side.
    camera_panels=(
        ("FPV", "/iris/standard_camera/image_raw"),
    ),
    kind="drone",
    control_help=(
        "W/S forward·reverse · A/D strafe · Q/E yaw · R/F up·down · "
        "T takeoff · L land · Z arm · X disarm · Space E-stop"
    ),
    has_throttle=True,
    has_arm=True,
    linear_max_mps=2.0,
    angular_max_rps=1.5,
    vertical_max_mps=1.0,
)


ALL_PLATFORMS: dict[str, PlatformSpec] = {
    p.name: p for p in (NOVA_CARTER, IRIS_DRONE)
}


def get_platform(name: str) -> PlatformSpec:
    if name not in ALL_PLATFORMS:
        raise KeyError(
            f"unknown platform {name!r}; "
            f"available: {list(ALL_PLATFORMS)}"
        )
    return ALL_PLATFORMS[name]


# ---------------------------------------------------------------------------
# Command publisher factories.
#
# Each platform exposes a single function that takes an rclpy Node and
# returns a (publish_command, publish_event) pair:
#
#   publish_command(intent: dict) -> None
#       intent ::= {linear_x, linear_y, linear_z, angular_z,
#                   emergency_stop, throttle, ...}
#
#   publish_event(event_name: str) -> None
#       event_name in {"arm", "disarm", "takeoff", "land"} (or platform-
#       specific names). Ground robots ignore events.
# ---------------------------------------------------------------------------
def build_publishers(platform: PlatformSpec, node) -> tuple[Callable, Callable]:
    """Return (publish_command, publish_event) for ``platform`` on ``node``.

    The returned closures are safe to call from rclpy callback threads.
    """
    if platform.kind == "ground":
        return _build_ground_publishers(platform, node)
    if platform.kind == "drone":
        return _build_drone_publishers(platform, node)
    raise ValueError(f"unknown platform kind {platform.kind!r}")


def _build_ground_publishers(platform: PlatformSpec, node):
    from geometry_msgs.msg import Twist
    pub = node.create_publisher(Twist, platform.cmd_vel_topic, 20)

    def publish_command(intent: dict):
        twist = Twist()
        if not intent.get("emergency_stop", False):
            twist.linear.x = float(intent.get("linear_x", 0.0))
            twist.linear.y = float(intent.get("linear_y", 0.0))
            twist.angular.z = float(intent.get("angular_z", 0.0))
        pub.publish(twist)

    def publish_event(_name: str):
        # Ground platforms have no arm/disarm/takeoff/land. Quietly drop.
        return

    return publish_command, publish_event


def _build_drone_publishers(platform: PlatformSpec, node):
    """
    Iris drone publishers.

    We try to import ``px4_msgs`` (the standard PX4 ROS 2 bridge package).
    If it's available, ARM / DISARM / OFFBOARD / TAKEOFF / LAND are issued
    as VehicleCommand messages on ``/fmu/in/vehicle_command`` and the
    velocity intent is published as a TrajectorySetpoint on
    ``/fmu/in/trajectory_setpoint``.

    If ``px4_msgs`` isn't installed, we fall back to publishing a plain
    geometry_msgs/TwistStamped on ``platform.cmd_vel_topic`` so a custom
    offboard controller node can interpret it. Events become log lines.
    """
    from geometry_msgs.msg import TwistStamped

    try:
        from px4_msgs.msg import (
            OffboardControlMode,
            TrajectorySetpoint,
            VehicleCommand,
        )
        from rclpy.qos import (
            QoSDurabilityPolicy,
            QoSHistoryPolicy,
            QoSProfile,
            QoSReliabilityPolicy,
        )
        px4_available = True
    except ImportError:
        px4_available = False

    if px4_available:
        node.get_logger().info(
            "Iris drone: PX4 px4_msgs available; using VehicleCommand + "
            "TrajectorySetpoint on /fmu/in/* topics."
        )
        px4_qos = QoSProfile(
            reliability=QoSReliabilityPolicy.BEST_EFFORT,
            durability=QoSDurabilityPolicy.TRANSIENT_LOCAL,
            history=QoSHistoryPolicy.KEEP_LAST,
            depth=1,
        )
        ts_pub = node.create_publisher(
            TrajectorySetpoint, "/fmu/in/trajectory_setpoint", px4_qos)
        oc_pub = node.create_publisher(
            OffboardControlMode, "/fmu/in/offboard_control_mode", px4_qos)
        vc_pub = node.create_publisher(
            VehicleCommand, "/fmu/in/vehicle_command", px4_qos)

        def _now_us():
            import time as _t
            return int(_t.time() * 1_000_000)

        def publish_command(intent: dict):
            # Always emit OffboardControlMode at >2Hz when offboard is on;
            # the dashboard's 30 Hz tick is more than enough.
            mode = OffboardControlMode()
            mode.timestamp = _now_us()
            mode.position = False
            mode.velocity = True
            mode.acceleration = False
            mode.attitude = False
            mode.body_rate = False
            oc_pub.publish(mode)

            ts = TrajectorySetpoint()
            ts.timestamp = _now_us()
            ts.position = [float("nan")] * 3
            ts.acceleration = [float("nan")] * 3
            ts.jerk = [float("nan")] * 3
            if intent.get("emergency_stop", False):
                ts.velocity = [0.0, 0.0, 0.0]
                ts.yaw = float("nan")
                ts.yawspeed = 0.0
            else:
                # PX4 uses NED frame -- forward is +x, right is +y, down is +z.
                # The dashboard speaks ENU-ish (forward +x, left +y, up +z),
                # which is what we'll convert here.
                lx = float(intent.get("linear_x", 0.0))   # forward
                ly = float(intent.get("linear_y", 0.0))   # left = +y
                lz = float(intent.get("linear_z", 0.0))   # up = +z
                yaw_rate = float(intent.get("angular_z", 0.0))
                ts.velocity = [lx, -ly, -lz]              # ENU -> NED
                ts.yaw = float("nan")
                ts.yawspeed = -yaw_rate
            ts_pub.publish(ts)

        def publish_event(name: str):
            cmd = VehicleCommand()
            cmd.timestamp = _now_us()
            cmd.from_external = True
            cmd.target_system = 1
            cmd.target_component = 1
            cmd.source_system = 1
            cmd.source_component = 1
            if name == "arm":
                cmd.command = VehicleCommand.VEHICLE_CMD_COMPONENT_ARM_DISARM
                cmd.param1 = 1.0
            elif name == "disarm":
                cmd.command = VehicleCommand.VEHICLE_CMD_COMPONENT_ARM_DISARM
                cmd.param1 = 0.0
            elif name == "takeoff":
                cmd.command = VehicleCommand.VEHICLE_CMD_NAV_TAKEOFF
                cmd.param1 = 0.0
                cmd.param7 = 2.5  # takeoff altitude m
            elif name == "land":
                cmd.command = VehicleCommand.VEHICLE_CMD_NAV_LAND
            elif name == "offboard":
                # Set custom mode -> OFFBOARD (PX4 reserved value 6)
                cmd.command = VehicleCommand.VEHICLE_CMD_DO_SET_MODE
                cmd.param1 = 1.0
                cmd.param2 = 6.0
            else:
                node.get_logger().warn(f"unknown drone event {name!r}")
                return
            vc_pub.publish(cmd)

        return publish_command, publish_event

    # Fallback: publish TwistStamped + log events.
    node.get_logger().warn(
        "Iris drone: px4_msgs NOT installed; publishing TwistStamped on "
        f"{platform.cmd_vel_topic}. Install px4_msgs for full PX4 control."
    )
    fallback_pub = node.create_publisher(TwistStamped, platform.cmd_vel_topic, 20)

    def publish_command(intent: dict):
        msg = TwistStamped()
        msg.header.stamp = node.get_clock().now().to_msg()
        msg.header.frame_id = "base_link"
        if not intent.get("emergency_stop", False):
            msg.twist.linear.x = float(intent.get("linear_x", 0.0))
            msg.twist.linear.y = float(intent.get("linear_y", 0.0))
            msg.twist.linear.z = float(intent.get("linear_z", 0.0))
            msg.twist.angular.z = float(intent.get("angular_z", 0.0))
        fallback_pub.publish(msg)

    def publish_event(name: str):
        node.get_logger().info(f"[fallback] drone event '{name}' (no px4_msgs)")

    return publish_command, publish_event
