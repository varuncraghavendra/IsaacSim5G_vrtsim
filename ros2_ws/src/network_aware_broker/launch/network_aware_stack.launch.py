"""Launch description for the network-aware teleop broker stack.

Brings up four nodes:

* ``ue_ingress_node`` -- listens on UDP :16000 for JSON teleop datagrams from
  the FastAPI web app (or any other UE-side process) and republishes them on
  ``/network/teleop_command``.
* ``metrics_ingress_node`` -- listens on UDP :16001 for JSON network metrics
  samples and republishes them on ``/network/metrics``.
* ``network_aware_broker`` -- the C++ broker. Reads its params from
  ``config/broker_params.yaml`` (continuous publish on ``/network/cmd_vel``
  with broker-side watchdog).
* ``rtf_clock_node`` -- publishes a scaled ``/clock`` (only used in
  deterministic replay mode; harmless otherwise).
"""
import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description():
    pkg_share = get_package_share_directory("network_aware_broker")
    config_file = os.path.join(pkg_share, "config", "broker_params.yaml")
    udp_port = int(os.environ.get("BROKER_UDP_PORT", "16000"))

    return LaunchDescription([
        Node(
            package="network_aware_broker",
            executable="ue_ingress_node",
            name="ue_ingress_node",
            output="screen",
            parameters=[{
                "udp_port": udp_port,
                "output_topic": "/network/teleop_command",
            }],
        ),
        Node(
            package="network_aware_broker",
            executable="metrics_ingress_node",
            name="metrics_ingress_node",
            output="screen",
            parameters=[{
                "udp_port": 16001,
                "output_topic": "/network/metrics",
            }],
        ),
        Node(
            package="network_aware_broker",
            executable="broker_main",
            name="network_aware_broker",
            output="screen",
            parameters=[config_file],
        ),
        Node(
            package="network_aware_broker",
            executable="rtf_clock_node",
            name="rtf_clock_node",
            output="screen",
            parameters=[{
                "rtf": 0.95,
                "period_ms": 10,
                "release_lookahead_ms": 0.0,
                "physics_step_ms": 20.0,
                "network_step_ms": 10.0,
                "deterministic_replay_enabled": False,
            }],
        ),
    ])
