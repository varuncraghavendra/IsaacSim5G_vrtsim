#!/usr/bin/env python3
"""
OpenAirInterface UDP bridge (bidirectional).

Sits between the simulated 5G radio (OAI RFSim or similar) and the local ROS 2
broker. Two independent forwarding threads share the same process:

* **Forward** (UE -> robot): packets arriving on ``--listen-port`` are
  forwarded to the broker's UDP ingress (default 127.0.0.1:16000).
* **Reverse** (robot -> UE): when ``--enable-reverse`` is set, telemetry and
  camera datagrams arriving on ``--rev-listen-port`` are forwarded to the
  UE-side teleop process (default 127.0.0.1:18500).

Both threads use a single recvfrom loop with a generous buffer (65536) so a
JPEG camera frame fits in one datagram.

Run it back-to-back with the broker:

    python3 integration/oai_udp_bridge.py --enable-reverse
"""
from __future__ import annotations

import argparse
import logging
import signal
import socket
import sys
import threading
from dataclasses import dataclass

logger = logging.getLogger("oai_udp_bridge")


@dataclass(frozen=True)
class ForwardSpec:
    tag: str
    listen_host: str
    listen_port: int
    out_host: str
    out_port: int


def forward_loop(spec: ForwardSpec, stop_event: threading.Event) -> None:
    rx = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
    rx.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
    rx.bind((spec.listen_host, spec.listen_port))
    rx.settimeout(0.5)
    tx = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
    logger.info(
        "[%s] listening on %s:%d -> %s:%d",
        spec.tag, spec.listen_host, spec.listen_port, spec.out_host, spec.out_port,
    )

    bytes_total = 0
    pkts_total = 0
    while not stop_event.is_set():
        try:
            data, src = rx.recvfrom(65536)
        except socket.timeout:
            continue
        except OSError as exc:
            logger.warning("[%s] recvfrom failed: %s", spec.tag, exc)
            continue
        try:
            tx.sendto(data, (spec.out_host, spec.out_port))
        except OSError as exc:
            logger.warning("[%s] sendto failed: %s", spec.tag, exc)
            continue
        bytes_total += len(data)
        pkts_total += 1
        # Don't log every camera frame; only small (control-plane) packets.
        if len(data) < 2048:
            logger.debug(
                "[%s] %d B  %s  -> %s:%d  (totals: %d pkts, %d B)",
                spec.tag, len(data), src, spec.out_host, spec.out_port,
                pkts_total, bytes_total,
            )


def main() -> int:
    parser = argparse.ArgumentParser(
        description="Forward UDP packets between OAI-exposed ports and ROS 2 ingress ports.",
    )
    # Forward (commands UE -> robot)
    parser.add_argument("--listen-host", default="0.0.0.0")
    parser.add_argument("--listen-port", type=int, default=17000,
                        help="UDP port this bridge listens on for inbound (UE) traffic.")
    parser.add_argument("--ros-host", default="127.0.0.1")
    parser.add_argument("--ros-port", type=int, default=16000,
                        help="UDP port of the ROS 2 UE ingress node.")

    # Reverse (telemetry / camera robot -> UE), optional
    parser.add_argument("--enable-reverse", action="store_true")
    parser.add_argument("--rev-listen-host", default="0.0.0.0")
    parser.add_argument("--rev-listen-port", type=int, default=17500,
                        help="Where the robot side sends telemetry/camera UDP "
                             "to OAI's downlink.")
    parser.add_argument("--rev-out-host", default="127.0.0.1")
    parser.add_argument("--rev-out-port", type=int, default=18500,
                        help="Where the UE-side teleop process listens for "
                             "telemetry/camera.")

    parser.add_argument("--verbose", "-v", action="store_true")
    args = parser.parse_args()

    logging.basicConfig(
        level=logging.DEBUG if args.verbose else logging.INFO,
        format="%(asctime)s %(levelname)s %(name)s: %(message)s",
    )

    specs = [ForwardSpec(
        tag="fwd-cmd",
        listen_host=args.listen_host, listen_port=args.listen_port,
        out_host=args.ros_host,       out_port=args.ros_port,
    )]
    if args.enable_reverse:
        specs.append(ForwardSpec(
            tag="rev-tlm",
            listen_host=args.rev_listen_host, listen_port=args.rev_listen_port,
            out_host=args.rev_out_host,       out_port=args.rev_out_port,
        ))

    stop_event = threading.Event()

    def handle_signal(_signum, _frame):
        logger.info("signal received, shutting down")
        stop_event.set()
    signal.signal(signal.SIGINT, handle_signal)
    signal.signal(signal.SIGTERM, handle_signal)

    threads = [
        threading.Thread(target=forward_loop, args=(s, stop_event),
                         name=f"oai-{s.tag}", daemon=True)
        for s in specs
    ]
    for t in threads:
        t.start()
    for t in threads:
        t.join()
    return 0


if __name__ == "__main__":
    sys.exit(main())
