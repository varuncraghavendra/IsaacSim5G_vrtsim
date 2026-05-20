#!/usr/bin/env python3
"""
Send synthetic NetworkMetrics samples to the broker's UDP ingress on :16001.

Useful for sanity-checking the broker reaction without OAI / a real radio
present. Emits a 5 Hz stream of values jittered within plausible LOS ranges.
Stop with Ctrl-C.
"""
from __future__ import annotations

import argparse
import json
import random
import signal
import socket
import sys
import time


def main() -> int:
    parser = argparse.ArgumentParser(description="Random NetworkMetrics UDP sender.")
    parser.add_argument("--host", default="127.0.0.1")
    parser.add_argument("--port", type=int, default=16001)
    parser.add_argument("--hz", type=float, default=5.0)
    args = parser.parse_args()

    sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
    period = 1.0 / max(0.1, args.hz)
    stop = False

    def handle(_signum, _frame):
        nonlocal stop
        stop = True
    signal.signal(signal.SIGINT, handle)
    signal.signal(signal.SIGTERM, handle)

    print(f"sending NetworkMetrics to {args.host}:{args.port} at {args.hz} Hz")
    while not stop:
        payload = {
            "one_way_latency_ms": round(random.uniform(10.0, 25.0), 2),
            "round_trip_latency_ms": round(random.uniform(20.0, 55.0), 2),
            "jitter_ms": round(random.uniform(1.0, 7.0), 2),
            "packet_error_rate": round(random.uniform(0.0, 0.08), 3),
            "snr_db": round(random.uniform(8.0, 28.0), 2),
            "rtf": 0.95,
            "los": True,
            "path_loss_db": round(random.uniform(40.0, 75.0), 2),
            "source": "sample_metrics_sender",
        }
        sock.sendto(json.dumps(payload).encode("utf-8"), (args.host, args.port))
        time.sleep(period)
    return 0


if __name__ == "__main__":
    sys.exit(main())
