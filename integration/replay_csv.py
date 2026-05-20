#!/usr/bin/env python3
"""
Replay a broker CSV log back into the broker's UDP ingress ports.

The broker can write a CSV of every teleop/metrics event when started with
``log_csv_path``. This script reads that CSV and re-fires every row at its
original wall-clock spacing, so you can deterministically reproduce a
recorded session without the original GUI / RFSim being live.
"""
from __future__ import annotations

import argparse
import csv
import json
import socket
import sys
import time
from typing import Iterable


def _send_teleop(sock: socket.socket, host: str, port: int, row: dict) -> None:
    payload = {
        "seq": int(row["seq"]),
        "linear_x": float(row["linear_x"]),
        "angular_z": float(row["angular_z"]),
        "desired_rtf": float(row["rtf"]),
        "emergency_stop": False,
        "source": "csv_replay",
    }
    sock.sendto(json.dumps(payload).encode("utf-8"), (host, port))


def _send_metrics(sock: socket.socket, host: str, port: int, row: dict) -> None:
    payload = {
        "one_way_latency_ms": float(row["latency_ms"]),
        "round_trip_latency_ms": float(row["rtt_ms"]),
        "jitter_ms": float(row["jitter_ms"]),
        "packet_error_rate": float(row["packet_error_rate"]),
        "snr_db": float(row["snr_db"]),
        "rtf": float(row["rtf"]),
        "los": str(row["los"]).strip().lower() == "true",
        "path_loss_db": float(row["path_loss_db"]),
        "source": "csv_replay",
    }
    sock.sendto(json.dumps(payload).encode("utf-8"), (host, port))


def _iter_rows(path: str) -> Iterable[dict]:
    with open(path, "r", encoding="utf-8") as handle:
        yield from csv.DictReader(handle)


def main() -> int:
    parser = argparse.ArgumentParser(
        description="Replay broker CSV log entries into the UDP ingress ports."
    )
    parser.add_argument("csv_file")
    parser.add_argument("--host", default="127.0.0.1")
    parser.add_argument("--teleop-port", type=int, default=16000)
    parser.add_argument("--metrics-port", type=int, default=16001)
    parser.add_argument("--time-scale", type=float, default=1.0,
                        help="Playback speed multiplier; >1 plays back faster.")
    args = parser.parse_args()

    rows = list(_iter_rows(args.csv_file))
    if not rows:
        print("CSV is empty, nothing to replay.", file=sys.stderr)
        return 1

    teleop_sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
    metrics_sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)

    t0 = float(rows[0]["wall_time_sec"])
    start = time.time()
    scale = max(args.time_scale, 1e-6)

    sent_teleop = sent_metrics = 0
    for row in rows:
        target = start + (float(row["wall_time_sec"]) - t0) / scale
        delay = target - time.time()
        if delay > 0.0:
            time.sleep(delay)

        event = row.get("event", "")
        if event == "teleop_rx":
            _send_teleop(teleop_sock, args.host, args.teleop_port, row)
            sent_teleop += 1
        elif event == "metrics_rx":
            _send_metrics(metrics_sock, args.host, args.metrics_port, row)
            sent_metrics += 1

    print(f"replay done: {sent_teleop} teleop + {sent_metrics} metrics events sent")
    return 0


if __name__ == "__main__":
    sys.exit(main())
