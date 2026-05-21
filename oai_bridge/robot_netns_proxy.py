#!/usr/bin/env python3
"""
robot_netns_proxy — runs inside the `robot-ns` netns built by
scripts/setup_robot_netns.sh.

Two relay legs, both pure UDP, no ROS dependency:

  DL  (commands, real PDSCH already happened):
      bind  oaitun_ue1's IP : DL_LISTEN_PORT
      send  veth_host_ip    : DL_FORWARD_PORT   (DSCP_EF, low-latency)

      Packets arriving here have already crossed gNB→UE PDSCH and been
      decapsulated onto the UE TUN. We just relay them across the veth
      pair to ue_teleop_bridge in the default ns.

  UL  (camera + pose + everything else, this is the real PUSCH leg):
      bind  veth_ns_ip      : UL_LISTEN_PORT
      send  operator_host   : OPERATOR_PORT      (DSCP_AF41, medium-low)

      Packets arriving here are written by Isaac (viewport_to_udp) and by
      ue_teleop_bridge (pose). We forward them out via routing, which —
      because the netns only has oaitun_ue1 as egress to OPERATOR_NET —
      hands them to OAI UE's TUN reader, encapsulates onto PUSCH, and
      ships them through the radio.

DSCP markings (IP_TOS):
  0xb8 = DSCP_EF   (Expedited Forwarding, 5G NR 5QI-82-ish, conversational
                    voice / signaling) for commands
  0x88 = DSCP_AF41 (Assured Forwarding 41, 5G NR 5QI-7/65-ish, interactive
                    video) for camera + pose uplinks

The 5QI mapping is set in the gNB QoS rules; see docs/URLLC_AND_UL_PATH.md
for the recommended PDU-session config. If your core doesn't honour the
DSCP-to-5QI map the marks are still useful for any midboxes between UPF
and the operator.
"""
from __future__ import annotations

import argparse
import os
import signal
import socket
import sys
import threading
import time


DSCP_EF   = 0xb8   # commands
DSCP_AF41 = 0x88   # interactive video / uplink


def _set_dscp(sock: socket.socket, dscp: int) -> None:
    try:
        sock.setsockopt(socket.IPPROTO_IP, socket.IP_TOS, dscp)
    except OSError as exc:
        print(f"[proxy][warn] set IP_TOS={hex(dscp)} failed: {exc}",
              file=sys.stderr)


class Relay(threading.Thread):
    """One-direction UDP relay with stats and DSCP marking on egress."""

    def __init__(
        self,
        name: str,
        bind_host: str, bind_port: int,
        dst_host: str, dst_port: int,
        dscp: int,
        stop_evt: threading.Event,
    ):
        super().__init__(name=name, daemon=True)
        self.name = name
        self._dst = (dst_host, dst_port)
        self._stop = stop_evt
        self._rx = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
        self._rx.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
        self._rx.bind((bind_host, bind_port))
        self._rx.settimeout(0.5)
        self._tx = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
        _set_dscp(self._tx, dscp)
        self.bind = (bind_host, bind_port)
        self.dscp = dscp
        self.count = 0
        self.bytes = 0
        self.last_log_t = time.monotonic()

    def run(self):
        print(
            f"[proxy:{self.name}] relay {self.bind[0]}:{self.bind[1]} "
            f"-> {self._dst[0]}:{self._dst[1]} dscp={hex(self.dscp)}",
            flush=True,
        )
        while not self._stop.is_set():
            try:
                data, _addr = self._rx.recvfrom(65536)
            except socket.timeout:
                self._maybe_log()
                continue
            except OSError as exc:
                print(f"[proxy:{self.name}] rx error: {exc}", flush=True)
                time.sleep(0.2)
                continue
            try:
                self._tx.sendto(data, self._dst)
                self.count += 1
                self.bytes += len(data)
            except OSError as exc:
                print(f"[proxy:{self.name}] tx error: {exc}", flush=True)
            self._maybe_log()

    def _maybe_log(self):
        now = time.monotonic()
        if now - self.last_log_t >= 5.0:
            print(
                f"[proxy:{self.name}] forwarded={self.count} "
                f"bytes={self.bytes}",
                flush=True,
            )
            self.last_log_t = now

    def close(self):
        try: self._rx.close()
        except Exception: pass
        try: self._tx.close()
        except Exception: pass


def main() -> int:
    ap = argparse.ArgumentParser(description=__doc__.split("\n\n", 1)[0])
    ap.add_argument("--ue-tun-ip", required=True,
                    help="UE TUN IP inside robot-ns (e.g. 10.0.0.4)")
    ap.add_argument("--veth-ns-ip", required=True,
                    help="robot-ns side of the veth (e.g. 192.168.244.2)")
    ap.add_argument("--veth-host-ip", required=True,
                    help="default-ns side of the veth, where the bridge "
                         "listens for the DL post-veth (e.g. 192.168.244.1)")
    ap.add_argument("--dl-listen-port", type=int,
                    default=int(os.environ.get("DL_LISTEN_PORT", "16000")),
                    help="UE-TUN-side port to listen for DL commands")
    ap.add_argument("--dl-forward-port", type=int,
                    default=int(os.environ.get("DL_FORWARD_PORT", "16005")),
                    help="default-ns port where ue_teleop_bridge listens")
    ap.add_argument("--ul-listen-port", type=int,
                    default=int(os.environ.get("UL_LISTEN_PORT", "17600")),
                    help="veth-ns-side port for UL ingress")
    ap.add_argument("--operator-host", required=True,
                    help="real operator host IP (on the docker bridge, "
                         "reachable from inside the 5G data network)")
    ap.add_argument("--operator-port", type=int,
                    default=int(os.environ.get("OPERATOR_PORT", "17500")),
                    help="dashboard uplink listener port")
    args = ap.parse_args()

    stop_evt = threading.Event()

    dl = Relay(
        "DL",
        bind_host=args.ue_tun_ip, bind_port=args.dl_listen_port,
        dst_host=args.veth_host_ip, dst_port=args.dl_forward_port,
        dscp=DSCP_EF,
        stop_evt=stop_evt,
    )
    ul = Relay(
        "UL",
        bind_host=args.veth_ns_ip, bind_port=args.ul_listen_port,
        dst_host=args.operator_host, dst_port=args.operator_port,
        dscp=DSCP_AF41,
        stop_evt=stop_evt,
    )

    def _stop(*_):
        print("[proxy] shutting down", flush=True)
        stop_evt.set()
    signal.signal(signal.SIGINT, _stop)
    signal.signal(signal.SIGTERM, _stop)

    dl.start()
    ul.start()
    try:
        while not stop_evt.is_set():
            stop_evt.wait(timeout=1.0)
    finally:
        dl.close()
        ul.close()
    return 0


if __name__ == "__main__":
    sys.exit(main())
