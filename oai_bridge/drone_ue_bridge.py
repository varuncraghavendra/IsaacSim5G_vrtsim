#!/usr/bin/env python3
"""
Drone UE-side bridge — OAI 5G data plane <-> PX4 SITL.

This process is the "robot side" of the IsaacSim5G drone teleop pipeline.
It MUST live downstream of the 5G UL/DL path. If the radio isn't
forwarding bytes, neither downlink commands nor uplink telemetry move
through this bridge, which is exactly the safety property the dashboard
relies on ("no OAI in the loop => drone is silent").

Downlink (commands from the operator):
    UDP JSON ─[5G DL: ext-dn → UPF → gNB → AIR → UE → oaitun_ue1
                       → robot_netns_proxy → veth → default ns]→
    listen_host:listen_port (default 192.168.244.1:16005)
    ─ this bridge ─→ MAVLink MANUAL_CONTROL / COMMAND_LONG ─→
    PX4 SITL (127.0.0.1:14540 by default — Pegasus binds its outbound
    MAVLink to this port; pymavlink learns PX4's source on first recv)
    ─→ Iris quadrotor in the city scene.

Uplink (telemetry to the operator):
    PX4 SITL emits LOCAL_POSITION_NED + ATTITUDE → this bridge subscribes
    via the same MAVLink connection → packs IST5 pose envelope at
    pose_rate_hz → sends to ul_host:ul_port (default 192.168.244.2:17600,
    the robot-ns side of the veth). robot_netns_proxy ships those packets
    out of oaitun_ue1 → real UL through the radio → operator host.

The bridge also runs an inert "hold" mode: if no DL command arrives for
``watchdog_ms`` it sends a centered MANUAL_CONTROL (sticks zeroed, throttle
at 0.5) so the drone parks instead of latching the last input. PX4 will
typically hover-hold in POSCTL/ALTCTL with this input.

Wire format (downlink JSON, identical to teleop_app/drone_app.py):
    {"pitch": float in [-1,1],
     "roll":  float in [-1,1],
     "yaw":   float in [-1,1],
     "throttle": float in [0,1],
     "emergency_stop": bool}
    or  {"type": "event", "name": "arm"|"disarm"|"takeoff"|"land"|
                                  "posctl"|"altctl"|"stabilized"|
                                  "offboard"|"manual"}

Wire format (uplink IST5 pose envelope — KIND_POSE):
    {"stamp": float, "frame_id": "px4_ned",
     "x":..,"y":..,"z":..,
     "vx":..,"vy":..,"vz":..,
     "yaw":..,                              # radians, ENU yaw
     "linear_x":..,"linear_y":..,"linear_z":..,
     "angular_z":..,
     "armed": bool, "mode": str}

PX4's LOCAL_POSITION_NED frame is North-East-Down. We convert to a
matching ENU triple (x=east, y=north, z=up) before sending uplink, since
that's what the dashboard's HUD + the channel bridge expect.
"""
from __future__ import annotations

import argparse
import json
import math
import os
import signal
import socket
import struct
import sys
import threading
import time
from dataclasses import dataclass
from pathlib import Path
from typing import Optional

# Shared IST5 envelope helpers.
sys.path.insert(0, str(Path(__file__).resolve().parent))
from protocol import KIND_POSE, pack_uplink  # noqa: E402

try:
    from pymavlink import mavutil
except ImportError as exc:
    print("[drone-ue] ERROR: pymavlink is required.\n"
          "          pip install pymavlink", file=sys.stderr)
    raise


# ---------------------------------------------------------------------------
# PX4 custom-mode constants (see px4_custom_mode.h).
PX4_MAIN_MODE_MANUAL      = 1
PX4_MAIN_MODE_ALTCTL      = 2
PX4_MAIN_MODE_POSCTL      = 3
PX4_MAIN_MODE_AUTO        = 4
PX4_MAIN_MODE_ACRO        = 5
PX4_MAIN_MODE_OFFBOARD    = 6
PX4_MAIN_MODE_STABILIZED  = 7
PX4_SUB_MODE_AUTO_TAKEOFF = 2
PX4_SUB_MODE_AUTO_LAND    = 6


@dataclass
class Stats:
    cmds_rx: int = 0
    cmds_rx_bad: int = 0
    events_rx: int = 0
    mavlink_sent: int = 0
    pose_tx: int = 0
    last_cmd_t: float = 0.0


# ---------------------------------------------------------------------------
class DroneUeBridge:
    def __init__(
        self, *,
        listen_host: str,
        listen_port: int,
        ul_host: str,
        ul_port: int,
        mavlink_url: str,
        watchdog_ms: float,
        mav_send_rate_hz: float,
        pose_rate_hz: float,
    ):
        self._stop = threading.Event()
        self.stats = Stats()

        self._watchdog_s = max(0.05, watchdog_ms / 1000.0)
        self._mav_send_period = 1.0 / max(1.0, mav_send_rate_hz)
        self._pose_send_period = 1.0 / max(1.0, pose_rate_hz)

        # Connect MAVLink first so we know the link works before we open
        # the network sockets the OAI side will hammer.
        print(f"[drone-ue] connecting MAVLink: {mavlink_url}")
        self.master = mavutil.mavlink_connection(mavlink_url, autoreconnect=True)
        self._target_system = 1
        self._target_component = 1

        # Latest joystick intent + timestamp. Send loop reads these.
        self._intent_lock = threading.Lock()
        self._intent = {"pitch": 0.0, "roll": 0.0,
                        "yaw": 0.0, "throttle": 0.5}
        self._last_intent_t = 0.0
        self._estop = False

        # Latest PX4 state snapshot the UL pose packer reads from.
        self._state_lock = threading.Lock()
        self._state = {
            "x": 0.0, "y": 0.0, "z": 0.0,
            "vx": 0.0, "vy": 0.0, "vz": 0.0,
            "yaw": 0.0,
            "armed": False,
            "mode": "—",
        }

        # Downlink socket.
        self._rx_sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
        self._rx_sock.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
        self._rx_sock.bind((listen_host, listen_port))
        self._rx_sock.settimeout(0.5)
        print(f"[drone-ue] DL listen: {listen_host}:{listen_port}")

        # Uplink socket. DSCP_EF for low-latency telemetry; ignored if not
        # supported on the host.
        self._tx_sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
        try:
            self._tx_sock.setsockopt(socket.IPPROTO_IP, socket.IP_TOS, 0xb8)
        except OSError:
            pass
        self._ul_addr = (ul_host, ul_port)
        print(f"[drone-ue] UL pose -> {ul_host}:{ul_port}")

    # ----------------------------------------------------------------- run
    def start(self):
        self._threads = [
            threading.Thread(target=self._rx_loop,
                             name="dl-rx", daemon=True),
            threading.Thread(target=self._mav_recv_loop,
                             name="mav-rx", daemon=True),
            threading.Thread(target=self._mav_send_loop,
                             name="mav-tx", daemon=True),
            threading.Thread(target=self._pose_tx_loop,
                             name="pose-tx", daemon=True),
            threading.Thread(target=self._stats_loop,
                             name="stats", daemon=True),
        ]
        for t in self._threads:
            t.start()

    def stop(self):
        self._stop.set()
        try: self._rx_sock.close()
        except Exception: pass
        try: self._tx_sock.close()
        except Exception: pass

    # ---------------------------------------------------------------- helpers
    @staticmethod
    def _mode_label(main_mode, sub_mode):
        labels = {
            PX4_MAIN_MODE_MANUAL:     "MANUAL",
            PX4_MAIN_MODE_ALTCTL:     "ALTCTL",
            PX4_MAIN_MODE_POSCTL:     "POSCTL",
            PX4_MAIN_MODE_AUTO:       "AUTO",
            PX4_MAIN_MODE_ACRO:       "ACRO",
            PX4_MAIN_MODE_OFFBOARD:   "OFFBOARD",
            PX4_MAIN_MODE_STABILIZED: "STABILIZED",
        }
        s = labels.get(main_mode, f"main={main_mode}")
        if main_mode == PX4_MAIN_MODE_AUTO:
            sub = {2: "TAKEOFF", 3: "LOITER", 4: "MISSION",
                   5: "RTL", 6: "LAND"}.get(sub_mode, f"sub={sub_mode}")
            return f"AUTO.{sub}"
        return s

    def _set_px4_mode(self, main_mode, sub_mode):
        custom_mode = (main_mode << 16) | (sub_mode << 24)
        self.master.mav.command_long_send(
            self._target_system, self._target_component,
            mavutil.mavlink.MAV_CMD_DO_SET_MODE, 0,
            mavutil.mavlink.MAV_MODE_FLAG_CUSTOM_MODE_ENABLED,
            float(main_mode), float(sub_mode), 0, 0, 0, 0)

    def _handle_event(self, name: str) -> bool:
        name = name.lower()
        try:
            if name == "arm":
                self.master.mav.command_long_send(
                    self._target_system, self._target_component,
                    mavutil.mavlink.MAV_CMD_COMPONENT_ARM_DISARM, 0,
                    1.0, 0, 0, 0, 0, 0, 0)
            elif name == "disarm":
                self.master.mav.command_long_send(
                    self._target_system, self._target_component,
                    mavutil.mavlink.MAV_CMD_COMPONENT_ARM_DISARM, 0,
                    0.0, 0, 0, 0, 0, 0, 0)
            elif name == "takeoff":
                self._set_px4_mode(PX4_MAIN_MODE_AUTO,
                                   PX4_SUB_MODE_AUTO_TAKEOFF)
            elif name == "land":
                self._set_px4_mode(PX4_MAIN_MODE_AUTO,
                                   PX4_SUB_MODE_AUTO_LAND)
            elif name == "offboard":
                self._set_px4_mode(PX4_MAIN_MODE_OFFBOARD, 0)
            elif name == "posctl":
                self._set_px4_mode(PX4_MAIN_MODE_POSCTL, 0)
            elif name == "altctl":
                self._set_px4_mode(PX4_MAIN_MODE_ALTCTL, 0)
            elif name == "stabilized":
                self._set_px4_mode(PX4_MAIN_MODE_STABILIZED, 0)
            elif name == "manual":
                self._set_px4_mode(PX4_MAIN_MODE_MANUAL, 0)
            else:
                print(f"[drone-ue] unknown event {name!r}")
                return False
            print(f"[drone-ue] event applied: {name}")
            return True
        except Exception as exc:
            print(f"[drone-ue] event {name!r} failed: {exc}")
            return False

    # ----------------------------------------------------------------- DL
    def _rx_loop(self):
        """Read JSON downlink datagrams and stash the latest intent."""
        while not self._stop.is_set():
            try:
                data, _addr = self._rx_sock.recvfrom(65536)
            except socket.timeout:
                continue
            except OSError:
                continue
            try:
                msg = json.loads(data.decode("utf-8"))
            except (UnicodeDecodeError, json.JSONDecodeError):
                self.stats.cmds_rx_bad += 1
                continue
            if not isinstance(msg, dict):
                self.stats.cmds_rx_bad += 1
                continue

            mtype = msg.get("type", "")
            if mtype == "event":
                name = str(msg.get("name", "")).strip()
                if name:
                    self.stats.events_rx += 1
                    self._handle_event(name)
                continue

            # Default: stick intent.
            try:
                with self._intent_lock:
                    self._intent["pitch"] = max(-1.0, min(1.0,
                        float(msg.get("pitch", 0.0))))
                    self._intent["roll"] = max(-1.0, min(1.0,
                        float(msg.get("roll", 0.0))))
                    self._intent["yaw"] = max(-1.0, min(1.0,
                        float(msg.get("yaw", 0.0))))
                    self._intent["throttle"] = max(0.0, min(1.0,
                        float(msg.get("throttle", 0.5))))
                    self._estop = bool(msg.get("emergency_stop", False))
                    self._last_intent_t = time.monotonic()
                self.stats.cmds_rx += 1
                self.stats.last_cmd_t = time.time()
            except (TypeError, ValueError):
                self.stats.cmds_rx_bad += 1

    # ------------------------------------------------------------- MAVLink
    def _mav_recv_loop(self):
        while not self._stop.is_set():
            try:
                msg = self.master.recv_match(blocking=True, timeout=1.0)
            except Exception:
                time.sleep(0.1); continue
            if msg is None:
                continue
            t = msg.get_type()
            if t == "HEARTBEAT":
                self._target_system = msg.get_srcSystem()
                self._target_component = msg.get_srcComponent()
                with self._state_lock:
                    self._state["armed"] = bool(msg.base_mode & 0x80)
                    cm = msg.custom_mode
                    self._state["mode"] = self._mode_label(
                        (cm >> 16) & 0xFF, (cm >> 24) & 0xFF)
            elif t == "LOCAL_POSITION_NED":
                # PX4 NED -> ENU. ENU.x = NED.y (east), ENU.y = NED.x
                # (north), ENU.z = -NED.z (up).
                with self._state_lock:
                    self._state["x"] = float(msg.y)
                    self._state["y"] = float(msg.x)
                    self._state["z"] = -float(msg.z)
                    self._state["vx"] = float(msg.vy)
                    self._state["vy"] = float(msg.vx)
                    self._state["vz"] = -float(msg.vz)
            elif t == "ATTITUDE":
                # PX4 yaw is NED yaw from north, CW positive. ENU yaw is
                # from east, CCW positive: yaw_enu = pi/2 - yaw_ned.
                yaw_enu = (math.pi / 2.0) - float(msg.yaw)
                # Wrap into [-pi, pi].
                while yaw_enu > math.pi:  yaw_enu -= 2 * math.pi
                while yaw_enu < -math.pi: yaw_enu += 2 * math.pi
                with self._state_lock:
                    self._state["yaw"] = yaw_enu

    def _mav_send_loop(self):
        next_t = time.monotonic()
        while not self._stop.is_set():
            with self._intent_lock:
                age = (time.monotonic() - self._last_intent_t
                       if self._last_intent_t else float("inf"))
                if self._estop or age > self._watchdog_s:
                    pitch = roll = yaw = 0.0
                    throttle = 0.5
                else:
                    pitch = self._intent["pitch"]
                    roll = self._intent["roll"]
                    yaw = self._intent["yaw"]
                    throttle = self._intent["throttle"]
            try:
                # MANUAL_CONTROL: x=pitch, y=roll, z=throttle [0..1000],
                # r=yaw, all int16. PX4 throttle convention: 0=down, 1000=up.
                self.master.mav.manual_control_send(
                    self._target_system,
                    int(max(-1.0, min(1.0, pitch))    * 1000),
                    int(max(-1.0, min(1.0, roll))     * 1000),
                    int(max( 0.0, min(1.0, throttle)) * 1000),
                    int(max(-1.0, min(1.0, yaw))      * 1000),
                    0)
                self.stats.mavlink_sent += 1
            except Exception:
                pass
            next_t += self._mav_send_period
            sleep_for = next_t - time.monotonic()
            if sleep_for > 0:
                time.sleep(sleep_for)
            else:
                next_t = time.monotonic()

    # ----------------------------------------------------------------- UL
    def _pose_tx_loop(self):
        next_t = time.monotonic()
        while not self._stop.is_set():
            with self._state_lock:
                s = dict(self._state)
            header = {
                "stamp": time.time(),
                "frame_id": "px4_enu",
                "x": s["x"], "y": s["y"], "z": s["z"],
                "yaw": s["yaw"],
                "linear_x": s["vx"],
                "linear_y": s["vy"],
                "linear_z": s["vz"],
                "angular_z": 0.0,
                "armed": s["armed"],
                "mode": s["mode"],
            }
            packet = pack_uplink(KIND_POSE, header, b"")
            try:
                self._tx_sock.sendto(packet, self._ul_addr)
                self.stats.pose_tx += 1
            except OSError:
                # netns proxy not up yet — keep going, packets will start
                # making it through once everything stabilises.
                pass
            next_t += self._pose_send_period
            sleep_for = next_t - time.monotonic()
            if sleep_for > 0:
                time.sleep(sleep_for)
            else:
                next_t = time.monotonic()

    # --------------------------------------------------------------- stats
    def _stats_loop(self):
        while not self._stop.is_set():
            time.sleep(5.0)
            s = self.stats
            with self._state_lock:
                st = dict(self._state)
            print(f"[drone-ue] stats  DL: cmds={s.cmds_rx} bad={s.cmds_rx_bad} "
                  f"events={s.events_rx}  MAV: sent={s.mavlink_sent}  "
                  f"UL: pose={s.pose_tx}  mode={st['mode']} "
                  f"armed={st['armed']} pos=({st['x']:.2f},{st['y']:.2f},"
                  f"{st['z']:.2f}) yaw={math.degrees(st['yaw']):.1f}°")


# ---------------------------------------------------------------------------
def main() -> int:
    ap = argparse.ArgumentParser(description=__doc__.split("\n\n", 1)[0])
    ap.add_argument("--listen-host",
                    default=os.environ.get("DL_LISTEN_HOST", "192.168.244.1"),
                    help="UDP DL listener host (veth-default-ns side).")
    ap.add_argument("--listen-port", type=int,
                    default=int(os.environ.get("DL_LISTEN_PORT", "16005")),
                    help="UDP DL listener port (forwarded by robot_netns_proxy "
                         "from oaitun_ue1).")
    ap.add_argument("--ul-host",
                    default=os.environ.get("UL_HOST", "192.168.244.2"),
                    help="Robot-ns side of the veth — UL ingress for the proxy.")
    ap.add_argument("--ul-port", type=int,
                    default=int(os.environ.get("UL_PORT", "17600")))
    ap.add_argument("--mavlink-url",
                    default=os.environ.get("MAVLINK_URL",
                                           "udpin:127.0.0.1:14540"),
                    help="pymavlink connection string for PX4 SITL.")
    ap.add_argument("--watchdog-ms", type=float,
                    default=float(os.environ.get("WATCHDOG_MS", "500")))
    ap.add_argument("--mav-send-hz", type=float,
                    default=float(os.environ.get("MAV_SEND_HZ", "50")),
                    help="MANUAL_CONTROL send rate to PX4.")
    ap.add_argument("--pose-rate-hz", type=float,
                    default=float(os.environ.get("POSE_RATE_HZ", "30")),
                    help="IST5 pose UL rate.")
    args = ap.parse_args()

    bridge = DroneUeBridge(
        listen_host=args.listen_host,
        listen_port=args.listen_port,
        ul_host=args.ul_host,
        ul_port=args.ul_port,
        mavlink_url=args.mavlink_url,
        watchdog_ms=args.watchdog_ms,
        mav_send_rate_hz=args.mav_send_hz,
        pose_rate_hz=args.pose_rate_hz,
    )
    bridge.start()

    stop = threading.Event()
    def _on_signal(*_):
        print("[drone-ue] shutting down")
        stop.set()
        bridge.stop()
    signal.signal(signal.SIGINT, _on_signal)
    signal.signal(signal.SIGTERM, _on_signal)

    stop.wait()
    return 0


if __name__ == "__main__":
    sys.exit(main())
