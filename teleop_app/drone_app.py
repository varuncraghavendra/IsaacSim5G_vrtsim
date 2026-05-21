"""
Operator-side dashboard for the IsaacSim5G drone teleop pipeline.

This dashboard is the only place a human touches in normal operation. It
is **strictly OAI-gated**:

    Browser ─ws─► drone_app.py ─UDP JSON─► EXT_DN_IP:EXT_DN_PORT
                                              │
                                              ▼
                                      OAI 5G data network
                                  (ext-dn → UPF → gNB → AIR → UE)
                                              │
                                              ▼
                                  oaitun_ue1 inside robot-ns
                                              │
                                              ▼
                                  robot_netns_proxy ─veth─► drone_ue_bridge
                                              │
                                              ▼
                                          PX4 SITL
                                              │
                                              ▼
                                       Iris drone in city
                                              │
                                              ▼
                                  PX4 LOCAL_POSITION_NED + ATTITUDE
                                              │
                                  drone_ue_bridge packs as IST5 pose ─►
                                  veth ─► robot_netns_proxy ─► oaitun_ue1
                                              │
                                              ▼
                                  5G UL ─► UPF ─► host bridge IP
                                              │
                                              ▼
                                  this dashboard (operator_udp_port)

There is NO localhost fallback to MAVLink. If the OAI stack isn't
forwarding bytes, the drone literally cannot hear the operator.

In addition to driving the browser, the dashboard mirrors every pose
sample it receives from the UE bridge onto a ZMQ PUB in the legacy
``MSG_FORMAT`` (``<ddddddddd``) on ``tcp://127.0.0.1:5556``. That is the
exact wire format ``main_paths_los_model_measurment_position.py`` and
``live_plot_position.py`` already consume, so the channel pipeline and
plotter work unchanged — they just see real drone pose instead of
synthetic trajectories.

Flags / env:

    --port                  HTTP port (TELEOP_PORT, default 8080).

    --downlink-host         where to send command JSON. Default
                            EXT_DN_IP (192.168.70.135) — the oai-ext-dn
                            container the 5G data plane uses for DL.
                            Set EXT_DN_IP=__none__ to refuse to send at
                            all (full blackhole, useful for testing the
                            safety property).
    --downlink-port         EXT_DN_PORT (default 16000).

    --camera-udp-port       UDP listener for IST5 camera + pose frames
                            arriving on the operator side after the 5G
                            UL (OPERATOR_UDP_PORT, default 17500).

    --gt-zmq-addr           ZMQ PUB bind address for the MSG_FORMAT
                            ground-truth republish (GT_ZMQ_ADDR,
                            default tcp://127.0.0.1:5556).

    --gt-disable            don't re-publish pose to ZMQ.

By design this app has no rclpy dependency; cargo doesn't have to be
sourced. ``pip install -r teleop_app/requirements.txt`` is enough.
"""
from __future__ import annotations

import argparse
import asyncio
import base64
import json
import math
import os
import socket
import struct
import sys
import threading
import time
from dataclasses import dataclass
from pathlib import Path
from typing import Optional

import uvicorn
from fastapi import FastAPI, WebSocket, WebSocketDisconnect
from fastapi.responses import HTMLResponse, JSONResponse
from fastapi.staticfiles import StaticFiles

# Shared IST5 envelope decoder.
sys.path.insert(0, str(Path(__file__).resolve().parent.parent / "oai_bridge"))
from protocol import KIND_CAMERA, KIND_POSE, unpack_uplink  # noqa: E402

APP_DIR = Path(__file__).resolve().parent
STATIC_DIR = APP_DIR / "static"

# MSG_FORMAT mirrors sim_isaac_trajectory.py / main_paths_los_model_*.py.
GT_MSG_FORMAT = "<ddddddddd"


# ---------------------------------------------------------------------------
@dataclass
class Config:
    http_port: int = 8080
    downlink_host: str = "192.168.70.135"
    downlink_port: int = 16000
    operator_udp_host: str = "0.0.0.0"
    operator_udp_port: int = 17500
    gt_zmq_addr: str = "tcp://127.0.0.1:5556"
    gt_disable: bool = False
    require_oai: bool = True

    @classmethod
    def from_args(cls) -> "Config":
        ap = argparse.ArgumentParser(description=__doc__.split("\n\n", 1)[0])
        ap.add_argument("--port", type=int,
                        default=int(os.environ.get("TELEOP_PORT", "8080")))
        ap.add_argument("--downlink-host",
                        default=os.environ.get("EXT_DN_IP", "192.168.70.135"))
        ap.add_argument("--downlink-port", type=int,
                        default=int(os.environ.get("EXT_DN_PORT", "16000")))
        ap.add_argument("--operator-udp-host",
                        default=os.environ.get("OPERATOR_UDP_HOST", "0.0.0.0"))
        ap.add_argument("--operator-udp-port", type=int,
                        default=int(os.environ.get("OPERATOR_UDP_PORT", "17500")))
        ap.add_argument("--gt-zmq-addr",
                        default=os.environ.get("GT_ZMQ_ADDR",
                                               "tcp://127.0.0.1:5556"))
        ap.add_argument("--gt-disable", action="store_true",
                        default=os.environ.get("GT_DISABLE", "") == "1")
        args = ap.parse_args()
        return cls(
            http_port=args.port,
            downlink_host=args.downlink_host,
            downlink_port=args.downlink_port,
            operator_udp_host=args.operator_udp_host,
            operator_udp_port=args.operator_udp_port,
            gt_zmq_addr=args.gt_zmq_addr,
            gt_disable=args.gt_disable,
        )


# ---------------------------------------------------------------------------
# Downlink — UDP JSON to ext-dn (the 5G data network ingress)
# ---------------------------------------------------------------------------
class Downlink:
    """Sends operator intent as JSON over UDP. Always toward the 5G data
    plane, never toward localhost MAVLink — that's the safety contract."""
    def __init__(self, cfg: Config):
        self.cfg = cfg
        self._sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
        # IP_TOS=DSCP_EF marks each command datagram as expedited
        # forwarding so the 5G QoS layer can map it onto a URLLC 5QI flow.
        try:
            self._sock.setsockopt(socket.IPPROTO_IP, socket.IP_TOS, 0xb8)
        except OSError:
            pass
        # Blackhole mode: if the operator explicitly says "no host", we
        # still serve the UI but don't transmit. Useful for proving the
        # safety property.
        self._enabled = cfg.downlink_host not in ("", "__none__")
        self._addr = (cfg.downlink_host, cfg.downlink_port)
        self.sent = 0
        if self._enabled:
            print(f"[drone-app] DL -> {self._addr[0]}:{self._addr[1]} "
                  f"(via OAI ext-dn)")
        else:
            print("[drone-app] DL DISABLED — running in blackhole mode")

    def send(self, payload: dict):
        if not self._enabled:
            return
        try:
            self._sock.sendto(
                json.dumps(payload, separators=(",", ":")).encode("utf-8"),
                self._addr)
            self.sent += 1
        except OSError:
            pass


# ---------------------------------------------------------------------------
# Uplink — IST5 listener on the operator side
# ---------------------------------------------------------------------------
class Uplink:
    """Receives IST5 envelopes from the UE side over OAI UL, demultiplexes
    pose vs camera, fans out to WebSocket subscribers, and republishes pose
    as MSG_FORMAT on ZMQ for the channel pipeline."""
    def __init__(self, cfg: Config):
        self.cfg = cfg
        self._stop = threading.Event()
        self._sock: Optional[socket.socket] = None
        self._subs: list[tuple[asyncio.Queue, asyncio.AbstractEventLoop]] = []
        self._sub_lock = threading.Lock()
        self._gt_pub = None
        self._t0_mono = time.monotonic()

        self.frames_rx = 0
        self.poses_rx = 0
        self.bytes_rx = 0
        self.last_pose: Optional[dict] = None
        # Last known mode/armed seen on the pose envelope.
        self.armed = False
        self.mode = "—"

    def start(self):
        sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
        sock.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
        try:
            sock.setsockopt(socket.SOL_SOCKET, socket.SO_RCVBUF, 4 * 1024 * 1024)
        except OSError:
            pass
        sock.bind((self.cfg.operator_udp_host, self.cfg.operator_udp_port))
        sock.settimeout(0.5)
        self._sock = sock
        print(f"[drone-app] UL listen: {self.cfg.operator_udp_host}:"
              f"{self.cfg.operator_udp_port}")

        if not self.cfg.gt_disable:
            try:
                import zmq
                ctx = zmq.Context.instance()
                self._gt_pub = ctx.socket(zmq.PUB)
                self._gt_pub.bind(self.cfg.gt_zmq_addr)
                print(f"[drone-app] GT republish (MSG_FORMAT) -> "
                      f"{self.cfg.gt_zmq_addr}")
            except Exception as exc:
                print(f"[drone-app] GT republish disabled: {exc}")

        self._thread = threading.Thread(
            target=self._rx_loop, name="ul-rx", daemon=True)
        self._thread.start()

    def stop(self):
        self._stop.set()
        try:
            if self._sock is not None:
                self._sock.close()
        except Exception:
            pass

    def subscribe(self, q: asyncio.Queue, loop: asyncio.AbstractEventLoop):
        with self._sub_lock:
            self._subs.append((q, loop))

    def unsubscribe(self, q: asyncio.Queue):
        with self._sub_lock:
            self._subs = [(qq, ll) for (qq, ll) in self._subs if qq is not q]

    def _push(self, env: dict):
        with self._sub_lock:
            subs = list(self._subs)
        for q, loop in subs:
            try:
                loop.call_soon_threadsafe(self._try_put, q, env)
            except RuntimeError:
                pass

    @staticmethod
    def _try_put(q: asyncio.Queue, env: dict):
        try:
            q.put_nowait(env)
        except asyncio.QueueFull:
            try:
                _ = q.get_nowait()
                q.put_nowait(env)
            except Exception:
                pass

    def _rx_loop(self):
        while not self._stop.is_set():
            try:
                data, _ = self._sock.recvfrom(65536)
            except socket.timeout:
                continue
            except OSError:
                continue
            self.bytes_rx += len(data)
            parsed = unpack_uplink(data)
            if parsed is None:
                continue
            kind, header, body = parsed
            if kind == KIND_POSE:
                self._handle_pose(header)
            elif kind == KIND_CAMERA:
                self._handle_camera(header, body)

    def _handle_pose(self, header: dict):
        self.poses_rx += 1
        try:
            armed = bool(header.get("armed", False))
            mode = str(header.get("mode", "—"))
        except Exception:
            armed, mode = False, "—"
        self.armed = armed
        self.mode = mode

        x = float(header.get("x", 0.0))
        y = float(header.get("y", 0.0))
        z = float(header.get("z", 0.0))
        vx = float(header.get("linear_x", 0.0))
        vy = float(header.get("linear_y", 0.0))
        vz = float(header.get("linear_z", 0.0))
        yaw = float(header.get("yaw", 0.0))
        speed = math.sqrt(vx*vx + vy*vy + vz*vz)

        env = {
            "type": "pose",
            "stamp": float(header.get("stamp", time.time())),
            "x": x, "y": y, "z": z,
            "yaw": yaw,
            "linear_x": vx, "linear_y": vy, "linear_z": vz,
            "speed": speed,
            "armed": armed,
            "mode": mode,
        }
        self.last_pose = env
        self._push(env)

        # MSG_FORMAT republish for the channel bridge.
        if self._gt_pub is not None:
            sim_time = time.monotonic() - self._t0_mono
            rtf = 1.0  # wall-clock pacing on the operator side.
            try:
                self._gt_pub.send(struct.pack(
                    GT_MSG_FORMAT,
                    sim_time, rtf, x, y, z, vx, vy, vz, yaw))
            except Exception:
                pass

    def _handle_camera(self, header: dict, body: bytes):
        self.frames_rx += 1
        self._push({
            "type": "camera",
            "topic": header.get("topic", ""),
            "stamp": float(header.get("stamp", time.time())),
            "width": int(header.get("width", 0)),
            "height": int(header.get("height", 0)),
            "data_b64": base64.b64encode(body).decode("ascii"),
        })


# ---------------------------------------------------------------------------
cfg = Config.from_args()
downlink = Downlink(cfg)
uplink = Uplink(cfg)

app = FastAPI(title="IsaacSim5G Drone Teleop")
app.mount("/static", StaticFiles(directory=str(STATIC_DIR)), name="static")


@app.on_event("startup")
async def _startup():
    uplink.start()


@app.on_event("shutdown")
async def _shutdown():
    uplink.stop()


@app.get("/")
async def root():
    return HTMLResponse((STATIC_DIR / "drone.html").read_text())


@app.get("/healthz")
async def healthz():
    return JSONResponse({
        "ok": True,
        "downlink": f"{cfg.downlink_host}:{cfg.downlink_port}"
                    if cfg.downlink_host not in ("", "__none__")
                    else "DISABLED",
        "downlink_sent": downlink.sent,
        "operator_udp": f"{cfg.operator_udp_host}:{cfg.operator_udp_port}",
        "frames_rx": uplink.frames_rx,
        "poses_rx": uplink.poses_rx,
        "bytes_rx": uplink.bytes_rx,
        "armed": uplink.armed,
        "mode": uplink.mode,
        "gt_publish": cfg.gt_zmq_addr if not cfg.gt_disable else "disabled",
    })


@app.websocket("/ws")
async def ws_endpoint(ws: WebSocket):
    await ws.accept()
    loop = asyncio.get_running_loop()
    q: asyncio.Queue = asyncio.Queue(maxsize=16)
    uplink.subscribe(q, loop)

    # Initial config push.
    await ws.send_text(json.dumps({
        "type": "config",
        "downlink": f"{cfg.downlink_host}:{cfg.downlink_port}"
                    if cfg.downlink_host not in ("", "__none__")
                    else "DISABLED",
        "operator_udp": f"{cfg.operator_udp_host}:{cfg.operator_udp_port}",
    }))

    last_status = 0.0

    async def pump():
        try:
            while True:
                env = await q.get()
                await ws.send_text(json.dumps(env))
        except (WebSocketDisconnect, RuntimeError):
            return

    async def status_loop():
        nonlocal last_status
        try:
            while True:
                await asyncio.sleep(0.25)
                last_status = time.time()
                await ws.send_text(json.dumps({
                    "type": "status",
                    "connected": uplink.poses_rx > 0,
                    "armed": uplink.armed,
                    "mode": uplink.mode,
                    "sent": downlink.sent,
                    "recv": uplink.poses_rx,
                    "frames": uplink.frames_rx,
                    "poses": uplink.poses_rx,
                }))
        except asyncio.CancelledError:
            return

    pump_task = asyncio.create_task(pump())
    status_task = asyncio.create_task(status_loop())

    try:
        while True:
            data = await ws.receive_text()
            try:
                msg = json.loads(data)
            except json.JSONDecodeError:
                continue
            t = msg.get("type", "stick")
            if t == "event":
                name = str(msg.get("name", "")).strip()
                if name:
                    downlink.send({"type": "event", "name": name})
            else:
                downlink.send({
                    "pitch": float(msg.get("pitch", 0.0)),
                    "roll": float(msg.get("roll", 0.0)),
                    "yaw": float(msg.get("yaw", 0.0)),
                    "throttle": float(msg.get("throttle", 0.5)),
                    "emergency_stop": bool(msg.get("emergency_stop", False)),
                })
    except WebSocketDisconnect:
        return
    finally:
        pump_task.cancel()
        status_task.cancel()
        uplink.unsubscribe(q)


def main():
    uvicorn.run(app, host="0.0.0.0", port=cfg.http_port, log_level="info")


if __name__ == "__main__":
    main()
