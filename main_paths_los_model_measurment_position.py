"""
bridge.py

Channel emulation bridge: Isaac Sim <-> OAI vrtsim

Paths mode: sends per-path parameters (delay, velocity, amplitude, phase, AoA, AoD)


Usage:
    Terminal 1:  Isaac Sim (or sim_isaac.py)
    Terminal 2:  python bridge.py --gnb-x 0 --gnb-y 0 --gnb-z 3
    Terminal 3:  OAI gNB (or sim_l1.py)
    Terminal 4:  OAI UE

Startup order:
    1. Isaac Sim (robot must be publishing before bridge sends anything)
    2. Bridge
    3. OAI gNB
    4. OAI UE
"""

import argparse
import struct
import time
import threading
import math

import numpy as np
import flatbuffers
import Phy.Taps as Taps
import Phy.Path as Path
import nnpy

try:
    import zmq
    HAS_ZMQ = True
except ImportError:
    HAS_ZMQ = False

from time_manager import TimeManager
from pose_buffer import PoseBuffer, RobotPose
from los_model import LOSChannelModel

# ─── Message formats ──────────────────────────────────────────────

ISAAC_MSG_FORMAT = "<ddddddddd"
ISAAC_MSG_SIZE = struct.calcsize(ISAAC_MSG_FORMAT)

L1_MSG_FORMAT = "<H6xQd"
L1_MSG_SIZE = struct.calcsize(L1_MSG_FORMAT)

# OAI PHY measurements: uint16 frame, uint8 slot, uint8 reserved, float range, float doppler, float velocity
MEAS_MSG_FORMAT = "<HBBfff"
MEAS_MSG_SIZE = struct.calcsize(MEAS_MSG_FORMAT)


# ─── Isaac receiver thread ────────────────────────────────────────

class MeasReceiver:
    """Receives PHY measurements (range, Doppler) from OAI gNB via nanomsg SUB."""
    def __init__(self, addr):
        self._addr = addr
        self._running = False
        self._thread = None
        self.msg_count = 0
        self.last_meas = None
        self._log_file = None

    def start(self, gt_log_path="/tmp/ground_truth.csv", est_log_path="/tmp/oai_estimates.csv"):
        self._est_log_path = est_log_path
        self._log_file = open(est_log_path, "w")
        self._log_file.write("# frame,slot,range_m,doppler_hz,velocity_mps\n")
        self._running = True
        self._thread = threading.Thread(target=self._run, daemon=True)
        self._thread.start()

    def stop(self):
        self._running = False
        if self._thread:
            self._thread.join(timeout=2.0)
        if self._log_file:
            self._log_file.close()

    def _run(self):
        sub = nnpy.Socket(nnpy.AF_SP, nnpy.SUB)
        sub.connect(self._addr)
        sub.setsockopt(nnpy.SUB, nnpy.SUB_SUBSCRIBE, b"")
        sub.setsockopt(nnpy.SOL_SOCKET, nnpy.RCVTIMEO, 100)
        print(f"[Meas] Subscribed to OAI measurements on {self._addr}")

        while self._running:
            try:
                raw = sub.recv()
            except Exception:
                continue

            if len(raw) < MEAS_MSG_SIZE:
                continue

            frame, slot, _, range_m, doppler_hz, velocity_mps = \
                struct.unpack(MEAS_MSG_FORMAT, raw[:MEAS_MSG_SIZE])

            self.last_meas = {
                'frame': frame,
                'slot': slot,
                'range_m': range_m,
                'doppler_hz': doppler_hz,
                'velocity_mps': velocity_mps,
            }
            self.msg_count += 1

            # Write to CSV
            if self._log_file:
                self._log_file.write(f"{frame},{slot},{range_m:.4f},{doppler_hz:.4f},{velocity_mps:.4f}\n")
                self._log_file.flush()

            if self.msg_count % 100 == 0:
                print(f"[Meas] Frame={frame} slot={slot} range={range_m:.2f}m "
                      f"doppler={doppler_hz:.1f}Hz vel={velocity_mps:.2f}m/s")

        sub.close()


class IsaacReceiver:
    def __init__(self, addr, pose_buffer, time_manager):
        self._addr = addr
        self._pose_buffer = pose_buffer
        self._time_manager = time_manager
        self._running = False
        self._thread = None
        self.msg_count = 0
        self.last_pose = None

    def start(self):
        self._running = True
        self._thread = threading.Thread(target=self._run, daemon=True)
        self._thread.start()

    def stop(self):
        self._running = False
        if self._thread:
            self._thread.join(timeout=2.0)

    def _run(self):
        if not HAS_ZMQ:
            print("[Isaac] ERROR: pyzmq not installed")
            return

        ctx = zmq.Context()
        sub = ctx.socket(zmq.SUB)
        sub.connect(self._addr)
        sub.setsockopt_string(zmq.SUBSCRIBE, "")
        sub.setsockopt(zmq.RCVTIMEO, 100)
        print(f"[Isaac] Subscribed to {self._addr}")

        while self._running:
            try:
                raw = sub.recv()
            except zmq.Again:
                continue

            if len(raw) < ISAAC_MSG_SIZE:
                continue

            wall_ns = time.monotonic_ns()
            sim_time, rtf, x, y, z, vx, vy, vz, heading = \
                struct.unpack(ISAAC_MSG_FORMAT, raw[:ISAAC_MSG_SIZE])

            pose = RobotPose(
                time_s=sim_time,
                x=x, y=y, z=z,
                vx=vx, vy=vy, vz=vz,
                heading=heading,
            )

            self._pose_buffer.add(pose)
            self._time_manager.update_isaac(sim_time, wall_ns)
            self.last_pose = pose
            self.msg_count += 1

        sub.close()
        ctx.term()


# ─── FlatBuffer builder ──────────────────────────────────────────

def build_paths_message(builder, paths_list, distance_m=0.0, apply_at_frame=0):
    """
    Build a FlatBuffer message with per-path parameters.

    paths_list: list of dicts with keys:
        delay_s, v_radial, amplitude, phase_rad, aoa_rad, aod_rad
    """
    builder.Clear()

    path_offsets = []

    # print(f"[Channnnel] Applied Frame = {apply_at_frame:d} | "
    #       f"distance = {distance_m:.2f}m | "
    #       )
    
    for p in paths_list:
        Path.PathStart(builder)
        Path.PathAddDelayS(builder, p['delay_s'])
        Path.PathAddAmplitude(builder, p['amplitude'])
        Path.PathAddPhaseRad(builder, p['phase_rad'])
        Path.PathAddAoaRad(builder, p['aoa_rad'])
        Path.PathAddAodRad(builder, p['aod_rad'])
        Path.PathAddVRadial(builder, p['v_radial'])
        # print(f"[Channnnel] path={p}"
        #      )
        path_offsets.append(Path.PathEnd(builder))

    Taps.TapsStartPathsVector(builder, len(path_offsets))
    for offset in reversed(path_offsets):
        builder.PrependUOffsetTRelative(offset)
    paths_vec = builder.EndVector()

    Taps.TapsStart(builder)
    Taps.TapsAddPaths(builder, paths_vec)
    Taps.TapsAddDistanceM(builder, distance_m)
    Taps.TapsAddApplyAtFrame(builder, apply_at_frame)
    msg = Taps.TapsEnd(builder)
    builder.Finish(msg)   
    return builder.Output()


# ─── Main ─────────────────────────────────────────────────────────

def main():
    parser = argparse.ArgumentParser(description="Isaac Sim <-> OAI Channel Bridge")
    parser.add_argument("--isaac-addr", default="tcp://127.0.0.1:5556")
    parser.add_argument("--oai-taps-addr", default="tcp://127.0.0.1:5555")
    parser.add_argument("--l1-timing-addr", default="tcp://127.0.0.1:5557")
    parser.add_argument("--gnb-x", type=float, default=0.0, help="gNB X position in meters")
    parser.add_argument("--gnb-y", type=float, default=0.0, help="gNB Y position in meters")
    parser.add_argument("--gnb-z", type=float, default=0.0, help="gNB Z position in meters")
    parser.add_argument("--gnb-boresight", type=float, default=0.0,
                        help="gNB boresight angle in radians")
    parser.add_argument("--fc-ghz", type=float, default=3.5, help="Carrier frequency in GHz")
    parser.add_argument("--log-every", type=int, default=100,
                        help="Log every N frames (default 100 = every 1s)")
    parser.add_argument("--meas-addr", default="tcp://127.0.0.1:5558",
                        help="Nanomsg address for OAI PHY measurements (SUB)")
    parser.add_argument("--gt-log", default="/tmp/ground_truth.csv",
                        help="Ground truth CSV output path")
    parser.add_argument("--est-log", default="/tmp/oai_estimates.csv",
                        help="OAI estimates CSV output path")
    args = parser.parse_args()

    # ── Components ────────────────────────────────────────────────
    time_mgr = TimeManager()
    pose_buf = PoseBuffer(capacity=200)
    gnb_pos = (args.gnb_x, args.gnb_y, args.gnb_z)

    channel = LOSChannelModel(
        gnb_pos=gnb_pos,
        gnb_boresight_rad=args.gnb_boresight,
        fc_hz=args.fc_ghz * 1e9,
    )

    fb_builder = flatbuffers.Builder(1024)

    # ── Nanomsg PUB → OAI ─────────────────────────────────────────
    taps_pub = nnpy.Socket(nnpy.AF_SP, nnpy.PUB)
    taps_pub.bind(args.oai_taps_addr)
    print(f"[Bridge] Paths PUB on {args.oai_taps_addr}")

    # ── Nanomsg SUB ← L1 frame timing ────────────────────────────
    l1_sub = nnpy.Socket(nnpy.AF_SP, nnpy.SUB)
    l1_sub.connect(args.l1_timing_addr)
    l1_sub.setsockopt(nnpy.SUB, nnpy.SUB_SUBSCRIBE, b"")
    l1_sub.setsockopt(nnpy.SOL_SOCKET, nnpy.RCVTIMEO, 100)
    print(f"[Bridge] L1 timing SUB on {args.l1_timing_addr}")

    # ── Isaac receiver thread ─────────────────────────────────────
    isaac = IsaacReceiver(args.isaac_addr, pose_buf, time_mgr)
    isaac.start()

    # ── OAI measurement receiver thread ───────────────────────────
    meas = MeasReceiver(args.meas_addr)
    meas.start(gt_log_path=args.gt_log, est_log_path=args.est_log)

    # ── Ground truth CSV ──────────────────────────────────────────
    gt_file = open(args.gt_log, "w")
    gt_file.write("# frame,distance_m,v_radial,doppler_hz,aoa_rad,robot_x,robot_y\n")

    # ── Wait for first Isaac pose before doing anything ───────────
    print("[Bridge] Waiting for first Isaac pose...")
    while isaac.last_pose is None:
        time.sleep(0.1)
    print(f"[Bridge] Got first pose: ({isaac.last_pose.x:.2f}, {isaac.last_pose.y:.2f}, {isaac.last_pose.z:.2f})")

    # ── State ─────────────────────────────────────────────────────
    frames_published = 0
    last_oai_frame = -1
    last_detail_log = time.monotonic()

    # ── Publish initial paths so OAI spin-wait can pass ───────────
    print("[Bridge] Publishing initial paths, waiting for OAI...")
    while True:
        # Keep publishing until L1 starts
        pose = isaac.last_pose
        distance = pose.distance_to(gnb_pos[0], gnb_pos[1], gnb_pos[2])
        paths_list = channel.get_paths(pose)
        msg = build_paths_message(fb_builder, paths_list, distance, apply_at_frame=0)
        taps_pub.send(msg)

        # Check if L1 message arrived
        try:
            raw = l1_sub.recv()
            if len(raw) >= L1_MSG_SIZE:
                print("[Bridge] L1 connected. Entering main loop.")
                # Process this first L1 message in the main loop
                break
        except Exception:
            time.sleep(0.01)
            continue

    # ── Main loop: driven by L1 frame timing ──────────────────────
    # Process the first L1 message we already received
    first_l1 = raw

    try:
        while True:
            # Use the buffered first message or receive new one
            if first_l1 is not None:
                raw = first_l1
                first_l1 = None
            else:
                try:
                    raw = l1_sub.recv()
                except Exception:
                    continue

            if len(raw) < L1_MSG_SIZE:
                continue

            # ── Parse L1 timing ───────────────────────────────────
            l1_wall_ns = time.monotonic_ns()
            oai_frame, sample_count, sample_rate = struct.unpack(L1_MSG_FORMAT, raw[:L1_MSG_SIZE])
            oai_sim_time = sample_count / sample_rate
            time_mgr.update_oai(oai_sim_time, l1_wall_ns)

            if oai_frame == last_oai_frame:
                continue
            last_oai_frame = oai_frame

            # ── Map to Isaac time ─────────────────────────────────
            target_isaac_time = time_mgr.oai_frame_to_isaac(l1_wall_ns, lookahead_s=0.01)

            # ── Look up robot pose ────────────────────────────────
            pose = pose_buf.get_at(target_isaac_time, max_extrapolation_s=0.1)
            if pose is None:
                continue

            # ── Compute paths from pose ───────────────────────────
            distance = pose.distance_to(gnb_pos[0], gnb_pos[1], gnb_pos[2])
            paths_list = channel.get_paths(pose)

            # ── Publish ───────────────────────────────────────────
            apply_at_frame = oai_frame + 1
            msg = build_paths_message(fb_builder, paths_list, distance, apply_at_frame=apply_at_frame)
            taps_pub.send(msg)
            frames_published += 1

            # ── Write ground truth CSV ────────────────────────────
            v_radial = paths_list[0]['v_radial']
            aoa_rad = paths_list[0]['aoa_rad']
            fc_hz = args.fc_ghz * 1e9
            doppler_gt = v_radial * fc_hz / 3e8
            gt_file.write(f"{apply_at_frame},{distance:.4f},{v_radial:.4f},{doppler_gt:.4f},{aoa_rad:.6f},{pose.x:.4f},{pose.y:.4f}\n")
            gt_file.flush()

            # ── Per-frame log ─────────────────────────────────────
            if frames_published % args.log_every == 0:
                n_paths = len(paths_list)
                # print(
                #     f"[Bridge] Frame={oai_frame:4d} | "
                #     f"robot=({pose.x:.1f},{pose.y:.1f},{pose.z:.1f}) "
                #     f"speed={pose.speed:.2f}m/s | "
                #     f"dist={distance:.2f}m | "
                #     f"paths={n_paths} | "
                #     f"v_radial={paths_list[0]['v_radial']:.1f}m/s | "
                #     f"sent={frames_published}"
                # )

            # ── Detailed status (every 10s) ───────────────────────
            now = time.monotonic()
            if now - last_detail_log > 10.0:
                drift = time_mgr.get_drift_report()
                latest = pose_buf.latest
                extrap_ms = (target_isaac_time - latest.time_s) * 1000 if latest else 0

                print(
                    f"[Bridge] ── STATUS ─────────────────────────────\n"
                    f"  Frames: sent={frames_published}\n"
                    f"  Isaac:  msgs={isaac.msg_count} "
                    f"buf={pose_buf.size} ({pose_buf.time_span_s:.2f}s)\n"
                    f"  Drift:  gap={drift.clock_gap_s*1000:+.1f}ms "
                    f"rate={drift.drift_rate*1000:.2f}ms/s "
                    f"({drift.severity})\n"
                    f"  Pose:   extrap={extrap_ms:+.1f}ms\n"
                    f"  Channel: dist={distance:.2f}m "
                    f"paths={len(paths_list)} "
                    f"v_radial={paths_list[0]['v_radial']:.1f}m/s"
                )
                last_detail_log = now

    except KeyboardInterrupt:
        print(f"\n[Bridge] Stopped. Sent {frames_published} path messages.")
    finally:
        isaac.stop()
        meas.stop()
        gt_file.close()
        taps_pub.close()
        l1_sub.close()


if __name__ == "__main__":
    main()