"""
sim_isaac_trajectory.py

Ground-truth pose publisher for the IsaacSim5G channel pipeline.

Historically this file generated synthetic trajectories (line, circle,
square, …) so the OAI channel bridge could run without Isaac. It still
does, but it now has one additional mode — ``--trajectory isaac`` — that
pulls real-time drone pose from Isaac Sim and re-publishes it in the same
wire format the rest of the pipeline already expects.

Wire format (unchanged):
    MSG_FORMAT = "<ddddddddd"
    (sim_time, rtf, x, y, z, vx, vy, vz, heading)
    Published at PUBLISH_INTERVAL on a ZMQ PUB socket.

Modes:
    line / circle / square / lshape / figure8 / random
        Synthetic trajectory — exactly the old behavior. Useful when
        Isaac isn't running and you want to exercise the channel
        bridge in isolation.

    isaac
        Subscribe to the drone-telemetry sidecar (running in Isaac
        Sim's Script Editor — see isaac/drone_telemetry.py) and
        re-publish each pose sample on the standard channel-bridge
        address. Velocities and heading come straight from Isaac;
        no synthetic generator is involved. This is the path used
        when you fly the city_drone_fpv Iris yourself via the
        teleop dashboard.

Usage:
    # Pull live pose from Isaac and feed the channel bridge.
    python sim_isaac_trajectory.py --trajectory isaac

    # Synthetic patterns (legacy):
    python sim_isaac_trajectory.py --trajectory line --start-x 3 --end-x 60 --speed 5
    python sim_isaac_trajectory.py --trajectory circle --center-x 30 --radius 20 --speed 5
"""

import argparse
import math
import struct
import time

try:
    import zmq
except ImportError:
    print("ERROR: pyzmq required. Install with: pip install pyzmq")
    exit(1)

MSG_FORMAT = "<ddddddddd"
PUBLISH_INTERVAL = 0.01


# ─── Trajectory generators ───────────────────────────────────────
# Each returns (x, y, vx, vy, heading) for a given sim_time

class LineTrajectory:
    """Back and forth along a straight line."""
    def __init__(self, start_x, start_y, end_x, end_y, speed):
        self.sx, self.sy = start_x, start_y
        self.dx = end_x - start_x
        self.dy = end_y - start_y
        self.length = math.sqrt(self.dx**2 + self.dy**2)
        self.speed = speed
        self.ux = self.dx / self.length
        self.uy = self.dy / self.length
        self.leg_time = self.length / speed
        self.name = f"Line ({start_x},{start_y})↔({end_x},{end_y})"

    def get_pose(self, t):
        cycle = 2.0 * self.leg_time
        tc = t % cycle
        if tc < self.leg_time:
            p = tc / self.leg_time
            x = self.sx + p * self.dx
            y = self.sy + p * self.dy
            vx = self.speed * self.ux
            vy = self.speed * self.uy
            heading = math.atan2(self.dy, self.dx)
        else:
            p = (tc - self.leg_time) / self.leg_time
            x = self.sx + self.dx - p * self.dx
            y = self.sy + self.dy - p * self.dy
            vx = -self.speed * self.ux
            vy = -self.speed * self.uy
            heading = math.atan2(-self.dy, -self.dx)
        return x, y, vx, vy, heading


class CircleTrajectory:
    """Circular orbit around a center point."""
    def __init__(self, center_x, center_y, radius, speed):
        self.cx, self.cy = center_x, center_y
        self.r = radius
        self.speed = speed
        self.omega = speed / radius
        self.period = 2 * math.pi * radius / speed
        self.name = f"Circle center=({center_x},{center_y}) r={radius}m"

    def get_pose(self, t):
        theta = self.omega * t
        x = self.cx + self.r * math.cos(theta)
        y = self.cy + self.r * math.sin(theta)
        vx = -self.speed * math.sin(theta)
        vy = self.speed * math.cos(theta)
        heading = math.atan2(vy, vx)
        return x, y, vx, vy, heading


class SquareTrajectory:
    """Square path around a center point."""
    def __init__(self, center_x, center_y, side, speed):
        self.speed = speed
        half = side / 2.0
        # Four corners: bottom-left → bottom-right → top-right → top-left
        self.corners = [
            (center_x - half, center_y - half),
            (center_x + half, center_y - half),
            (center_x + half, center_y + half),
            (center_x - half, center_y + half),
        ]
        self.side = side
        self.perimeter = 4 * side
        self.lap_time = self.perimeter / speed
        self.name = f"Square center=({center_x},{center_y}) side={side}m"

    def get_pose(self, t):
        tc = t % self.lap_time
        dist = tc * self.speed
        seg_length = self.side

        # Which segment (0-3)
        seg = int(dist / seg_length)
        if seg >= 4:
            seg = 3
        seg_progress = (dist - seg * seg_length) / seg_length

        # Current and next corner
        c0 = self.corners[seg % 4]
        c1 = self.corners[(seg + 1) % 4]

        x = c0[0] + seg_progress * (c1[0] - c0[0])
        y = c0[1] + seg_progress * (c1[1] - c0[1])

        dx = c1[0] - c0[0]
        dy = c1[1] - c0[1]
        d = math.sqrt(dx**2 + dy**2)
        vx = self.speed * dx / d
        vy = self.speed * dy / d
        heading = math.atan2(dy, dx)

        return x, y, vx, vy, heading


class LShapeTrajectory:
    """L-shaped path: go forward, turn 90°, go forward, then reverse back."""
    def __init__(self, start_x, start_y, leg1, leg2, speed):
        self.speed = speed
        self.sx, self.sy = start_x, start_y
        self.leg1 = leg1
        self.leg2 = leg2
        # Waypoints: start → corner → end
        self.waypoints = [
            (start_x, start_y),
            (start_x + leg1, start_y),
            (start_x + leg1, start_y + leg2),
        ]
        self.total_dist = leg1 + leg2
        self.half_time = self.total_dist / speed
        self.name = f"L-shape start=({start_x},{start_y}) legs={leg1}m+{leg2}m"

    def _interp(self, dist):
        """Get position and direction at a given distance along the L path."""
        if dist <= self.leg1:
            p = dist / self.leg1
            w0, w1 = self.waypoints[0], self.waypoints[1]
        else:
            p = (dist - self.leg1) / self.leg2
            w0, w1 = self.waypoints[1], self.waypoints[2]

        p = max(0, min(1, p))
        x = w0[0] + p * (w1[0] - w0[0])
        y = w0[1] + p * (w1[1] - w0[1])
        dx = w1[0] - w0[0]
        dy = w1[1] - w0[1]
        return x, y, dx, dy

    def get_pose(self, t):
        cycle = 2.0 * self.half_time
        tc = t % cycle

        if tc < self.half_time:
            dist = tc * self.speed
            x, y, dx, dy = self._interp(dist)
        else:
            dist = self.total_dist - (tc - self.half_time) * self.speed
            x, y, dx, dy = self._interp(dist)
            dx, dy = -dx, -dy

        d = math.sqrt(dx**2 + dy**2)
        vx = self.speed * dx / d if d > 0 else 0
        vy = self.speed * dy / d if d > 0 else 0
        heading = math.atan2(dy, dx)
        return x, y, vx, vy, heading


class Figure8Trajectory:
    """Figure-8 (lemniscate) trajectory."""
    def __init__(self, center_x, center_y, radius, speed):
        self.cx, self.cy = center_x, center_y
        self.a = radius
        self.speed = speed
        # Approximate period using numerical integration would be exact,
        # but for simplicity use a parametric rate
        self.omega = speed / (radius * 1.5)  # approximate
        self.name = f"Figure-8 center=({center_x},{center_y}) r={radius}m"

    def get_pose(self, t):
        theta = self.omega * t
        # Lemniscate of Bernoulli: x = a*cos(t)/(1+sin²(t)), y = a*sin(t)*cos(t)/(1+sin²(t))
        s = math.sin(theta)
        c = math.cos(theta)
        denom = 1 + s * s
        x = self.cx + self.a * c / denom
        y = self.cy + self.a * s * c / denom

        # Numerical velocity via finite difference
        dt = 1e-5
        s2 = math.sin(theta + self.omega * dt)
        c2 = math.cos(theta + self.omega * dt)
        d2 = 1 + s2 * s2
        x2 = self.cx + self.a * c2 / d2
        y2 = self.cy + self.a * s2 * c2 / d2

        vx = (x2 - x) / dt
        vy = (y2 - y) / dt

        # Normalize to desired speed
        v = math.sqrt(vx**2 + vy**2)
        if v > 0:
            vx = self.speed * vx / v
            vy = self.speed * vy / v

        heading = math.atan2(vy, vx)
        return x, y, vx, vy, heading


class RandomWalkTrajectory:
    """Random walk with smooth turns. Changes direction periodically."""
    def __init__(self, center_x, center_y, area_size, speed):
        import random
        self.cx, self.cy = center_x, center_y
        self.half = area_size / 2
        self.speed = speed
        self.rng = random.Random(42)
        # Pre-generate waypoints
        self.waypoints = [(center_x, center_y)]
        for _ in range(1000):
            wx = center_x + self.rng.uniform(-self.half, self.half)
            wy = center_y + self.rng.uniform(-self.half, self.half)
            self.waypoints.append((wx, wy))
        self._build_segments()
        self.name = f"RandomWalk center=({center_x},{center_y}) area={area_size}m"

    def _build_segments(self):
        self.seg_dists = []
        self.total_dist = 0
        for i in range(len(self.waypoints) - 1):
            dx = self.waypoints[i+1][0] - self.waypoints[i][0]
            dy = self.waypoints[i+1][1] - self.waypoints[i][1]
            d = math.sqrt(dx**2 + dy**2)
            self.seg_dists.append(d)
            self.total_dist += d

    def get_pose(self, t):
        dist = (t * self.speed) % self.total_dist
        cumulative = 0
        for i, sd in enumerate(self.seg_dists):
            if cumulative + sd >= dist:
                p = (dist - cumulative) / sd if sd > 0 else 0
                w0 = self.waypoints[i]
                w1 = self.waypoints[i + 1]
                x = w0[0] + p * (w1[0] - w0[0])
                y = w0[1] + p * (w1[1] - w0[1])
                dx = w1[0] - w0[0]
                dy = w1[1] - w0[1]
                d = math.sqrt(dx**2 + dy**2)
                vx = self.speed * dx / d if d > 0 else 0
                vy = self.speed * dy / d if d > 0 else 0
                heading = math.atan2(dy, dx)
                return x, y, vx, vy, heading
            cumulative += sd
        # Fallback
        return self.cx, self.cy, 0, 0, 0


# ─── Isaac live source ────────────────────────────────────────────
#
# Bridge from the Isaac drone telemetry sidecar (isaac/drone_telemetry.py)
# to the channel pipeline. The sidecar publishes JSON on a ZMQ PUB; we
# subscribe, repack into MSG_FORMAT, and forward on the standard
# channel-bridge address — so main_paths_los_model_measurment_position.py
# is unaware Isaac (rather than a synthetic generator) is the source.

def run_isaac_source(pub, isaac_src_addr, log_every=200):
    """Subscribe to the Isaac sidecar's pose feed and re-publish in
    MSG_FORMAT on ``pub`` until interrupted."""
    try:
        import zmq
    except ImportError:
        print("[SimIsaac] ERROR: pyzmq required for --trajectory isaac")
        return

    ctx = zmq.Context.instance()
    sub = ctx.socket(zmq.SUB)
    sub.connect(isaac_src_addr)
    sub.setsockopt_string(zmq.SUBSCRIBE, "")
    # Receive timeout so Ctrl-C is responsive even when Isaac hasn't sent
    # anything yet.
    sub.setsockopt(zmq.RCVTIMEO, 500)
    print(f"[SimIsaac] Source: live Isaac sidecar at {isaac_src_addr}")
    print(f"[SimIsaac] Waiting for first pose sample...")

    msg_count = 0
    first_seen = False
    last_log_t = time.monotonic()
    try:
        while True:
            try:
                payload = sub.recv_json()
            except zmq.Again:
                continue
            if not first_seen:
                first_seen = True
                print(f"[SimIsaac] First Isaac pose received: "
                      f"({payload.get('x', 0):.2f}, {payload.get('y', 0):.2f}, "
                      f"{payload.get('z', 0):.2f})")
            sim_time = float(payload.get("sim_time", 0.0))
            rtf = float(payload.get("rtf", 1.0))
            x = float(payload.get("x", 0.0))
            y = float(payload.get("y", 0.0))
            z = float(payload.get("z", 0.0))
            vx = float(payload.get("vx", 0.0))
            vy = float(payload.get("vy", 0.0))
            vz = float(payload.get("vz", 0.0))
            heading = float(payload.get("heading", 0.0))
            pub.send(struct.pack(
                MSG_FORMAT, sim_time, rtf, x, y, z, vx, vy, vz, heading))
            msg_count += 1
            now = time.monotonic()
            if msg_count % log_every == 0 or now - last_log_t >= 5.0:
                speed = math.sqrt(vx**2 + vy**2 + vz**2)
                print(f"[SimIsaac] t={sim_time:6.2f}s rtf={rtf:.2f} "
                      f"pos=({x:7.2f},{y:7.2f},{z:6.2f}) "
                      f"v={speed:5.2f}m/s hdg={math.degrees(heading):6.1f}° "
                      f"sent={msg_count}")
                last_log_t = now
    except KeyboardInterrupt:
        print(f"\n[SimIsaac] Stopped after {msg_count} messages.")
    finally:
        sub.close(0)


# ─── Main ─────────────────────────────────────────────────────────

def main():
    parser = argparse.ArgumentParser(description="Simulated Isaac Sim Robot")
    parser.add_argument("--addr", default="tcp://127.0.0.1:5556",
                        help="ZMQ PUB bind address")
    parser.add_argument("--trajectory",
                        choices=["line", "circle", "square", "lshape",
                                 "figure8", "random", "isaac"],
                        default="line",
                        help="Trajectory type. 'isaac' bridges live drone "
                             "pose from the Isaac Script Editor sidecar.")
    parser.add_argument("--isaac-src",
                        default="tcp://127.0.0.1:5570",
                        help="ZMQ SUB address for --trajectory isaac. Must "
                             "match ISAAC_POSE_PUB_ADDR in drone_telemetry.py.")
    parser.add_argument("--start-x", type=float, default=3.0)
    parser.add_argument("--start-y", type=float, default=0.0)
    parser.add_argument("--end-x", type=float, default=60.0)
    parser.add_argument("--end-y", type=float, default=0.0)
    parser.add_argument("--center-x", type=float, default=30.0, help="Center X for circle/square/figure8/random")
    parser.add_argument("--center-y", type=float, default=0.0, help="Center Y for circle/square/figure8/random")
    parser.add_argument("--radius", type=float, default=20.0, help="Radius for circle/figure8")
    parser.add_argument("--side", type=float, default=40.0, help="Side length for square")
    parser.add_argument("--leg1", type=float, default=30.0, help="First leg length for L-shape")
    parser.add_argument("--leg2", type=float, default=20.0, help="Second leg length for L-shape")
    parser.add_argument("--area", type=float, default=50.0, help="Area size for random walk")
    parser.add_argument("--height", type=float, default=0.0, help="Robot Z height")
    parser.add_argument("--speed", type=float, default=5.0, help="Robot speed in m/s")
    args = parser.parse_args()

    # ZMQ PUB shared by all modes.
    print(f"[SimIsaac] Publishing on {args.addr}")
    ctx = zmq.Context()
    pub = ctx.socket(zmq.PUB)
    pub.bind(args.addr)

    # Live Isaac mode: short-circuit and run the bridge loop.
    if args.trajectory == "isaac":
        try:
            run_isaac_source(pub, args.isaac_src)
        finally:
            pub.close()
            ctx.term()
        return

    # Synthetic trajectory modes.
    if args.trajectory == "line":
        traj = LineTrajectory(args.start_x, args.start_y, args.end_x, args.end_y, args.speed)
    elif args.trajectory == "circle":
        traj = CircleTrajectory(args.center_x, args.center_y, args.radius, args.speed)
    elif args.trajectory == "square":
        traj = SquareTrajectory(args.center_x, args.center_y, args.side, args.speed)
    elif args.trajectory == "lshape":
        traj = LShapeTrajectory(args.start_x, args.start_y, args.leg1, args.leg2, args.speed)
    elif args.trajectory == "figure8":
        traj = Figure8Trajectory(args.center_x, args.center_y, args.radius, args.speed)
    elif args.trajectory == "random":
        traj = RandomWalkTrajectory(args.center_x, args.center_y, args.area, args.speed)

    print(f"[SimIsaac] Trajectory: {traj.name}")
    print(f"[SimIsaac] Speed: {args.speed} m/s")

    sim_time = 0.0
    next_publish = time.monotonic()
    msg_count = 0

    try:
        while True:
            x, y, vx, vy, heading = traj.get_pose(sim_time)
            z = args.height
            vz = 0.0
            rtf = 1.0

            msg = struct.pack(MSG_FORMAT,
                              sim_time, rtf,
                              x, y, z,
                              vx, vy, vz,
                              heading)
            pub.send(msg)

            msg_count += 1
            sim_time += PUBLISH_INTERVAL

            if msg_count % 500 == 0:
                speed = math.sqrt(vx**2 + vy**2)
                print(f"[SimIsaac] t={sim_time:.1f}s pos=({x:.2f}, {y:.2f}) "
                      f"vel=({vx:.2f}, {vy:.2f}) speed={speed:.1f}m/s heading={math.degrees(heading):.1f}°")

            next_publish += PUBLISH_INTERVAL
            sleep_time = next_publish - time.monotonic()
            if sleep_time > 0:
                time.sleep(sleep_time)
            else:
                missed = int(-sleep_time / PUBLISH_INTERVAL)
                next_publish += missed * PUBLISH_INTERVAL

    except KeyboardInterrupt:
        print(f"\n[SimIsaac] Stopped after {msg_count} messages ({sim_time:.1f}s)")
    finally:
        pub.close()
        ctx.term()


if __name__ == "__main__":
    main()