#!/usr/bin/env python
"""
| File: city_drone_fpv.py
| Description: Loads the NVIDIA CityDemoPack (Isaac Sim AEC) and spawns an Iris
|   quadrotor with a forward-facing wide-FOV FPV camera. PX4 SITL auto-launches
|   and exposes MAVLink on UDP 14550 for QGroundControl.
|
| Run: /home/varun/isaacsim/python.sh examples/city_drone_fpv.py
"""

import carb
from isaacsim import SimulationApp

simulation_app = SimulationApp({"headless": False})

import os
import omni.timeline
import omni.kit.viewport.utility as vp_utils
from pxr import Sdf, Usd, UsdGeom, UsdPhysics, Gf
import numpy as np
from omni.isaac.core.world import World
from omni.isaac.core.utils.prims import is_prim_path_valid

from pegasus.simulator.params import ROBOTS
from pegasus.simulator.logic.backends.px4_mavlink_backend import (
    PX4MavlinkBackend,
    PX4MavlinkBackendConfig,
)
from pegasus.simulator.logic.vehicles.multirotor import Multirotor, MultirotorConfig
from pegasus.simulator.logic.interface.pegasus_interface import PegasusInterface

from scipy.spatial.transform import Rotation

# Local NVIDIA CityDemoPack USD (downloaded under examples/isaac_city_demo).
CITY_USD = "/home/varun/PegasusSimulator/examples/isaac_city_demo/CityDemopack/World_CityDemopack.usd"

DRONE_PRIM = "/World/quadrotor"
DRONE_BODY = f"{DRONE_PRIM}/body"
# FPV camera lives at the stage root (NOT under the drone). Its world pose is
# driven manually from the drone body's world matrix every frame, which side-
# steps any unit-conversion issues USD applies when a child prim crosses a
# different metersPerUnit boundary (the CityDemoPack USD is in cm).
FPV_CAMERA_PRIM = "/World/fpv_cam"

# Camera offset in the drone body's local frame (meters): 30 cm forward, 5 cm up.
FPV_CAM_OFFSET = (0.30, 0.0, 0.05)

# Spawn position. Z is well above any reasonable building so the drone
# free-falls onto whichever city mesh is directly under (0, 0) — with the
# city's meshes given physics colliders below, that's the road/plaza surface.
SPAWN_XY = (0.0, 0.0)
SPAWN_Z = 50.0
USD_WARMUP_FRAMES = 240            # frames to pump so the USD payload streams
# The CityDemoPack USD coordinates are in centimeters (bbox spans -2000..+12302
# when interpreted as the stage's meter units, i.e. 100× too big). Scale the
# entire /World/layout by 0.01 so 1 unit in the city USD = 1 meter in our
# stage — drone physics and city geometry now match scale.
CITY_TO_METERS = 0.01


class PegasusApp:
    def __init__(self):
        self.timeline = omni.timeline.get_timeline_interface()

        # PX4 SITL networking: keep MAVLink on localhost. QGC connects to UDP 14550.
        os.environ["PX4_SIM_HOST_ADDR"] = "127.0.0.1"
        os.environ["PX4_GCS_HOST_ADDR"] = "127.0.0.1"

        # Pin PX4's home GPS to match Pegasus's world origin (Lisbon, from
        # extensions/.../config/configs.yaml). Without this, PX4 boots with its
        # default home (Zurich) and EKF has to drag the position estimate ~1500 km
        # when the first GPS sample arrives — which destabilizes attitude.
        # These must be set BEFORE PegasusInterface autolaunches PX4.
        os.environ["PX4_HOME_LAT"] = "38.736832"
        os.environ["PX4_HOME_LON"] = "-9.137977"
        os.environ["PX4_HOME_ALT"] = "90.0"

        self.pg = PegasusInterface()
        # Make sure Pegasus's GPS reprojection uses the same origin we just told PX4.
        self.pg.set_global_coordinates(
            latitude=38.736832, longitude=-9.137977, altitude=90.0
        )
        # Lower physics rate so the renderer can keep up under the heavy city
        # scene; this lets us enable lockstep without PX4's IMU/GPS feed stalling.
        self.pg._world_settings["physics_dt"] = 1.0 / 100.0
        self.pg._world_settings["rendering_dt"] = 1.0 / 60.0
        self.pg._world = World(**self.pg._world_settings)
        self.world = self.pg.world

        if not os.path.exists(CITY_USD):
            carb.log_error(f"[city] {CITY_USD} not found")
            simulation_app.close()
            raise SystemExit(1)

        carb.log_warn(f"[city] loading {CITY_USD}")
        self.pg.load_environment(CITY_USD)

        # Fallback lighting: the CityDemoPack's DomeLight references a cloud HDR
        # that isn't reachable, so the scene renders pitch black. A local
        # DistantLight (sun) + DomeLight guarantee the city is visible.
        self._add_fallback_lights()

        # Pump the app so the USD payload (city geometry) actually streams in —
        # load_environment is async, so the meshes don't exist until the
        # underlying USD has loaded.
        carb.log_warn(f"[city] streaming USD payload ({USD_WARMUP_FRAMES} frames)...")
        for _ in range(USD_WARMUP_FRAMES):
            simulation_app.update()

        # Convert city's cm-units to meters so the drone scale matches.
        self._set_layout_transform(scale=CITY_TO_METERS, translate=(0.0, 0.0, 0.0))

        # Apply convex-hull colliders AFTER scaling so PhysX sees the meshes
        # at their final size.
        self._apply_city_colliders()

        # Locate a grass/park surface in the now-meter-scaled world. Then
        # translate the layout so the chosen spawn lands at world origin —
        # this keeps float precision sharp and aligns GPS with PX4 home.
        spawn_pose = self._find_spawn_pose()
        if spawn_pose is None:
            spawn_xyz = (SPAWN_XY[0], SPAWN_XY[1], SPAWN_Z)
            self._add_ground_plane(0.0)
        else:
            cx, cy, top_z = spawn_pose
            self._set_layout_transform(
                scale=CITY_TO_METERS, translate=(-cx, -cy, 0.0)
            )
            spawn_xyz = (0.0, 0.0, top_z + 0.5)
            self._add_ground_plane(top_z)
        carb.log_warn(f"[city] spawn position {spawn_xyz}")

        config_multirotor = MultirotorConfig()
        # Lockstep stays OFF: the city scene is too heavy for the renderer to keep
        # up with synchronized physics, which jams the Pegasus<->PX4 TCP link and
        # makes EKF diverge. Wall-clock mode trades determinism for stability.
        mavlink_cfg = PX4MavlinkBackendConfig({
            "vehicle_id": 0,
            "enable_lockstep": False,
            "update_rate": 100.0,
            "px4_autolaunch": True,
            "px4_dir": self.pg.px4_path,
            "px4_vehicle_model": self.pg.px4_default_airframe,
        })
        config_multirotor.backends = [PX4MavlinkBackend(mavlink_cfg)]

        Multirotor(
            DRONE_PRIM,
            ROBOTS["Iris"],
            0,
            list(spawn_xyz),
            Rotation.from_euler("XYZ", [0.0, 0.0, 0.0], degrees=True).as_quat(),
            config=config_multirotor,
        )

        self.world.reset()
        self._attach_fpv_camera()

        self.stop_sim = False
        self._viewport_pinned = False
        # Pin the viewport to the FPV camera after a few frames so the drone
        # body's transform has settled (otherwise the cam may be pinned while
        # the body is still at the unscaled spawn pose, producing a bad frame).
        self._pin_at_step = 30

    def _add_fallback_lights(self):
        from pxr import UsdLux
        stage = self.world.stage
        sun = UsdLux.DistantLight.Define(stage, "/World/FallbackSun")
        sun.CreateIntensityAttr(3000.0)
        sun.CreateAngleAttr(0.53)
        UsdGeom.Xformable(sun.GetPrim()).AddRotateXYZOp().Set(Gf.Vec3f(-45.0, 30.0, 0.0))
        dome = UsdLux.DomeLight.Define(stage, "/World/FallbackDome")
        dome.CreateIntensityAttr(500.0)
        carb.log_warn("[city] fallback DistantLight + DomeLight added")

    def _collect_meshes(self, root_prim, results):
        """Recursively collect every UsdGeom.Mesh under root_prim, descending
        into instance prototypes so we cover the CityDemoPack's instanceable
        references (where most of the geometry lives)."""
        if root_prim.IsA(UsdGeom.Mesh):
            results.append(root_prim)
        if root_prim.IsInstance():
            proto = root_prim.GetPrototype()
            if proto and proto.IsValid():
                for child in proto.GetChildren():
                    self._collect_meshes(child, results)
            return
        for child in root_prim.GetChildren():
            self._collect_meshes(child, results)

    def _apply_city_colliders(self):
        """Apply convex-hull collision to every mesh under /World/layout. The
        CityDemoPack ships without physics colliders; without this the drone
        falls straight through the visible road geometry."""
        stage = self.world.stage
        layout = stage.GetPrimAtPath("/World/layout")
        if not layout or not layout.IsValid():
            carb.log_warn("[city] /World/layout missing; no colliders applied")
            return
        meshes = []
        self._collect_meshes(layout, meshes)
        for prim in meshes:
            try:
                UsdPhysics.CollisionAPI.Apply(prim)
                UsdPhysics.MeshCollisionAPI.Apply(prim).CreateApproximationAttr().Set("convexHull")
            except Exception:
                pass
        carb.log_warn(f"[city] applied convexHull collision to {len(meshes)} meshes")

    def _find_named_pose(self, keywords, label):
        """Find the largest mesh whose path matches any keyword; return
        (center_x, center_y, top_z). None if no match."""
        stage = self.world.stage
        layout = stage.GetPrimAtPath("/World/layout")
        if not layout or not layout.IsValid():
            return None
        cache = UsdGeom.BBoxCache(
            0.0,
            includedPurposes=[UsdGeom.Tokens.default_, UsdGeom.Tokens.render],
            useExtentsHint=True,
        )
        best = None
        best_area = 0.0
        for prim in Usd.PrimRange(layout):
            path = str(prim.GetPath()).lower()
            if not any(k in path for k in keywords):
                continue
            try:
                rng = cache.ComputeWorldBound(prim).ComputeAlignedRange()
            except Exception:
                continue
            if rng.IsEmpty():
                continue
            size = rng.GetSize()
            area = float(size[0]) * float(size[1])
            if area > best_area:
                best_area = area
                best = (prim.GetPath(), rng)
        if best is None:
            carb.log_warn(f"[city] no {label} prim found")
            return None
        path, rng = best
        mn, mx = rng.GetMin(), rng.GetMax()
        cx = (float(mn[0]) + float(mx[0])) / 2.0
        cy = (float(mn[1]) + float(mx[1])) / 2.0
        top_z = float(mx[2])
        carb.log_warn(
            f"[city] {label} prim {path}: center=({cx:.2f},{cy:.2f}) "
            f"top_z={top_z:.2f} (area={best_area:.0f})"
        )
        return cx, cy, top_z

    def _find_spawn_pose(self):
        """Prefer grass/park (drone on grass per user request); fall back to
        any 'floor' / 'ground' / 'plaza' prim; finally fall back to streets."""
        pose = self._find_named_pose(
            ["grass", "lawn", "park", "green"], "grass/park"
        )
        if pose is not None:
            return pose
        pose = self._find_named_pose(
            ["plaza", "floor", "ground"], "plaza/ground"
        )
        if pose is not None:
            return pose
        return self._find_named_pose(["street"], "street")

    def _set_layout_transform(self, scale, translate):
        """Reset /World/layout's xform to: scale uniform by `scale`, then
        translate by `translate` (in stage meters). Both ops are reapplied
        each call so subsequent calls can update the translation while keeping
        the scale fixed."""
        stage = self.world.stage
        layout = stage.GetPrimAtPath("/World/layout")
        if not layout or not layout.IsValid():
            return
        xform = UsdGeom.Xformable(layout)
        xform.ClearXformOpOrder()
        xform.AddScaleOp().Set(Gf.Vec3f(float(scale), float(scale), float(scale)))
        tx, ty, tz = translate
        xform.AddTranslateOp().Set(Gf.Vec3d(float(tx), float(ty), float(tz)))
        carb.log_warn(
            f"[city] /World/layout xform: scale={scale} translate=({tx:.1f},{ty:.1f},{tz:.1f})"
        )

    def _add_ground_plane(self, z):
        """Invisible flat physics collider as a backstop in case some city
        meshes still don't have working colliders."""
        stage = self.world.stage
        cube = UsdGeom.Cube.Define(stage, "/World/PhysicsGround")
        cube.CreateSizeAttr(1.0)
        xform = UsdGeom.Xformable(cube.GetPrim())
        xform.ClearXformOpOrder()
        xform.AddTranslateOp().Set(Gf.Vec3d(0.0, 0.0, z - 0.5))
        xform.AddScaleOp().Set(Gf.Vec3f(100000.0, 100000.0, 1.0))
        UsdPhysics.CollisionAPI.Apply(cube.GetPrim())
        UsdGeom.Imageable(cube.GetPrim()).CreateVisibilityAttr().Set("invisible")
        carb.log_warn(f"[city] invisible safety ground plane at z={z:.2f}")

    def _attach_fpv_camera(self):
        """Create the FPV camera as a top-level prim and prepare the offset
        matrix that maps cam-local space into the drone body's frame. The
        actual world pose is set every frame in _update_fpv_camera_pose()."""
        if not is_prim_path_valid(DRONE_BODY):
            carb.log_warn(f"[fpv] {DRONE_BODY} not found; FPV camera skipped")
            return
        stage = self.world.stage
        cam_prim = stage.DefinePrim(FPV_CAMERA_PRIM, "Camera")
        cam = UsdGeom.Camera(cam_prim)

        # True fisheye projection (Omniverse-specific). focal=2.5/aperture=18.5
        # gives ~180 deg FOV under fisheyeEquidistant.
        cam_prim.CreateAttribute(
            "cameraProjectionType", Sdf.ValueTypeNames.Token
        ).Set("fisheyeEquidistant")
        cam.CreateFocalLengthAttr(2.5)
        cam.CreateHorizontalApertureAttr(18.5)
        cam.CreateVerticalApertureAttr(18.5)
        cam.CreateClippingRangeAttr(Gf.Vec2f(0.05, 10000.0))

        # Cam-in-body offset matrix (USD is row-vector: p_body = p_cam * M).
        # First row of M = where cam +X lands in body frame, etc.
        #   cam +X → body -Y  (screen right = drone right)
        #   cam +Y → body +Z  (screen up    = drone up)
        #   cam +Z → body -X  (so cam looks down its -Z = body +X = forward)
        ox, oy, oz = FPV_CAM_OFFSET
        self._cam_in_body = Gf.Matrix4d(
            0.0, -1.0, 0.0, 0.0,
            0.0, 0.0, 1.0, 0.0,
            -1.0, 0.0, 0.0, 0.0,
            ox, oy, oz, 1.0,
        )

        # Single TransformOp that we overwrite every frame.
        xform = UsdGeom.Xformable(cam_prim)
        xform.ClearXformOpOrder()
        self._cam_xform_op = xform.AddTransformOp()
        self._cam_xform_op.Set(self._cam_in_body)
        carb.log_warn(f"[fpv] fisheye camera created at {FPV_CAMERA_PRIM}")

    def _update_fpv_camera_pose(self):
        """Drive the FPV cam's world transform from the drone body's current
        world matrix. Called every frame so the cam tracks physics exactly."""
        if not getattr(self, "_cam_xform_op", None):
            return
        stage = self.world.stage
        body = stage.GetPrimAtPath(DRONE_BODY)
        if not body or not body.IsValid():
            return
        body_world = UsdGeom.Xformable(body).ComputeLocalToWorldTransform(
            Usd.TimeCode.Default()
        )
        cam_world = self._cam_in_body * body_world
        self._cam_xform_op.Set(cam_world)

    def _pin_viewport_to_fpv(self):
        try:
            vp = vp_utils.get_active_viewport()
            if vp is not None:
                vp.camera_path = FPV_CAMERA_PRIM
                carb.log_warn(f"[fpv] viewport pinned to {FPV_CAMERA_PRIM}")
        except Exception as e:
            carb.log_warn(f"[fpv] viewport pin failed: {e}")

    def run(self):
        self.timeline.play()
        step = 0
        while simulation_app.is_running() and not self.stop_sim:
            self.world.step(render=True)
            self._update_fpv_camera_pose()
            step += 1
            if not self._viewport_pinned and step == self._pin_at_step:
                self._pin_viewport_to_fpv()
                self._viewport_pinned = True

        carb.log_warn("PegasusApp Simulation App is closing.")
        self.timeline.stop()
        simulation_app.close()


def main():
    PegasusApp().run()


if __name__ == "__main__":
    main()
