# Architecture

## End-to-end path

### Command (downlink, operator -> robot)
```
Browser UI
  -> WebSocket /ws (FastAPI teleop_app)
  -> UDP 16000 (UE ingress)
  -> [OAI uplink, optional]
  -> ROS 2 ue_ingress_node
  -> Time Broker (admission + shaping)
  -> /cmd_vel
  -> nova_carter_bridge (passthrough, NO integration)
  -> Isaac Sim ROS 2 Twist subscriber graph
  -> Nova Carter articulation (physics)
```

### Feedback (uplink, robot -> operator)
```
Isaac Sim (physics step)
  -> ROS 2 Odometry publisher graph  -> /odom
  -> ROS 2 Camera publisher graph    -> /front_stereo_camera/left/image_raw(+/compressed)
  -> [OAI downlink, optional via integration/oai_udp_bridge.py --enable-reverse]
  -> teleop_app feedback subscriber (rclpy, in-process)
  -> WebSocket /ws
  -> Browser UI (pose readout + <img> camera viewport)
```

## Main ROS 2 topics

- `/network/teleop_command` — normalized teleop command ingress
- `/network/metrics` — network metrics ingress from OAI / RFSim / obstacle model
- `/cmd_vel` — adaptive broker output to robot (consumed by Isaac directly)
- `/odom` — robot odometry **from Isaac Sim** (no longer synthesized in Python)
- `/front_stereo_camera/left/image_raw[/compressed]` — front camera (from Isaac)
- `/network/step_complete` — Isaac/bridge step heartbeat
- `/network/clock_state` — broker timing state
- `/network/robot_telemetry` — fused telemetry for dashboards

## Why kinematics moved fully into Isaac Sim

The previous bridge integrated `linear_x` and `angular_z` in Python when
`--fallback-kinematic` was enabled (which was the default). That made the
ROS 2 stack appear functional even when Isaac wasn't actually moving the
robot, because the bridge faked the odometry. The new design:

- **Single source of truth for motion**: Isaac's physics engine. Whatever
  position / velocity Isaac produces is what the operator sees.
- **Pure passthrough bridge**: easier to reason about, no hidden state,
  no parallel kinematic model that can drift from sim ground truth.
- **Real wheel slip, real collisions, real dynamics**: free side effect of
  letting the physics engine own the integration.

## Borrowed ideas and their implementation

### ROS-NetSim-inspired
- Transparent ROS-facing integration: robot application still sees normal ROS topics
- Synchronism updates: broker state topic summarizes time / queue / delay state
- Packet interception boundary: implemented as UDP bridge scripts

### DANCERS-inspired
- Decoupled time stepping: `physics_step_ms`, `network_step_ms`, `publish_rate_hz`
- Explicit coordination barrier: command release may wait for `/network/step_complete`
- Measurable co-simulation overhead: deterministic CSV log replay

## Improvements over DANCERS for this use case

- OAI RFSim aware timing instead of ns-3 / Wi-Fi-only assumptions
- RTF-aware adaptive scheduling
- Hot-reload obstacle masks instead of init-only static obstacle scene
- Deterministic event logging + replay for reproducibility
- Isaac Sim / Nova Carter path instead of Gazebo/PX4-only integration
- **Operator-in-the-loop video feedback over the simulated 5G downlink**
  (browser sees what the robot's camera sees, end-to-end latency includes
  the modelled radio link)
