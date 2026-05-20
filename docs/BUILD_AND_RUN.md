# Build and run

## 1. ROS 2 workspace

```bash
cd ros2_ws
source /opt/ros/humble/setup.bash
rosdep install --from-paths src --ignore-src -r -y
colcon build --symlink-install
source install/setup.bash
```

## 2. Teleop GUI

The teleop app now subscribes to ROS 2 feedback topics in-process. Source your
ROS 2 install in the same shell before running it.

```bash
cd teleop_app
python3 -m venv .venv
source .venv/bin/activate
pip install -r requirements.txt
source /opt/ros/humble/setup.bash   # so rclpy is importable
python3 app.py
```

Open: `http://127.0.0.1:8080`

Optional environment overrides:

```bash
export ODOM_TOPIC=/odom
export CAMERA_TOPIC=/front_stereo_camera/left/image_raw
export CAMERA_FPS_LIMIT=15
```

## 3. Launch broker stack

```bash
cd ros2_ws
source /opt/ros/humble/setup.bash
source install/setup.bash
ros2 launch network_aware_broker network_aware_stack.launch.py
```

## 4. OAI RFSim bridge (forward only)

```bash
python3 integration/oai_udp_bridge.py \
  --listen-port 17000 \
  --ros-port 16000
```

### 4b. OAI RFSim bridge (forward + reverse, so feedback also rides OAI)

```bash
python3 integration/oai_udp_bridge.py \
  --listen-port 17000 --ros-port 16000 \
  --enable-reverse \
  --rev-listen-port 17500 --rev-out-port 18500
```

## 5. Isaac bridge (passthrough -- kinematics are owned by Isaac)

```bash
python3 isaac/nova_carter_bridge.py \
  --robot-prim /World/Nova_Carter \
  --cmd-in-topic /cmd_vel \
  --cmd-out-topic /cmd_vel \
  --step-rate-hz 50
```

There is no longer a `--fallback-kinematic` flag. Motion is computed by Isaac
Sim's physics engine; the bridge is a pure command relay plus a
`/network/step_complete` heartbeat. Pose comes back on `/odom` from Isaac, and
the front camera comes back on `/front_stereo_camera/left/image_raw` (or its
`/compressed` companion).

## 6. Dynamic obstacle bridge (optional)

```bash
python3 isaac/dynamic_obstacle_bridge.py \
  --obstacles-file config/obstacles.example.json \
  --gnb-x 0.0 --gnb-y 0.0 \
  --metrics-port 16001
```

## 7. Example OAI commands

gNB:
```bash
sudo ./nr-softmodem -O conf/gnb_v.conf --rfsim --rfsimulator.serveraddr server
```

UE:
```bash
sudo ./nr-uesoftmodem -O conf/ue.conf --rfsim
```

## 8. Smoke tests

Teleop ingress:
```bash
ros2 topic echo /network/teleop_command
```

Adaptive output:
```bash
ros2 topic echo /cmd_vel
```

Pose feedback (now from Isaac, not from the Python integrator):
```bash
ros2 topic echo /odom
```

Camera frame rate:
```bash
ros2 topic hz /front_stereo_camera/left/image_raw
```

Clock state:
```bash
ros2 topic echo /network/clock_state
```

Telemetry:
```bash
ros2 topic echo /network/robot_telemetry
```

## 9. Browser-side validation

With `app.py` running, open `http://127.0.0.1:8080`:

- the **camera viewport** (right panel) should display frames as soon as Isaac
  starts publishing on the camera topic
- the **x / y / yaw / speed** tiles should update at roughly the publish rate
  of `/odom`
- the **cam age** and **pose age** indicators should stay under ~200 ms when
  the feedback path is healthy
