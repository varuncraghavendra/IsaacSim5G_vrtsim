# Isaac helpers

Scripts in this directory live in two categories:

## Run inside Isaac Sim (Script Editor)

* **`fix_and_start_camera.py`** -- the main one. Captures the left + right
  `front_hawk` cameras with Isaac's bundled Python (3.11), pipes the
  bytes via TCP sockets to a helper subprocess running system Python
  3.10 + rclpy, and that subprocess publishes the real ROS 2 topics:

      /Nova_Carter_ROS/front_hawk/left/image_raw   (+ /compressed)
      /Nova_Carter_ROS/front_hawk/right/image_raw  (+ /compressed)

  These are the topics the dashboard's two camera panels subscribe to.
  See the comment block at the top of the file for usage.

* **`enable_camera_graph.py`** -- diagnostic helper. Enumerates every
  OmniGraph in the current stage, prints whether each is enabled, and
  force-enables `/World/Nova_Carter_ROS/front_hawk`. Use this only if
  you want to try to get the *native* Isaac OmniGraph publishers working;
  in practice we use `fix_and_start_camera.py` instead because the
  OmniGraph path has been unreliable.

## Run on the host (with ROS 2 sourced)

* **`nova_carter_bridge.py`** -- OAI-mode only. Relays
  `/network/cmd_vel` (broker output) to `/cmd_vel` (Isaac input) at
  30 Hz with a 500 ms watchdog. Not needed for direct mode.

* **`dynamic_obstacle_bridge.py`** -- watches a JSON file of obstacles
  and synthesises `NetworkMetrics` for the broker so LOS/NLOS changes
  show up in the simulated link quality.
