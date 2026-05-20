# Validation checklist

- Build workspace successfully with ROS 2 Humble
- Launch `network_aware_stack.launch.py`
- Verify `/network/teleop_command`, `/network/metrics`, `/cmd_vel`, `/network/clock_state`, `/network/robot_telemetry`
- Send UDP teleop JSON to port `16000`
- Send UDP metrics JSON to port `16001`
- Confirm `/cmd_vel` is gated until `/network/step_complete`
- Enable deterministic CSV logging and replay the CSV through `integration/replay_csv.py`
