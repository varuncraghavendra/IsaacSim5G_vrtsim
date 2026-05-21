# Time-aligned robot state for plotting and post-hoc analysis.
#
# Published by isaac/plotter_publisher.py at ~30 Hz on topic /plotter.
# Computed from /chassis/odom (pose + twist) and /clock (sim_time -> rtf).
#
# heading is yaw in radians, derived from the odom quaternion via atan2.
# rtf is a rolling-window slope of d(sim_time)/d(wall_time); 1.0 means
# Isaac is running real-time, <1.0 means it's slower than wall-clock.

builtin_interfaces/Time sim_time
float64 rtf
float64 x
float64 y
float64 z
float64 vx
float64 vy
float64 vz
float64 heading
