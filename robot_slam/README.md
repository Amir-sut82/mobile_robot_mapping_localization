# robot_slam

This package implements the custom C++ 2D SLAM pipeline used by the workspace. It contains:

- odometry-based differential-drive motion prediction;
- LiDAR scan-to-map matching with finite-difference gradient ascent;
- Bresenham ray tracing and a clamped log-odds occupancy grid;
- periodic PGM/YAML map export.

See the workspace [README](../README.md) for the complete installation, launch commands, topics, formulas, frame conventions, planner/localization interaction, parameters, and limitations.

Quick start:

```bash
ros2 launch robot_slam slam.launch.py
ros2 run teleop_twist_keyboard teleop_twist_keyboard
```

The launch publishes `/map` and `/slam/pose`, consumes `/ekf/odom` and `/scan`, and saves `slam_map.pgm`/`slam_map.yaml` under `~/maps` by default.
