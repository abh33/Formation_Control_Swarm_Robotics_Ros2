# Decisions

Log of key design decisions and the reasoning behind them.

## robots.yaml: nested vs flat parallel arrays

Chose nested (grouped by robot name) over flattened parallel arrays to avoid
manual parsing and length-mismatch risk. Loaded natively via ROS 2 parameter
overrides.

## Avoidance trigger: reactive vs centrally computed

<!-- open question — currently leaning reactive per-robot sensor values -->

## Localization source for robot_controller

Pose comes from `camera_vision_sensor_node`, not from `robot_driver`'s own
odometry — matches the original's "robots don't know their own position,
the camera tells them" design.