# Design

## robots.yaml

Nested ROS 2 parameter YAML, grouping each robot's fields (`aruco_id`, `x`, `y`)
under its own name key. Loaded via
`automatically_declare_parameters_from_overrides=True` +
`get_parameters_by_prefix('robots')`.

```yaml
robots:
  robot_1:
    aruco_id: 1
    x: 0.0
    y: 0.0
```

## Topics

<!-- table of topic name, type, publisher, subscriber(s) -->

## Behavior switching

<!-- go-to-goal <-> obstacle-avoidance state machine details -->