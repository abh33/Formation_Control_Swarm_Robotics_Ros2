# Roadmap

## V1 — Centralized, faithful recreation

- [x] Environment setup, CoppeliaSim connection
- [ ] Robot spawning (`simulation_manager`)
- [ ] `robot_driver`: handle caching, `cmd_vel` sub, sensor publishing
- [ ] `camera_vision_sensor_node`: localization
- [ ] `robot_controller`: go-to-goal + avoidance
- [ ] `goal_allocation_node`
- [ ] Multi-robot scaling
- [ ] Shape formation
- [ ] Visualization
- [ ] Documentation pass

## V2 — Modularized centralized

<!-- planner/allocator/collision-checker/controller split -->

## V3 — Decentralized

<!-- robots exchange state with neighbors, no master -->