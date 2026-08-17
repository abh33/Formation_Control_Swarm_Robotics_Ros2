# Architecture

## Node overview

| Node | Responsibility |
|---|---|
| `simulation_manager` | Connect to CoppeliaSim, load scene, spawn/remove robots, start/monitor sim, publish sim state |
| `robot_driver` (xN) | Connect to its robot, control wheels from `cmd_vel`, publish raw sensor values |
| `robot_controller` (xN) | Go-to-goal + obstacle-avoidance behavior switching |
| `camera_vision_sensor_node` | Read CoppeliaSim vision sensor, detect robots, publish localization |
| `goal_allocation_node` | Allocate goals per robot from a target shape |

## Data flow

<!-- diagram or short description: CoppeliaSim <- ZeroMQ <- {sim_manager, robot_driver xN} <- topics <- controller layer -->

## Design principles

- Keep nodes above `robot_driver` / `simulation_manager` platform-agnostic where possible
- Launch files stay thin — orchestration only, no logic
- Avoidance behavior is a discrete state machine, not continuous reactive blending

## Open questions

<!-- e.g. how to reconcile per-node "connect to CoppeliaSim" calls with the platform-agnostic goal -->