# Architecture

## Tiers

The system is split into two tiers, and this split is the single most
important architectural boundary in the project:

**Driver tier** — the only nodes allowed to touch the CoppeliaSim API
directly. Everything here is simulator-specific and is meant to be
swappable (for Gazebo, or real hardware) without touching anything above
it.

**Logic tier** — platform-agnostic. Communicates only via standard ROS
topics/messages. Has no knowledge of CoppeliaSim, object handles, or
simulation-specific concepts.

This boundary exists so that switching to Gazebo or real hardware later
only requires swapping the driver-tier nodes; the logic tier should not
need to change at all.

## Package layout

Package structure mirrors the tier boundary — one package per *tier*, not
one package per node:

- **`swarm_interfaces`** — all custom message definitions (`RobotPose`,
  `RobotPoseArray`, `RobotGoal`, `RobotGoalArray`, `ProximitySensor`)
- **`swarm_coppelia_drivers`** — driver tier: `simulation_manager`,
  `robot_driver`, `camera_vision_sensor_node`
- **`swarm_control`** — logic tier: `goal_allocation_node`,
  `robot_controller`
- **`swarm_bringup`** — launch files and config (robot YAML configs) only;
  contains no node logic of its own

## Nodes

### Driver tier

**`simulation_manager`**
Connects to CoppeliaSim, loads the scene, validates it, spawns robots
(same model, each with a unique name + ArUco marker), starts the
simulation, monitors and publishes `/simulation_state`. Owns the shared
robot-name ↔ ArUco-ID mapping (loaded from the `robots` YAML config block
and exposed to other nodes via ROS parameters).

**`robot_driver`** (×N, one per robot)
Connects to CoppeliaSim, waits for `/simulation_state == RUNNING`, finds
its own robot via a `robot_name` launch parameter, subscribes to `cmd_vel`,
converts it to wheel velocities via differential-drive kinematics, and
publishes proximity sensor readings (with a `detected` flag, so occlusion
isn't mistaken for "no obstacle"). Pure hardware-abstraction layer — no
concept of goals or poses.

**`camera_vision_sensor_node`**
Connects to CoppeliaSim, pulls the overhead vision sensor feed, detects
ArUco markers, computes each robot's pose (world-frame x/y via homography,
plus heading), and publishes `RobotPoseArray` with a per-robot `detected`
flag for markers not seen in a given frame. Also owns the homography
calibration routine (see `docs/design.md`).

### Logic tier

**`goal_allocation_node`**
Interactive goal-allocation UI: displays the live camera feed in an
OpenCV window, lets the user select goal points either by double-clicking
N points (SELECT_N mode) or by drawing a curve that gets resampled into N
evenly-spaced points (SHAPE_DRAW mode). Matches robots to goals via
Hungarian assignment (minimizing total travel distance) and publishes the
result on `/robot_goal_list`.

**`robot_controller`** (×N, one per robot)
Go-to-goal control + collision avoidance. Subscribes to its own entry in
`/robot_pose_list` and `/robot_goal_list`, publishes `cmd_vel` for
`robot_driver` to consume. Currently in progress — see `docs/roadmap.md`.

## Cross-cutting rules

- No CoppeliaSim object handles are ever passed between nodes. Each
  driver-tier node independently connects to CoppeliaSim and looks up its
  own objects by name. All cross-tier and cross-node communication is
  standard ROS messages.
- Driver-tier nodes gate their behavior on `/simulation_state == RUNNING`
  before acting. Logic-tier nodes generally do not re-gate on this — the
  driver tier is the single place that decision needs to be enforced.
- Positions are published in **world coordinates (meters)**, not pixels —
  see `docs/decisions.md` for why.
