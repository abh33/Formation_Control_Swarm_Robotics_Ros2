# Roadmap

## Staged plan

- **V1** — centralized, faithful recreation of the 2017 original
  (in progress — see status below)
- **V2** — same centralized approach, modularized into separate
  planner / allocator / collision-checker / controller nodes
- **V3** — decentralized: robots exchange state with neighbors, no
  central coordinator

## V1 node status

| Node | Status | Notes |
|---|---|---|
| `simulation_manager` | Done | Connects, loads/validates scene, spawns robots, publishes `/simulation_state` |
| `robot_driver` | Done | Gates on sim state, `cmd_vel` → wheel velocities, proximity sensor publishing with `detected` flag |
| `camera_vision_sensor_node` | Done | ArUco detection, homography-based pose publishing, working calibration pipeline |
| `goal_allocation_node` | Done | SELECT_N and SHAPE_DRAW modes both working end-to-end; Hungarian matching; publishes `/robot_goal_list` |
| `robot_controller` | In progress | Pose/goal subscription skeleton written; go-to-goal P-control and collision avoidance not yet implemented |

## Immediate next steps

1. **`robot_controller`: go-to-goal P-control.** Subscribe to own pose
   (`/robot_pose_list`) and own goal (`/robot_goal_list`), compute
   heading error (normalized to `[-π, π]`) and distance to goal, publish
   proportional `cmd_vel`. Guard against acting before both a pose
   (`detected == True`) and a goal (explicit "goal received" flag, since
   a default-initialized `RobotGoal()` at (0,0) is indistinguishable from
   a real goal at the origin otherwise) are available. Add a stopping
   distance threshold to avoid chasing noise-level position error.
2. **`robot_controller`: collision avoidance.** Decide discrete
   state-machine vs. blended-vector response (see `docs/design.md`);
   decide where the pairwise trigger-detection logic lives (see
   `docs/decisions.md` — leaning toward pulling a lightweight
   `collision_checker`-style node forward into V1).
3. Once `robot_controller` is working end-to-end with static goals,
   revisit single-shot vs. continuous goal rematching (see
   `docs/decisions.md`) now that live avoidance motion exists to actually
   observe the tradeoff against.
4. Move on to V2: split into separate planner / allocator /
   collision-checker / controller nodes.

## Known accepted limitations (not being chased further right now)

- ~1cm world-position error and ~2-4° heading error after calibration —
  attributed to ArUco detection noise and homography not modeling lens
  distortion; well within tolerance for current work.
- Two `robot_config*.yaml` variants beyond the main swarm config and the
  17-robot calibration rig exist in `swarm_bringup/config/` and could be
  confused for one another; worth renaming more descriptively at some
  point but not urgent.
