# abv_guidance

Trajectory generation and goal management for the Air Bearing Vehicle (ABV) platform.

---

## Purpose

The `abv_guidance` package is responsible for generating desired vehicle motion commands. It produces target poses or trajectories that are consumed by the control stack.

Guidance defines _where the vehicle should go_, but does not compute low-level actuation commands.

---

## Role in the GNC Pipeline

`abv_guidance` sits between navigation and the goal manager.

It:

- Receives the current estimated vehicle state
- Receives the obstacle set on `abv/scene/obstacles`
- Generates a desired pose or trajectory
- Sends waypoints one at a time to `abv_goal_manager`, advancing when `abv/controller/status` reports `ARRIVED`
- Publishes the upcoming path for visualization

The guidance layer does not perform state estimation or thruster allocation.

---

## Guidance Modes

Selected by the `type` field of [`AbvGuidanceCommand`](../interface/abv_msgs.md#abvguidancecommand) (lowercase or capitalized):

### 1. `file` — File-Based Trajectory

- Reads waypoints from `abv_bringup`'s installed `config/path.csv`
- Outputs sequential target poses, with optional per-waypoint timeout and tolerance overrides
- Useful for repeatable experiments

See [Running the ABV Software](../../getting_started/running_abv.md#path) for the file format, including the `# PreviewLength: N` header.

### 2. `line` — Straight-Line Generator

- Sends the commanded goal (`goal_state.position`) as a single waypoint
- Used by `abv_gui` for click-to-go goals when no obstacles are placed

### 3. `avoid` — Collision-Avoiding Path

- Plans an A* path on a grid of `CollisionAvoid.GridResolution` meters per cell (8-connected) from the vehicle's live pose to the goal, keeping the robot footprint inside the `Scene` bounds and clear of every obstacle
- Re-plans in the background at `CollisionAvoid.ReplanRate` Hz, so the path reacts to obstacles added or removed while it runs. If a cycle finds no path, the previous plan is kept
- Sends the next path node as each waypoint, and finishes once the vehicle is within one grid cell of the goal
- **Heading:** a single trajectory heading is chosen once, when the command is received — `0` if the goal is ahead of the start in +x (bearing within ±90°), otherwise `π` — and every intermediate waypoint holds that heading. Only the final waypoint uses the commanded yaw
- Fails to start (and returns to `IDLE`) if the goal is outside the scene bounds, blocked, or unreachable
- The robot footprint is checked as an axis-aligned square of side `max(TableView.RobotWidth, TableView.RobotLength)`, regardless of heading
- Used by `abv_gui` for click-to-go goals when obstacles are placed

### Timeouts

- **Path duration** — if the command's `duration` is not `-1`, the whole path is stopped (with a `STOP`) when it elapses.
- **Per-waypoint timeout** — `Guidance.StateMachine.WaypointTimeout` (or a per-waypoint override). When it fires, the waypoint is accepted if the error is within its tolerance × `Guidance.StateMachine.WaypointTimeoutToleranceScale`; otherwise the path is aborted with a `STOP`.

---

## Architecture

Core components include:

- `StateMachine`  
  Manages guidance modes and transitions (`STARTUP` → `IDLE` → `GENERATE_PATH` → `SEND_WAYPOINT` → `WAITING_FOR_EXECUTION` → `WAITING_FOR_ARRIVAL`). Waits in `STARTUP` until navigation data arrives.

- `RosCommandSource`  
  Receives `abv/guidance/command` and hands it to the state machine.

- `IPathGenerator` implementations:
  - `FromFileGenerator` — loads and serves waypoint sequences from `path.csv`
  - `StraightLineGenerator` — the single-waypoint `line` mode
  - `CollisionAvoidPathGenerator` — the A* `avoid` mode

- `Scene`  
  Holds the fixed table bounds (`Scene.XMin/XMax/YMin/YMax`) and the live obstacle set from `abv/scene/obstacles`, and answers bounds/collision queries using `AxisAlignedBoundingBox`.

- `PathWatchdog`  
  Cancellable timer used for the path-duration and per-waypoint timeouts.

## Topics

### Subscribed

- `/abv/state`  
  Estimated state from `abv_navigation`.

- `/abv/guidance/command`  
  Commands the guidance state machine to begin executing a trajectory — see [`AbvGuidanceCommand`](../interface/abv_msgs.md#abvguidancecommand). Published by `abv_gui` (pose goals as `line`/`avoid`, and `File` paths), `abv_commander` (`path` command), and `abv_bridge` (external autonomy waypoints).

- `/abv/controller/status`  
  Arrival status from `abv_goal_manager`, used to determine when to advance to the next waypoint.

- `/abv/scene/obstacles`  
  Obstacle set (full replace on every message), published by `abv_gui`.

### Published

- `/abv/controller/command`  
  Target waypoints (`pose`) for `abv_goal_manager`, and `STOP` on timeout/abort.

- `/abv/guidance/status`  
  Current state-machine state.

- `/abv/guidance/path`  
  The remaining waypoints of the active path (empty when idle), drawn by `abv_gui`.

- `/abv/heartbeat`  
  Liveness, as `guidance`.
