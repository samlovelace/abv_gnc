# abv_msgs

Custom ROS 2 message and service definitions for the Air Bearing Vehicle (ABV) platform.

---

## Purpose

The `abv_msgs` package defines all custom ROS 2 message and service types used across the ABV software stack.

It serves as the interface layer between:

- Navigation
- Guidance
- Control
- Teleoperation
- Visualization
- External tools

All inter-package communication within the ABV stack relies on these message definitions.

---

## Role in the System

`abv_msgs` sits at the foundation of the architecture.

abv_msgs  
 ├── abv_navigation  
 ├── abv_guidance  
 ├── abv_goal_manager  
 ├── abv_control  
 ├── abv_thrusters  
 ├── abv_policy_registry  
 ├── abv_rl-cpp  
 ├── abv_bridge  
 ├── abv_simulator  
 ├── abv_teleop  
 ├── abv_commander  
 └── abv_gui

This package contains no nodes and no runtime logic.  
It defines shared data structures only.

---

## Dependencies

This package declares a dependency on `std_msgs`. `AbvState` also embeds a `builtin_interfaces/Time` timestamp, which is available through the ROS 2 interface generators.

---

# Message Definitions

## AbvVec3

### Message Definition (msg/AbvVec3.msg)

- `x` - a float representing the x axis
- `y` - a float representing the y axis
- `yaw` - a float representing the yaw axis

### Description

Used to represent the 3 degrees of freedom of the ABV (planar x, y position or velocity, plus yaw). Used as a building block within several other messages, including `AbvState`, `AbvControllerCommand`, and `AbvPath`.

---

## AbvNodeStatus

### Message Definition (msg/AbvNodeStatus.msg)

- `node_name` - the name of the reporting node, e.g. `"goal_manager"`, `"guidance"`
- `node_state` - the current state of that node's internal state machine, as a string

### Description

A small, reusable status block embedded in other status messages (`AbvControllerStatus`, `AbvGuidanceStatus`) so any component can report which state machine state it is currently in without each message duplicating the same two fields.

- Published by: `abv_goal_manager`, `abv_guidance` (as part of their respective status messages)

---

## AbvHeartbeat

### Message Definition (msg/AbvHeartbeat.msg)

- `node_name` - the name of the node emitting the heartbeat

### Description

A lightweight liveness signal, used on two topics:

- **`abv/heartbeat`** — every long-running node constructs a `HeartbeatPublisher` (from `abv_common`) at startup, which publishes this message at `Heartbeat.Rate`. `abv_gui`'s `NodeHealthPanel` marks a node "Disconnected" if no heartbeat arrives within `Heartbeat.StaleAfter` seconds.
  - Published by: `abv_goal_manager` (`goal_manager`), `abv_control` (`control`), `abv_thrusters` (`thrusters`), `abv_policy_registry` (`policy_registry`), `abv_navigation` (`navigation`), `abv_guidance` (`guidance`), `abv_bridge` (`bridge`)
  - Subscribed by: `abv_gui`
- **`abv/policy/status`** — control-policy nodes register themselves with `abv_policy_registry`; every `node_name` seen here is treated as a selectable policy.
  - Published by: `abv_control` (`control`), `abv_rl-cpp` (`rl`)
  - Subscribed by: `abv_policy_registry`

---

## AbvControllerCommand

### Message Definition (msg/AbvControllerCommand.msg)

- `type` - a string representing the type of command (see below)
- `is_global` - if true, `data` is interpreted in the world frame rather than the vehicle body frame (velocity and direction types)
- `data` (`AbvVec3`) - the commanded pose, velocity, or direction/control input; meaning depends on `type`
- `tolerance` (`AbvVec3`) - per-axis arrival tolerance for this command (pose/velocity types only; ≤ 0 on an axis falls back to `Control.Arrival.Tolerance`)
- `thrusters` - an 8-char `'0'`/`'1'` string used only when `type` is `"thruster"` (see `AbvThrusterStatus` for the bit-order convention)

### Description

This message commands `abv_goal_manager`. Supported `type` values:

- `"pose"` — drive to the pose in `data` (world frame).
- `"vel"` / `"velocity"` — track the velocity in `data`.
- `"direction"` — apply `data` directly as a control input (used by `abv_teleop` and the GUI's axis buttons). Must be refreshed at least every 250 ms or the goal manager stops.
- `"thruster"` — fire the `thrusters` pattern directly, bypassing feedback control.
- `"STOP"` (also `"stop"`) — zero the thrusters and clear the goal.
- `"clear"` — clear the goal without commanding the thrusters (sent by `abv_policy_registry` after a policy switch).

`pose` and `vel` only move the vehicle while a control policy is active; `direction`, `thruster`, and `STOP` always reach the thrusters.

- Published by: `abv_commander`, `abv_teleop`, `abv_gui`, `abv_guidance`, `abv_policy_registry`
- Subscribed by: `abv_goal_manager`

---

## AbvControllerStatus

### Message Definition (msg/AbvControllerStatus.msg)

- `status` (`AbvNodeStatus`) - the goal manager's node name (`goal_manager`) and state machine state
- `arrival` - the arrival status: `IDLE` (0), `RUNNING` (1), or `ARRIVED` (2)
- `nav_ok` - mirrors whether navigation data was fresh and valid at publish time

### Description

This message informs other components of the goal-tracking state inside `abv_goal_manager`. `abv_guidance` uses the arrival state to determine when to send the next waypoint. (The applied force/torque formerly carried here now lives in `AbvThrusterStatus`.)

- Published by: `abv_goal_manager`
- Subscribed by: `abv_guidance`, `abv_gui`, `abv_bridge`

---

## AbvControlContext

### Message Definition (msg/AbvControlContext.msg)

- `pose` (`AbvVec3`) - current estimated pose
- `velocity` (`AbvVec3`) - current estimated velocity (body frame for body-frame velocity goals)
- `goal` (`AbvVec3`) - current goal pose or velocity
- `error` (`AbvVec3`) - goal minus current, yaw wrapped to [-π, π]
- `goal_type` - `POSE` (0) or `VELOCITY` (1)

### Description

Published by `abv_goal_manager` once per state-machine tick while in `POSE_CONTROL` / `VELOCITY_CONTROL`, so any control-policy node can compute a control input without the goal manager knowing which or how many policies are listening.

- Published by: `abv_goal_manager` (`abv/controller/context`)
- Subscribed by: `abv_control`, `abv_rl-cpp`

---

## AbvThrusterCommand

### Message Definition (msg/AbvThrusterCommand.msg)

- `type` - `"direction"` or `"thruster"`
- `is_global` - true if `data` is in the world frame; `abv_thrusters` converts it to the body frame
- `data` (`AbvVec3`) - control effort (fx, fy, tz), for type `"direction"`
- `thrusters` - 8-char `'0'`/`'1'` string, for type `"thruster"`
- `policy_name` - the control policy that computed this command (`"control"`, `"rl"`), or empty for `abv_goal_manager`'s direct thruster/direction/stop writes

### Description

The input to `abv_thrusters`. Commands with a non-empty `policy_name` are dropped unless it matches `abv/policy/active`; commands with an empty `policy_name` always pass.

- Published by: `abv_goal_manager`, `abv_control`, `abv_rl-cpp` (`abv/thrusters/command`)
- Subscribed by: `abv_thrusters`

---

## AbvThrusterStatus

### Message Definition (msg/AbvThrusterStatus.msg)

- `thrusters` - an 8-char `'0'`/`'1'` string; index `i` corresponds to thruster `(i+1)`, matching the `Thrusters.Allocation` column order in `config.yaml`
- `fx`, `fy`, `tz` - the applied force in x, y and torque about z resulting from that firing pattern

### Description

Reports which of the 8 thrusters are currently firing and the resulting applied force/torque. `abv_navigation` uses the force/torque as the EKF control input; `abv_gui` lights up the corresponding thruster glyphs and plots the control input.

- Published by: `abv_thrusters` (`abv/thrusters/status`)
- Subscribed by: `abv_navigation`, `abv_gui`

---

## AbvGuidanceCommand

### Message Definition (msg/AbvGuidanceCommand.msg)

- `type` - the type of trajectory to execute: `"line"`, `"file"`, or `"avoid"` (capitalized forms also accepted)
- `duration` - the duration to execute the trajectory for, in seconds (or `-1` to run until arrival)
- `goal_state` (`AbvState`) - the goal state at the end of the trajectory (used by `line` and `avoid`)
- `arrival_tolerance` (`AbvState`) - per-axis tolerance used to determine arrival (≤ 0 falls back to the configured default)

### Description

Provides a way of commanding the `abv_guidance` node to coordinate sending a sequence of waypoints to `abv_goal_manager`. See [`abv_guidance`](../gnc/abv_guidance.md#guidance-modes).

- Published by: `abv_gui` (pose goals and paths), `abv_commander` (`path` command), `abv_bridge` (external waypoints)
- Subscribed by: `abv_guidance`

---

## AbvGuidanceStatus

### Message Definition (msg/AbvGuidanceStatus.msg)

- `status` (`AbvNodeStatus`) - the guidance node's name and state machine state

### Description

Reports the current state of the guidance state machine (e.g. idle, executing a trajectory) to observers such as `abv_gui`.

- Published by: `abv_guidance`
- Subscribed by: `abv_gui`

---

## AbvPath

### Message Definition (msg/AbvPath.msg)

- `waypoints` (`AbvVec3[]`) - the remaining waypoints of the active path

### Description

The upcoming portion of the path `abv_guidance` is executing, for visualization. Empty when guidance is idle. For `file` paths, the length is limited by the `# PreviewLength` header in `path.csv`.

- Published by: `abv_guidance` (`abv/guidance/path`)
- Subscribed by: `abv_gui`

---

## AbvObstacle / AbvObstacleArray

### Message Definition (msg/AbvObstacle.msg, msg/AbvObstacleArray.msg)

- `AbvObstacle`: `x`, `y` - center in the world frame (m); `radius` (m)
- `AbvObstacleArray`: `obstacles` (`AbvObstacle[]`)

### Description

The full set of obstacles on the table. Each message replaces the previous set entirely (there are no per-obstacle IDs); an empty array clears all obstacles. Used by `abv_guidance`'s `avoid` mode.

- Published by: `abv_gui` (`abv/scene/obstacles`)
- Subscribed by: `abv_guidance`

---

## AbvPolicyActive

### Message Definition (msg/AbvPolicyActive.msg)

- `policy_name` - the active control policy, or empty if none has been activated

### Description

The single source of truth for which control policy may drive `abv/thrusters/command`. Re-published periodically.

- Published by: `abv_policy_registry` (`abv/policy/active`)
- Subscribed by: `abv_control`, `abv_rl-cpp`, `abv_thrusters`, `abv_gui`

---

## AbvPolicyRegistry / AbvPolicyEntry

### Message Definition (msg/AbvPolicyRegistry.msg, msg/AbvPolicyEntry.msg)

- `AbvPolicyRegistry`: `policies` (`AbvPolicyEntry[]`)
- `AbvPolicyEntry`: `policy_name`, `alive`

### Description

Every policy the registry has ever seen, with its current liveness. Re-published periodically.

- Published by: `abv_policy_registry` (`abv/policy/registry`)
- Subscribed by: `abv_gui` (policy selector)

---

## AbvResponse

### Message Definition (msg/AbvResponse.msg)

- `ack` - true once the receiving node has accepted/acknowledged the request
- `complete` - true once the requested action has finished executing

### Description

A minimal two-field acknowledgement message used where a component needs to distinguish "I received this" (`ack`) from "I finished this" (`complete`), rather than a single boolean. Not currently used by any node.

---

## AbvState

### Message Definition (msg/AbvState.msg)

- `timestamp` (`builtin_interfaces/Time`) - time of the last real measurement/update (not publish time)
- `valid` - true if this is fresh EKF output from a real measurement; false if free-running dead-reckoning has exceeded the configured `MaxDeadReckonDuration` bound
- `position` (`AbvVec3`) - the x, y, yaw position of the ABV
- `velocity` (`AbvVec3`) - the x, y, yaw velocity of the ABV

### Description

Used to represent the estimated position and velocity of the ABV in its 3 degrees of freedom, along with whether that estimate is currently trustworthy. This is the primary output of `abv_navigation` and the data type embedded in `AbvGuidanceCommand`'s goal/tolerance fields.

- Published by: `abv_navigation` (`abv/state`), `abv_simulator` (`abv/sim/state`), `abv_bridge` (`abv/sim/gazebo_state`)
- Subscribed by: `abv_guidance`, `abv_goal_manager`, `abv_thrusters`, `abv_gui`, `abv_bridge`, `abv_navigation` (sim), `abv_simulator` (external propagation)

---

# Service Definitions

## AbvPolicySwitch

### Service Definition (srv/AbvPolicySwitch.srv)

Request:

- `policy_name` - the policy to activate (e.g. `"control"`, `"rl"`)

Response:

- `success` - whether the switch happened (also true if it was already active)
- `message` - human-readable result

### Description

Asks `abv_policy_registry` to switch the active control policy. Rejected if the policy isn't registered or has gone stale. On success, the registry publishes the new `abv/policy/active` and clears the current goal.

- Called by: `abv_gui` (`PolicySelectorPanel`), or `ros2 service call /abv/policy/switch abv_msgs/srv/AbvPolicySwitch "{policy_name: control}"`
- Served by: `abv_policy_registry` (`abv/policy/switch`)

---

## Notes

- All angular quantities are assumed to be in radians unless otherwise specified.
- All positions are expressed in the planar world frame unless otherwise specified.
- Message definitions should remain backward compatible whenever possible.
