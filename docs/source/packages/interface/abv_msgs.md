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
 ├── abv_controller  
 ├── abv_bridge  
 ├── abv_teleop  
 ├── abv_commander  
 └── abv_gui

This package contains no nodes and no runtime logic.  
It defines shared data structures only.

---

## Dependencies

This package depends on:

- std_msgs
- geometry_msgs
- sensor_msgs
- builtin_interfaces

These dependencies are used within custom message definitions (e.g. `AbvState` embeds a `builtin_interfaces/Time` timestamp).

---

# Message Definitions

## AbvVec3

### Message Definition (msg/AbvVec3.msg)

- `x` - a float representing the x axis
- `y` - a float representing the y axis
- `yaw` - a float representing the yaw axis

### Description

Used to represent the 3 degrees of freedom of the ABV (planar x, y position or velocity, plus yaw). Used as a building block within several other messages, including `AbvState` and `AbvControllerCommand`.

---

## AbvNodeStatus

### Message Definition (msg/AbvNodeStatus.msg)

- `node_name` - the name of the reporting node, e.g. `"controller"`, `"guidance"`
- `node_state` - the current state of that node's internal state machine, as a string

### Description

A small, reusable status block embedded in other status messages (`AbvControllerStatus`, `AbvGuidanceStatus`) so any component can report which state machine state it is currently in without each message duplicating the same two fields.

- Published by: `abv_controller`, `abv_guidance` (as part of their respective status messages)

---

## AbvHeartbeat

### Message Definition (msg/AbvHeartbeat.msg)

- `node_name` - the name of the node emitting the heartbeat

### Description

A lightweight liveness signal. Every long-running node constructs a `HeartbeatPublisher` (from `abv_common`) at startup, which publishes this message on the shared `abv/heartbeat` topic at the rate configured in `Heartbeat.Rate`. `abv_gui`'s `NodeHealthPanel` subscribes to this topic and marks a node "Disconnected" if no heartbeat arrives within `Heartbeat.StaleAfter` seconds.

- Published by: `abv_controller`, `abv_navigation`, `abv_guidance`, `abv_bridge`
- Subscribed by: `abv_gui`

---

## AbvControllerCommand

### Message Definition (msg/AbvControllerCommand.msg)

- `type` - a string representing the type of command, i.e. `"thruster"`, `"pose"`, `"velocity"`
- `is_global` - if true, `data`/`tolerance` are interpreted in the world frame rather than the vehicle body frame
- `data` (`AbvVec3`) - the commanded pose or velocity, or a thruster direction vector; meaning depends on `type`
- `tolerance` (`AbvVec3`) - per-axis arrival tolerance for this command (pose/velocity types only)
- `thrusters` - an 8-char `'0'`/`'1'` string used only when `type` is `"thruster"` (see `AbvThrusterStatus` for the bit-order convention)

### Description

This message is used to command the `abv_controller` module to perform some form of control, whether that be pose or velocity control, or a manual thrust vector.

- `type` is `"pose"` or `"velocity"` — `data` represents the desired value for the x, y, and yaw axes.
- `type` is `"thruster"` — `thrusters` represents which of the 8 thrusters to fire directly, bypassing feedback control.

- Published by: `abv_commander`, `abv_teleop`, `abv_gui`, `abv_guidance`
- Subscribed by: `abv_controller`

---

## AbvControllerStatus

### Message Definition (msg/AbvControllerStatus.msg)

- `status` (`AbvNodeStatus`) - the controller's node name and state machine state
- `fx`, `fy`, `tz` - the theoretical applied force in x, y and the applied torque about z at the current time
- `arrival` - the arrival status of the feedback controller: `IDLE` (0), `RUNNING` (1), or `ARRIVED` (2)
- `nav_ok` - mirrors whether navigation data was fresh (not stale) at publish time

### Description

This message is used to inform other components of the feedback control state occurring within `abv_controller`. The `abv_guidance` module needs to know the arrival state of the control system to determine when to send the next goal. The `abv_navigation` module needs the theoretical applied force/torque to aid in the Kalman Filter state estimation.

- Published by: `abv_controller`
- Subscribed by: `abv_guidance`, `abv_navigation`, `abv_gui`, `abv_bridge`

---

## AbvGuidanceCommand

### Message Definition (msg/AbvGuidanceCommand.msg)

- `type` - the type of trajectory to execute
- `duration` - the duration to execute the trajectory for, in seconds (or `-1` to run until arrival)
- `goal_state` (`AbvState`) - the goal state at the end of the trajectory
- `arrival_tolerance` (`AbvState`) - per-axis position/velocity tolerance used to determine arrival

### Description

Provides a way of commanding the `abv_guidance` node to coordinate sending a sequence of waypoints to `abv_controller`.

- Published by: `abv_commander` (`path` command)
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

## AbvThrusterStatus

### Message Definition (msg/AbvThrusterStatus.msg)

- `thrusters` - an 8-char `'0'`/`'1'` string; index `i` corresponds to thruster `(i+1)`, matching the `Control.Thrusters.Allocation` column order in `config.yaml`

### Description

Reports which of the 8 thrusters are currently firing, primarily so `abv_gui`'s table-top view can light up the corresponding thruster glyphs on the vehicle render.

- Published by: `abv_controller`
- Subscribed by: `abv_gui`

---

## AbvResponse

### Message Definition (msg/AbvResponse.msg)

- `ack` - true once the receiving node has accepted/acknowledged the request
- `complete` - true once the requested action has finished executing

### Description

A minimal two-field acknowledgement message used where a component needs to distinguish "I received this" (`ack`) from "I finished this" (`complete`), rather than a single boolean.

---

## AbvState

### Message Definition (msg/AbvState.msg)

- `timestamp` (`builtin_interfaces/Time`) - time of the last real measurement/update (not publish time)
- `valid` - true if this is fresh EKF output from a real measurement; false if free-running dead-reckoning has exceeded the configured `MaxDeadReckonDuration` bound
- `position` (`AbvVec3`) - the x, y, yaw position of the ABV
- `velocity` (`AbvVec3`) - the x, y, yaw velocity of the ABV

### Description

Used to represent the estimated position and velocity of the ABV in its 3 degrees of freedom, along with whether that estimate is currently trustworthy. This is the primary output of `abv_navigation` and the data type embedded in `AbvGuidanceCommand`'s goal/tolerance fields.

- Published by: `abv_navigation` (`abv/state`), `abv_simulator` (`abv/sim/state`)
- Subscribed by: `abv_guidance`, `abv_controller`, `abv_gui`, `abv_bridge`

---

# Service Definitions

## AbvControlAction

### Service Definition (srv/AbvControlAction.srv)

Request:

- `pose` (`AbvVec3`) - current estimated pose
- `vel` (`AbvVec3`) - current estimated velocity
- `goal` (`AbvVec3`) - current goal pose or velocity
- `error` (`AbvVec3`) - current state error (goal minus current)
- `goal_type` - whether the goal is a pose or velocity target

Response:

- `action` (`AbvVec3`) - the control action (e.g. thrust direction) to apply
- `is_global` - whether `action` is expressed in the world frame or the vehicle body frame

### Description

Called by `abv_controller`'s `ExternalControlPolicy` when `Control.ControlPolicy` is set to `External` in `config.yaml`. Instead of computing control effort internally (as the built-in `PidControlPolicy` does), the controller sends the current state error over this service and applies whatever action the external service returns — this is the hook used to swap in an externally-trained policy (e.g. `abv_rl` / `abv_rl-cpp`) without changing `abv_controller` itself.

- Called by: `abv_controller`
- Served by: an external control policy node (e.g. `abv_rl`/`abv_rl-cpp`), only when running in `External` control mode

---

## Notes

- All angular quantities are assumed to be in radians unless otherwise specified.
- All positions are expressed in the planar world frame unless otherwise specified.
- Message definitions should remain backward compatible whenever possible.
