# abv_goal_manager

Command intake, goal tracking, and arrival detection for the Air Bearing Vehicle (ABV) platform.

---

## Purpose

The `abv_goal_manager` package is the single entry point for vehicle-level commands. It receives commands on `abv/controller/command`, runs the goal state machine, computes the state error against the current goal, and reports arrival status back to guidance.

It does **not** compute control effort itself. For pose and velocity goals it publishes an `AbvControlContext` (current state, goal, and error) that whichever control policy is active — [`abv_control`](abv_control.md) or `abv_rl-cpp` — turns into a control input. Direct thruster, direction, and stop commands bypass the control policies and are written straight to [`abv_thrusters`](abv_thrusters.md).

---

## Role in the GNC Pipeline

`abv_goal_manager` sits between guidance/operator tools and the control policies.

It:

- Receives commands from `abv_guidance`, `abv_gui`, `abv_commander`, `abv_teleop`, and `abv_policy_registry`
- Receives the estimated vehicle state from `abv_navigation`
- Publishes `AbvControlContext` on `abv/controller/context` every tick while in pose or velocity control
- Publishes direct thruster/direction commands on `abv/thrusters/command` (with an empty `policy_name`, so they are never gated by the active policy)
- Publishes its state-machine state, arrival status, and navigation health on `abv/controller/status`

Pose and velocity goals only move the vehicle once a control policy has been activated through [`abv_policy_registry`](abv_policy_registry.md). Thruster and direction commands work regardless.

---

## Command Types

Selected by the `type` field of [`AbvControllerCommand`](../interface/abv_msgs.md#abvcontrollercommand):

| `type` | State entered | Behavior |
|---|---|---|
| `thruster` | `THRUSTER_CONTROL` | Fires the 8-character `thrusters` pattern directly |
| `direction` | `DIRECTION_CONTROL` | Applies `data` as a raw control input (body or world frame per `is_global`). Reverts to `IDLE` and zeroes thrusters if no new input arrives within 250 ms |
| `pose` | `POSE_CONTROL` | Drives to the `data` pose (x, y, yaw) using `tolerance`, falling back to `Control.Arrival.Tolerance` per axis when ≤ 0 |
| `vel` / `velocity` | `VELOCITY_CONTROL` | Tracks the `data` velocity in world or body frame |
| `STOP` | `IDLE` | Publishes a zero thruster command and clears the goal |
| `CLEAR` | `IDLE` | Clears the goal **without** publishing a thruster command (sent by `abv_policy_registry` after a policy switch) |

Type strings are matched case-insensitively for the common spellings (`pose`/`Pose`, `STOP`/`stop`/`Stop`, …).

`pose`, `vel`, and global-frame `direction` commands are rejected if navigation data is not fresh and valid (see `Control.NavDataTimeout`).

---

## State Machine

States: `STARTUP`, `IDLE`, `THRUSTER_CONTROL`, `DIRECTION_CONTROL`, `POSE_CONTROL`, `VELOCITY_CONTROL`, `NAV_FAULT`.

If navigation data goes stale or invalid while in `POSE_CONTROL` or `VELOCITY_CONTROL` (or a global-frame `DIRECTION_CONTROL`), the state machine zeroes the thrusters and enters `NAV_FAULT`. Once navigation recovers, it automatically resumes the pose/velocity goal it was executing; a fault during `DIRECTION_CONTROL` resolves back to `IDLE`.

Arrival is reported as `ARRIVED` once every axis of the error has stayed within tolerance for `Control.Arrival.Duration` seconds.

---

## Architecture

- `CommandHandler`: Subscribes to `abv/controller/command` and transitions the `StateMachine`
- `StateMachine`: Main run loop at `Control.StateMachine.Rate`; also publishes `abv/controller/status` at the same rate
- `GoalManager`: Holds the active goal, computes errors, publishes `AbvControlContext` and direct `AbvThrusterCommand`s, and determines arrival status

---

## Topics

### Subscribed

- `/abv/controller/command` — commands, see above
- `/abv/state` — estimated vehicle state from `abv_navigation` (via `abv_common`'s `RosNavigationListener`)

### Published

- `/abv/controller/context` — [`AbvControlContext`](../interface/abv_msgs.md#abvcontrolcontext) for the active control policy
- `/abv/thrusters/command` — direct thruster/direction/stop commands
- `/abv/controller/status` — [`AbvControllerStatus`](../interface/abv_msgs.md#abvcontrollerstatus): state, arrival, `nav_ok`
- `/abv/heartbeat` — liveness, as `goal_manager`
