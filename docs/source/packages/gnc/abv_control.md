# abv_control

Classical feedback control laws for the Air Bearing Vehicle (ABV) platform.

---

## Purpose

The `abv_control` package hosts the on-board classical control law (PID today). It is one of the selectable control policies — registered as `control` — alongside the RL policy in `abv_rl-cpp`. Only the policy currently activated by [`abv_policy_registry`](abv_policy_registry.md) computes and publishes control input.

---

## Role in the GNC Pipeline

`abv_control` sits between [`abv_goal_manager`](abv_goal_manager.md) and [`abv_thrusters`](abv_thrusters.md).

It:

- Receives `AbvControlContext` (current state, goal, error) from `abv_goal_manager`
- Ignores it unless `abv/policy/active` names `control`
- Computes a control input with the configured `IControlLaw`
- Publishes it as a world-frame `direction` command on `abv/thrusters/command`, tagged `policy_name: control`

When the policy becomes active again after being inactive, the control law's internal state (integrator, previous error/time) is reset so the first tick back doesn't see a huge `dt`.

---

## Control Laws

Selected with `ControlLaw.Law` in `config.yaml`:

- `PID` — per-axis PID on the pose error, gains from `ControlLaw.Pid.Kp/Ki/Kd`
- `LQR`, `MPC` — reserved; not implemented yet, falls back to PID with a warning

Any other value also falls back to PID.

---

## Architecture

- `IControlLaw`: Interface for any control law hosted by this node (`computeAction()`, `reset()`)
- `PidControlLaw`: PID implementation
- `ControlLawHandler`: Wires topics, tracks whether `control` is the active policy, and runs the law on each context message

---

## Topics

### Subscribed

- `/abv/controller/context` — from `abv_goal_manager`
- `/abv/policy/active` — from `abv_policy_registry`

### Published

- `/abv/thrusters/command` — control input for `abv_thrusters`
- `/abv/heartbeat` — liveness, as `control`
- `/abv/policy/status` — policy registration heartbeat, as `control`
