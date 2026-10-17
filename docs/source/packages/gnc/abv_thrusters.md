# abv_thrusters

Thruster allocation, policy arbitration, and hardware drivers for the Air Bearing Vehicle (ABV) platform.

---

## Purpose

The `abv_thrusters` package is the only package that drives the thrusters. It converts a continuous control input into a discrete 8-thruster firing pattern and sends it to the hardware (or the simulator), and it decides which control-policy commands are allowed through.

---

## Role in the GNC Pipeline

`abv_thrusters` is the last stage of the pipeline.

It:

- Receives `AbvThrusterCommand`s from `abv_goal_manager`, `abv_control`, and `abv_rl-cpp`
- **Drops** any command whose `policy_name` is set but doesn't match `abv/policy/active`. Commands with an empty `policy_name` (`abv_goal_manager`'s direct thruster/direction/stop writes) always pass. Until a policy is activated, every policy-sourced command is dropped
- Rotates world-frame (`is_global`) inputs into the body frame using the latest `abv/state` yaw
- Discretizes and allocates the input to thrusters, and sends the pattern to the driver
- Publishes the firing pattern and resulting applied force/torque on `abv/thrusters/status`

---

## Allocation

1. **Discretization** — each axis of the control input is mapped to -1/0/+1 with a Schmitt trigger using `Thrusters.InputDiscretization.On/Off`.
2. **Mapping** — selected with `Thrusters.AllocationStrategy`:
   - `LookupTable` (default) — fixed lookup from the thrust-direction vector to a firing pattern
   - `Matrix` — pseudo-inverse of the `Thrusters.Allocation` matrix, firing thrusters whose solution exceeds `Thrusters.Allocation.Threshold`

`Thrusters.Force` and `Thrusters.MomentArm` are used to compute the applied force/torque reported on `abv/thrusters/status`.

---

## Drivers

The driver is chosen at **compile time** from the target architecture:

- **ARM (Jetson)** — `GpioThrusterDriver`, using JETGPIO on the pins in `Thrusters.ThrusterDriver.GPIO.pins`. Requires root, which is why `jetson.launch.py` runs this node under `sudo`.
- **x86** — `UdpThrusterDriver`, sending the firing pattern over UDP to `127.0.0.1:6969` for [`abv_simulator`](../tools/abv_simulator.md).

---

## Architecture

- `ThrusterCommandHandler`: Topic wiring, active-policy check, world→body conversion
- `ThrusterCommander`: Discretization, mapper selection, driver ownership
- `IThrusterMapper` → `LookupTableThrusterMapper`, `MatrixThrusterMapper`
- `IThrusterDriver` → `GpioThrusterDriver`, `UdpThrusterDriver`

---

## Topics

### Subscribed

- `/abv/thrusters/command` — [`AbvThrusterCommand`](../interface/abv_msgs.md#abvthrustercommand)
- `/abv/policy/active` — from `abv_policy_registry`
- `/abv/state` — yaw for frame conversion

### Published

- `/abv/thrusters/status` — [`AbvThrusterStatus`](../interface/abv_msgs.md#abvthrusterstatus), consumed by `abv_navigation` (EKF control input) and `abv_gui`
- `/abv/heartbeat` — liveness, as `thrusters`
