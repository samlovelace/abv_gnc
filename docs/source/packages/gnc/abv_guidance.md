# abv_guidance

Trajectory generation and goal management for the Air Bearing Vehicle (ABV) platform.

---

## Purpose

The `abv_guidance` package is responsible for generating desired vehicle motion commands. It produces target poses or trajectories that are consumed by the control stack.

Guidance defines _where the vehicle should go_, but does not compute low-level actuation commands.

---

## Role in the GNC Pipeline

`abv_guidance` sits between navigation and control.

It:

- Receives the current estimated vehicle state
- Generates a desired pose or trajectory
- Publishes desired state information to `abv_controller`

The guidance layer does not perform state estimation or thruster allocation.

---

## Guidance Modes

The package typically supports multiple goal generation strategies.

### 1. File-Based Trajectory

- Reads waypoints from a file (e.g., CSV or YAML)
- Outputs sequential target poses
- Useful for repeatable experiments

### 2. Straight-Line Generator

- Generates a simple linear trajectory between two poses
- Useful for basic motion validation

Mode selection is handled by the guidance state machine.

---

## Architecture

Core components include:

- `StateMachine`  
  Manages guidance modes and transitions.

- `FromFileGenerator`  
  Loads and serves waypoint sequences from file.

- `StraightLineGenerator`  
  Generates simple interpolated trajectories.

## Topics

### Subscribed

- `/abv/state`  
  Estimated state from `abv_navigation`.

- `/abv/guidance/command`  
  Commands the guidance state machine to begin executing a trajectory — see [`AbvGuidanceCommand`](../interface/abv_msgs.md#abvguidancecommand). Published by `abv_commander` (`path` command) or another external command source.

- `/abv/controller/status`  
  Arrival status from `abv_controller`, used to determine when to advance to the next waypoint.

### Published

- `/abv/controller/command`  
  Target waypoints for the controller.
