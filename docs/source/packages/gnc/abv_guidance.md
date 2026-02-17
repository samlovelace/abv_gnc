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
- Publishes desired state information to `abv_control`

The guidance layer does not perform state estimation or thruster allocation.

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

''TODO: Add blurb about generic command source''

### Published

- `/abv/command`  
  Target waypoints for the controller.

---
