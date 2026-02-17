# abv_control

Feedback control and thruster command computation for the Air Bearing Vehicle (ABV) platform.

---

## Purpose

The `abv_control` package is responsible for converting desired vehicle behavior into low-level thruster commands. It implements the control logic, state machine handling, and hardware interface required to actuate the vehicle.

The package supports both closed-loop waypoint control and direct thruster control modes.

---

## Role in the GNC Pipeline

`abv_control` sits downstream of navigation and guidance.

It:

- Receives the estimated vehicle state from `abv_navigation`
- Receives pose or velocity waypoints from `abv_guidance`
- Computes control efforts
- Converts control efforts into discrete thruster commands
- Sends commands to the hardware interface

This package does not perform state estimation or trajectory generation.

## Control Modes

The control stack typically supports two operating modes:

### 1. Waypoint (Pose Control) Mode

- Accepts a desired pose (x, y, yaw) or velocity (vx, vy, w)
- Computes state error
- Applies feedback control
- Generates thruster commands internally

### 2. Direct (Thruster Control) Mode

- Accepts externally computed control inputs
- Converts control inputs to thruster firing sequences
- Does not perform internal feedback control

Mode selection is handled in the command topic. ''TODO: add link to abv_msgs/AbvCommand''

---

## Architecture

Core components include:

- `Controller`  
  Computes control effort from state error.

- `StateMachine`  
  Manages operating modes and control flow.

- `ThrusterCommander`  
  Converts control efforts into discrete thruster activation sequences.

- `Vehicle`  
  Encapsulates control logic.

- `GpioThrusterDriver`  
  Sends commands directly to hardware via GPIO.

- `UdpThrusterDriver`  
  Sends thruster commands via UDP (only used for simulation).

---

## Topics

### Subscribed

- `/abv/state`  
  Estimated vehicle state from `abv_navigation`.

- `/abv/command`  
  Commands from `abv_guidance` or `abv_teleop`.

### Published

- `/abv/controller_status`  
  Arrival status and theoretical thrust vector.

---
