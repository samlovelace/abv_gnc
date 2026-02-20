# abv_msgs

Custom ROS 2 message definitions for the Air Bearing Vehicle (ABV) platform.

---

## Purpose

The `abv_msgs` package defines all custom ROS 2 message types used across the ABV software stack.

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
 ├── abv_control  
 ├── abv_teleop  
 └── abv_gui

This package contains no nodes and no runtime logic.  
It defines shared data structures only.

---

## Dependencies

This package depends on:

- std_msgs
- geometry_msgs
- sensor_msgs

These dependencies are used within custom message definitions.

---

# Message Definitions

Below is a template for each message type.  
Fill in the fields and descriptions as needed.

---

## AbvCommand

### Message Definition (msg/AbvCommand.msg)

- `type` - a string representing the type of command, i.e "thruster", "pose", "velocity"
- `data` - a vector of 3 values, the meaning of which depends on the value of `type` (see Description below)

### Description

This message is used to command the `abv_control` module to perform some form of control, whether that be pose or velocity control, or a manual thrust vector.

- `type` is "pose" or "velocity - `data` represents the desired cartesian value for the x, y, and yaw axes.
- `type` is "thruster" - `data` represents the desired thrust direction vector in the body-frame

- Published by: `abv_commander`
- Subscribed by: `abv_controller`

---

## AbvControllerStatus

### Message Definition (msg/AbvControllerStatus.msg)

- `fx` - the theoretical applied force in the x direction at the current time
- `fy` - the theoretical applied force in the y direction at the current time
- `fz` - the theoretical applied force in the z direction at the current time

- `arrival` - the arrival status of the feedback controller

### Description

This message is used to inform other components of the feedback control state ocurring within `abv_controller`. The `abv_guidance` module needs to know the arrival state of the control system to determine when to send the next goal. The `abv_navigation` module needs the theoretical applied force to aid in the Kalman Filter state estimation.

- Published by: `abv_controller`
- Subscribed by: `abv_guidance` & `abv_navigation`

---

## AbvGuidanceCommand

### Message Definition (msg/AbvGuidanceCommand.msg)

<insert message fields here>

### Description

Describe the structure of guidance commands:

- Desired pose?
- Velocity?
- Waypoint index?
- Mode selection?

---

## AbvResponse

### Message Definition (msg/AbvResponse.msg)

<insert message fields here>

### Description

Describe how acknowledgements or responses are represented.

---

## AbvState

### Message Definition (msg/AbvState.msg)

<insert message fields here>

### Description

Describe the estimated vehicle state:

- Position?
- Velocity?
- Orientation?
- Frame convention?
- Units?

---

## AbvVec3

### Message Definition (msg/AbvVec3.msg)

<insert message fields here>

### Description

Explain the purpose of this vector type:

- Why not use geometry_msgs/Vector3?
- Is it planar-specific?

---

## Euler

### Message Definition (msg/Euler.msg)

<insert message fields here>

### Description

Describe the Euler angle convention:

- Rotation order (e.g., ZYX)?
- Units (radians)?
- Frame reference?

---

## Quaternion

### Message Definition (msg/Quaternion.msg)

<insert message fields here>

### Description

Describe quaternion representation:

- Field ordering (x, y, z, w)?
- Normalization requirement?
- Frame definition?

---

## RobotState

### Message Definition (msg/RobotState.msg)

<insert message fields here>

### Description

Explain how this differs from AbvState.

---

## Vec3

### Message Definition (msg/Vec3.msg)

<insert message fields here>

### Description

Explain how this differs from AbvVec3 or geometry_msgs/Vector3.

---

## VehicleWaypoint

### Message Definition (msg/VehicleWaypoint.msg)

<insert message fields here>

### Description

Describe the waypoint structure:

- Position?
- Orientation?
- Tolerances?
- Timeout?
- Identifier?

---

## Notes

- All angular quantities are assumed to be in radians unless otherwise specified.
- All positions are expressed in the planar world frame unless otherwise specified.
- Message definitions should remain backward compatible whenever possible.
