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

## AbvControllerCommand

### Message Definition (msg/AbvControllerCommand.msg)

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

- `goal_state` - the goal state at the end of a trajectory
- `type` - the type of trajectory to execute
- `duration` the duration to execute the trajectory for

### Description

Provides a way for commanding the `abv_guidance` node to coordinate sending a sequence of waypoints to the `abv_controller`.

---

## AbvResponse

### Message Definition (msg/AbvResponse.msg)

<insert message fields here>

### Description

Describe how acknowledgements or responses are represented.

---

## AbvState

### Message Definition (msg/AbvState.msg)

- `position` - the x, y, yaw position of the abv
- `velocity` - the x, y, yaw velocity of the abv

### Description

## Used to represent the position and velocity of the abv in its 3 degrees of freedom. Used as the data type for publishing navigation data.

## AbvVec3

### Message Definition (msg/AbvVec3.msg)

- `x` - a float representing the x axis
- `y` - a float representing the y axis
- `yaw` - a float representing the yaw axis

### Description

Used to represent the 3 degrees of freedom of the abv. Used as the data type in the `AbvState` msg to represent the position and velocity of the abv.

## Notes

- All angular quantities are assumed to be in radians unless otherwise specified.
- All positions are expressed in the planar world frame unless otherwise specified.
- Message definitions should remain backward compatible whenever possible.
