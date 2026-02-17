# abv_navigation

State estimation and vehicle state tracking for the Air Bearing Vehicle platform.

## Purpose

The `abv_navigation` package estimates the 3-DOF pose (x, y, yaw) and velocity of the vehicle
using sensor measurements (OptiTrack, simulation, etc.) and publishes the vehicle state
for downstream guidance and control modules.

## Role in the GNC Pipeline

`abv_navigation` sits at the front of the GNC pipeline.

It:

- Consumes raw state measurements (e.g., OptiTrack or simulation data)
- Filters and processes measurements
- Publishes the estimated vehicle state

The estimated state is consumed by:

- `abv_guidance`
- `abv_control`

This package does not depend on guidance or control logic.

## Published Topics

- `/abv/state` — Estimated pose and velocity

## Dependencies

- `abv_msgs`
- Eigen
- libmotioncapture
