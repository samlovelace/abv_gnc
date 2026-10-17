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
- `abv_goal_manager`
- `abv_thrusters` (yaw, for world→body frame conversion)
- `abv_gui` and `abv_bridge`

This package does not depend on guidance or control logic.

---

## Architecture

Core classes within this package include:

- `VehicleStateTracker`: Owns the state-tracking loop. Selects and initializes the configured `IStateFetcher`, feeds its raw measurements into the `ExtendedKalmanFilter`, and hands the filtered result to `RosStatePublisher`.
- `IStateFetcher`: Interface for acquiring raw state measurements, so the tracker isn't tied to any one sensing source. Implementations: `OptitrackStateFetcher_LibMocap` (real hardware, via `libmotioncapture`) and `SimulatedStateFetcher` (reads `abv/sim/state`). Selected by `Navigation.Interface`: `optitrack` or `simulated`. When running `abv_simulator` with `optitrack`, the simulator emulates the OptiTrack VRPN server, so the OptiTrack fetcher is exercised in simulation too.
- `ExtendedKalmanFilter`: Fuses raw measurements over time to produce a smoothed pose/velocity estimate, and free-runs (dead-reckons) between measurements up to `Navigation.MaxDeadReckonDuration` before the output is marked invalid. Also accepts the latest applied force/torque from `abv_thrusters`' `AbvThrusterStatus` as an EKF control input.
- `RosStatePublisher`: Converts the internal `AbvState` representation to the `abv_msgs::msg::AbvState` IDL type and publishes it.

## Topics

### Subscribed

- `/abv/sim/state`  
  Simulated vehicle state, consumed instead of real sensor data when `Navigation.Interface: simulated` (via `SimulatedStateFetcher`).

- `/abv/thrusters/status`  
  Applied force/torque (`fx`, `fy`, `tz`) from `abv_thrusters`, used as the EKF's control input.

### Published

- `/abv/state`  
  Estimated pose and velocity.

## Dependencies

- `abv_msgs`
- Eigen
- libmotioncapture
