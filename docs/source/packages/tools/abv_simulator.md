# abv_simulator

A numerical integration module for simulating the full state of the ABV.

## Purpose

The `abv_simulator` package is responsible for simulating how the ABV would move based on the thrusters that were fired. This package is only needed for running/testing the GNC stack without running on the actual ABV.

## Usage

Listens for thruster firing patterns over UDP on port `6969`, sent by `abv_thrusters`' `UdpThrusterDriver` (used automatically on x86 builds). Computes the applied force/torque from the fired thrusters (using the same `Thrusters.AllocationStrategy` as `abv_thrusters`) and propagates the vehicle state. Additionally, has the ability to add noise to simulate a noisy sensor module.

The vehicle spawns inset from the table's bottom-right corner (the world origin) by half the robot footprint (`TableView.RobotLength/RobotWidth`), so the whole body starts on the table.

### How the simulated state is delivered

Chosen by `Navigation.Interface`:

- `optitrack` — the simulator runs an `OptitrackEmulator` VRPN server streaming the configured `Navigation.RigidBodyName`, so `abv_navigation` runs exactly as it would against the real motion-capture system.
- anything else (e.g. `simulated`) — the state is published on `/abv/sim/state` for `abv_navigation`'s simulated interface.

### Propagation mode

Chosen by `Control.Dynamics.PropagationMode`:

- `internal` — the simulator integrates the dynamics itself.
- `external` — an external physics simulator (via `abv_bridge`) owns the dynamics. The simulator publishes the applied force/torque as a `geometry_msgs/WrenchStamped` on `Control.Dynamics.WrenchCommandTopic` (default `mujoco/wrench_cmd`, `frame_id` = `Control.Dynamics.WrenchTargetLink`) and reads the resulting state back from `Control.Dynamics.ExternalStateTopic` (default `abv/sim/gazebo_state`).

### Navigation dropout testing

Setting `Navigation.SimulateDropout: true` injects random bursts of missing navigation data, for exercising `abv_goal_manager`'s `NAV_FAULT` handling and the EKF's dead-reckoning bound.

## Topics

### Subscribed

- `Control.Dynamics.ExternalStateTopic` (default `/abv/sim/gazebo_state`) — external propagation only

### Published

- `/abv/sim/state` — the simulated state of the ABV (non-`optitrack` interface only)
- `Control.Dynamics.WrenchCommandTopic` (default `/mujoco/wrench_cmd`) — external propagation only
