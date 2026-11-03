# Architecture

This document outlines the architecture of the `autobot` stack and accompanying tools. For the physical platform these packages run on, see [System Overview](system_overview.md); for the message-level view of what publishes/subscribes to what, see [Data Flow](data_flow.md).

## Package layout

The stack is organized into four groups of ROS 2 packages:

- **GNC pipeline** — the core sense/decide/act loop that runs on board the vehicle: `autobot_navigation`, `autobot_guidance`, `autobot_goal_manager`, `autobot_control` / `autobot_rl-cpp` (control policies), `autobot_policy_registry`, and `autobot_thrusters`.
- **Interfaces** — the shared vocabulary every other package builds on: `autobot_msgs` (message/service definitions) and `autobot_bridge` (translates between `autobot_msgs` and external systems).
- **Operator tools** — ground-station software for commanding and observing the vehicle: `autobot_gui`, `autobot_commander`, `autobot_teleop`.
- **System/infrastructure** — supporting packages with no GNC logic of their own: `autobot_bringup` (launch/config), `autobot_common` (shared C++ library), `autobot_description` (URDF/visualization).

See [Packages](../packages/index.md) for a per-package summary table and links to each package's own documentation page.

## System diagram

The GNC stack runs on board the vehicle. It accepts commands from two kinds of sources — a human operator on the ground-station laptop, or an external autonomy module talking through `autobot_bridge` — and the pipeline has no notion of which kind of source a command came from, since both publish the same `autobot_msgs` command topics.

![Architecture diagram: the on-board GNC pipeline (autobot_bridge, autobot_guidance, autobot_goal_manager, the autobot_control / autobot_rl-cpp control policies selected by autobot_policy_registry, autobot_thrusters, autobot_navigation) accepting commands from either a human operator laptop or an external autonomy module, and driving the thruster hardware](/_static/architecture.png)

Source: [`docs/diagrams/architecture.puml`](https://github.com/samlovelace/autobot/blob/main/docs/diagrams/architecture.puml) (rendered with PlantUML — see `docs/build.sh`).

## The GNC pipeline

The on-board packages form a pipeline, each stage consuming the previous stage's output over a ROS 2 topic:

- **`autobot_navigation`** estimates the vehicle's pose and velocity with an Extended Kalman Filter, fed by either OptiTrack (real hardware) or `autobot_simulator` (sim), and publishes the result as `abv/state`.
- **`autobot_guidance`** turns a guidance command into a sequence of pose targets — a single straight-line goal, a file of waypoints, or a collision-avoiding A\* path around obstacles on `abv/scene/obstacles` — forwarding one at a time as `abv/controller/command` and waiting for `abv/controller/status` to report arrival before advancing.
- **`autobot_goal_manager`** owns the active goal: it runs the goal state machine, computes the state error, and publishes it as `abv/controller/context` for the control policies. Direct thruster/direction/stop commands skip the policies and go straight to `abv/thrusters/command`.
- **Control policies** — **`autobot_control`** (PID, registered as `control`) and **`autobot_rl-cpp`** (ONNX RL policy, registered as `rl`) both listen to `abv/controller/context`, but only the one named on `abv/policy/active` publishes a control input on `abv/thrusters/command`.
- **`autobot_policy_registry`** discovers the policy nodes via `abv/policy/status`, and switches the active one at runtime through the `abv/policy/switch` service. It starts with **no** active policy, so an operator must pick one before pose/velocity goals have any effect.
- **`autobot_thrusters`** drops commands from inactive policies, rotates world-frame inputs into the body frame, maps the input onto discrete thruster firings, and drives the GPIO hardware (or `autobot_simulator` over UDP). It reports the firing pattern and applied force/torque on `abv/thrusters/status`, which `autobot_navigation` uses as the EKF control input.

Every stage also participates in a heartbeat/status layer independent of this pipeline: each node publishes `abv/heartbeat` for liveness, and `autobot_goal_manager` / `autobot_guidance` publish a status message (`AbvControllerStatus`, `AbvGuidanceStatus`) carrying their current state-machine state, all consumed by `autobot_gui`'s health/status panels.

## Commanding the pipeline

Two kinds of sources can command the vehicle, and the pipeline treats them identically:

- **A human operator**, from the ground-station laptop, via [`autobot_gui`](../packages/tools/autobot_gui.md) (table-top click-to-command or the command panel), [`autobot_commander`](../packages/tools/autobot_commander.md) (CLI), or [`autobot_teleop`](../packages/tools/autobot_teleop.md) (keyboard/controller) — which publish onto `abv/guidance/command` (GUI pose goals and paths) and `abv/controller/command` (velocity, thruster, direction, and stop commands). The GUI also publishes obstacles on `abv/scene/obstacles` and selects the active control policy.
- **An external autonomy module**, via [`autobot_bridge`](../packages/interface/autobot_bridge.md), which converts its `ptera_msgs` waypoint/state messages to and from the same `autobot_msgs` topics, so the autonomy module never needs to know about `autobot_msgs` directly.

All inter-package messages used by either path are defined once, in `autobot_msgs`, so every package — pipeline stage, operator tool, or bridge — shares the same vocabulary. See [Data Flow](data_flow.md) for the full topic-by-topic breakdown, and [Operation](../operation/index.md) for how a human operator drives the GUI in practice.

## System/infrastructure

`autobot_bringup` holds the launch files (`jetson.launch.py`, `gnc.launch.py`) and the single `config.yaml` that every node reads through `autobot_common`'s `ConfigurationManager` — this is the one place tuning values (control-law gains under `ControlLaw`, thruster allocation under `Thrusters`, scene bounds, network addresses) live. `autobot_common` is a shared C++ library, not a node: it provides the `RosTopicManager` publish/subscribe wrapper, `ConfigurationManager`, `HeartbeatPublisher`, `DataLogger`, and other infrastructure that every other C++ package links against, so those concerns are implemented once. `autobot_description` provides the URDF and RViz configuration used to visualize the vehicle, independent of the running GNC stack.
