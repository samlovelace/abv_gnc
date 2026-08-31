# Architecture

This document outlines the architecture of the `abv_gnc` stack and accompanying tools. For the physical platform these packages run on, see [System Overview](system_overview.md); for the message-level view of what publishes/subscribes to what, see [Data Flow](data_flow.md).

## Package layout

The stack is organized into four groups of ROS 2 packages:

- **GNC pipeline** — the core sense/decide/act loop that runs on board the vehicle: `abv_navigation`, `abv_guidance`, `abv_controller`.
- **Interfaces** — the shared vocabulary every other package builds on: `abv_msgs` (message/service definitions) and `abv_bridge` (translates between `abv_msgs` and external systems).
- **Operator tools** — ground-station software for commanding and observing the vehicle: `abv_gui`, `abv_commander`, `abv_teleop`.
- **System/infrastructure** — supporting packages with no GNC logic of their own: `abv_bringup` (launch/config), `abv_common` (shared C++ library), `abv_description` (URDF/visualization).

See [Packages](../packages/index.md) for a per-package summary table and links to each package's own documentation page.

## System diagram

The GNC stack runs on board the vehicle. It accepts commands from two kinds of sources — a human operator on the ground-station laptop, or an external autonomy module talking through `abv_bridge` — and the pipeline has no notion of which kind of source a command came from, since both publish the same `abv_msgs` command topics.

![Architecture diagram: the on-board GNC pipeline (abv_bridge, abv_guidance, abv_navigation, abv_controller) accepting commands from either a human operator laptop or an external autonomy module, and driving thruster hardware or an optional external control policy](/_static/architecture.png)

Source: [`docs/diagrams/architecture.puml`](https://github.com/samlovelace/abv_gnc/blob/main/docs/diagrams/architecture.puml) (rendered with PlantUML — see `docs/build.sh`).

## The GNC pipeline

The three on-board packages form a straight-line pipeline, each stage consuming the previous stage's output over a ROS 2 topic:

- **`abv_navigation`** estimates the vehicle's pose and velocity with an Extended Kalman Filter, fed by either OptiTrack (real hardware) or `abv_simulator` (sim), and publishes the result as `abv/state`.
- **`abv_guidance`** turns a single command or a file of waypoints into a sequence of pose/velocity targets, forwarding one at a time to the controller as `abv/controller/command`, and waiting for `abv/controller/status` to report arrival before advancing.
- **`abv_controller`** closes the feedback loop: it computes control effort from the state error (via a `PidControlPolicy`, or an `ExternalControlPolicy` that delegates to an external service — see `AbvControlAction` in [`abv_msgs`](../packages/interface/abv_msgs.md#abvcontrolaction)) and maps that effort onto discrete thruster firings.

Every stage also participates in a heartbeat/status layer independent of this pipeline: each node publishes `abv/heartbeat` for liveness and a status message (`AbvControllerStatus`, `AbvGuidanceStatus`) carrying its current state-machine state, both consumed by `abv_gui`'s health/status panels.

## Commanding the pipeline

Two kinds of sources can command the vehicle, and the pipeline treats them identically:

- **A human operator**, from the ground-station laptop, via [`abv_gui`](../packages/tools/abv_gui.md) (table-top click-to-command or the command panel), [`abv_commander`](../packages/tools/abv_commander.md) (CLI), or [`abv_teleop`](../packages/tools/abv_teleop.md) (keyboard/controller) — all three publish directly onto `abv/controller/command` / `abv/guidance/command`.
- **An external autonomy module**, via [`abv_bridge`](../packages/interface/abv_bridge.md), which converts its `ptera_msgs` waypoint/state messages to and from the same `abv_msgs` topics, so the autonomy module never needs to know about `abv_msgs` directly.

All inter-package messages used by either path are defined once, in `abv_msgs`, so every package — pipeline stage, operator tool, or bridge — shares the same vocabulary. See [Data Flow](data_flow.md) for the full topic-by-topic breakdown, and [Operation](../operation/index.md) for how a human operator drives the GUI in practice.

## System/infrastructure

`abv_bringup` holds the launch files (`jetson.launch.py`, `gnc.launch.py`, `jetson-rl.launch.py`) and the single `config.yaml` that every node reads through `abv_common`'s `ConfigurationManager` — this is the one place tuning values (control gains, thruster allocation, network addresses) live. `abv_common` is a shared C++ library, not a node: it provides the `RosTopicManager` publish/subscribe wrapper, `ConfigurationManager`, `HeartbeatPublisher`, `DataLogger`, and other infrastructure that every other C++ package links against, so those concerns are implemented once. `abv_description` provides the URDF and RViz configuration used to visualize the vehicle, independent of the running GNC stack.
