# abv_bridge

Translates between `abv_msgs` and external interfaces (an external autonomy framework, and Gazebo simulation).

---

## Purpose

The `abv_bridge` package converts messages between the ABV's native `abv_msgs` types and external message types (`ptera_msgs`, and Gazebo simulation topics), so the GNC pipeline can integrate with systems outside this repo — an external autonomy stack, or a physics simulator — without any other package needing to depend on those external interfaces directly.

It contains no control, guidance, or navigation logic of its own; it is purely a translation layer.

---

## Role in the System

`abv_bridge` sits alongside the GNC pipeline rather than inside it — it observes the same topics an external observer would (`abv/state`, `abv/controller/status`) and republishes them in another vocabulary, and does the reverse for incoming external commands.

---

## Architecture

Each conversion direction is a small, focused class, constructed with an incoming and outgoing topic name and wired up once in `main()`:

- `NavigationConvertor`: `abv_msgs::AbvState` → `ptera_msgs::RobotState` (ABV state out to the external autonomy stack).
- `ControllerStatusConvertor`: `abv_msgs::AbvControllerStatus` → `ptera_msgs::ControllerStatus` (controller status out to the external autonomy stack).
- `WaypointConvertor`: `ptera_msgs::VehicleWaypoint` → `abv_msgs::AbvGuidanceCommand` (external waypoint commands in, forwarded to `abv_guidance`).
- `GazeboStateConvertor`: `ptera_msgs::RobotState` → `abv_msgs::AbvState` (Gazebo's simulated physics feedback in, so `abv_controller` can consume it as a native `AbvState` when `Dynamics.PropagationMode: external`).

A `HeartbeatPublisher` runs alongside these so `abv_bridge` shows up in `abv_gui`'s Node Health panel like any other long-running node.

---

## Topics

### Subscribed

- `/abv/state`  
  Re-published externally via `NavigationConvertor`.

- `/abv/controller/status`  
  Re-published externally via `ControllerStatusConvertor`.

- `robot/vehicle/waypoint` (external, `ptera_msgs/VehicleWaypoint`)  
  Converted to `abv/guidance/command` via `WaypointConvertor`.

- `gazebo/robot/state` (external, `ptera_msgs/RobotState`)  
  Converted to `abv/sim/gazebo_state` via `GazeboStateConvertor`.

### Published

- `robot/state` (external, `ptera_msgs/RobotState`)

- `robot/vehicle/controller_status` (external, `ptera_msgs/ControllerStatus`)

- `abv/guidance/command`  
  Forwarded to `abv_guidance`.

- `abv/sim/gazebo_state`  
  Forwarded to `abv_controller` for external state propagation.

- `abv/heartbeat`

See [Data Flow](../../overview/data_flow.md) for the full topic table, including direction across each conversion.
