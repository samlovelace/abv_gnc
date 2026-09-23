# abv_gui

Desktop ground-station GUI for live telemetry and commanding the ABV.

---

## Purpose

The `abv_gui` package is a native Qt6 desktop application ("ABV Ground Station") that gives an operator a single window for watching the vehicle's live state and sending it commands, without needing a terminal. It is the primary tool referenced in the [Quick Start](../../operation/index.md) guide.

Like `abv_commander` and `abv_teleop`, it is a command *source* — it publishes the same `abv/controller/command` / `abv/guidance/command` topics as any other tool, and has no special standing with the GNC pipeline.

## Usage

Run it directly, or via `abv_bringup`'s `gnc.launch.py` (started automatically when launched with `sim:=true`):

```bash
ros2 run abv_gui abv_gui
```

## Architecture

Core components:

- `TableTopView` — an always-visible, to-scale top-down render of the physical table with the vehicle's live pose (a heading-aware glyph with thruster nozzles that light up when firing). Click-drag-release proposes a goal pose (shown as a ghost outline); a confirm popup then either publishes it via `CommandPanel::sendPoseCommand()` or discards it. Knows nothing about ROS itself — it only emits a `goalPoseSelected(x, y, yaw)` signal and leaves the decision of what "sending" means to the caller.
- `CommandPanel` — tabbed pose/velocity/path/thruster command panel, equivalent to `abv_commander`'s CLI prompts but with live spin boxes. Mirrors the vehicle's current pose into the Pose tab's fields (skipping any field the user is actively editing) so commands start from the current pose rather than zero.
- `LivePlot` — a scrolling time-series chart with autoscaling and an optional live-value readout, used three times: Position, Velocity, and Control Input.
- `StatusPanel` — displays the guidance and controller state-machine states and the controller's arrival state (`IDLE` / `RUNNING` / `ARRIVED`).
- `NodeHealthPanel` — one indicator per expected node (`controller`, `navigation`, `guidance`, `bridge`), driven by `abv/heartbeat`: starts Disconnected, turns Connected on first heartbeat, and reverts to Disconnected if heartbeats stop arriving. A separate **Comms** indicator (via `NetworkPinger`) does a genuine network-layer ping to the robot host, independent of ROS/DDS, so a comms failure can be told apart from a crashed node.
- `TopicAdapter<MsgT, T>` — a generic bridge from a ROS subscription to a Qt signal (`newDataVariant`), used throughout so that ROS callback data is only ever consumed on the Qt GUI thread rather than racing user interaction on the same widgets.

## Topics

### Subscribed

- `/abv/state`  
  Drives the Position/Velocity plots, the table view's vehicle glyph, and the Command Panel's pose-sync.

- `/abv/controller/status`  
  Drives the Control Input plot and the Status Panel's controller/arrival state.

- `/abv/guidance/status`  
  Drives the Status Panel's guidance state.

- `/abv/controller/thrusters`  
  Lights up firing thrusters on the table view's vehicle glyph.

- `/abv/heartbeat`  
  Drives the Node Health panel's per-node Connected/Disconnected indicators.

### Published

- `/abv/controller/command`  
  Pose, velocity, and thruster commands from the Command Panel and the table view's click-to-set-goal.

- `/abv/guidance/command`  
  Path commands from the Command Panel.
