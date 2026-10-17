# abv_gui

Desktop ground-station GUI for live telemetry and commanding the ABV.

---

## Purpose

The `abv_gui` package is a native Qt6 desktop application ("ABV Ground Station") that gives an operator a single window for watching the vehicle's live state and sending it commands, without needing a terminal. It is the primary tool referenced in the [Quick Start](../../operation/index.md) guide.

Like `abv_commander` and `abv_teleop`, it is a command *source* with no special standing in the GNC pipeline: pose goals go to `abv_guidance` on `abv/guidance/command`, and velocity, thruster, direction, and stop commands go to `abv_goal_manager` on `abv/controller/command`. It is also where an operator places obstacles for collision avoidance and selects the active control policy.

## Usage

Run it directly, or via `abv_bringup`'s `gnc.launch.py` (started automatically when launched with `sim:=true`):

```bash
ros2 run abv_gui abv_gui
```

At startup no control policy is active:

![abv_gui at startup with the "No Active Policy" banner](/_static/abv_gui_no_policy.png)

After switching to the `control` policy and sending a goal from the table view:

![abv_gui with "Active Policy: CONTROL", a goal and planned path on the table view, and the vehicle in POSE_CONTROL](/_static/abv_gui_policy_with_goal.png)

## Architecture

Core components:

- `ActivePolicyBanner` — full-width banner at the top of the window showing the active control policy from `abv/policy/active`, or **No Active Policy** when none has been selected.
- `TableTopView` — an always-visible, to-scale top-down render of the physical table (sized by `TableView.Width/Height`, origin at the table's bottom-right corner) with the vehicle's live pose (a heading-aware glyph with thruster nozzles that light up when firing), placed obstacles, and the planned path from `abv/guidance/path`. Knows nothing about ROS itself — it only emits signals and leaves the decision of what "sending" means to the caller.
  - **Left-click-drag-release** proposes a goal pose (shown as a ghost outline); a confirm popup (**Send Goal**) then sends it via `CommandPanel::sendPoseCommand()`, or discards it.
  - **Right-click-drag-release** proposes a circular obstacle (drag distance sets the radius); confirming **Add Obstacle** adds it to the obstacle set and republishes the full set on `abv/scene/obstacles`.
  - **Right-click** (no drag) offers **Clear Obstacles**, which publishes an empty set.
- `CommandPanel` — tabbed command panel with live spin boxes:
  - **Pose** — sends the pose to `abv_guidance` as a `line` command, or `avoid` if any obstacles are placed (the same path as a table-view goal). Mirrors the vehicle's current pose into its fields (skipping any field the user is actively editing) so commands start from the current pose rather than zero.
  - **Velocity** — sends a `vel` command to `abv_goal_manager`; the **Body Frame** checkbox sends it in the body frame instead of the world frame.
  - **Path** — sends a `File` path with a duration to `abv_guidance` (runs `path.csv`).
  - **Thrusters** — per-thruster (`T1`–`T8`) and per-axis buttons that send `thruster` / `direction` commands at 10 Hz while held, and a zero command on release. These bypass the control policies, so they work without one selected.
- **Stop** button — sends `STOP` on `abv/controller/command`.
- `PolicySelectorPanel` ("Control Policy") — lists the policies on `abv/policy/registry`; **Switch** calls the `abv/policy/switch` service and shows the result.
- `LivePlot` — a scrolling time-series chart with autoscaling and an optional live-value readout, used three times: Position, Velocity, and Control Input.
- `StatusPanel` — displays the guidance and goal-manager state-machine states and the arrival state (`IDLE` / `RUNNING` / `ARRIVED`).
- `NodeHealthPanel` — one indicator per expected node (`goal_manager`, `navigation`, `guidance`, `bridge`, `control`, `thrusters`), driven by `abv/heartbeat`: starts Disconnected, turns Connected on first heartbeat, and reverts to Disconnected if no heartbeat arrives within `Heartbeat.StaleAfter`. A separate **Comms** indicator (via `NetworkPinger`) pings the robot host (`Navigation.Network.Local.Ip`) every `Heartbeat.PingIntervalMs`, independent of ROS/DDS, so a comms failure can be told apart from a crashed node.
- `TopicAdapter<MsgT, T>` — a generic bridge from a ROS subscription to a Qt signal (`newDataVariant`), used throughout so that ROS callback data is only ever consumed on the Qt GUI thread rather than racing user interaction on the same widgets.

## Topics

### Subscribed

- `/abv/state`  
  Drives the Position/Velocity plots, the table view's vehicle glyph, and the Command Panel's pose-sync.

- `/abv/thrusters/status`  
  Drives the Control Input plot (applied force/torque) and lights up firing thrusters on the table view's vehicle glyph.

- `/abv/controller/status`  
  Drives the Status Panel's goal-manager/arrival state.

- `/abv/guidance/status`  
  Drives the Status Panel's guidance state.

- `/abv/guidance/path`  
  The planned path drawn on the table view.

- `/abv/policy/registry`, `/abv/policy/active`  
  Populate the policy selector and the active-policy banner.

- `/abv/heartbeat`  
  Drives the Node Health panel's per-node Connected/Disconnected indicators.

### Published

- `/abv/guidance/command`  
  Pose goals (`line` / `avoid`) from the table view and Pose tab, and `File` path commands.

- `/abv/controller/command`  
  Velocity, thruster, direction, and `STOP` commands.

- `/abv/scene/obstacles`  
  The full set of placed obstacles, republished whenever it changes.

### Services called

- `/abv/policy/switch`  
  From the policy selector's **Switch** button.
