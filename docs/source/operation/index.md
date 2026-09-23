<!-- screenshot captured 2026-08-30, re-capture manually if abv_gui's layout changes -->

# Quick Start

This page walks through the fastest path from "software is installed" to "the ABV is moving and I'm watching it happen": confirming both machines are ready, starting the GNC stack, starting the operator GUI, checking comms, and sending a first pose command. It assumes you've already completed [Installation](../getting_started/installation.md) on both machines; see [Running the ABV Software](../getting_started/running_abv.md) for the full detail behind each command used here.

## 1. Confirm both machines are up to date

The stack has two sides that both need to be built and current before a session:

- **On the Jetson Orin** (on board the vehicle): `ssh` in, `cd` to the workspace, `git pull`, then re-run `./scripts/setup.sh` if dependencies changed, or just `colcon build` if only code changed. Re-source with `source install/setup.bash`.
- **On the operator laptop**: same steps, run locally. The laptop only needs the operator tools (`abv_gui`, `abv_commander`, `abv_teleop`) — the [Installation](../getting_started/installation.md) `./scripts/setup.sh` builds everything, so this is the same process on both machines.

Verify each machine with `ros2 pkg list | grep abv_` per [Installation](../getting_started/installation.md#verify-installation).

## 2. Start the GNC stack (on the Jetson)

From the workspace root on the Jetson:

```bash
source install/setup.bash
ros2 launch abv_bringup jetson.launch.py
```

This launches `abv_guidance`, `abv_navigation`, and `abv_controller` directly (not via `ros2 run`, since thruster GPIO access needs root — see [`abv_bringup`](../packages/system/abv_bringup.md)). Leave this running for the rest of the session.

## 3. Start the operator GUI (on the laptop)

In a new terminal on the operator laptop:

```bash
source install/setup.bash
ros2 run abv_gui abv_gui
```

This opens the "ABV Ground Station" window: a live table-top view of the vehicle on the left, live position/velocity/control plots in the middle, and a command/status/health panel on the right. See [`abv_gui`](../packages/tools/abv_gui.md) for a full breakdown of every panel.

![The ABV Ground Station window, showing the table-top view with the vehicle and firing thrusters, live position/velocity/control-input plots, the pose command panel, status panel, and node health panel](/_static/abv_gui_screenshot.png)

## 4. Check comms before commanding anything

Before sending any command, check the **Node Health** panel (bottom right):

- Each of `controller`, `navigation`, `guidance`, and `bridge` should show **Connected** — this is driven by each node's heartbeat, and will flip to Disconnected if a node stops publishing.
- The separate **Comms** indicator is a network-level ping to the robot host, independent of ROS/DDS — useful for telling "the network is down" apart from "a node crashed."

If a node shows Disconnected, check that its process is still running in the Jetson terminal from step 2 before commanding the vehicle.

## 5. Send a pose command from the GUI

Once comms look good, the table-top view is the fastest way to command a pose:

1. Click and drag on the table view where you want the vehicle to go — releasing shows a ghost outline of the proposed goal pose.
2. A small confirm menu pops up showing the pose; choose **Send Goal** to publish it, or click elsewhere to discard it.

The **Command Panel** (right side) offers the same pose/velocity/thruster/path commands as [`abv_commander`](../packages/tools/abv_commander.md), if you need more precision than a click-drag, or want to run a full waypoint path from `abv_bringup`'s `path.csv`.

## Next steps

- [`abv_gui`](../packages/tools/abv_gui.md) — full reference for every panel in the ground station.
- [Running the ABV Software](../getting_started/running_abv.md) — command-line alternatives (`abv_commander`, `abv_teleop`) and the full pose/velocity/path command reference.
- [Architecture](../overview/architecture.md) and [Data Flow](../overview/data_flow.md) — what's actually happening under the hood once a command is sent.
