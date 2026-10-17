# abv_bringup

Central launch files and configuration for the ABV stack.

---

## Purpose

The `abv_bringup` package holds no source code of its own — it is the single place that launch files and the shared `config.yaml` live, so that starting the stack, and tuning it, doesn't require touching any other package.

---

## Launch files

- **`gnc.launch.py`** — the general-purpose launch file. Starts `abv_goal_manager`, `abv_control`, `abv_thrusters`, `abv_navigation`, `abv_guidance`, `abv_bridge` (node name `abv_ptera_bridge`), `abv_policy_registry`, and `abv_rl-cpp` (node name `rl_policy_node`) as regular ROS nodes. Accepts a `sim` launch argument (default `false`); when `sim:=true`, also starts `abv_simulator`, `abv_gui`, and `abv_commander`. Use this off the Jetson (e.g. running the stack fully in simulation) or on any hardware without GPIO restrictions.
- **`jetson.launch.py`** — the Jetson-specific launch file. Starts `abv_guidance`, `abv_navigation`, `abv_goal_manager`, `abv_control`, `abv_thrusters`, `abv_rl-cpp`, and `abv_policy_registry` via `sudo ./scripts/run.sh <package>` rather than `ros2 run` — see [Why `run.sh`](#why-runsh-instead-of-ros2-run) below — so it must be launched from the workspace root. It does **not** start `abv_bridge`. `abv_thrusters` and `abv_rl-cpp` are started through `sudo bash -c` with an explicit `LD_LIBRARY_PATH` covering the workspace, ROS, JetsonGPIO, ONNX Runtime, and CUDA 12.6 libraries; these paths are hard-coded for the lab Jetson (`/home/optimus/...`) and need editing on any other machine.

Both launch files include `abv_policy_registry`, which is required whenever a control-policy node runs: without it `abv/policy/active` is never published and `abv_thrusters` drops every policy-sourced command. Even with it, **no policy is active at startup** — select one from `abv_gui` or `abv/policy/switch` (see [`abv_policy_registry`](../gnc/abv_policy_registry.md)).

### Why `run.sh` instead of `ros2 run`

On the Jetson, `abv_thrusters` needs direct GPIO access to fire the thrusters, which requires root. `sudo ros2 run <pkg> <exe>` doesn't reliably pick up the workspace's sourced environment under `sudo`, so the Jetson launch file instead shells out to `scripts/run.sh <package_name>` (at the workspace root), which explicitly sources `/opt/ros/humble/setup.bash` and `install/setup.bash` before `exec`-ing the binary directly out of `install/<package>/lib/<package>/`.

## Configuration

`config/config.yaml` is loaded at startup by every node through `abv_common`'s `ConfigurationManager`. It's organized by subsystem:

| Section | Used by | Contents |
|---|---|---|
| `Guidance` | `abv_guidance` | `StateMachine.Rate`, default `WaypointTimeout`, and `WaypointTimeoutToleranceScale` (relaxed-tolerance multiplier applied when a waypoint times out) |
| `Navigation` | `abv_navigation`, `abv_simulator`, `abv_gui` | `Interface` (`optitrack` \| `simulated`), `Rate`, OptiTrack `RigidBodyName`, `MaxDeadReckonDuration`, `SimulateDropout` (simulator only), and the `Network.Local/Server` addresses used for motion capture (the local IP is also what `abv_gui` pings) |
| `Control` | `abv_goal_manager`, `abv_common`, `abv_simulator` | `StateMachine.Rate`, `Arrival.Tolerance`/`Duration`, `NavDataTimeout` (seconds before nav data counts as stale), and `Dynamics` — vehicle mass/inertia/thruster force/moment arm for simulation, plus `PropagationMode` (`internal` \| `external`), `ExternalStateTopic`, `WrenchCommandTopic`, `WrenchTargetLink` for external propagation |
| `ControlLaw` | `abv_control` | `Law` (`PID`; `LQR`/`MPC` reserved) and `Pid.Kp/Ki/Kd` |
| `Thrusters` | `abv_thrusters`, `abv_simulator` | `InputDiscretization.On/Off`, `AllocationStrategy` (`LookupTable` \| `Matrix`), `Allocation.x/y/yaw/Threshold`, `ThrusterDriver` (`Type`, `GPIO.pins`), `Force`, `MomentArm` |
| `Heartbeat` | every node, `abv_gui`, `abv_policy_registry` | `Rate`, `StaleAfter` (when a node or policy counts as disconnected), `PingIntervalMs` (GUI comms ping) |
| `TableView` | `abv_gui`, `abv_guidance`, `abv_simulator` | Table dimensions and vehicle footprint (also used as the collision footprint and the simulator spawn offset) |
| `Scene` | `abv_guidance` | `XMin/XMax/YMin/YMax` — the region the vehicle must stay within |
| `CollisionAvoid` | `abv_guidance` | `GridResolution` (A* cell size, m) and `ReplanRate` (Hz) |

The control policy is no longer chosen in config — it is selected at runtime through [`abv_policy_registry`](../gnc/abv_policy_registry.md).

`config/path.csv` is the waypoint file consumed by `abv_guidance`'s `file` mode (via `abv_commander`'s `path` command or the GUI's Path tab) — see [Running the ABV Software](../../getting_started/running_abv.md#path) for the file format. `config/vid1.csv` is an alternate waypoint file; nothing reads it by default, so copy it over `path.csv` to use it.
