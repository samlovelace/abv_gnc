# autobot_bringup

Central launch files and configuration for the ABV stack.

---

## Purpose

The `autobot_bringup` package holds no source code of its own — it is the single place that launch files and the shared `config.yaml` live, so that starting the stack, and tuning it, doesn't require touching any other package.

---

## Launch files

- **`gnc.launch.py`** — the general-purpose launch file. Starts `autobot_goal_manager`, `autobot_control`, `autobot_thrusters`, `autobot_navigation`, `autobot_guidance`, `autobot_bridge` (node name `autobot_ptera_bridge`), `autobot_policy_registry`, and `autobot_rl-cpp` (node name `rl_policy_node`) as regular ROS nodes. Accepts a `sim` launch argument (default `false`); when `sim:=true`, also starts `autobot_simulator`, `autobot_gui`, and `autobot_commander`. Use this off the Jetson (e.g. running the stack fully in simulation) or on any hardware without GPIO restrictions.
- **`jetson.launch.py`** — the Jetson-specific launch file. Starts `autobot_guidance`, `autobot_navigation`, `autobot_goal_manager`, `autobot_control`, `autobot_thrusters`, `autobot_rl-cpp`, and `autobot_policy_registry` via `sudo ./scripts/run.sh <package>` rather than `ros2 run` — see [Why `run.sh`](#why-runsh-instead-of-ros2-run) below — so it must be launched from the workspace root. It does **not** start `autobot_bridge`. `autobot_thrusters` and `autobot_rl-cpp` are started through `sudo bash -c` with an explicit `LD_LIBRARY_PATH` covering the workspace, ROS, JetsonGPIO, ONNX Runtime, and CUDA 12.6 libraries; these paths are hard-coded for the lab Jetson (`/home/optimus/...`) and need editing on any other machine.

Both launch files include `autobot_policy_registry`, which is required whenever a control-policy node runs: without it `abv/policy/active` is never published and `autobot_thrusters` drops every policy-sourced command. Even with it, **no policy is active at startup** — select one from `autobot_gui` or `abv/policy/switch` (see [`autobot_policy_registry`](../gnc/autobot_policy_registry.md)).

### Why `run.sh` instead of `ros2 run`

On the Jetson, `autobot_thrusters` needs direct GPIO access to fire the thrusters, which requires root. `sudo ros2 run <pkg> <exe>` doesn't reliably pick up the workspace's sourced environment under `sudo`, so the Jetson launch file instead shells out to `scripts/run.sh <package_name>` (at the workspace root), which explicitly sources `/opt/ros/humble/setup.bash` and `install/setup.bash` before `exec`-ing the binary directly out of `install/<package>/lib/<package>/`.

## Configuration

`config/config.yaml` is loaded at startup by every node through `autobot_common`'s `ConfigurationManager`. It's organized by subsystem:

| Section          | Used by                                                       | Contents                                                                                                                                                                                                                                                                                                                                    |
| ---------------- | ------------------------------------------------------------- | ------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------- |
| `Guidance`       | `autobot_guidance`                                            | `StateMachine.Rate`, default `WaypointTimeout`, and `WaypointTimeoutToleranceScale` (relaxed-tolerance multiplier applied when a waypoint times out)                                                                                                                                                                                        |
| `Navigation`     | `autobot_navigation`, `autobot_simulator`, `autobot_gui`      | `Interface` (`optitrack` \| `simulated`), `Rate`, OptiTrack `RigidBodyName`, `MaxDeadReckonDuration`, `SimulateDropout` (simulator only), and the `Network.Local/Server` addresses used for motion capture (the local IP is also what `autobot_gui` pings)                                                                                  |
| `Control`        | `autobot_goal_manager`, `autobot_common`, `autobot_simulator` | `StateMachine.Rate`, `Arrival.Tolerance`/`Duration`, `NavDataTimeout` (seconds before nav data counts as stale), and `Dynamics` — vehicle mass/inertia/thruster force/moment arm for simulation, plus `PropagationMode` (`internal` \| `external`), `ExternalStateTopic`, `WrenchCommandTopic`, `WrenchTargetLink` for external propagation |
| `ControlLaw`     | `autobot_control`                                             | `Law` (`PID`; `LQR`/`MPC` reserved) and `Pid.Kp/Ki/Kd`                                                                                                                                                                                                                                                                                      |
| `Thrusters`      | `autobot_thrusters`, `autobot_simulator`                      | `InputDiscretization.On/Off`, `AllocationStrategy` (`LookupTable` \| `Matrix`), `Allocation.x/y/yaw/Threshold`, `ThrusterDriver` (`Type`, `GPIO.pins`), `Force`, `MomentArm`                                                                                                                                                                |
| `Heartbeat`      | every node, `autobot_gui`, `autobot_policy_registry`          | `Rate`, `StaleAfter` (when a node or policy counts as disconnected), `PingIntervalMs` (GUI comms ping)                                                                                                                                                                                                                                      |
| `TableView`      | `autobot_gui`, `autobot_guidance`, `autobot_simulator`        | Table dimensions and vehicle footprint (also used as the collision footprint and the simulator spawn offset)                                                                                                                                                                                                                                |
| `Scene`          | `autobot_guidance`                                            | `XMin/XMax/YMin/YMax` — the region the vehicle must stay within                                                                                                                                                                                                                                                                             |
| `CollisionAvoid` | `autobot_guidance`                                            | `GridResolution` (A\* cell size, m) and `ReplanRate` (Hz)                                                                                                                                                                                                                                                                                   |

The control policy is no longer chosen in config — it is selected at runtime through [`autobot_policy_registry`](../gnc/autobot_policy_registry.md).

`config/path.csv` is the waypoint file consumed by `autobot_guidance`'s `file` mode (via `autobot_commander`'s `path` command or the GUI's Path tab) — see [Running the ABV Software](../../getting_started/running_abv.md#path) for the file format. `config/vid1.csv` is an alternate waypoint file; nothing reads it by default, so copy it over `path.csv` to use it.
