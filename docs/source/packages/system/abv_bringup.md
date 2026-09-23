# abv_bringup

Central launch files and configuration for the ABV stack.

---

## Purpose

The `abv_bringup` package holds no source code of its own — it is the single place that launch files and the shared `config.yaml` live, so that starting the stack, and tuning it, doesn't require touching any other package.

---

## Launch files

- **`gnc.launch.py`** — the general-purpose launch file. Starts `abv_guidance`, `abv_navigation`, `abv_controller`, and `abv_bridge` via `ros2 run`. Accepts a `sim` launch argument (default `false`); when `sim:=true`, also starts `abv_simulator`, `abv_gui`, and `abv_commander`. Use this off the Jetson (e.g. running the stack fully in simulation) or on any hardware without GPIO restrictions.
- **`jetson.launch.py`** — the Jetson-specific launch file. Starts the same three GNC nodes (`abv_guidance`, `abv_navigation`, `abv_controller`), but via `sudo ./scripts/run.sh <package>` rather than `ros2 run` — see [Why `run.sh`](#why-runsh-instead-of-ros2-run) below.
- **`jetson-rl.launch.py`** — identical to `jetson.launch.py`, plus `abv_rl-cpp` as a fourth process, launched with the extra `LD_LIBRARY_PATH` entries its ONNX Runtime / JetsonGPIO / CUDA dependencies need on the Jetson.

### Why `run.sh` instead of `ros2 run`

On the Jetson, `abv_controller` needs direct GPIO access to fire the thrusters, which requires root. `sudo ros2 run <pkg> <exe>` doesn't reliably pick up the workspace's sourced environment under `sudo`, so the Jetson launch files instead shell out to `scripts/run.sh <package_name>` (at the workspace root), which explicitly sources `/opt/ros/humble/setup.bash` and `install/setup.bash` before `exec`-ing the binary directly out of `install/<package>/lib/<package>/`.

## Configuration

`config/config.yaml` is loaded at startup by every node through `abv_common`'s `ConfigurationManager`. It's organized by subsystem:

- `Guidance` — state machine rate, default waypoint timeout.
- `Navigation` — sensing interface (`optitrack`, or simulated), rate, OptiTrack rigid-body name, dead-reckoning bound, and the local/server network addresses used for motion capture.
- `Control` — control policy selection (`PID` | `External`), state machine rate, arrival tolerance/duration, PID gains, thruster input discretization, thruster allocation strategy (`LookupTable` | `Matrix`) and pin mapping, and vehicle dynamics parameters (used for simulation).
- `Heartbeat` — per-node heartbeat rate and the staleness window `abv_gui` uses to mark a node disconnected.
- `TableView` — physical table dimensions and vehicle footprint, used by `abv_gui`'s table-top view.

`config/path.csv` is an example waypoint file consumed by `abv_guidance`'s file-based trajectory mode (via `abv_commander`'s `path` command) — see [Running the ABV Software](../../getting_started/running_abv.md#path) for the file format.
