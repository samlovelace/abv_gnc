# Running the ABV Software

Assuming you have successfully installed the software, there are a handful of different ways to run the system.

## Running the GNC Packages

The on-board GNC nodes should be launched first. From the repo root, source the environment:

```bash
source install/setup.bash
```

Then launch the GNC packages. A Jetson-specific launch file is available which handles hardware restrictions on the Jetson Orin. It must be run from the repo root, since it starts each node through `./scripts/run.sh` with `sudo`:

```bash
ros2 launch autobot_bringup jetson.launch.py
```

A general launch file is also available for simulation or other hardware:

```bash
ros2 launch autobot_bringup gnc.launch.py            # GNC nodes only
ros2 launch autobot_bringup gnc.launch.py sim:=true  # + autobot_simulator, autobot_gui, autobot_commander
```

| Node                                                           | `jetson.launch.py` | `gnc.launch.py`  |
| -------------------------------------------------------------- | ------------------ | ---------------- |
| `autobot_guidance`, `autobot_navigation`                       | ✓                  | ✓                |
| `autobot_goal_manager`, `autobot_control`, `autobot_thrusters` | ✓                  | ✓                |
| `autobot_policy_registry`, `autobot_rl-cpp`                    | ✓                  | ✓                |
| `autobot_bridge` (as `autobot_ptera_bridge`)                   | —                  | ✓                |
| `autobot_simulator`, `autobot_gui`, `autobot_commander`        | —                  | with `sim:=true` |

See [`autobot_bringup`](../packages/system/autobot_bringup.md) for details.

## Select a Control Policy

**No control policy is active at startup.** Until one is selected, `autobot_thrusters` drops every command from the control policies, so pose, velocity, and path commands are accepted but the vehicle doesn't move. Teleop and direct thruster commands are not affected.

Select a policy from `autobot_gui`'s policy selector (pick `control` for PID or `rl` for the RL policy, then click **Switch**), or from the command line:

```bash
ros2 service call /abv/policy/switch autobot_msgs/srv/AbvPolicySwitch "{policy_name: control}"
```

The active policy is shown in the banner at the top of `autobot_gui`. Switching policies clears the current goal. See [`autobot_policy_registry`](../packages/gnc/autobot_policy_registry.md).

## Commanding the Vehicle

There are a handful of different ways to command the vehicle: Teleop, Pose, Velocity, and Path. The quickest, for pose goals, is clicking on `autobot_gui`'s table view — see [Quick Start](../operation/index.md).

### Teleop

Using the `autobot_teleop` package, a keyboard or game controller can be used to send commands to the GNC system while it is running. In a new terminal, run:

```bash
ros2 run autobot_teleop autobot_teleop
```

By default, this listens for input from the arrow keys on the keyboard. The full key mapping is outlined in the `autobot_teleop` package documentation.

### Pose

Sends a desired x, y, yaw pose directly to `autobot_goal_manager` (bypassing `autobot_guidance`). In a new terminal, run:

```bash
ros2 run autobot_commander autobot_commander
```

Then follow the prompts:

| Prompt                         | Input               |
| ------------------------------ | ------------------- |
| `Command:`                     | `pose`              |
| `Enter goal pose (x, y, yaw):` | e.g. `1.0 0.5 1.57` |

Units are meters and radians. This sends a single pose command; the vehicle only moves once a control policy is active.

### Velocity

Sends a desired x, y, yaw velocity directly to `autobot_goal_manager`. In a new terminal, run:

```bash
ros2 run autobot_commander autobot_commander
```

Then follow the prompts:

| Prompt                                | Input              |
| ------------------------------------- | ------------------ |
| `Command:`                            | `vel`              |
| `Enter goal velocity (xd, yd, yawd):` | e.g. `0.2 0.0 0.1` |

Units are m/s and rad/s. This sends a single velocity command; the vehicle only moves once a control policy is active.

### Path

Uses the `autobot_guidance` package to execute a sequence of waypoints loaded from `path.csv` in the `autobot_bringup` config folder. In a new terminal, run:

```bash
ros2 run autobot_commander autobot_commander
```

Then follow the prompts:

| Prompt          | Input                                             |
| --------------- | ------------------------------------------------- |
| `Command:`      | `path`                                            |
| `Type:`         | `file`                                            |
| `Duration (s):` | duration in seconds, or `-1` to run the full path |

`autobot_guidance` also supports `line` and `avoid` path types, but both need a goal pose, which `autobot_commander` can't supply — send those from `autobot_gui` instead (see [`autobot_guidance`](../packages/gnc/autobot_guidance.md#guidance-modes)).

`autobot_guidance` will send each waypoint from `path.csv` to `autobot_goal_manager` in sequence, waiting until the ABV has arrived within tolerance of each pose before advancing to the next.

Each line in `path.csv` is one waypoint: `x,y,yaw[,timeout[,x_tol,y_tol,yaw_tol]]`

| Column                | Required | Meaning                                                                                                                                                                                                                                                                                                                                                         |
| --------------------- | -------- | --------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------- |
| `x,y,yaw`             | yes      | goal pose (m, m, rad)                                                                                                                                                                                                                                                                                                                                           |
| `timeout`             | no       | seconds to wait for arrival before giving up on this waypoint; omit or use `-1` for the configured default (`Guidance.StateMachine.WaypointTimeout`)                                                                                                                                                                                                            |
| `x_tol,y_tol,yaw_tol` | no       | per-axis arrival tolerance override (m, m, rad) for this waypoint only; must all three be present together, and `timeout` must be present (use `-1` as a placeholder) if you want to override tolerance without also overriding timeout. Falls back to the tolerance sent with the `path` command, then to the configured default (`Control.Arrival.Tolerance`) |

For example, `0,0,0,-1,0.02,0.02,0.02` uses the default timeout but tightens the arrival tolerance to 2cm/2cm/~1.1&deg; for that one waypoint - useful for a final precision waypoint in an otherwise loosely-toleranced transit path.

When a waypoint times out, `autobot_guidance` checks the current error against a relaxed tolerance (the waypoint's tolerance × `Guidance.StateMachine.WaypointTimeoutToleranceScale`). If within it, the waypoint counts as reached and the path continues; otherwise the path is aborted and a `STOP` is sent.

A line starting with `#` is a `key: value` header. The only supported key is `PreviewLength`, e.g. `# PreviewLength: 3`, which limits how many upcoming waypoints are published on `abv/guidance/path` (and drawn in `autobot_gui`). By default all remaining waypoints are shown.
