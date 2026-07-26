# Running the ABV Software

Assuming you have successfully installed the software, there are a handful of different ways to run the system.

## Running the GNC Packages

The `abv_guidance`, `abv_navigation`, and `abv_controller` packages should be launched first. From the repo root, source the environment:

```bash
source install/setup.bash
```

Then launch the GNC packages. A Jetson-specific launch file is available which handles hardware restrictions on the Jetson Orin:

```bash
ros2 launch abv_bringup jetson.launch.py
```

A general launch file is also available for simulation or other hardware:

```bash
ros2 launch abv_bringup gnc.launch.py
```

## Commanding the Vehicle

There are a handful of different ways to command the vehicle: Teleop, Pose, Velocity, and Path.

### Teleop

Using the `abv_teleop` package, a keyboard or game controller can be used to send commands to the GNC system while it is running. In a new terminal, run:

```bash
ros2 run abv_teleop abv_teleop
```

By default, this listens for input from the arrow keys on the keyboard. The full key mapping is outlined in the `abv_teleop` package documentation.

### Pose

Sends a desired x, y, yaw pose directly to `abv_controller`. In a new terminal, run:

```bash
ros2 run abv_commander abv_commander
```

Then follow the prompts:
| Prompt | Input |
|---|---|
| `command:` | `pose` |
| `goal pose (x y yaw):` | e.g. `1.0 0.5 1.57` |

Units are meters and radians. This sends a single pose command to `abv_controller`.

### Velocity

Sends a desired x, y, yaw velocity directly to `abv_controller`. In a new terminal, run:

```bash
ros2 run abv_commander abv_commander
```

Then follow the prompts:
| Prompt | Input |
|---|---|
| `command:` | `vel` |
| `goal velocity (x y yaw):` | e.g. `0.2 0.0 0.1` |

Units are m/s and rad/s. This sends a single velocity command to `abv_controller`.

### Path

Uses the `abv_guidance` package to execute a sequence of waypoints loaded from `path.csv` in the `abv_bringup` config folder. In a new terminal, run:

```bash
ros2 run abv_commander abv_commander
```

Then follow the prompts:
| Prompt | Input |
|---|---|
| `command:` | `path` |
| `Type:` | `file` |
| `Duration:` | duration in seconds, or `-1` to run the full path |

`abv_guidance` will send each waypoint from `path.csv` to `abv_controller` in sequence, waiting until the ABV has arrived within tolerance of each pose before advancing to the next.

Each line in `path.csv` is one waypoint: `x,y,yaw[,timeout[,x_tol,y_tol,yaw_tol]]`
| Column | Required | Meaning |
|---|---|---|
| `x,y,yaw` | yes | goal pose (m, m, rad) |
| `timeout` | no | seconds to wait for arrival before giving up on this waypoint; omit or use `-1` for the configured default (`Guidance.StateMachine.WaypointTimeout`) |
| `x_tol,y_tol,yaw_tol` | no | per-axis arrival tolerance override (m, m, rad) for this waypoint only; must all three be present together, and `timeout` must be present (use `-1` as a placeholder) if you want to override tolerance without also overriding timeout. Falls back to the tolerance sent with the `path` command, then to the configured default (`Control.Arrival.Tolerance`)|

For example, `0,0,0,-1,0.02,0.02,0.02` uses the default timeout but tightens the arrival tolerance to 2cm/2cm/~1.1&deg; for that one waypoint - useful for a final precision waypoint in an otherwise loosely-toleranced transit path.
