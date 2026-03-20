# Running

Assuming you have successfully installed the software, there are a handful different ways of running the system.

## Running the GNC Packages

The `abv_guidance`, `abv_navigation` and `abv_control` packages should be launched first. To do so, from the `abv_gnc` root directory, first source the environment by doing

```bash
source install/setup.bash
```

Then, launch the GNC packages. Note, due to hardware restrictions on the Jetson Orin, a special run script is used. A Jeston specific launch file is available. To run it, from the `abv_gnc` directory, run

```bash
ros2 launch abv_bringup jetson.launch.py
```

A general launch file for the GNC packages is also available for simulation or other hardware not requiring special scripts.

```bash
ros2 launch abv_bringup gnc.launch.py
```

## Commanding the Vehicle

There are a handful of different ways to command the vehicle

- Teleop
- Path
- Pose
- Velocity

### Teleop

Using the `abv_teleop` package, the computer keyboard or a game controller can be used to send commands the the GNC system while running. To use the `abv_teleop` package for sending commands, open a new terminal and run the `abv_teleop` package.

```bash
ros2 run abv_teleop abv_teleop
```

By default, this package is listening for input from the arrow keys (and a couple others) on the computer keyboard. The mapping for each key is outlined in the `abv_teleop` package documentation. # TODO: link to this page somehow

### Pose

This mode allows a user to send a desired x, y, yaw pose to the `abv_controller` directly. To do this, first run the `abv_commander` package in a new terminal

```bash
ros2 run abv_commander abv_commander
```

When prompted for a command, input `pose` and press enter. When prompted for a goal pose, enter the desired x, y, yaw pose (units: meters and radians) and press enter. This will send a single pose command to the `abv_controller` package.

### Velocity

This mode allows a user to send a desired x, y, yaw velocity to the `abv_controller` directly. To do this, first run the `abv_commander` package in a new terminal

```bash
ros2 run abv_commander abv_commander
```

When prompted for a command, input `vel` and press enter. When prompted for a goal velocity, enter the desired x, y, yaw velocities (units: m/s and rad/s) and press enter. This will send a single velocity command to the `abv_controller` package.

### Path

This mode utilizes the `abv_guidance` package to handle sending a sequence of waypoints to the ABV. It requires usage of the `abv_commander` command line tool. To run the `abv_commander`, in a new terminal

```bash
ros2 run abv_commander abv_commander
```

- When prompted for `command:` input `path`.
- When prompted for `Type:` input `file`. (NOTE: `file` is the only mode supported for the latest software)
- When prompted for `Duration:` input a desired duration in seconds, or -1 to allow the full path to be executed.

Inputting the above data into the `abv_commander` tool will send a command to the `abv_guidance` package to load the `path.csv` file from the `abv_bringup` config folder. This `path.csv` contains a sequence of desired x, y, yaw values for the ABV to execute. The `abv_guidance` pacakge will send these waypoints to the `abv_controller` package and wait until the ABV has arrived within some tolerance of the goal pose, before sending the next waypoint in the sequence.
