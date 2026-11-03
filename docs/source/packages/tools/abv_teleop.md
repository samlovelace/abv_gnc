# autobot_teleop

Remote teleop commander to manually drive the ABV.

## Purpose

The `autobot_teleop` package is responsible for taking user inputs from the configured device (arrow keys, game controller) and converting that to the corresponding body-frame `direction` command sent to `autobot_goal_manager`.

## Usage

Run the `autobot_teleop` package from the `autobot` folder

```bash
sudo ./scripts/run.sh autobot_teleop
```

After running the program, using the computer keyboard is enabled by default. The mapping of the keys to command is shown below. Note, all commands are in the ABV body frame. Meaning, regardless of the orientation of the ABV, pressing the `arrow up` key will drive the ABV forward from its current perspective.

- `arrow up` +x
- `arrow down` -x
- `arrow right` +y
- `arrow left` -y
- `e` +yaw
- `q` -yaw

## Topics

### Published

- `/abv/controller/command` - publishes `direction` commands, which `autobot_goal_manager` applies in its `DIRECTION_CONTROL` state. These bypass the control policies, so teleop works even with no policy selected. If input stops for more than 250 ms, `autobot_goal_manager` zeroes the thrusters.
