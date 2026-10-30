# abv_commander

Command line tool for sending commands to the ABV GNC stack.

## Purpose

The `abv_commander` package can be used to send various commands to the ABV GNC stack. It is a command line tool where a user can input the desired fields for pose and/or velocity waypoints, as well as sending a desired path for the ABV.

## Available Commands

After running the package, a list of available commands can be displayed by entering `help` into the command line prompt. The list of available commands is also outlined below

- `pose` - sends a desired x, y, yaw pose to `abv_goal_manager`
- `vel` - sends a desired x, y, yaw velocity to `abv_goal_manager` (world frame)
- `stop` - sends a `STOP` command, so `abv_goal_manager` zeroes the thrusters and clears the current goal
- `path` - sends a path command (type and duration) to `abv_guidance`. Only `file` is usable from here; `line` and `avoid` need a goal pose, which the CLI doesn't prompt for

Pose and velocity commands only move the vehicle once a control policy is active — see [`abv_policy_registry`](../gnc/abv_policy_registry.md).

## Topics

### Published

- `/abv/controller/command`  
  `pose`, `vel`, and `stop` commands, to `abv_goal_manager`.

- `/abv/guidance/command`  
  `path` commands, to `abv_guidance`.
