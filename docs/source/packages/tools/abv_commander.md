# abv_commander

Command line tool for sending commands to the ABV GNC stack.

## Purpose

The `abv_commander` package can be used to send various commands to the ABV GNC stack. It is a command line tool where a user can input the desired fields for pose and/or velocity waypoints, as well as sending a desired path for the ABV.

## Available Commands

After running the package, a list of available commands can be displayed by entering `help` into the command line prompt. The list of available commands is also outlined below

- `pose` - sends a desired x, y, yaw pose to the `abv_controller` module
- `vel` - sends a desired x, y, yaw velocity to the `abv_controller` module
- `stop` - sends a [0, 0, 0] thrust vector so that the `abv_controller` will stop firing the thrusters
- `path` - sends a desired path command to the `abv_guidance` module

## Topics

### Published

- `/abv/command`  
  Sends commands to `abv_controller` or `abv_guidance`.
