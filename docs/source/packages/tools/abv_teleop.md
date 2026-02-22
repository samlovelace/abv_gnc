# abv_teleop

Remote teleop commander to manually drive the ABV.

## Purpose

The `abv_teleop` package is responsible for taking user inputs from the configured device (arrow keys, game controller) and converting that to the corresponding thrust vector sent to the ABV.

## Usage

Run the `abv_teleop` package from the `abv_gnc` folder

```bash
sudo ./run.sh abv_teleop
```

After running the program, using the computer keyboard is enabled by default. The mapping of the keys to command is shown below. Note, all commands are in the ABV body frame.

- `arrow up` +x
- `arrow down` -x
- `arrow right` +y
- `arrow left` -y
- `e` +yaw
- `q` -yaw

## Topics

### Published

- `/abv/command` - publishes the desired thrust vector to the ABV
