# Installation

## System Requirements

The ABV software stack is primarily developed and tested on:

- Ubuntu 22.04
- NVIDIA Jetson Orin (ARM64)

The stack may build on other Ubuntu 22.04 systems, but hardware-specific components (e.g., GPIO drivers) are intended for Jetson platforms.

---

## Prerequisites

Before installing, ensure the following:

- Active internet connection
- `git` is installed

If `git` is not installed:

```bash
sudo apt update
sudo apt install git
```

## Clone the Repository

Clone the main repo

```bash
git clone https://github.com/samlovelace/abv_gnc.git
cd abv_gnc
```

The `setup.sh` script performs a full environment setup, including:

- Installing ROS 2 Humble
- Installing required system dependencies
- Building the workspace
- Configuring the environment

To run the setup script:

```bash
./scripts/setup.sh
```

## Verify Installation

After installation, source the workspace:

```bash
source install/setup.bash
```

You can verify the installation by checking that the packages are discoverable:

```bash
ros2 pkg list | grep abv_
```

If setup worked properly, you should see the following packages in the output:

- `abv_bridge`
- `abv_bringup`
- `abv_commander`
- `abv_common`
- `abv_controller`
- `abv_description`
- `abv_gui`
- `abv_guidance`
- `abv_msgs`
- `abv_navigation`
- `abv_rl`
- `abv_rl-cpp`
- `abv_simulator`
- `abv_teleop`
