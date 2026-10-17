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

The `setup.sh` script performs a full environment setup. It is safe to re-run (steps that are already done are skipped) and works either as a normal user with `sudo` or as root inside a container. It:

- Configures a UTF-8 locale and the ROS 2 apt repository, then installs ROS 2 Humble (`ros-humble-desktop`, `xacro`, `rosbridge-suite`), colcon, rosdep, and the system dependencies (Eigen, yaml-cpp, SFML, Boost, Qt 6 + Qt Charts)
- Adds `source /opt/ros/humble/setup.bash` to `~/.bashrc` and initializes/updates `rosdep`
- Builds third-party libraries from source into `~/libs`: `plog`, `libmotioncapture`, and — on `aarch64` (Jetson) only — `JETGPIO`
- Clones and builds the `ptera` underlay workspace into `~/robot_ws` (skipping `ptera_sim`), which provides the `ptera_msgs` used by `abv_bridge`
- Runs `rosdep install` and builds this repo as an overlay with `colcon build --symlink-install -DCMAKE_BUILD_TYPE=RelWithDebInfo`

To run the setup script:

```bash
./scripts/setup.sh
```

### ONNX Runtime (required for `abv_rl-cpp`)

`setup.sh` does **not** install ONNX Runtime, but `abv_rl-cpp` will not build without it, so a full `colcon build` fails on a clean machine until it's installed. Install ONNX Runtime (system-wide under `/usr/local`, or anywhere and pass `--cmake-args -DORT_ROOT=<path>`), or skip the package with `colcon build --packages-skip abv_rl-cpp` if you don't need the RL policy. On the Jetson, `jetson.launch.py` expects ONNX Runtime under `/home/optimus/onnxruntime` and the CUDA 12.6 libraries under `/usr/local/cuda-12.6`. See [`abv_rl-cpp`](../packages/gnc/abv_rl.md#build-requirements).

The Python `abv_rl` package additionally expects a conda environment named `safetyrl-env` (see `src/abv_rl/abv_rl/environment.yml`); it is only needed for training and ONNX export.

## Docker (development)

The `docker/` folder holds development helpers for testing the setup script in a clean environment. All of them must be run from the repo root.

| Script | What it does |
|---|---|
| `docker/build_amd64.sh` | Builds the `abv_gnc-amd64` image from `docker/dockerfile` (based on `osrf/ros:humble-desktop`) and runs `scripts/setup.sh` inside it, with the repo mounted at `/abv_gnc` |
| `docker/run.sh` | Starts an interactive `abv_gnc-amd64` container with host networking, `--privileged`, `/dev`, and X11 forwarding (for `abv_gui`), with the repo mounted at `/abv_gnc` |
| `docker/test_arm64.sh` | Installs QEMU arm64 emulation if needed, then builds an `abv_gnc-arm64` image and runs `scripts/setup.sh` in it, to check the Jetson setup path. **Deletes the repo's `build/` and `install/` folders first** |
| `docker/rocky-linux/setup.sh <image.tar>` | For Rocky Linux hosts: installs Docker CE and X11 utilities, adds you to the `docker` group, and loads a saved image tarball. Afterwards, run `./docker/run.sh` |

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
- `abv_control`
- `abv_description`
- `abv_goal_manager`
- `abv_gui`
- `abv_guidance`
- `abv_msgs`
- `abv_navigation`
- `abv_policy_registry`
- `abv_rl`
- `abv_rl-cpp`
- `abv_simulator`
- `abv_teleop`
- `abv_thrusters`
