#!/bin/bash
##########################################################################
# ABV GNC - Environment Setup
#
# Installs ROS2 Humble, system dependencies, and third-party libraries
# required to build and run the ABV GNC stack.
#
# Usage:
#   chmod +x setup.sh
#   ./setup.sh
#
# Assumptions:
#   - Ubuntu 22.04 (Jammy) on x86_64 or ARM64 (Jetson Orin)
#   - Internet access for apt and git
#   - sudo privileges
#
# What this script does:
#   1. Installs ROS2 Humble Desktop
#   2. Installs system dependencies via apt
#   3. Builds and installs third-party libs from source (plog, libmotioncapture, JETGPIO)
#   4. Installs librealsense2 (apt on x86_64, source+RSUSB on Jetson/aarch64)
#      and clones realsense-ros into the workspace so colcon builds it
#   5. Builds the ABV ROS2 packages via colcon
#
# Third-party libs are cloned to ~/libs by default (see LIBS_DIR)
##########################################################################

# where to clone libs that need built from source
LIBS_DIR=~/libs

# verify these against https://github.com/IntelRealSense/librealsense/releases
# and https://github.com/IntelRealSense/realsense-ros/releases before relying
# on them long-term - pinned here for a reproducible install, not guaranteed
# to stay the right choice as new releases come out
REALSENSE_SDK_TAG=v2.59.1
REALSENSE_ROS_TAG=4.59.1

# List of required system packages
DEPENDENCIES=(
    build-essential
    cmake
    curl
    vim
    tmux
    libeigen3-dev
    libyaml-cpp-dev
    libsfml-dev
    libboost-system-dev
    libboost-thread-dev
    qt6-base-dev
    libqt6charts6-dev
)

# Function to check and install a package
check_and_install() {
    PKG="$1"
    CUSTOM_INSTALL_FUNC="$2"

    if dpkg -s "$PKG" >/dev/null 2>&1; then
        echo "[✔] $PKG is already installed."
    else
        echo "[✘] $PKG is not installed."

        if [ -n "$CUSTOM_INSTALL_FUNC" ] && declare -f "$CUSTOM_INSTALL_FUNC" > /dev/null; then
            echo "[↪] Using custom install function: $CUSTOM_INSTALL_FUNC"
            "$CUSTOM_INSTALL_FUNC"
        else
            echo "[↪] Installing $PKG via apt..."
            sudo apt update && sudo apt install -y "$PKG"
        fi
    fi
}

clone_and_checkout() {
    LIB="$1"
    URL="$2"
    TAG="$3"
    DIR="$4"

    # Clone if missing
    if [ ! -d "$DIR/$LIB" ]; then
        git clone https://github.com/"$URL".git "$DIR/$LIB"
    fi

    cd "$DIR/$LIB"
    git checkout "$TAG"
}

install_from_source() {
    LIB="$1"
    URL="$2"
    TAG="$3"
    DIR="$4"
    shift 4
    CMAKE_ARGS=("$@")

    clone_and_checkout $LIB $URL $TAG $DIR

    # Always update submodules (safe even if none exist)
    echo "[↪] Updating submodules for $LIB..."
    git submodule update --init --recursive

    mkdir -p build
    cd build

    echo "[↪] Running CMake for $LIB with args: ${CMAKE_ARGS[*]}"

    cmake -DCMAKE_BUILD_TYPE=Release "${CMAKE_ARGS[@]}" ..
    make -j"$(nproc)"
    sudo make install
}

install_librealsense() {
    ARCH="$(uname -m)"

    if [ "$ARCH" = "aarch64" ]; then
        # Jetson's L4T kernel isn't one of the Ubuntu HWE kernels
        # librealsense2-dkms supports, so the prebuilt kernel module fails to
        # build there. The documented workaround is the RSUSB backend, which
        # talks to the camera over USB directly instead of a kernel module -
        # this skips the dkms apt package entirely and builds librealsense2
        # from source with that backend forced on.
        echo "[realsense] aarch64 (Jetson) detected - building librealsense2 from source with RSUSB backend..."
        install_from_source librealsense IntelRealSense/librealsense "$REALSENSE_SDK_TAG" "$LIBS_DIR" \
            "-DFORCE_RSUSB_BACKEND=ON" \
            "-DBUILD_EXAMPLES=OFF" \
            "-DBUILD_GRAPHICAL_EXAMPLES=OFF"
    else
        echo "[realsense] Installing librealsense2 via apt..."
        sudo mkdir -p /etc/apt/keyrings
        curl -sSf https://librealsense.realsenseai.com/Debian/librealsenseai.asc \
            | gpg --dearmor | sudo tee /etc/apt/keyrings/librealsenseai.gpg >/dev/null
        sudo apt-get install -y apt-transport-https
        echo "deb [signed-by=/etc/apt/keyrings/librealsenseai.gpg] https://librealsense.realsenseai.com/Debian/apt-repo $(lsb_release -cs) main" \
            | sudo tee /etc/apt/sources.list.d/librealsense.list
        sudo apt update
        sudo apt-get install -y librealsense2-dkms librealsense2-utils librealsense2-dev
    fi
}

install_ros() {
    echo "[ROS] Starting installation of ROS 2 Humble..."

    # --- Configure UTF-8 Locale ---
    if ! locale | grep -q "UTF-8"; then
        echo "[ROS] Configuring UTF-8 locale..."
        sudo apt install -y locales
        locale-gen en_US en_US.UTF-8
        update-locale LC_ALL=en_US.UTF-8 LANG=en_US.UTF-8
        export LANG=en_US.UTF-8
    fi

    # --- Set Timezone Non-Interactively ---
    echo "[ROS] Setting timezone to America/New_York..."
    ln -fs /usr/share/zoneinfo/America/New_York /etc/localtime
    DEBIAN_FRONTEND=noninteractive sudo apt install -y tzdata
    dpkg-reconfigure -f noninteractive tzdata

    # --- Add Required Tools ---
    DEBIAN_FRONTEND=noninteractive sudo apt install -y software-properties-common curl gnupg lsb-release

    # --- Add Universe Repo (auto-confirm) ---
    echo "[ROS] Adding universe repository..."
    add-apt-repository -y universe

    # --- Add ROS 2 GPG Key ---
    echo "[ROS] Adding ROS 2 GPG key..."
    curl -sSL https://raw.githubusercontent.com/ros/rosdistro/master/ros.key \
        -o /usr/share/keyrings/ros-archive-keyring.gpg

    # --- Add ROS 2 Repository ---
    echo "[ROS] Adding ROS 2 repository..."
    echo "deb [arch=$(dpkg --print-architecture) signed-by=/usr/share/keyrings/ros-archive-keyring.gpg] \
          http://packages.ros.org/ros2/ubuntu $(lsb_release -cs) main" \
          > /etc/apt/sources.list.d/ros2.list

    # --- Install ROS 2 Humble and Tools ---
    echo "[ROS] Installing ROS 2 Humble and tools..."
    sudo apt update
    DEBIAN_FRONTEND=noninteractive sudo apt install -y ros-humble-desktop \
        ros-humble-xacro
        python3-colcon-common-extensions \
        python3-rosdep \
        python3-vcstool

    # --- Source ROS Environment ---
    echo "source /opt/ros/humble/setup.bash" >> ~/.bashrc
    source /opt/ros/humble/setup.bash
    echo "[✔] ROS 2 Humble installed and environment sourced."
}

###########################################################################
## Script run logic
###########################################################################

# get current dir so we can come back at the end to build the packages
REPO_ROOT="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"

# make sure things are updated
sudo apt update

# treat ros2 install special
check_and_install "ros-humble-desktop" "install_ros"

# install system deps
for pkg in "${DEPENDENCIES[@]}"; do
    check_and_install "$pkg"
done

# install deps from source
install_from_source plog SergiusTheBest/plog 1.1.10 $LIBS_DIR
install_from_source libmotioncapture samlovelace/libmotioncapture main "$LIBS_DIR" \
    "-DCMAKE_INSTALL_PREFIX=/usr/local"

# librealsense2 SDK - apt on x86_64, source+RSUSB on Jetson/aarch64 (see
# install_librealsense). Not gated behind check_and_install: the source-build
# path doesn't register as a dpkg package, so that guard would never
# recognize it as installed on Jetson - same reasoning as plog/
# libmotioncapture above, which are unconditionally re-run every time too.
install_librealsense

# custom steps for JETGPIO
clone_and_checkout JETGPIO Rubberazer/JETGPIO v1.2 "$LIBS_DIR"
echo "orinagx" > hardware
make && sudo make install
# end JETGPIO custom

# robot_ws setup
ROBOT_WS=~/robot_ws
mkdir -p "$ROBOT_WS"/src
clone_and_checkout ptera samlovelace/ptera main "$ROBOT_WS"/src

# realsense-ros (the realsense2_camera node that autobot_camera_realsense's
# launch/camera.launch.py wraps) - cloned as source rather than apt-installed
# so it builds the same way on both x86_64 and Jetson/aarch64, since Intel
# only publishes prebuilt ros-humble-realsense2-camera debs for x86_64
clone_and_checkout realsense-ros IntelRealSense/realsense-ros "$REALSENSE_ROS_TAG" "$ROBOT_WS"/src

# resolve ROS package.xml dependencies (realsense-ros needs cv_bridge,
# diagnostic_updater, etc.) for everything now sitting in ROBOT_WS/src
source /opt/ros/humble/setup.bash
if [ ! -f /etc/ros/rosdep/sources.list.d/20-default.list ]; then
    sudo rosdep init
fi
rosdep update
rosdep install --from-paths "$ROBOT_WS"/src --ignore-src -r -y

cd "$ROBOT_WS" && colcon build --packages-skip ptera_sim

# build the main workspace
cd "$REPO_ROOT"
source /opt/ros/humble/setup.bash
source "$ROBOT_WS"/install/setup.bash
colcon build
