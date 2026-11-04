#!/bin/bash
##########################################################################
# Autobot - Environment Setup
#
# Installs ROS2 Humble, system dependencies, Node.js, and third-party
# libraries required to build and run the Autobot robotics stack.
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
#   3. Installs Node.js 22 LTS and npm for the Vite web GUI
#   4. Builds and installs third-party libraries from source
#   5. Installs librealsense2 (apt on x86_64, source+RSUSB on Jetson)
#   6. Clones realsense-ros into the workspace
#   7. Builds the ROS2 packages via colcon (including the web GUI)
#
# Third-party libraries are cloned to ~/libs by default.
##########################################################################

# Fail on the first error so callers (e.g. docker/test_arm64.sh) see it.
# No -u: ROS setup.bash reads unset variables.
set -eo pipefail

# Where to clone libraries that need to be built from source
LIBS_DIR=~/libs

# Pinned RealSense versions
REALSENSE_SDK_TAG=v2.59.1
REALSENSE_ROS_TAG=4.59.1

# Node.js major version
NODEJS_MAJOR=22

# Required system packages
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

###########################################################################
## Package Installation
###########################################################################

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

###########################################################################
## Node.js Installation
###########################################################################

install_nodejs() {
    echo "[Node.js] Checking Node.js installation..."

    local CURRENT_MAJOR=0

    if command -v node >/dev/null 2>&1; then
        CURRENT_MAJOR=$(node -p "process.versions.node.split('.')[0]")
    fi

    if [ "$CURRENT_MAJOR" -ge "$NODEJS_MAJOR" ] && command -v npm >/dev/null 2>&1; then
        echo "[✔] Node.js $(node --version) already installed."
        echo "[✔] npm $(npm --version) already installed."
        return
    fi

    echo "[Node.js] Installing Node.js ${NODEJS_MAJOR}..."

    # Configure NodeSource repository
    curl -fsSL "https://deb.nodesource.com/setup_${NODEJS_MAJOR}.x" | sudo -E bash -

    # Install Node.js (includes npm)
    sudo apt install -y nodejs

    # Verify installation
    if ! command -v node >/dev/null 2>&1 || ! command -v npm >/dev/null 2>&1; then
        echo "[✘] Node.js/npm installation failed."
        exit 1
    fi

    echo "[✔] Node.js $(node --version)"
    echo "[✔] npm $(npm --version)"
}

###########################################################################
## Source Library Installation
###########################################################################

clone_and_checkout() {
    LIB="$1"
    URL="$2"
    TAG="$3"
    DIR="$4"

    # Clone if missing
    if [ ! -d "$DIR/$LIB" ]; then
        git clone "https://github.com/$URL.git" "$DIR/$LIB"
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

    clone_and_checkout "$LIB" "$URL" "$TAG" "$DIR"

    # Update submodules
    echo "[↪] Updating submodules for $LIB..."
    git submodule update --init --recursive

    mkdir -p build
    cd build

    echo "[↪] Running CMake for $LIB with args: ${CMAKE_ARGS[*]}"

    cmake -DCMAKE_BUILD_TYPE=Release "${CMAKE_ARGS[@]}" ..
    make -j"$(nproc)"
    sudo make install
}

###########################################################################
## RealSense Installation
###########################################################################

install_librealsense() {
    ARCH="$(uname -m)"

    if [ "$ARCH" = "aarch64" ]; then

        # Jetson L4T kernels are not supported by librealsense2-dkms.
        # Build from source using the RSUSB backend instead.

        echo "[RealSense] ARM64 detected - building with RSUSB backend..."

        install_from_source \
            librealsense \
            IntelRealSense/librealsense \
            "$REALSENSE_SDK_TAG" \
            "$LIBS_DIR" \
            "-DFORCE_RSUSB_BACKEND=ON" \
            "-DBUILD_EXAMPLES=OFF" \
            "-DBUILD_GRAPHICAL_EXAMPLES=OFF"

    else

        echo "[RealSense] Installing librealsense2 via apt..."

        sudo mkdir -p /etc/apt/keyrings

        curl -sSf https://librealsense.realsenseai.com/Debian/librealsenseai.asc \
            | gpg --dearmor \
            | sudo tee /etc/apt/keyrings/librealsenseai.gpg >/dev/null

        sudo apt-get install -y apt-transport-https

        echo "deb [signed-by=/etc/apt/keyrings/librealsenseai.gpg] https://librealsense.realsenseai.com/Debian/apt-repo $(lsb_release -cs) main" \
            | sudo tee /etc/apt/sources.list.d/librealsense.list

        sudo apt update

        sudo apt-get install -y \
            librealsense2-dkms \
            librealsense2-utils \
            librealsense2-dev
    fi
}

###########################################################################
## ROS 2 Installation
###########################################################################

install_ros() {
    echo "[ROS] Starting installation of ROS 2 Humble..."

    # Configure UTF-8 locale
    if ! locale | grep -q "UTF-8"; then
        echo "[ROS] Configuring UTF-8 locale..."

        sudo apt install -y locales
        sudo locale-gen en_US en_US.UTF-8
        sudo update-locale LC_ALL=en_US.UTF-8 LANG=en_US.UTF-8

        export LANG=en_US.UTF-8
    fi

    # Configure timezone
    echo "[ROS] Setting timezone to America/New_York..."

    sudo ln -fs /usr/share/zoneinfo/America/New_York /etc/localtime

    DEBIAN_FRONTEND=noninteractive sudo apt install -y tzdata
    sudo dpkg-reconfigure -f noninteractive tzdata

    # Install repository tools
    DEBIAN_FRONTEND=noninteractive sudo apt install -y \
        software-properties-common \
        curl \
        gnupg \
        lsb-release

    # Enable Universe repository
    echo "[ROS] Adding universe repository..."
    sudo add-apt-repository -y universe

    # Add ROS 2 GPG key and repository, unless already configured.
    # A second entry with a different Signed-By (e.g. alongside the
    # ros2-apt-source package) makes apt refuse to read its sources.
    if grep -Rqs "packages.ros.org/ros2" /etc/apt/sources.list /etc/apt/sources.list.d/; then
        echo "[ROS] ROS 2 repository already configured."
    else
        echo "[ROS] Adding ROS 2 GPG key..."

        curl -sSL \
            https://raw.githubusercontent.com/ros/rosdistro/master/ros.key \
            | sudo tee /usr/share/keyrings/ros-archive-keyring.gpg >/dev/null

        echo "[ROS] Adding ROS 2 repository..."

        echo "deb [arch=$(dpkg --print-architecture) signed-by=/usr/share/keyrings/ros-archive-keyring.gpg] \
http://packages.ros.org/ros2/ubuntu $(lsb_release -cs) main" \
            | sudo tee /etc/apt/sources.list.d/ros2.list
    fi

    # Install ROS 2 and development tools
    echo "[ROS] Installing ROS 2 Humble and tools..."

    sudo apt update

    DEBIAN_FRONTEND=noninteractive sudo apt install -y \
        ros-humble-desktop \
        ros-humble-xacro \
        ros-humble-rosbridge-suite \
        python3-colcon-common-extensions \
        python3-rosdep \
        python3-vcstool

    # Configure shell environment
    if ! grep -Fxq "source /opt/ros/humble/setup.bash" ~/.bashrc; then
        echo "source /opt/ros/humble/setup.bash" >> ~/.bashrc
    fi

    source /opt/ros/humble/setup.bash

    echo "[✔] ROS 2 Humble installed."
}

###########################################################################
## Main Setup
###########################################################################

# Determine repository root
REPO_ROOT="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"

echo "========================================"
echo "       Autobot Environment Setup"
echo "========================================"
echo "Repository: $REPO_ROOT"
echo

# Update package lists
sudo apt update

###########################################################################
## Install ROS 2
###########################################################################

check_and_install "ros-humble-desktop" "install_ros"

###########################################################################
## Install System Dependencies
###########################################################################

echo "[System] Installing dependencies..."

for pkg in "${DEPENDENCIES[@]}"; do
    check_and_install "$pkg" ""
done

###########################################################################
## Install Node.js and npm
###########################################################################

# Must be installed before colcon builds autobot_bringup,
# which invokes npm to compile the Vite frontend.

install_nodejs

###########################################################################
## Install Third-Party Libraries
###########################################################################

echo "[Libraries] Installing third-party libraries..."

install_from_source \
    plog \
    SergiusTheBest/plog \
    1.1.10 \
    "$LIBS_DIR"

install_from_source \
    libmotioncapture \
    samlovelace/libmotioncapture \
    main \
    "$LIBS_DIR" \
    "-DCMAKE_INSTALL_PREFIX=/usr/local"

###########################################################################
## Install RealSense
###########################################################################

install_librealsense

###########################################################################
## Install JETGPIO
###########################################################################

clone_and_checkout \
    JETGPIO \
    Rubberazer/JETGPIO \
    v1.2 \
    "$LIBS_DIR"

echo "orinagx" > hardware

make -j"$(nproc)"
sudo make install

###########################################################################
## Set Up Supporting ROS 2 Workspace
###########################################################################

ROBOT_WS=~/robot_ws

mkdir -p "$ROBOT_WS/src"

# Ptera
clone_and_checkout \
    ptera \
    samlovelace/ptera \
    main \
    "$ROBOT_WS/src"

# RealSense ROS 2 wrapper
clone_and_checkout \
    realsense-ros \
    IntelRealSense/realsense-ros \
    "$REALSENSE_ROS_TAG" \
    "$ROBOT_WS/src"

###########################################################################
## Resolve ROS Dependencies
###########################################################################

source /opt/ros/humble/setup.bash

if [ ! -f /etc/ros/rosdep/sources.list.d/20-default.list ]; then
    sudo rosdep init
fi

rosdep update

rosdep install \
    --from-paths "$ROBOT_WS/src" \
    --ignore-src \
    -r \
    -y

###########################################################################
## Build Supporting Workspace
###########################################################################

echo "[ROS] Building supporting workspace..."

cd "$ROBOT_WS"

colcon build --packages-skip ptera_sim

###########################################################################
## Build Autobot Workspace
###########################################################################

echo "[Autobot] Building main workspace..."

cd "$REPO_ROOT"

source /opt/ros/humble/setup.bash
source "$ROBOT_WS/install/setup.bash"

# autobot_bringup CMake handles npm ci and npm run build.
colcon build

echo
echo "========================================"
echo "       Autobot Setup Complete!"
echo "========================================"
echo
echo "Node.js: $(node --version)"
echo "npm:     $(npm --version)"
echo
