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
            apt update && apt install -y "$PKG"
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
    make install
}

LIBS_DIR=~/libs

# List of required packages
DEPENDENCIES=(
    build-essential
    cmake
    git
    curl
    libeigen3-dev
    libyaml-cpp-dev
    libsfml-dev
    libboost-system-dev 
    libboost-thread-dev
)

for pkg in "${DEPENDENCIES[@]}"; do
    check_and_install "$pkg"
done

install_from_source plog SergiusTheBest/plog 1.1.10 $LIBS_DIR
install_from_source libmotioncapture samlovelace/libmotioncapture main "$LIBS_DIR" 

# custom steps for JETGPIO 
clone_and_checkout JETGPIO Rubberazer/JETGPIO v1.2 "$LIBS_DIR"
echo "orinagx" > hardware
make && make install
# end JETGPIO custom 

# TODO: ROS2 custom install function 
