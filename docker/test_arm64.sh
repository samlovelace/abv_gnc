#!/bin/bash
##########################################################################
# Utilizes the dockerfile-arm64 file to test the setup script in an arm 
# emulated docker container. Primarily a development tool, not meant to be
# used for running the software in a production sense. 
#
# Usage:
#   ./docker/test_arm64.sh <package_name>
#
# Must be run from the workspace root.
##########################################################################
set -e

REPO_ROOT="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"

# remove build/install folders if testing build in container 
rm -rf "$REPO_ROOT"/build "$REPO_ROOT"/install

if ! docker run --rm --platform linux/arm64 alpine uname -m 2>/dev/null | grep -q aarch64; then
    echo "Installing QEMU arm64 support..."
    docker run --privileged --rm tonistiigi/binfmt --install arm64
fi

docker build --platform linux/arm64 -t abv_gnc-arm64 -f docker/dockerfile .
docker run --platform linux/arm64 -it \
    -v "$REPO_ROOT":/abv_gnc \
    abv_gnc-arm64 \
    bash /abv_gnc/scripts/setup.sh