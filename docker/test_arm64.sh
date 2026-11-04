#!/usr/bin/env bash
##########################################################################
# Autobot - ARM64 Docker Build Test
#
# Tests the Autobot setup script inside an emulated ARM64 container.
# Intended for development and architecture compatibility testing.
#
# Usage:
#   ./docker/test_arm64.sh
#
# Can be run from any directory.
##########################################################################

set -euo pipefail

REPO_ROOT="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
SCRIPT_DIR="$REPO_ROOT/docker"

IMAGE_NAME="autobot-arm64"

echo "==============================================="
echo "       Autobot ARM64 Setup & Build Test"
echo "==============================================="

# Verify ARM64 emulation
if ! docker run --rm --platform linux/arm64 alpine uname -m 2>/dev/null | grep -q aarch64; then
    echo "[ARM64] Installing QEMU support..."
    docker run --privileged --rm tonistiigi/binfmt --install arm64
fi

# Build ARM64 Docker image
echo "[Docker] Building ARM64 image..."

docker build \
    --platform linux/arm64 \
    --load \
    -t "$IMAGE_NAME" \
    -f "$SCRIPT_DIR/dockerfile-arm64" \
    "$REPO_ROOT"

# Run setup inside ARM64 container
echo "[Docker] Running ARM64 setup..."

# Only allocate a TTY when there is one, so this also works in CI
TTY_ARGS=()
if [ -t 0 ] && [ -t 1 ]; then
    TTY_ARGS=(-it)
fi

docker run \
    --rm \
    --platform linux/arm64 \
    "${TTY_ARGS[@]}" \
    -v "$REPO_ROOT":/autobot:ro \
    -v autobot-arm64-build:/autobot-build \
    --entrypoint /bin/bash \
    "$IMAGE_NAME" \
    -c '
        set -e

        echo "[ARM64] Architecture: $(uname -m)"

        echo "[ARM64] Copying repository..."
        cp -a /autobot/. /autobot-build/

        cd /autobot-build

        echo "[ARM64] Running setup..."
        bash scripts/setup.sh
    '
echo
echo "[✔] ARM64 build test completed successfully."