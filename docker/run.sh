#!/bin/bash
##########################################################################
# runs a container from the abv_gnc-amd64 image, mounting the workspace 
# Primarily a development tool, not meant to be used for running the 
# software in a production sense. 
#
# Usage:
#   ./docker/run.sh
#
# Must be run from the workspace root.
##########################################################################
set -e

REPO_ROOT="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"

# allow local docker containers to connect to X server
xhost +local:docker

docker run -it \
  --network host \
  --privileged \
  -e DISPLAY=$DISPLAY \
  -e QT_X11_NO_MITSHM=1 \
  -v /tmp/.X11-unix:/tmp/.X11-unix \
  -v /dev:/dev \
  -v "$REPO_ROOT":/abv_gnc \
  abv_gnc-amd64:latest \
  bash