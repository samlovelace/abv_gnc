#!/bin/bash
##########################################################################
# Utilizes the dockerfile file to run the setup script. 
# Primarily a development tool, not meant to be used for running the 
# software in a production sense. 
#
# Usage:
#   ./docker/run_amd64.sh
#
# Must be run from the workspace root.
##########################################################################
set -e

REPO_ROOT="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"

docker build  -t autobot-amd64 -f docker/dockerfile .
docker run -it \
    -v "$REPO_ROOT":/autobot \
    autobot-amd64 \
    bash /autobot/scripts/setup.sh