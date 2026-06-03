#!/bin/bash
##########################################################################
# Setup script for running abv_gnc on Rocky Linux
# Installs Docker CE and X11 forwarding dependencies
#
# Usage:
#   ./setup.sh <path_to_image.tar>
#
# Must be run from the workspace root.
##########################################################################
set -e

if [ -z "$1" ]; then
    echo "Usage: ./setup.sh <path_to_image.tar>"
    exit 1
fi

IMAGE_TAR="$1"

if [ ! -f "$IMAGE_TAR" ]; then
    echo "Error: image file not found at $IMAGE_TAR"
    exit 1
fi

echo "==> Adding Docker CE repository..."
sudo dnf config-manager --add-repo https://download.docker.com/linux/rhel/docker-ce.repo

echo "==> Installing Docker CE..."
sudo dnf install -y docker-ce docker-ce-cli containerd.io docker-compose-plugin

echo "==> Starting and enabling Docker service..."
sudo systemctl enable --now docker

echo "==> Adding $USER to docker group..."
sudo usermod -aG docker $USER

echo "==> Installing X11 utilities..."
sudo dnf install -y xorg-x11-server-utils

echo "==> Loading Docker image..."
sudo docker load -i "$IMAGE_TAR"

echo "==> Setup complete!"
echo "    Open a new terminal for docker group changes to take effect,"
echo "    then run ./docker/run.sh to start the container."