#!/bin/bash
set -e

# Install wget if not present
if ! command -v wget &> /dev/null; then
    echo "wget not found, installing..."
    sudo apt update && sudo apt install -y wget
fi

# Download and install Miniconda for ARM
wget https://repo.anaconda.com/miniconda/Miniconda3-latest-Linux-aarch64.sh
bash Miniconda3-latest-Linux-aarch64.sh

# Reload shell and disable base auto-activation
source ~/.bashrc
conda config --set auto_activate_base false

echo "########### Miniconda installed successfully ######################"
echo "########### Creating Conda Environment from yml file ##############"
conda env create -f ../abv_rl/environment.yml

echo "############ Adding ROS2 Python libs to conda env #################"
echo "/opt/ros/humble/lib/python3.10/site-packages" > \
    ~/miniconda3/envs/abv_rl/lib/python3.*/site-packages/ros2.pth

echo "########### Deactivating Conda Env before runtime ################"
conda deactivate