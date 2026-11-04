#!/usr/bin/env bash
set -e

# Detect ROS Distro
if [ -z "$ROS_DISTRO" ]; then
    echo -e "\e[1;33m[workspace_setup] Warning: ROS_DISTRO not detected. Defaulting to 'humble'.\e[0m"
    ROS_DISTRO="humble"
else
    echo -e "\e[1;34m[workspace_setup] Detected ROS Distro: $ROS_DISTRO\e[0m"
fi

DDS_PACKAGE="ros-${ROS_DISTRO}-rmw-cyclonedds-cpp"

echo -e "\e[1;34m[workspace_setup] Starting system dependency and environment setup...\e[0m"

# 1. Install CycloneDDS if not present
if dpkg -l | grep -q "${DDS_PACKAGE}"; then
    echo -e "\e[1;32m[workspace_setup] ${DDS_PACKAGE} is already installed.\e[0m"
else
    echo -e "\e[1;33m[workspace_setup] ${DDS_PACKAGE} not found. Installing via apt-get...\e[0m"
    sudo apt-get update
    sudo apt-get install -y "${DDS_PACKAGE}"
fi

# 2. Add sourcing of this workspace to ~/.bashrc if not already there
BASHRC="$HOME/.bashrc"
SETUP_LINE="source ~/construct_ws/install/setup.bash"

# Comment out old manual exports in .bashrc to avoid conflict
if grep -q "export RMW_IMPLEMENTATION=" "$BASHRC" || grep -q "export CYCLONEDDS_URI=" "$BASHRC"; then
    echo -e "\e[1;33m[workspace_setup] Commenting out existing hardcoded DDS environment exports in ~/.bashrc to avoid conflicts...\e[0m"
    sed -i 's/^export RMW_IMPLEMENTATION=/# export RMW_IMPLEMENTATION=/g' "$BASHRC"
    sed -i 's/^export CYCLONEDDS_URI=/# export CYCLONEDDS_URI=/g' "$BASHRC"
fi

if grep -Fxq "$SETUP_LINE" "$BASHRC"; then
    echo -e "\e[1;32m[workspace_setup] Sourcing of workspace already registered in ~/.bashrc.\e[0m"
else
    echo -e "\e[1;33m[workspace_setup] Registering workspace sourcing in ~/.bashrc...\e[0m"
    echo "" >> "$BASHRC"
    echo "# ROS 2 workspace environment auto-setup" >> "$BASHRC"
    echo "$SETUP_LINE" >> "$BASHRC"
fi

echo -e "\e[1;32m[workspace_setup] Setup completed successfully!\e[0m"
echo -e "\e[1;34mPlease source ~/.bashrc or open a new terminal to apply changes.\e[0m"
