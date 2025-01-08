#!/bin/bash

# Set to exit on error
set -e

read -p 'Team number used during installation: ' TEAM_NUMBER
if ! [[ "$TEAM_NUMBER" =~ ^[0-9]+$ ]]
    then
        echo "Please enter an integer for team number"
        exit 1
fi

# Constants to be updated
ROS_VERSION=jazzy

# Unset environment variables
echo "Removing ROS-related environment variables from ~/.bashrc"
sed -i '/export ROS_DOMAIN_ID/d' ~/.bashrc
sed -i "/source \/opt\/ros\/$ROS_VERSION\/setup.bash/d" ~/.bashrc
sed -i '/export PIP_BREAK_SYSTEM_PACKAGES=1/d' ~/.bashrc

# Remove ROS packages and dependencies
echo "Uninstalling ROS and related dependencies"
sudo apt purge -y ros-$ROS_VERSION-desktop python3-rosdep python3-colcon-common-extensions python3-pip
sudo apt autoremove -y
sudo apt clean

# Remove ROS keyring and repository
echo "Removing ROS keyring and repository"
sudo rm -f /usr/share/keyrings/ros-archive-keyring.gpg
sudo rm -f /etc/apt/sources.list.d/ros2.list
sudo apt update

# Uninstall build-essential and git
echo "Uninstalling build-essential and git"
sudo apt purge -y build-essential git
sudo apt autoremove -y
sudo apt clean

# Uninstall curl
echo "Uninstalling curl"
sudo apt purge -y curl
sudo apt autoremove -y
sudo apt clean

# uninstall this hidden file
sudo rm -rf /opt/ros

# Final cleanup
echo "Performing final cleanup"
sudo apt autoremove -y
sudo apt clean

echo "Uninstallation finished!"
