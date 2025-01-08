#!/bin/bash

# Set to exit on error
set -e

read -p 'Team number: ' TEAM_NUMBER
if ! [[ "$TEAM_NUMBER" =~ ^[0-9]+$ ]]
    then
        echo "Please enter integer for team number"
fi
TEAM_NUMBER=$((TEAM_NUMBER))

# Constants to be updated
ROS_VERSION=jazzy

# Update and upgrade
sudo apt install -y software-properties-common
sudo add-apt-repository universe -y
sudo apt update && sudo apt upgrade -y

# Install build-essential
sudo apt install -y build-essential

# install Git
sudo apt install -y git

# Setup ROS 2 and dependencies
sudo apt install -y curl
sudo curl -sSL https://raw.githubusercontent.com/ros/rosdistro/master/ros.key -o /usr/share/keyrings/ros-archive-keyring.gpg
echo "deb [arch=$(dpkg --print-architecture) signed-by=/usr/share/keyrings/ros-archive-keyring.gpg] http://packages.ros.org/ros2/ubuntu $(. /etc/os-release && echo $UBUNTU_CODENAME) main" | sudo tee /etc/apt/sources.list.d/ros2.list > /dev/null
sudo apt update
sudo apt install -y ros-$ROS_VERSION-desktop python3-rosdep python3-colcon-common-extensions python3-pip
sudo rosdep init
rosdep update
echo "export ROS_DOMAIN_ID=$TEAM_NUMBER" >> ~/.bashrc
echo "source /opt/ros/$ROS_VERSION/setup.bash" >> ~/.bashrc
echo "export PIP_BREAK_SYSTEM_PACKAGES=1" >> ~/.bashrc
source ~/.bashrc

echo "Installation finished!"
