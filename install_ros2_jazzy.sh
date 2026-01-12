#!/bin/bash

if ! grep -q "Ubuntu" /etc/os-release 2>/dev/null; then
	echo "This script only works for Ubuntu"
	exit 1
fi
ubuntu_v=$(lsb_release -rs)
echo "Detected Ubuntu $ubuntu_v"

if [[ "$ubuntu_version" < "Ubuntu 22.04" ]]; then
    echo "Ubuntu 22.04 or newer is needed"
fi


echo "Installing Dependencies"
sudo apt install -y software-properties-common
sudo add-apt-repository universe -y
sudo apt update && sudo apt install -y curl gnupg lsb-release
sudo curl -sSL https://raw.githubusercontent.com/ros/rosdistro/master/ros.key -o /usr/share/keyrings/ros-archive-keyring.gpg
echo "deb [arch=$(dpkg --print-architecture) signed-by=/usr/share/keyrings/ros-archive-keyring.gpg] http://packages.ros.org/ros2/ubuntu $(. /etc/os-release && echo $UBUNTU_CODENAME) main" | sudo tee /etc/apt/sources.list.d/ros2.list > /dev/null
sudo apt update

echo "Installing ROS2 Jazzy"
sudo apt install -y ros-jazzy-desktop

echo "Installing ROS2 Tools"
sudo apt install -y \
        ros-dev-tools \
        ros-jazzy-ros2-control \
        ros-jazzy-ros2-controllers \
        ros-jazzy-controller-manager \
        ros-jazzy-joint-state-broadcaster \
        ros-jazzy-joint-state-publisher-gui \
        ros-jazzy-robot-state-publisher \
        ros-jazzy-rviz2 \
        ros-jazzy-xacro \
        python3-colcon-common-extensions \
        python3-rosdep \
        python3-vcstool

sudo rosdep init
source /opt/ros/jazzy/setup.bash

echo "Finished installing ROS2 Jazzy"

