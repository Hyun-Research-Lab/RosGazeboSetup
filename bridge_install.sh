#!/usr/bin/bash
set -e
if [ $EUID != 0 ]; then
    sudo "$0" "$@"
    exit $?
fi

if [ -z "$1" ]; then
    echo "Base path not provided"
    exit 1
fi

echo "Installing ROS2 Ignition Gazebo Bridge"
# https://github.com/gazebosim/ros_gz/tree/humble

#Assuming ignition gazebo fortress is installed and ros2 humble is installed

export GZ_VERSION=fortress

# Setup the workspace
mkdir -p $1/ros_ign_gz_bridge/src
pushd $1/ros_ign_gz_bridge/src

# Download needed software
git clone https://github.com/gazebosim/ros_gz.git -b humble


#Install dependencies
popd
pushd $1/ros_ign_gz_bridge
rosdep install -r --from-paths src -i -y --rosdistro humble

# Source ROS distro's setup.bash
source /opt/ros/humble/setup.bash

sudo apt-get update  #resolves missing dependencies error
sudo apt-get upgrade -y

# Build and install into workspace
popd
pushd $1/ros_ign_gz_bridge
colcon build
popd

echo "Finished"