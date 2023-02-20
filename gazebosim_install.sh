#!/usr/bin/bash
# https://github.com/gazebosim/ros_gz/tree/humble

#Assuming ignition gazebo fortress is installed and ros2 humble is installed

export GZ_VERSION=fortress

# Setup the workspace
mkdir -p ~/ws/src
pushd ~/ws/src

# Download needed software
git clone https://github.com/gazebosim/ros_gz.git -b humble


#Install dependencies
popd
pushd ~/ws
rosdep install -r --from-paths src -i -y --rosdistro humble

# Source ROS distro's setup.bash
source /opt/ros/humble/setup.bash

sudo apt-get update  #resolves missing dependencies error
sudo apt-get upgrade -y

# Build and install into workspace
popd
pushd ~/ws
colcon build
popd
