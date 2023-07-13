#!/usr/bin/bash
set -e

sudo rm -rf $1/ros2_humble/
sudo rm -rf $1/.ros/
sudo rm -rf /opt/ros/
sudo rm -rf /etc/ros/

sudo rm -rf $1/ign_ws/
sudo rm -rf $1/ros_gz_bridge/

sudo rm -rf /opt/drake/
echo "Uninstalled"