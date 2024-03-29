#!/usr/bin/bash
set -e
# https://docs.ros.org/en/humble/Installation/Alternatives/Ubuntu-Development-Setup.html

if [ $EUID != 0 ]; then
    sudo "$0" "$@"
    exit $?
fi

if [ -z "$1" ]; then
    echo "Base path not provided"
    exit 1
fi
echo "Base path: $1"
# check for UTF-8
sudo apt-get update && sudo apt-get install locales -y
sudo locale-gen en_US en_US.UTF-8
sudo update-locale LC_ALL=en_US.UTF-8 LANG=en_US.UTF-8
export LANG=en_US.UTF-8

#add ros 2 apt-get repository
sudo apt-get install software-properties-common -y
sudo add-apt-repository universe -y

#Now add the ROS 2 GPG key with apt-get.
sudo apt-get update && sudo apt-get install -y curl
sudo curl -sSL https://raw.githubusercontent.com/ros/rosdistro/master/ros.key -o /usr/share/keyrings/ros-archive-keyring.gpg

#add repository to your sources list
sudo echo "deb [arch=$(dpkg --print-architecture) signed-by=/usr/share/keyrings/ros-archive-keyring.gpg] http://packages.ros.org/ros2/ubuntu $(. /etc/os-release && echo $UBUNTU_CODENAME) main" | sudo tee /etc/apt/sources.list.d/ros2.list > /dev/null

#Install common packages.
sudo apt-get update 
sudo apt-get install -y \
  python3-flake8-docstrings \
  python3-pip \
  python3-pytest-cov \
  ros-dev-tools

#Install packages for Ubuntu version 22.04
sudo apt-get install -y \
   python3-flake8-blind-except \
   python3-flake8-builtins \
   python3-flake8-class-newline \
   python3-flake8-comprehensions \
   python3-flake8-deprecated \
   python3-flake8-import-order \
   python3-flake8-quotes \
   python3-pytest-repeat \
   python3-pytest-rerunfailures

#Install packages for Ubuntu version 20.04
# python3 -m pip install -U \
#    flake8-blind-except \
#    flake8-builtins \
#    flake8-class-newline \
#    flake8-comprehensions \
#    flake8-deprecated \
#    flake8-import-order \
#    flake8-quotes \
#    pytest-repeat \
#    pytest-rerunfailures

#create workspace and clone repos
echo "Creating workspace"
mkdir -p $1/ros2_humble/src
echo "workspace created in $1/ros2_humble/src"
pushd $1/ros2_humble
vcs import --input https://raw.githubusercontent.com/ros2/ros2/humble/ros2.repos src

#install dependencies
sudo apt-get upgrade -y

sudo rosdep init
rosdep update
rosdep install --from-paths src --ignore-src -y --skip-keys "fastcdr rti-connext-dds-6.0.1 urdfdom_headers"

#build code (takes a while, around 40 min?)
# cd ~/ros2_humble/
colcon build --symlink-install

#setup environment
# Replace ".bash" with your shell if you're not using bash
# Possible values are: setup.bash, setup.sh, setup.zsh
. $1/ros2_humble/install/local_setup.bash

popd #return to original directory

echo "ROS2 Humble installed"