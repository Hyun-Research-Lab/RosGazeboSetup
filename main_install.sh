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

echo "Installing Ignition Gazebo 6"

#https://gazebosim.org/docs/fortress/install_ubuntu_src

#Install tools
sudo apt-get install -y python3-pip wget lsb-release gnupg curl

pip install vcstool || pip3 install vcstool
pip install -U colcon-common-extensions || pip3 install -U colcon-common-extensions

#assuming install path is not at $HOME/.local:
#but if it is, and you get "command not found" errors: debug with:
# pip show vcstool || pip3 show vcstool | grep Locatio
# pip show colcon-common-extensions || pip3 show colcon-common-extensions | grep Location

# export PATH=$PATH:$HOME/.local/bin/

#install git 
sudo apt-get install -y git

#get soures
echo "making dir"
mkdir -p $1/ign_ws/src
echo "dir made"
pushd $1/ign_ws/src

# retrieve all the Ignition libraries sources
wget https://raw.githubusercontent.com/ignition-tooling/gazebodistro/master/collection-fortress.yaml

vcs import < collection-fortress.yaml

#Install dependencies

#Add packages.osrfoundation.org to the apt sources list:
sudo wget https://packages.osrfoundation.org/gazebo.gpg -O /usr/share/keyrings/pkgs-osrf-archive-keyring.gpg
echo "deb [arch=$(dpkg --print-architecture) signed-by=/usr/share/keyrings/pkgs-osrf-archive-keyring.gpg] http://packages.osrfoundation.org/gazebo/ubuntu-stable $(lsb_release -cs) main" | sudo tee /etc/apt/sources.list.d/gazebo-stable.list > /dev/null
sudo apt-get update

#install all dependencies in Ubuntu Bionic, Focal or Jammy:
sudo apt-get -y install \
  $(sort -u $(find . -iname 'packages-'`lsb_release -cs`'.apt' -o -iname 'packages.apt' | grep -v '/\.git/') | sed '/ignition\|sdf/d' | tr '\n' ' ')

#The Ignition Libraries require the gcc compiler version 8 or higher. (Windows requires Visual Studio 2019).

#assuming not using ubuntu bionic

#Building the Ignition Libraries
pushd $1/ign_ws/
colcon graph
colcon build --merge-install
. $1/ign_ws/install/setup.bash

#uninstall
#rm -rf ~/$1/ign_ws/
popd
popd

echo "Ignition Gazebo 6 installed"
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