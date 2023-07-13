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