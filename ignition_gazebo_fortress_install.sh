#!/usr/bin/bash
#https://gazebosim.org/docs/fortress/install_ubuntu_src

#Install tools
sudo apt-get install python3-pip wget lsb-release gnupg curl

pip install vcstool || pip3 install vcstool
pip install -U colcon-common-extensions || pip3 install -U colcon-common-extensions

#assuming install path is not at $HOME/.local:
#but if it is, and you get "command not found" errors: debug with:
# pip show vcstool || pip3 show vcstool | grep Locatio
# pip show colcon-common-extensions || pip3 show colcon-common-extensions | grep Location

# export PATH=$PATH:$HOME/.local/bin/

#install git 
sudo apt-get install git

#get soures
mkdir -p ~/workspace/src
pushd ~/workspace/src

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
pushd ~/workspace/
colcon graph
colcon build --merge-install
. ~/workspace/install/setup.bash

#uninstall
#rm -rf ~/workspace
popd
popd