#not tested yet lol https://drake.mit.edu/apt.html#stable-releases
#interface with ros https://github.com/RobotLocomotion/drake-ros
sudo apt-get update
sudo apt-get install --no-install-recommends \
  ca-certificates gnupg lsb-release wget


wget -qO- https://drake-apt.csail.mit.edu/drake.asc | gpg --dearmor - \
  | sudo tee /etc/apt/trusted.gpg.d/drake.gpg >/dev/null

echo "deb [arch=amd64] https://drake-apt.csail.mit.edu/$(lsb_release -cs) $(lsb_release -cs) main" \
  | sudo tee /etc/apt/sources.list.d/drake.list >/dev/null

sudo apt-get update
sudo apt-get install --no-install-recommends drake-dev

#install CMAKE for c++
#https://github.com/RobotLocomotion/drake-external-examples/tree/main/drake_cmake_installed_apt 
sudo apt-get --no-install-recommends install build-essential cmake 