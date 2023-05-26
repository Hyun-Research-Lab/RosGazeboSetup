# RosGazeboSetup
script to set up ROS 2 Humble + Ignition Gazebo Fortress + bridging. 

run to get executable permission
`sudo -s`
`chmod +x main_install.sh`

Run main_install.sh to install all
`./main_install.sh`


# Can run individual files separately if breaks happen, main runs all three in order, Ros, Gazebo, Bridge


# insert into bashrc

assumes inclusion of drake
```
export PATH=$PATH:/home/tsai169/.local/bin
export PATH="/opt/drake/bin${PATH:+:${PATH}}"
export PYTHONPATH="/opt/drake/lib/python$(python3 -c 'import sys; print("{0}.{1}".format(*sys.version_info))')/site-packages${PYTHONPATH:+:${PYTHONPATH}}"
source /opt/ros/humble/setup.bash
source ~/drake/install/setup.bash
. ~/ros2_humble/install/local_setup.bash
. ~/ws/install/local_setup.bash
. ~/main_workspace/install/local_setup.bash
```