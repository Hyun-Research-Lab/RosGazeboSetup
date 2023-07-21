# RosGazeboSetup

# Fast Python Install
Run install handler
`./install_handler.sh <SOURCE?> <PATH> <OPTIONS>`
SOURCE is an int value from 0-1, 0 to not source in `~/.bashrc` and 1 to source and add paths. Defaults to 1 if left blank.
PATH is the path to the directory you want to install the files to.
PATH defaults to `/home/$USER` if left blank, which installs to user home directory.
OPTIONS defaults to `all` if left blank, which installs ROS 2 Humble + Ignition Gazebo Fortress + bridging.
OPTIONS can be a list of 0-4 (space separated for multi option), where each number corresponds to the following:
0. Main Script (ROS 2 Humble + Ignition Gazebo Fortress + bridging)
1. ROS 2 Humble
2. Ignition Gazebo Fortress
3. Bridging
4. Drake




# Using individual scripts
script to set up ROS 2 Humble + Ignition Gazebo Fortress + bridging. 

run to get executable permission
`sudo chmod +x main_install.sh`

Run main_install.sh to install ROS 2 Humble + Ignition Gazebo Fortress + bridging.
`./main_install.sh <PATH>`

PATH is the path to the directory you want to install the files to.
Normally can specify `/home/$USER` to install to user home directory.

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

If install files are locked behind sudo perms and not user perms:
`sudo chmod -R a+rwx <file path>`