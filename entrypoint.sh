#!/bin/bash

# Define some useful aliases
#echo 'alias run_force_estimator_node="ros2 run sensors_and_observers force_estimator_node"' >> ~/.bashrc

# Navigate to the right directory and source it
cd /home/user/ros/ws
. /opt/ros/${ROS_DISTRO}/setup.sh
. install/setup.bash
echo v0.1
exec ros2 run trackball_interface trackball_interface
