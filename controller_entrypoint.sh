#!/bin/bash

# Navigate to the right directory and source it
cd /home/user/ros/ws
. /opt/ros/${ROS_DISTRO}/setup.sh
. install/setup.bash

exec ros2 run controllers test_trajectory_publisher
