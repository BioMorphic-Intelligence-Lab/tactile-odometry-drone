# Build arguments
ARG ROS_DISTRO=humble

# Base image
FROM ros:${ROS_DISTRO}-ros-base

# Restate the arg to make it available in later stage
ARG ROS_DISTRO

# Add copy of local workspace 
WORKDIR /home/user/ros
ADD onboard/src/px4_msgs ./ws/src/px4_msgs
ADD onboard/src/controllers ./ws/src/controllers
ADD onboard/src/estimators ./ws/src/estimators

WORKDIR /home/user/ros/ws
RUN . /opt/ros/${ROS_DISTRO}/setup.sh && colcon build --packages-select px4_msgs
RUN . /opt/ros/${ROS_DISTRO}/setup.sh && colcon build --packages-select controllers
RUN . /opt/ros/${ROS_DISTRO}/setup.sh && colcon build --packages-select estimators

# Add the entrypoint script
ADD controller_entrypoint.sh /entrypoint.sh
RUN chmod +x /entrypoint.sh
ENTRYPOINT ["/entrypoint.sh"]
CMD ["bash"]
