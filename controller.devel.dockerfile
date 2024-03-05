# Build arguments
ARG ROS_DISTRO=humble

# Base image
FROM ros:${ROS_DISTRO}-ros-base

# Restate the arg to make it available in later stage
ARG ROS_DISTRO

# Add copy of local workspace 
WORKDIR /home/user/ros
ADD onboard/src ./ws/src

WORKDIR /home/user/ros/ws
RUN . /opt/ros/${ROS_DISTRO}/setup.sh && colcon build
# Add the entrypoint script
ADD controller_entrypoint.sh /entrypoint.sh
RUN chmod +x /entrypoint.sh
ENTRYPOINT ["/entrypoint.sh"]
CMD ["bash"]
