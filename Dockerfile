# Build arguments
ARG ROS_DISTRO=humble

# Base image
FROM ros:${ROS_DISTRO}-ros-base

# Restate the arg to make it available in later stage
ARG ROS_DISTRO

# An ARG declared before a FROM is outside of a build stage, so it can’t be used in any instruction after a FROM

# Install additional ros packages and other libraries
RUN apt-get update && DEBIAN_FRONTEND=noninteractive apt-get upgrade -yq && apt-get install -y \
    git \
    make \
    unzip \
    wget \
    && rm -rf /var/lib/apt/lists/*


# Add copy of local workspace and setup script
WORKDIR /home/user/ros
ADD onboard/ ./ws/
WORKDIR /home/user/ros/ws

# Setup workspace
RUN . /opt/ros/${ROS_DISTRO}/setup.sh && colcon build --packages-select sensors

# Add the entrypoint script
ADD entrypoint.sh /entrypoint.sh
RUN chmod +x /entrypoint.sh
ENTRYPOINT ["/entrypoint.sh"]
CMD ["bash"]
