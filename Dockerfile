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
    libusb-1.0-0 \
    libusb-1.0-0-dev \
    && rm -rf /var/lib/apt/lists/*


# Add copy of local workspace and setup script
WORKDIR /home/user/ros
ADD onboard/src ./ws/src
WORKDIR /home/user/ros/ws
#CMD rm -r build install log
CMD rosdep install phidgets_drivers
# Setup workspace
# RUN . /opt/ros/${ROS_DISTRO}/setup.sh && colcon build --packages-select libphidget22
RUN . /opt/ros/${ROS_DISTRO}/setup.sh && colcon build --packages-select phidgets_drivers
#RUN . /opt/ros/${ROS_DISTRO}/setup.sh && colcon build --packages-select trackball_interface

# copy udev rules
COPY onboard/src/phidgets_drivers/phidgets_api/debian/udev /etc/udev/rules.d/99-phidgets.rules

# Add the entrypoint script
ADD entrypoint.sh /entrypoint.sh
RUN chmod +x /entrypoint.sh
ENTRYPOINT ["/entrypoint.sh"]
CMD ["bash"]
