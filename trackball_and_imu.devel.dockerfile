# Build arguments
ARG ROS_DISTRO=humble

# Base image
FROM ros:${ROS_DISTRO}-ros-base

# Restate the arg to make it available in later stage
ARG ROS_DISTRO

# An ARG declared before a FROM is outside of a build stage, so it can’t be used in any instruction after a FROM

# Add copy of local workspace and setup script
WORKDIR /home/user/ros
ADD onboard/src ./ws/src
WORKDIR /home/user/ros/ws
#CMD rm -r build install log
RUN apt-get -y update; apt-get -y install curl
RUN curl -fsSL https://www.phidgets.com/downloads/setup_linux | bash - &&\
apt-get install -y libphidget22
# Setup workspace
RUN . /opt/ros/${ROS_DISTRO}/setup.sh && colcon build --packages-select trackball_interface

# copy udev rules
#COPY onboard/src/phidgets_drivers/phidgets_api/debian/udev /etc/udev/rules.d/99-phidgets.rules

# Add the entrypoint script
ADD trackball_and_imu_entrypoint.sh /entrypoint.sh
RUN chmod +x /entrypoint.sh
ENTRYPOINT ["/entrypoint.sh"]
CMD ["bash"]
