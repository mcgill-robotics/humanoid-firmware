#!/bin/bash

# # Update and install necessary packages
apt-get update
apt-get install -y \
    python3-pip \
    python3-catkin-tools \
    iproute2 \
    iputils-ping \
    libusb-1.0-0 \
    curl \
    udev \
    libfontconfig1 \
    libusb-dev \
    libxft-dev \
    usbutils \
    ros-${ROS_DISTRO}-rosserial-arduino \
    ros-${ROS_DISTRO}-rosserial

pip install odrive

# sudo bash -c "curl https://cdn.odriverobotics.com/files/odrive-udev-rules.rules > /etc/udev/rules.d/91-odrive.rules && udevadm control --reload-rules && udevadm trigger"
sudo bash -c "mkdir -p /etc/udev/rules.d/ && curl https://cdn.odriverobotics.com/files/odrive-udev-rules.rules > /etc/udev/rules.d/91-odrive.rules && udevadm control --reload-rules && udevadm trigger"

# # # No command should follow immediately after the backslash on the previous line

# # Clean up the package lists to keep the container clean
# rm -rf /var/lib/apt/lists/*

# Source the ROS environment
source /opt/ros/noetic/setup.bash
echo "source /opt/ros/noetic/setup.bash" >>~/.bashrc

# Execute the command provided to the docker container
exec "$@"
