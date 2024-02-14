#!/bin/bash

# # Update and install necessary packages
apt-get update
apt-get install -y \
    libusb-1.0-0 \
    iproute2 \
    python3-pip \
    iputils-ping \
    curl \
    udev \
    libfontconfig1 \
    libusb-dev \
    libxft-dev \
    usbutils

pip install odrive

# sudo bash -c "curl https://cdn.odriverobotics.com/files/odrive-udev-rules.rules > /etc/udev/rules.d/91-odrive.rules && udevadm control --reload-rules && udevadm trigger"
sudo bash -c "mkdir -p /etc/udev/rules.d/ && curl https://cdn.odriverobotics.com/files/odrive-udev-rules.rules > /etc/udev/rules.d/91-odrive.rules && udevadm control --reload-rules && udevadm trigger"

# # # No command should follow immediately after the backslash on the previous line

# # Clean up the package lists to keep the container clean
# rm -rf /var/lib/apt/lists/*

# Source the ROS environment
source /opt/ros/noetic/setup.bash

# Execute the command provided to the docker container
exec "$@"
