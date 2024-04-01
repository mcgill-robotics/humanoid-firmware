#!/bin/bash

# # Update and install necessary packages
apt-get update
apt-get install -y \
    python3-pip \
    python3-catkin-tools \
    iproute2 \
    net-tools \
    iputils-ping \
    libusb-1.0-0 \
    curl \
    udev \
    libfontconfig1 \
    libusb-dev \
    libxft-dev \
    usbutils \
    ros-${ROS_DISTRO}-rosserial-arduino \
    ros-${ROS_DISTRO}-rosserial \
    xfce4 \
    xfce4-goodies \
    tightvncserver


pip install odrive

# Set up VNC server
# WARNING: For demonstration purposes only, set a secure password in production!
export USER=root
echo "my_password" | vncpasswd -f >/root/.vnc/passwd
chmod 600 /root/.vnc/passwd
vncserver -kill :1 || true
vncserver :1 -geometry 1280x800 -depth 24

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
