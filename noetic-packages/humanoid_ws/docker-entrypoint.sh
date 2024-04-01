#!/bin/bash

# Update and install necessary packages
apt-get update
# apt-get install -y \
#     python3-pip \
#     python3-catkin-tools \
#     iproute2 \
#     net-tools \
#     iputils-ping \
#     libusb-1.0-0 \
#     curl \
#     udev \
#     libfontconfig1 \
#     libusb-dev \
#     libxft-dev \
#     usbutils \
#     ros-${ROS_DISTRO}-rosserial-arduino \
#     ros-${ROS_DISTRO}-rosserial \
#     xfce4 \
#     xfce4-goodies \
#     x11vnc \
#     xvfb \
#     tightvncserver \
#     novnc \
#     python3-websockify \
#     python3-numpy

# # Install websockify to bridge between noVNC and the VNC server
# apt-get install -y websockify

# # Install a desktop environment
# apt-get install -y ubuntu-desktop
# apt-get install -y gnome-session gdm3

pip install odrive scipy pynput pygame scikit-learn

# Set up X11/VNC server environment
Xvfb :1 -screen 0 1280x800x24 &
export DISPLAY=:1

# Start X11VNC
x11vnc -passwd my_password -display :1 -N -forever &

# Start noVNC
cd /opt/noVNC/utils && ./launch.sh --vnc localhost:5900 &

# sudo bash -c "curl https://cdn.odriverobotics.com/files/odrive-udev-rules.rules > /etc/udev/rules.d/91-odrive.rules && udevadm control --reload-rules && udevadm trigger"
sudo bash -c "mkdir -p /etc/udev/rules.d/ && curl https://cdn.odriverobotics.com/files/odrive-udev-rules.rules > /etc/udev/rules.d/91-odrive.rules && udevadm control --reload-rules && udevadm trigger"

# Clean up the package lists to keep the container clean
rm -rf /var/lib/apt/lists/*

# Source the ROS environment
source /opt/ros/noetic/setup.bash
echo "source /opt/ros/noetic/setup.bash; source devel/setup.bash" >>~/.bashrc
echo "alias sros='source /opt/ros/noetic/setup.bash; source devel/setup.bash'" >>~/.bashrc

# Execute the command provided to the docker container
exec "$@"
