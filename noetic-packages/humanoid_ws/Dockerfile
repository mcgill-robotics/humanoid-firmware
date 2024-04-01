FROM ros:noetic-ros-core-focal

# Install necessary packages (including TigerVNC server)
RUN apt-get update && \
    apt-get install -y \
    python3-pip \
    python3-catkin-tools \
    iproute2 \
    net-tools \
    iputils-ping \
    curl

RUN  apt-get install -y \
    udev \
    libusb-1.0-0 \
    libfontconfig1 \
    libusb-dev \
    libxft-dev \
    usbutils \
    ros-${ROS_DISTRO}-rosserial-arduino \
    ros-${ROS_DISTRO}-rosserial 

RUN pip install odrive scipy pynput pygame scikit-learn

# Use a base image that includes 'debconf-utils' or install it
RUN apt-get update && apt-get install -y debconf-utils

# Preconfigure selections: here choosing gdm3 over lightdm
# Replace 'package-name package/question select option' with actual package name and selection details
RUN echo "gdm3 gdm3/default-display-manager select" | debconf-set-selections
RUN echo "lightdm lightdm/default-display-manager select false" | debconf-set-selections

# Now install your packages
# RUN apt-get install -y \
#     gdm3 \
#     ubuntu-desktop \
#     gnome-session \
#     xfce4 \
#     xfce4-goodies

# Install websockify to bridge between noVNC and the VNC server
RUN apt-get install -y websockify

RUN  apt-get install -y \
    python3-websockify \
    python3-numpy 

RUN apt-get install -y \
    x11vnc \
    xvfb \
    tigervnc-standalone-server \
    tightvncserver \
    novnc


# Expose VNC port (5901 by default)
EXPOSE 5901
EXPOSE 5900

RUN apt-get clean
