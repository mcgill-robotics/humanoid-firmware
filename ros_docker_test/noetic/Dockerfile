FROM ros:noetic-ros-core-focal

# Install necessary packages (including TigerVNC server)
RUN apt-get update && \
    apt-get install -y tigervnc-standalone-server && \
    apt-get clean

# Expose VNC port (5901 by default)
EXPOSE 5901
