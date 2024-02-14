# # Use ROS Humble as the base image
# FROM ros:humble

# # Avoid prompts from apt
# ENV DEBIAN_FRONTEND=noninteractive

# # Install dependencies
# RUN apt-get update && apt-get install -y \
#     git \
#     python3-pip \
#     iproute2 \
#     iputils-ping \
#     && rm -rf /var/lib/apt/lists/*

# # Setup the workspace
# RUN mkdir /microros_ws \
#     && cd /microros_ws \
#     && git clone -b foxy https://github.com/micro-ROS/micro_ros_setup.git src/micro_ros_setup \
#     && . /opt/ros/foxy/setup.sh \
#     && apt update && rosdep update \
#     && rosdep install --from-paths src --ignore-src -y \
#     && colcon build \
#     && echo "source /microros_ws/install/local_setup.bash" >> ~/.bashrc \


#     # Set the working directory
#     WORKDIR /microros_ws

# # Command to run when starting the container
# CMD ["/bin/bash"]
