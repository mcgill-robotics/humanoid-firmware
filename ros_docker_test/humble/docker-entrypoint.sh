#!/bin/bash

bash

# # Update and install necessary packages
apt-get update
apt-get install -y \
    python3-pip \
    iproute2 \
    iputils-ping

source /opt/ros/humble/setup.bash

# Execute the command provided to the docker container
exec "$@"
