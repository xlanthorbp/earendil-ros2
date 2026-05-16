#!/bin/bash

echo "Starting Earendil Bot on Raspberry Pi 5..."

# Ensure xhost permissions are set if you ever need to view rqt on the Pi itself 
# (Though usually you run rviz on your laptop, not the Pi)
xhost +local:root 2>/dev/null || true

# Run the Docker container optimized for the real robot.
# Key differences from the sim version:
# 1. --privileged and -v /dev:/dev gives ROS access to your physical USB/Serial sensors (Arduino, LiDAR).
# 2. --network host allows your laptop to see the ROS topics over Wi-Fi.
# 3. No NVIDIA/GPU flags since the Pi 5 doesn't use them.

docker run -it --rm \
    --name earendil_real \
    --privileged \
    -v /dev:/dev \
    --network host \
    -e DISPLAY=$DISPLAY \
    -v /tmp/.X11-unix:/tmp/.X11-unix:rw \
    earendil-real-image "$@"
