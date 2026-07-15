# ====================================================================
# STAGE 1: Builder (Optimized for Raspberry Pi 5 / ARM64)
# ====================================================================
FROM ros:humble-ros-base AS builder

ENV DEBIAN_FRONTEND=noninteractive
WORKDIR /real_ws

# 1. Install build tools and python dependencies
RUN apt-get update && apt-get install -y \
    python3-colcon-common-extensions \
    python3-rosdep \
    build-essential \
    && rm -rf /var/lib/apt/lists/*

# 2. Copy package.xml files first to utilize Docker layer caching.
COPY src/earendil_bot/package.xml src/earendil_bot/package.xml

# 3. Install all ROS dependencies defined in package.xml automatically
# Notice we are using ros-base, which strips out heavy GUI tools like Gazebo/Rviz
RUN apt-get update \
    && rosdep update \
    && rosdep install --from-paths src --ignore-src -y \
    && rm -rf /var/lib/apt/lists/*

# 4. Copy the rest of the source code
COPY src/ src/

# 5. Build the workspace
RUN . /opt/ros/humble/setup.sh && \
    colcon build --cmake-args -DCMAKE_BUILD_TYPE=Release


# ====================================================================
# STAGE 2: Runtime (Slim image for the Pi)
# ====================================================================
FROM ros:humble-ros-base AS runtime

ENV DEBIAN_FRONTEND=noninteractive
WORKDIR /real_ws

# 1. Copy package.xml again to install dependencies in the runtime image
COPY src/earendil_bot/package.xml src/earendil_bot/package.xml

# 2. Install runtime dependencies (I2C tools removed since IMU is on Arduino)
RUN apt-get update \
    && rosdep update \
    && rosdep install --from-paths src --ignore-src -y \
    && rm -rf /var/lib/apt/lists/*

# 3. Copy ONLY the compiled 'install' folder from the builder stage.
COPY --from=builder /real_ws/install /real_ws/install

# 4. Setup entrypoint
COPY entrypoint.sh /entrypoint.sh
RUN chmod +x /entrypoint.sh

ENTRYPOINT ["/entrypoint.sh"]
CMD ["ros2", "launch", "earendil_bot", "tunnel_hardware.launch.py"]
