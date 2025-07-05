# Use the official ROS 2 Iron base image
FROM ros:iron-ros-base

# Set environment variables
ENV DEBIAN_FRONTEND=noninteractive \
    ROS_DISTRO=iron

# Update and install common dependencies
RUN apt-get update && apt-get install -y --no-install-recommends \
    python3-colcon-common-extensions \
    build-essential \
    git \
    curl \
    wget \
    lsb-release \
    ros-iron-turtlesim \
    && rm -rf /var/lib/apt/lists/*

# Source ROS 2 setup.bash in every shell
RUN echo "source /opt/ros/$ROS_DISTRO/setup.bash" >> /root/.bashrc

# Set working directory
WORKDIR /ros2_ws

# Default command
CMD ["tail", "-f", "/dev/null"]
