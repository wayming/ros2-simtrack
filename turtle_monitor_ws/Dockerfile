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
    ros-iron-turtlebot3-gazebo \
    ros-iron-turtlebot3-msgs \
    && rm -rf /var/lib/apt/lists/*

# Source ROS 2 setup.bash in every shell
RUN echo "source /opt/ros/$ROS_DISTRO/setup.bash" >> /root/.bashrc

RUN useradd -m -s /bin/bash -u 1000 rosuser \
    && echo "rosuser ALL=(ALL) NOPASSWD:ALL" >> /etc/sudoers

# Set working directory
WORKDIR /ros2_ws

USER rosuser

# Default command
CMD ["tail", "-f", "/dev/null"]
