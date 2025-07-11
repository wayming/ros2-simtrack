# Use the official ROS 2 Jazzy base image
FROM ros:jazzy-ros-base

# Default UID/GID for build-time argument (can be overridden)
ARG HOST_UID=1000
ARG HOST_GID=1000

# Environment setup
ENV DEBIAN_FRONTEND=noninteractive \
    ROS_DISTRO=jazzy \
    TURTLEBOT3_MODEL=burger

# Set up ROS sources and update
RUN apt-get update && apt-get install -y --no-install-recommends \
    curl \
    gnupg \
    lsb-release \
    git \
    build-essential \
    python3-colcon-common-extensions \
    python3-pip \
    sudo \
    wget \
    && rm -rf /var/lib/apt/lists/*

# Install Gazebo Harmonic
RUN curl https://packages.osrfoundation.org/gazebo.gpg --output /usr/share/keyrings/pkgs-osrf-archive-keyring.gpg && \
    echo "deb [arch=$(dpkg --print-architecture) signed-by=/usr/share/keyrings/pkgs-osrf-archive-keyring.gpg] http://packages.osrfoundation.org/gazebo/ubuntu-stable $(lsb_release -cs) main" \
    > /etc/apt/sources.list.d/gazebo-stable.list && \
    apt-get update && \
    apt-get install -y gz-harmonic

# Install Cartographer
RUN apt-get install -y ros-jazzy-cartographer ros-jazzy-cartographer-ros

# Install Navigation2
RUN apt-get install -y ros-jazzy-navigation2 ros-jazzy-nav2-bringup

# Colcon common extensions  
RUN apt-get install python3-colcon-common-extensions

WORKDIR /ros2_ws
RUN . /opt/ros/jazzy/setup.sh && colcon build --symlink-install

# Create group if not exists, then user
RUN if ! getent group $HOST_GID >/dev/null; then \
        groupadd -g $HOST_GID hostgroup; \
    fi && \
    if ! id -u $HOST_UID >/dev/null 2>&1; then \
        useradd -m -u $HOST_UID -g $HOST_GID appuser \
        mkdir -p /home/appuser && \
        chown $HOST_UID:$HOST_GID /home/appuser; \
    fi

USER $HOST_UID:$HOST_GID

RUN echo 'export ROS_DOMAIN_ID=30 #TURTLEBOT3' >> ~/.bashrc && \
    echo 'source /opt/ros/jazzy/setup.bash' >> ~/.bashrc && \
    echo 'source /ros2_ws/turtle_monitor_ws/install/setup.bash' >> ~/.bashrc


# Default command
CMD ["tail", "-f", "/dev/null"]

