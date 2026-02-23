FROM ubuntu:22.04
ENV DEBIAN_FRONTEND=noninteractive
SHELL ["/bin/bash", "-c"]

# 1) Base utilities
RUN apt-get update && apt-get install -y \
    ca-certificates curl gnupg lsb-release \
    build-essential cmake git nano \
    python3-pip python3-venv \
    sudo \
    iproute2 iputils-ping net-tools netcat-openbsd \
    xauth x11-apps \
    && rm -rf /var/lib/apt/lists/*

# 2) Add ROS 2 apt repo
RUN curl -sSL https://raw.githubusercontent.com/ros/rosdistro/master/ros.key \
    | gpg --dearmor -o /usr/share/keyrings/ros-archive-keyring.gpg && \
    echo "deb [arch=$(dpkg --print-architecture) signed-by=/usr/share/keyrings/ros-archive-keyring.gpg] \
    http://packages.ros.org/ros2/ubuntu $(lsb_release -cs) main" \
    > /etc/apt/sources.list.d/ros2.list

# 3) Install ROS 2 Humble + Gazebo classic integration + controls + MoveIt
RUN apt-get update && apt-get install -y \
    ros-humble-desktop \
    ros-humble-gazebo-ros-pkgs \
    ros-humble-gazebo-ros2-control \
    ros-humble-ros2-control ros-humble-ros2-controllers \
    ros-humble-moveit* \
    python3-rosdep python3-colcon-common-extensions python3-vcstool \
    && rm -rf /var/lib/apt/lists/*

# 4) rosdep
RUN rosdep init || true && rosdep update

# Workspace
ENV WS=/ws
RUN mkdir -p $WS/src
WORKDIR $WS

# Always have ROS sourced
RUN echo "source /opt/ros/humble/setup.bash" >> /root/.bashrc