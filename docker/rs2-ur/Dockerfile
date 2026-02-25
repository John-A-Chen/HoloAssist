FROM osrf/ros:humble-desktop-full

ENV DEBIAN_FRONTEND=noninteractive
SHELL ["/bin/bash", "-c"]

# Core dev + UR + MoveIt + DDS
RUN apt-get update && apt-get install -y \
    git nano sudo \
    build-essential cmake \
    python3-colcon-common-extensions \
    python3-vcstool \
    python3-rosdep \
    iproute2 iputils-ping net-tools \
    xauth x11-apps \
    ros-humble-ros2-control \
    ros-humble-ros2-controllers \
    ros-humble-moveit \
    ros-humble-rmw-cyclonedds-cpp \
    ros-humble-cyclonedds \
    && rm -rf /var/lib/apt/lists/*

# rosdep
RUN rosdep init || true && rosdep update

# Workspace
ENV WS=/ws
RUN mkdir -p $WS/src
WORKDIR $WS

# -------- Non-root user (optional but recommended) --------
ARG USERNAME=ros
ARG USER_UID=1000
ARG USER_GID=1000

RUN groupadd --gid ${USER_GID} ${USERNAME} \
  && useradd -m -s /bin/bash --uid ${USER_UID} --gid ${USER_GID} ${USERNAME} \
  && echo "${USERNAME} ALL=(ALL) NOPASSWD:ALL" > /etc/sudoers.d/${USERNAME} \
  && chmod 0440 /etc/sudoers.d/${USERNAME} \
  && chown -R ${USER_UID}:${USER_GID} /ws

USER ${USERNAME}

RUN echo "source /opt/ros/humble/setup.bash" >> /home/${USERNAME}/.bashrc
WORKDIR /ws
