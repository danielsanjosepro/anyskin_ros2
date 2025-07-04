ARG ROS_DISTRO=humble
 
FROM osrf/ros:${ROS_DISTRO}-desktop AS base

ENV ROS_DISTRO=${ROS_DISTRO}

# Create a non-root user
ARG USERNAME=ros
ARG USER_UID=1000
ARG USER_GID=$USER_UID
ARG DEBIAN_FRONTEND=noninteractive

SHELL ["/bin/bash", "-c"]

# Delete existing user if it exists
RUN if getent passwd ${USER_UID}; then \
    userdel -r $(getent passwd ${USER_UID} | cut -d: -f1); \
    fi

# Delete existing group if it exists
RUN if getent group ${USER_GID}; then \
    groupdel $(getent group ${USER_GID} | cut -d: -f1); \
    fi

RUN groupadd --gid $USER_GID $USERNAME \
  && useradd -s /bin/bash --uid $USER_UID --gid $USER_GID -m $USERNAME \
  && mkdir /home/$USERNAME/.config && chown $USER_UID:$USER_GID /home/$USERNAME/.config

# Set up sudo
RUN apt-get update \
  && apt-get install -y sudo \
  && echo $USERNAME ALL=\(root\) NOPASSWD:ALL > /etc/sudoers.d/$USERNAME\
  && chmod 0440 /etc/sudoers.d/$USERNAME \
  && rm -rf /var/lib/apt/lists/*


RUN apt-get update && \
    DEBIAN_FRONTEND=noninteractive apt-get install -y --no-install-recommends \
    vim \
    build-essential \
    cmake \
    wget \
    git \
    unzip \
    pip \
    python3-venv \
    python3-ament-package \
    python3-flake8 \
    python3-rosdep \
    python3-setuptools \
    python3-vcstool \
    python3-colcon-common-extensions \
    cmake \
    libpoco-dev \
    libeigen3-dev \
    dpkg


# Symlink python3 to python
RUN ln -s /usr/bin/python3 /usr/bin/python

USER $USERNAME

RUN mkdir -p /home/ros/ros2_ws/src

WORKDIR /home/ros/ros2_ws/src

RUN git clone https://github.com/raunaqbhirangi/anyskin.git --recursive && \
    cd anyskin && \
    sudo pip install -e . && \
    touch COLCON_IGNORE

FROM base AS overlay

ENV ROS_DISTRO=${ROS_DISTRO}

COPY --chown=ros:ros . /home/ros/ros2_ws/src/anyskin_ros2

WORKDIR /home/ros/ros2_ws
RUN source /opt/ros/${ROS_DISTRO}/setup.bash && colcon build


