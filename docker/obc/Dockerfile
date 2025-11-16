# syntax=docker/dockerfile:1.6
ARG BASE_IMAGE=arm64v8/ubuntu:22.04
FROM ${BASE_IMAGE}

LABEL maintainer="A&M Subsea ROV" \
      description="ROS 2 runtime image for the onboard Raspberry Pi 5 computer"

ARG USERNAME=rov
ARG USER_UID=1000
ARG USER_GID=1000
ARG ROS_DISTRO=humble

ENV DEBIAN_FRONTEND=noninteractive \
    TZ=UTC \
    LANG=C.UTF-8 \
    ROS_DISTRO=${ROS_DISTRO} \
    ROS_VERSION=2 \
    ROS_PYTHON_VERSION=3

RUN apt-get update && apt-get install -y --no-install-recommends \
        build-essential \
        pkg-config \
        python3 \
        python3-pip \
        python3-venv \
        python3-dev \
        git \
        curl \
        gnupg2 \
        ca-certificates \
        tzdata \
        sudo \
        lsb-release \
        software-properties-common \
        iproute2 \
        iputils-ping \
        net-tools \
        i2c-tools \
        libgpiod2 \
        libatlas-base-dev \
        udev \
    && add-apt-repository universe \
    && rm -rf /var/lib/apt/lists/*

RUN mkdir -p /etc/apt/keyrings \
    && curl -sSL https://raw.githubusercontent.com/ros/rosdistro/master/ros.key \
    | gpg --dearmor -o /etc/apt/keyrings/ros-archive-keyring.gpg \
    && echo "deb [arch=$(dpkg --print-architecture) signed-by=/etc/apt/keyrings/ros-archive-keyring.gpg] http://packages.ros.org/ros2/ubuntu $(. /etc/os-release && echo $UBUNTU_CODENAME) main" \
    | tee /etc/apt/sources.list.d/ros2.list >/dev/null

RUN apt-get update && apt-get install -y --no-install-recommends \
        ros-${ROS_DISTRO}-ros-base \
        python3-colcon-common-extensions \
        python3-rosdep \
        ros-dev-tools \
        python3-opencv \
        python3-numpy \
        gstreamer1.0-tools \
        gstreamer1.0-plugins-good \
        gstreamer1.0-plugins-bad \
    && rm -rf /var/lib/apt/lists/*

RUN groupadd --gid ${USER_GID} ${USERNAME} \
    && useradd --uid ${USER_UID} --gid ${USER_GID} -m ${USERNAME} \
    && usermod -aG dialout,video ${USERNAME} \
    && echo "${USERNAME} ALL=(ALL) NOPASSWD:ALL" >/etc/sudoers.d/${USERNAME}

WORKDIR /workspace
RUN chown ${USER_UID}:${USER_GID} /workspace

COPY --chown=${USER_UID}:${USER_GID} software/onboard /workspace/src/onboard

COPY --chown=${USER_UID}:${USER_GID} docker/obc/scripts/entrypoint.sh /usr/local/bin/obc-entrypoint.sh
RUN chmod +x /usr/local/bin/obc-entrypoint.sh

USER ${USERNAME}
ENV PATH="/home/${USERNAME}/.local/bin:${PATH}" \
    WORKSPACE=/workspace

# Placeholder for mission software build commands
# RUN . /opt/ros/${ROS_DISTRO}/setup.sh && colcon build --symlink-install

CMD ["/usr/local/bin/obc-entrypoint.sh", "bash"]
