# syntax=docker/dockerfile:1.6
ARG BASE_IMAGE=arm64v8/ubuntu:22.04
FROM ${BASE_IMAGE}

LABEL maintainer="A&M Subsea ROV" \
      description="ROS 2-enabled operator console image for Raspberry Pi 5 or desktop"

ARG USERNAME=operator
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
        python3 \
        python3-pip \
        python3-venv \
        git \
        curl \
        gnupg2 \
        sudo \
        ca-certificates \
        tzdata \
        lsb-release \
        software-properties-common \
        xauth \
        x11-apps \
        gstreamer1.0-tools \
        gstreamer1.0-plugins-good \
        gstreamer1.0-plugins-bad \
        ffmpeg \
        libgl1 \
        mesa-utils \
        net-tools \
        iproute2 \
        wget \
    && add-apt-repository universe \
    && rm -rf /var/lib/apt/lists/*

RUN mkdir -p /etc/apt/keyrings \
    && curl -sSL https://raw.githubusercontent.com/ros/rosdistro/master/ros.key \
    | gpg --dearmor -o /etc/apt/keyrings/ros-archive-keyring.gpg \
    && echo "deb [arch=$(dpkg --print-architecture) signed-by=/etc/apt/keyrings/ros-archive-keyring.gpg] http://packages.ros.org/ros2/ubuntu $(. /etc/os-release && echo $UBUNTU_CODENAME) main" \
    | tee /etc/apt/sources.list.d/ros2.list >/dev/null

RUN apt-get update && apt-get install -y --no-install-recommends \
        ros-${ROS_DISTRO}-desktop \
        python3-colcon-common-extensions \
        python3-rosdep \
        ros-dev-tools \
        python3-opencv \
        python3-numpy \
        python3-pyqt6 \
        python3-pyqt5 \
        python3-pyside6 \
    && rm -rf /var/lib/apt/lists/*

RUN groupadd --gid ${USER_GID} ${USERNAME} \
    && useradd --uid ${USER_UID} --gid ${USER_GID} -m ${USERNAME} \
    && usermod -aG video,render ${USERNAME} \
    && echo "${USERNAME} ALL=(ALL) NOPASSWD:ALL" >/etc/sudoers.d/${USERNAME}

WORKDIR /workspace
RUN chown ${USER_UID}:${USER_GID} /workspace

COPY --chown=${USER_UID}:${USER_GID} software/ground_station /workspace/src/ground_station

COPY --chown=${USER_UID}:${USER_GID} docker/gcs/scripts/entrypoint.sh /usr/local/bin/gcs-entrypoint.sh
RUN chmod +x /usr/local/bin/gcs-entrypoint.sh

USER ${USERNAME}
ENV PATH="/home/${USERNAME}/.local/bin:${PATH}" \
    WORKSPACE=/workspace

# Placeholder for operator UI install
# RUN pip3 install -r /workspace/src/ground_station/requirements.txt

CMD ["/usr/local/bin/gcs-entrypoint.sh", "bash"]
