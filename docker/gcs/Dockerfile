# syntax=docker/dockerfile:1.6
ARG BASE_IMAGE=arm64v8/ubuntu:22.04
FROM ${BASE_IMAGE}

LABEL maintainer="A&M Subsea ROV" \
      description="Template ground-control-station image for Raspberry Pi 5"

ARG USERNAME=operator
ARG USER_UID=1000
ARG USER_GID=1000

ENV DEBIAN_FRONTEND=noninteractive \
    TZ=UTC \
    LANG=C.UTF-8

RUN apt-get update && apt-get install -y --no-install-recommends \
        python3 \
        python3-pip \
        python3-venv \
        git \
        curl \
        sudo \
        ca-certificates \
        tzdata \
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
    && rm -rf /var/lib/apt/lists/*

RUN groupadd --gid ${USER_GID} ${USERNAME} \
    && useradd --uid ${USER_UID} --gid ${USER_GID} -m ${USERNAME} \
    && usermod -aG video,render ${USERNAME} \
    && echo "${USERNAME} ALL=(ALL) NOPASSWD:ALL" >/etc/sudoers.d/${USERNAME}

WORKDIR /workspace
RUN chown ${USER_UID}:${USER_GID} /workspace

COPY --chown=${USER_UID}:${USER_GID} scripts/entrypoint.sh /usr/local/bin/gcs-entrypoint.sh
RUN chmod +x /usr/local/bin/gcs-entrypoint.sh

USER ${USERNAME}
ENV PATH="/home/${USERNAME}/.local/bin:${PATH}" \
    WORKSPACE=/workspace

# Placeholder for operator UI install
# COPY ../../software/ground_station/ /workspace/ui/
# RUN pip3 install -r /workspace/ui/requirements.txt

CMD ["/usr/local/bin/gcs-entrypoint.sh", "bash"]
