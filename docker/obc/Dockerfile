# syntax=docker/dockerfile:1.6
ARG BASE_IMAGE=arm64v8/ubuntu:22.04
FROM ${BASE_IMAGE}

LABEL maintainer="A&M Subsea ROV" \
      description="Template onboard computer image for Raspberry Pi 5"

ARG USERNAME=rov
ARG USER_UID=1000
ARG USER_GID=1000

ENV DEBIAN_FRONTEND=noninteractive \
    TZ=UTC \
    LANG=C.UTF-8

RUN apt-get update && apt-get install -y --no-install-recommends \
        build-essential \
        pkg-config \
        python3 \
        python3-pip \
        python3-venv \
        python3-dev \
        git \
        curl \
        ca-certificates \
        tzdata \
        sudo \
        iproute2 \
        iputils-ping \
        net-tools \
        i2c-tools \
        libgpiod2 \
        libatlas-base-dev \
    && rm -rf /var/lib/apt/lists/*

RUN groupadd --gid ${USER_GID} ${USERNAME} \
    && useradd --uid ${USER_UID} --gid ${USER_GID} -m ${USERNAME} \
    && usermod -aG dialout,video ${USERNAME} \
    && echo "${USERNAME} ALL=(ALL) NOPASSWD:ALL" >/etc/sudoers.d/${USERNAME}

WORKDIR /workspace
RUN chown ${USER_UID}:${USER_GID} /workspace

COPY --chown=${USER_UID}:${USER_GID} scripts/entrypoint.sh /usr/local/bin/obc-entrypoint.sh
RUN chmod +x /usr/local/bin/obc-entrypoint.sh

USER ${USERNAME}
ENV PATH="/home/${USERNAME}/.local/bin:${PATH}" \
    WORKSPACE=/workspace

# Placeholder for mission software install
# COPY ../../software/onboard/ /workspace/app/
# RUN pip3 install -r /workspace/app/requirements.txt

CMD ["/usr/local/bin/obc-entrypoint.sh", "bash"]
