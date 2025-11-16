# Docker Images

This directory houses container definitions for the Raspberry Pi-based onboard computer (OBC) and the ground control station (GCS).

- `obc/`: Runtime image that deploys mission software to the Pi 5 inside the vehicle.
- `gcs/`: Runtime image for the operator console Pi 5 or companion desktop Pi handling control + visualization.

Each subdirectory contains a `Dockerfile`, build context, and helper scripts required to package and deploy the platform. Keep hardware-specific configuration (udev rules, overlays, system tuning) alongside the Dockerfile so flashing a Pi stays reproducible.

Use `docker-compose.yml` in this folder to build or run both images on a development host (buildx will emulate `linux/arm64` if the host is x86). Override or extend the compose services as the software matures.
