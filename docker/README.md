# Docker Images

This directory houses container definitions for the Raspberry Pi-based onboard computer (OBC) and the ground control station (GCS). Both images now include a full ROS 2 (default Humble) environment plus the dependencies required by the ROS nodes in `software/`.

- `obc/`: Runtime image that deploys mission software to the Pi 5 inside the vehicle.
- `gcs/`: Runtime image for the operator console Pi 5 or companion desktop Pi handling control + visualization.

During the build we copy the mission sources into the image so they are immediately available:

- `software/onboard/**` → `/workspace/src/onboard` inside the OBC container.
- `software/ground_station/**` → `/workspace/src/ground_station` inside the GCS container.

Each subdirectory contains a `Dockerfile`, build context, and helper scripts required to package and deploy the platform. Keep hardware-specific configuration (udev rules, overlays, system tuning) alongside the Dockerfile so flashing a Pi stays reproducible.

Use `docker-compose.yml` in this folder to build or run both images on a development host (buildx will emulate `linux/arm64` if the host is x86). Override or extend the compose services as the software matures. Set `ROS_DISTRO` via a build argument if you need a different ROS 2 distribution:

```
docker compose build \
  --build-arg ROS_DISTRO=humble \
  obc gcs
```
