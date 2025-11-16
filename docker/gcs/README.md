# Ground Control Station (GCS) Image

ROS 2 desktop container for the operator-facing Raspberry Pi 5 (or desktop Pi) that displays the HMI, ingests telemetry, and forwards joystick/video streams. It ships with ROS 2 Humble (override via `ROS_DISTRO`) plus Qt bindings required by the telemetry UI scaffold.

## Build

```
docker buildx build \
  --platform linux/arm64 \
  -f docker/gcs/Dockerfile \
  -t subsea-rov/gcs:dev \
  .

# Switch ROS distro:
# docker buildx build --build-arg ROS_DISTRO=iron -f docker/gcs/Dockerfile .
```

Override build arguments just like the OBC image to align usernames, base OS, or preinstalled dependencies.

## Run with GUI forwarding

```
xhost +local:root  # allow local containers to use the X server

docker run -it --rm \
  --env DISPLAY=$DISPLAY \
  --volume /tmp/.X11-unix:/tmp/.X11-unix \
  --device /dev/input \
  -v $(pwd)/software/ground_station:/workspace/ui \
  subsea-rov/gcs:dev
```

Integrate joystick devices (`/dev/input/js*`), network interfaces, or logging volumes as required by the ground-station software.

The entrypoint sources `/opt/ros/${ROS_DISTRO}/setup.bash` before launching the requested command. Point `WORKSPACE_SETUP` to a colcon overlay if you build one inside the container.

The build copies `software/ground_station` into `/workspace/src/ground_station`, so the latest ground-station scripts are always present even when you run the container without bind-mounting the repository.
