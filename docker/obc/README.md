# Onboard Computer (OBC) Image

ROS 2-enabled container for the Raspberry Pi 5 that sits inside the vehicle. It supplies Python, build tools, ROS 2 Humble (override via `ROS_DISTRO`), and Pi-specific interfaces (I2C, GPIO) needed for mission software.

## Build

```
docker buildx build \
  --platform linux/arm64 \
  -f docker/obc/Dockerfile \
  -t subsea-rov/obc:dev \
  .

# Switch ROS distro:
# docker buildx build --build-arg ROS_DISTRO=iron -f docker/obc/Dockerfile .
```

Pass custom arguments as needed:

- `--build-arg BASE_IMAGE=arm64v8/ubuntu:24.04`
- `--build-arg USERNAME=rov --build-arg USER_UID=1001`

## Run on a Pi 5

```
docker run -it --rm \
  --net=host \
  --privileged \
  -v /dev:/dev \
  -v $(pwd)/software/onboard:/workspace/app \
  subsea-rov/obc:dev
```

Mount the mission code into `/workspace/app` (or bake it in during the build) and configure environment variables such as `WORKSPACE_SETUP` if you need to source additional setup scripts before launching ROS nodes.

The default entrypoint sources `/opt/ros/${ROS_DISTRO}/setup.bash`. If you build a colcon workspace inside `/workspace`, export `WORKSPACE_SETUP=/workspace/install/setup.bash` so the entrypoint loads the overlay before executing your command. Source files from `software/onboard` are baked into `/workspace/src/onboard` during the image build so the container always has a copy even when you do not mount the live repository.
