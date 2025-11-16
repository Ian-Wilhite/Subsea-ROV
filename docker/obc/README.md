# Onboard Computer (OBC) Image

Template container for the Raspberry Pi 5 that sits inside the vehicle. It supplies Python, build tools, and Pi-specific interfaces (I2C, GPIO) needed for mission software.

## Build

```
docker buildx build \
  --platform linux/arm64 \
  -t subsea-rov/obc:dev \
  docker/obc
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
