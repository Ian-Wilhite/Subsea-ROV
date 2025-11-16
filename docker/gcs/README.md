# Ground Control Station (GCS) Image

Template container for the operator-facing Raspberry Pi 5 (or desktop Pi) that displays the HMI, ingests telemetry, and forwards joystick/video streams.

## Build

```
docker buildx build \
  --platform linux/arm64 \
  -t subsea-rov/gcs:dev \
  docker/gcs
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
