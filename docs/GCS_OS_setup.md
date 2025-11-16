# Ground Control Station OS & Configuration

The GCS must bridge two independent links:
1. The Blue Robotics Navigator flight controller (via MAVLink over Ethernet/USB).
2. The Insta360 camera payloads (via USB/serial for control and UVC for video).

## Recommended Base OS

- **Ubuntu 22.04 LTS (64-bit, desktop flavor)** on Raspberry Pi 5 using Canonical's official image. This keeps parity with the Docker base images and has first-party kernel/device support for USB 3, video4linux, and Vulkan drivers that BlueOS expects.
- Install the latest Raspberry Pi firmware (`sudo rpi-eeprom-update -a`) and enable USB boot/PCIe tuning per Canonical release notes.
- Keep the kernel on the GA channel (5.15 or newer) to ensure the BlueOS companion requirements (`cgroups v1`, `iptables-legacy`) remain available to Docker.

## Software Stack

1. **BlueOS Companion Interface**
   - BlueOS normally runs on the companion computer (OBC) that connects directly to the Navigator. The GCS does not have to run BlueOS itself but must be capable of discovering and proxying BlueOS services.
   - Install `mavproxy`, `mavutil`, or `mavros` to communicate through the Navigator's Ethernet bridge (default 192.168.2.1).
   - Configure NetworkManager with a static IP on the tether NIC (e.g., `192.168.2.2/24`) so the GCS can reach BlueOS web UI, MAVLink endpoints, and video streams when connected directly.

2. **Docker + compose**
   - Install Docker Engine on Ubuntu (`sudo apt-get install docker.io docker-compose-plugin`).
   - Enable rootless or add the `operator` user to the `docker` group.
   - Use the provided `docker/gcs` image for ROS-based apps; mount `/dev/input`, `/dev/video*`, and `/run/udev` into the container for joystick/camera access.

3. **ROS 2 Humble or Iron**
   - Prefer the same ROS 2 distribution used on the OBC (Humble recommended). Install host-side tools (`ros-humble-desktop`, `colcon`, `cyclonedds`).
   - Configure FastDDS/CycloneDDS XML profiles to match QoS across the tethered network.

## FC Connectivity

- The Navigator exposes MAVLink over Ethernet (UDP 14550) and over USB serial when BlueOS is running on the companion. To reach it from the GCS:
  1. Set the tether NIC to static `192.168.2.2` and connect to the companion's switch or direct Ethernet.
  2. Start `mavproxy.py --master=udp:192.168.2.1:14550 --out udp:127.0.0.1:14551` or use `ros2 run mavros mavros_node fcu_url:=udp://@192.168.2.1:14550`.
  3. If using USB, ensure `cdc_acm`/`usbserial` modules are loaded and the user has access to `/dev/ttyACM*`; set `fcu_url:=/dev/ttyACM0:115200`.
- Keep `iptables-legacy` installed so MAVLink routing utilities function inside Docker.

## Insta360 USB/Serial Connectivity

- Run the GCS on Ubuntu so the kernel provides UVC (video) and UAC (audio) drivers for Insta360 cameras. Use the official Insta360 SDK or `libusb` to control capture modes.
- Udev rules:
  - Create `/etc/udev/rules.d/99-insta360.rules` to set ownership/group for the camera VID/PID so Docker containers can access `/dev/video*` without root.
  - Example: `SUBSYSTEM=="video4linux", ATTRS{idVendor}=="2d96", MODE="0660", GROUP="video"`.
- For serial/command channels (if the camera exposes them), match on the corresponding USB interface and symlink to `/dev/insta360_serial` for easier mounting into containers.

## Network & Services

- Enable Avahi (`sudo apt install avahi-daemon`) so `.local` hostnames (e.g., `blueos.local`) resolve automatically.
- Configure systemd-networkd or NetworkManager profiles that switch between tethered and lab Wi-Fi without disrupting the static 192.168.2.x link.
- Optionally install `zerotier` or `tailscale` when remote operation is required; expose MAVLink and camera RTSP streams through VPN only when secured.

## Summary

Use Ubuntu 22.04 LTS on the GCS Pi 5, install Docker + ROS 2 + MAVLink tooling, configure a static tether interface, and add udev rules for Insta360 devices. This setup lets the GCS reach the Navigator via BlueOS (UDP 14550) while simultaneously ingesting video/control streams from the Insta360 cameras through USB/serial.
