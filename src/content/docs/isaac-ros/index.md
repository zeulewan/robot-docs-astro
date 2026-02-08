---
title: Isaac ROS Container
description: GPU-accelerated ROS 2 perception container setup, packages, FastDDS fix, and troubleshooting.
---

## Quick Start

```bash
# Start container (from committed image, NOT isaac-ros-cli)
docker run -d --name isaac_ros_dev_container \
  --runtime nvidia --network host --ipc host --privileged \
  -e NVIDIA_VISIBLE_DEVICES=all \
  -e NVIDIA_DRIVER_CAPABILITIES=all \
  -v ~/workspaces/isaac_ros-dev:/workspaces/isaac_ros-dev \
  --entrypoint /usr/local/bin/scripts/workspace-entrypoint.sh \
  -e HOST_USER_UID=$(id -u) -e HOST_USER_GID=$(id -g) -e USERNAME=admin \
  isaac-ros-apriltag:latest \
  bash -c "sleep infinity"

# Attach a shell
docker exec -it -u admin isaac_ros_dev_container bash

# Stop
docker stop isaac_ros_dev_container && docker rm isaac_ros_dev_container
```

## DO NOT use `isaac-ros activate`

:::danger
`isaac-ros activate` (`/usr/bin/isaac-ros`) will **delete a stopped container and recreate from the base NVIDIA image**, losing all customizations (foxglove-bridge, ffmpeg, FastDDS fix, entrypoint scripts).
:::

Safe for attaching to an **already running** container (just does `docker exec bash`), but never use it to start one.

## Image

| | |
|---|---|
| **Current image** | `isaac-ros-apriltag:latest` (~19.8 GB, hand-built via `docker commit`) |
| **New base (pulled)** | `nvcr.io/nvidia/isaac/ros:noble-ros2_jazzy_d3e84470d576702a380478a513fb3fc6-amd64` |

### Migration Plan

Switching to a reproducible Dockerfile. See [Dockerfile Plan](/robot-docs/isaac-ros/dockerfile-plan/).

New base: **full NVIDIA image** (noble + ros2_jazzy layers) with ROS 2 Jazzy (50+ packages), TensorRT, PyTorch, nav2, rviz2, foxglove-bridge, slam_toolbox, OpenCV, and CUDA dev tools.

**Custom additions on top (Dockerfile):**
- `ros-jazzy-isaac-ros-apriltag` -- GPU AprilTag
- `ros-jazzy-isaac-ros-visual-slam` -- cuVSLAM
- `ros-jazzy-isaac-ros-nvblox` -- GPU 3D reconstruction
- `ros-jazzy-foxglove-compressed-video-transport` -- H.264 NVENC for Foxglove (community package, NOT in base)
- `ros-jazzy-ffmpeg-encoder-decoder` + `ffmpeg` -- NVENC encoding
- FastDDS no-SHM XML at `/etc/fastdds_no_shm.xml`
- Entrypoint scripts for foxglove-bridge + H.264 republisher auto-start

**NOT needed in container:**
- Unitree SDK/packages -- runs on G1's Jetson Orin, not workstation
- System ROS 2 on host -- conflicts with Isaac Sim's Python 3.11

## Installed Packages (on top of base)

```
ros-jazzy-isaac-ros-apriltag                   # GPU AprilTag detection
ros-jazzy-foxglove-bridge                      # WebSocket bridge (port 8765)
ros-jazzy-foxglove-compressed-video-transport   # H.264 NVENC video for Foxglove
ros-jazzy-ffmpeg-encoder-decoder               # ffmpeg with NVENC
ffmpeg                                         # CLI tool
```

## Custom Configs Baked Into Image

### 1. FastDDS UDP-only transport XML

**Path:** `/tmp/fastdds_no_shm.xml`
(Future Dockerfile should move to `/etc/fastdds_no_shm.xml`)

Fixes SHM transport breakage between Isaac Sim (host) and container. Without it, topics are discoverable but data doesn't flow.

Full XML content:

```xml
<?xml version="1.0" encoding="UTF-8" ?>
<dds xmlns="http://www.eprosima.com/XMLSchemas/fastRTPS_Profiles">
    <profiles>
        <transport_descriptors>
            <transport_descriptor>
                <transport_id>udp_transport</transport_id>
                <type>UDPv4</type>
            </transport_descriptor>
        </transport_descriptors>
        <participant profile_name="default_participant" is_default_profile="true">
            <rtps>
                <userTransports>
                    <transport_id>udp_transport</transport_id>
                </userTransports>
                <useBuiltinTransports>false</useBuiltinTransports>
            </rtps>
        </participant>
    </profiles>
</dds>
```

### 2. `FASTRTPS_DEFAULT_PROFILES_FILE` env var

Set in `/etc/profile.d/fastdds-fix.sh` -- login shells pick it up automatically.

### 3. Foxglove-bridge auto-start

**Path:** `/usr/local/bin/scripts/entrypoint_additions/50-foxglove-bridge.sh`

Auto-starts foxglove-bridge on port 8765 at container launch. Also sets the FastDDS env var.

```bash
#!/bin/bash
export FASTRTPS_DEFAULT_PROFILES_FILE=/tmp/fastdds_no_shm.xml
source /opt/ros/jazzy/setup.bash
ros2 launch foxglove_bridge foxglove_bridge_launch.xml port:=8765 &
```

## Running Services

### Foxglove Bridge

- **Port:** 8765 (WebSocket)
- **Connect from Mac:** `ws://100.101.214.44:8765` (Tailscale) or `ws://workstation:8765`
- **Auto-starts** via entrypoint addition when container uses workspace-entrypoint

### H.264 NVENC Republisher

Start manually after container is up:

```bash
docker exec -d isaac_ros_dev_container bash -c \
  "export FASTRTPS_DEFAULT_PROFILES_FILE=/tmp/fastdds_no_shm.xml && \
   source /opt/ros/jazzy/setup.bash && \
   ros2 run image_transport republish raw foxglove --ros-args \
     -r in:=/front_stereo_camera/left/image_rect_color \
     -r out/foxglove:=/front_stereo_camera/left/compressed_video \
     -p out.foxglove.encoder:=h264_nvenc \
     -p out.foxglove.gop_size:=10 \
     -p out.foxglove.bit_rate:=5000000 \
     -p out.foxglove.qmax:=10 \
     -p 'out.foxglove.encoder_av_options:=forced-idr:1,preset:p1,tune:ll'"
```

- Subscribes to raw `/front_stereo_camera/left/image_rect_color`
- Publishes H.264 NVENC encoded to `/front_stereo_camera/left/compressed_video`
- H.265 does NOT work with Foxglove browser (can't decode keyframes)
- NVENC requires `gop_size` > 1

## Host Wrappers

### `~/bin/ros2`

Runs `ros2` inside the container with FastDDS fix:

```bash
docker exec isaac_ros_dev_container bash -c \
  "export FASTRTPS_DEFAULT_PROFILES_FILE=/tmp/fastdds_no_shm.xml && \
   source /opt/ros/jazzy/setup.bash && \
   ros2 $*"
```

## Troubleshooting

### Topics visible but no data flowing

FastDDS XML file or env var is missing. Recreate:

```bash
docker exec isaac_ros_dev_container bash -c 'cat > /tmp/fastdds_no_shm.xml << "XML"
<?xml version="1.0" encoding="UTF-8" ?>
<dds xmlns="http://www.eprosima.com/XMLSchemas/fastRTPS_Profiles">
    <profiles>
        <transport_descriptors>
            <transport_descriptor>
                <transport_id>udp_transport</transport_id>
                <type>UDPv4</type>
            </transport_descriptor>
        </transport_descriptors>
        <participant profile_name="default_participant" is_default_profile="true">
            <rtps>
                <userTransports>
                    <transport_id>udp_transport</transport_id>
                </userTransports>
                <useBuiltinTransports>false</useBuiltinTransports>
            </rtps>
        </participant>
    </profiles>
</dds>
XML'
```

### Foxglove says "Check that WebSocket server is reachable"

foxglove-bridge isn't running. Start it:

```bash
docker exec -d isaac_ros_dev_container bash -c \
  "export FASTRTPS_DEFAULT_PROFILES_FILE=/tmp/fastdds_no_shm.xml && \
   source /opt/ros/jazzy/setup.bash && \
   ros2 launch foxglove_bridge foxglove_bridge_launch.xml port:=8765"
```

### Foxglove connects but no data

foxglove-bridge started without FastDDS fix. Kill and restart:

```bash
docker exec isaac_ros_dev_container bash -c \
  "pkill -f foxglove_bridge; sleep 1; \
   export FASTRTPS_DEFAULT_PROFILES_FILE=/tmp/fastdds_no_shm.xml && \
   source /opt/ros/jazzy/setup.bash && \
   ros2 launch foxglove_bridge foxglove_bridge_launch.xml port:=8765 &"
```

### Container missing packages after restart

`isaac-ros activate` recreated from base image. Use `docker run` from `isaac-ros-apriltag:latest` instead.
