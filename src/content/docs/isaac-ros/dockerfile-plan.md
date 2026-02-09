---
title: Dockerfile Plan
description: Reproducible Dockerfile for the Isaac ROS container.
---

Reproducible Dockerfile for the `isaac_ros_dev_container`.

---

## Context

The container (19.8 GB) was built interactively via `docker exec` + `docker commit`. No Dockerfile exists.

Target robot: Unitree G1 humanoid (Jetson Orin onboard). Container serves both simulation and real robot modes.

---

## Decisions

| Decision | Choice |
|---|---|
| **Base image** | `nvcr.io/nvidia/isaac/ros:noble-ros2_jazzy_d3e84470d576702a380478a513fb3fc6-amd64` (~19 GB, already pulled) |
| **docker-compose.yml** | Yes |
| **Launch method** | `docker compose up -d` with `workspace-entrypoint.sh` |
| **Container name** | `isaac_ros_dev_container` (required by `~/bin/ros2` wrapper and `isaac-ros activate`) |
| **Unitree packages** | NOT needed -- G1 runs `unitree_ros2` on its Orin, publishes standard ROS 2 topics |

`isaac-ros activate` compatibility: partial. Can set `base_image_keys` in `~/.config/isaac-ros-cli/config.yaml`, but if the container is stopped/missing, it recreates from scratch. Always start with `docker compose up -d` first.

---

## NVIDIA Reference Dockerfiles

At `https://github.com/NVIDIA-ISAAC-ROS/isaac-ros-cli` under `docker/`:

| File | What it does | Size |
|------|-------------|------|
| `Dockerfile.isaac_ros` | **Our base.** Ubuntu Noble + apt repos (CUDA, Isaac ROS, Jetson, ROS2) + `ros-jazzy-ros-base` + entrypoint scripts | ~1.46 GB |
| `Dockerfile.noble` | CUDA 13.0 dev + TensorRT + VPI + CV-CUDA + PyTorch + Triton + dev libs | ~15 GB+ |
| `Dockerfile.ros2_jazzy` | Full ROS2 Jazzy: 50+ packages, nav2, rviz2, foxglove, custom OpenCV | ~3 GB+ |

---

## Packages to Add on Top of Base

Base already includes foxglove-bridge, image-transport, nav2, rviz2, slam_toolbox, TensorRT, PyTorch, OpenCV, and CUDA dev tools. Only these need adding:

```
ros-jazzy-isaac-ros-apriltag          # GPU AprilTag detection
ros-jazzy-isaac-ros-visual-slam       # cuVSLAM (visual SLAM)
ros-jazzy-isaac-ros-nvblox            # GPU 3D reconstruction
ros-jazzy-foxglove-compressed-video-transport  # H.264 NVENC for Foxglove (community package, NOT in base)
ros-jazzy-ffmpeg-encoder-decoder      # ffmpeg with NVENC
ffmpeg                                # CLI tool
```

---

## Custom Configs

### 1. FastDDS no-SHM XML

Currently at `/tmp/fastdds_no_shm.xml` (ephemeral). Bake into `/etc/fastdds_no_shm.xml`.

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

### 2. `admin` user

`useradd -m -s /bin/bash -G sudo admin` with UID 996 (matching host `zeul`). Required by `isaac-ros activate`.

### 3. `FASTRTPS_DEFAULT_PROFILES_FILE` env var

Do not use `.bashrc` -- workspace-entrypoint.sh overwrites it. Use `/etc/profile.d/fastdds-fix.sh` pointing to `/etc/fastdds_no_shm.xml`.

### 4. Foxglove-bridge auto-start

At `/usr/local/bin/scripts/entrypoint_additions/50-foxglove-bridge.sh`. Must set `FASTRTPS_DEFAULT_PROFILES_FILE` explicitly -- entrypoint sources scripts, not login shells.

```bash
#!/bin/bash
export FASTRTPS_DEFAULT_PROFILES_FILE=/etc/fastdds_no_shm.xml
source /opt/ros/jazzy/setup.bash
ros2 launch foxglove_bridge foxglove_bridge_launch.xml port:=8765 &
```

### 5. H.264 NVENC republisher auto-start

At `/usr/local/bin/scripts/entrypoint_additions/60-h264-republisher.sh`.

```bash
#!/bin/bash
export FASTRTPS_DEFAULT_PROFILES_FILE=/etc/fastdds_no_shm.xml
source /opt/ros/jazzy/setup.bash
ros2 run image_transport republish raw foxglove --ros-args \
  -r in:=/front_stereo_camera/left/image_rect_color \
  -r out/foxglove:=/front_stereo_camera/left/compressed_video \
  -p out.foxglove.encoder:=h264_nvenc \
  -p out.foxglove.gop_size:=10 \
  -p out.foxglove.bit_rate:=5000000 \
  -p out.foxglove.qmax:=10 \
  -p 'out.foxglove.encoder_av_options:=forced-idr:1,preset:p1,tune:ll' &
```

### AprilTag node launch with topic remapping

Default `isaac_ros_apriltag` subscribes to `/image` and `/camera_info`. Remap for carter warehouse:

```bash
ros2 launch isaac_ros_apriltag isaac_ros_apriltag.launch.py \
  --ros-args \
  -r image:=/front_stereo_camera/left/image_rect_color \
  -r camera_info:=/front_stereo_camera/left/camera_info
```

Consider adding as an entrypoint addition script (e.g. `60-apriltag.sh`).

### NOT needed in Dockerfile (auto-generated or runtime junk)

- `/etc/libibverbs.d/*`, `/etc/nvsciipc*`, `/etc/cufile.json` -- created by CUDA/NVIDIA package installs
- `/etc/ghostscript/*`, `/etc/java/*`, `/etc/pulse/*` -- created by dependency installs
- `/tmp/launch_params_*`, `/tmp/*.log` -- runtime temp files

---

## Build Process

### Step 1: Backups

```bash
# Tag old hand-built image
docker tag isaac-ros-apriltag:latest isaac-ros-apriltag:old
```

Full base image already pulled.

### Step 2: Create `~/workspaces/isaac_ros-dev/Dockerfile`

~40 lines: apt install Isaac ROS perception + foxglove H.264 transport + ffmpeg, create admin user, write FastDDS XML to `/etc/`, add `/etc/profile.d/fastdds-fix.sh` (NOT .bashrc -- entrypoint overwrites it), create entrypoint addition scripts, set CMD/WORKDIR.

foxglove-bridge, image-transport, nav2, rviz2, TensorRT, PyTorch, OpenCV already in base -- do NOT re-install.

### Step 3: Create `~/workspaces/isaac_ros-dev/docker-compose.yml`

Container name `isaac_ros_dev_container`, runtime nvidia, host networking, ipc host, privileged, workspace bind mount.

### Step 4: Build

```bash
docker build -t isaac-ros-apriltag:latest ~/workspaces/isaac_ros-dev/
```

### Step 5: Sanity checks (throwaway containers)

```bash
docker run --rm isaac-ros-apriltag:latest dpkg -l | grep isaac-ros-apriltag
docker run --rm isaac-ros-apriltag:latest id admin
docker run --rm isaac-ros-apriltag:latest cat /etc/fastdds_no_shm.xml
docker run --rm isaac-ros-apriltag:latest cat /etc/profile.d/fastdds-fix.sh
docker run --rm isaac-ros-apriltag:latest cat /usr/local/bin/scripts/entrypoint_additions/50-foxglove-bridge.sh
docker run --rm isaac-ros-apriltag:latest cat /usr/local/bin/scripts/entrypoint_additions/60-h264-republisher.sh
```

### Step 6: Swap container

```bash
docker stop isaac_ros_dev_container && docker rm isaac_ros_dev_container
docker compose -f ~/workspaces/isaac_ros-dev/docker-compose.yml up -d
```

### Step 7: Update `~/bin/ros2` wrapper

Change FastDDS XML path from `/tmp/` to `/etc/`.

### Step 8: Live verification

- `ros2 topic list` from host
- `isaac-ros activate` works

### Step 9: Cleanup (optional, after everything works)

```bash
docker rmi isaac-ros-apriltag:old   # Remove old 19GB hand-built image
```

### Step 10: Update skill docs

---

## Files to Create/Modify

| File | Action |
|------|--------|
| `~/workspaces/isaac_ros-dev/Dockerfile` | Create |
| `~/workspaces/isaac_ros-dev/docker-compose.yml` | Create |
| `~/bin/ros2` | Update FastDDS XML path |

---

## Verification Checklist

1. `docker build` completes without errors
2. `dpkg -l | grep isaac-ros-apriltag` -- package present
3. `id admin` -- UID 996
4. `cat /etc/fastdds_no_shm.xml` -- config present
5. `FASTRTPS_DEFAULT_PROFILES_FILE` set via `/etc/profile.d/fastdds-fix.sh` (NOT .bashrc)
6. Entrypoint addition scripts exist and are executable
7. `docker compose up -d` then `ros2 topic list` from host works
8. `isaac-ros activate` can attach to running container (never use it to start one)
9. foxglove-bridge auto-starts on port 8765
10. H.264 NVENC republisher auto-starts, `/front_stereo_camera/left/compressed_video` topic exists
11. Foxglove Studio connects to `ws://100.101.214.44:8765` and shows camera feed
12. `/tag_detections` topic publishes when AprilTag node is launched

---

## Lessons Learned

- **`isaac-ros activate` is destructive** -- if container is stopped, it `docker rm`s and recreates from base image. All customizations lost.
- **`.bashrc` gets overwritten** -- workspace-entrypoint.sh copies `/etc/skel/.[^.]*` over the user's home dir. Use `/etc/profile.d/` for persistent env vars.
- **Entrypoint addition scripts must set their own env** -- they're `source`d, not login shells, so `/etc/profile.d/` doesn't apply. Each script must explicitly `export FASTRTPS_DEFAULT_PROFILES_FILE=...`.
- **Headless Isaac Sim needs explicit extension enable** -- `SimulationApp({"headless": True})` does NOT auto-enable `isaacsim.ros2.bridge`. Call `ext_manager.set_extension_enabled_immediate("isaacsim.ros2.bridge", True)` before loading scenes.
- **FastDDS SHM is the silent killer** -- topics discoverable across host/container but data doesn't flow. Both the XML file AND env var must exist.
- **H.265 NVENC doesn't work with Foxglove** -- browser can't decode keyframes. Use H.264 only.
