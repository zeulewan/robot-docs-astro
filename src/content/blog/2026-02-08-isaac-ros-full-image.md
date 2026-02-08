---
title: "Switching to Full NVIDIA Isaac ROS Image"
date: 2026-02-08
authors:
  - zeul
tags:
  - isaac-ros
  - docker
  - unitree-g1
---

Switching the Isaac ROS container from a minimal base image with manual package installs to the full NVIDIA `noble-ros2_jazzy` image.

<!-- more -->

## Background

The `isaac_ros_dev_container` was built interactively -- packages installed via `docker exec`, then saved with `docker commit`. No Dockerfile, non-reproducible. If `isaac-ros activate` runs on a stopped container, it wipes everything and recreates from the base image.

## The Decision

NVIDIA provides three Dockerfile layers that stack into one image:

1. **`Dockerfile.isaac_ros`** (~1.5 GB) -- Ubuntu Noble, ROS base, apt repos
2. **`Dockerfile.noble`** (~15 GB) -- TensorRT, PyTorch, CUDA dev, VPI, Triton
3. **`Dockerfile.ros2_jazzy`** (~3 GB) -- 50+ ROS 2 packages, nav2, rviz2, foxglove-bridge, slam_toolbox

Current container uses only Tier 1 + manually installed packages, ending up at ~19 GB anyway. Full image is the same size but includes nav2, rviz2, slam_toolbox, and many more packages.

## Why Now

Unitree G1 development needs both simulation (Isaac Sim) and real robot (Jetson Orin) support. The full image includes everything:

- **foxglove-bridge** (pre-installed) -- WebSocket visualization
- **nav2** (pre-installed) -- navigation stack for G1
- **rviz2** (pre-installed) -- 3D visualization
- **slam_toolbox** (pre-installed) -- SLAM algorithms
- **CycloneDDS** (pre-installed) -- matches G1's DDS middleware

Only a few packages need adding on top: Isaac ROS perception (AprilTag, cuVSLAM, nvblox), H.264 NVENC transport, and custom entrypoint scripts.

## Image Pulled

```
nvcr.io/nvidia/isaac/ros:noble-ros2_jazzy_d3e84470d576702a380478a513fb3fc6-amd64
```

Most layers already existed locally from the base image -- pull was fast.

## Next Steps

- Write the Dockerfile (~40 lines on top of the full base)
- Create docker-compose.yml
- Build, verify, and swap the container
- Update `~/bin/ros2` wrapper to use `/etc/fastdds_no_shm.xml` instead of `/tmp/`
