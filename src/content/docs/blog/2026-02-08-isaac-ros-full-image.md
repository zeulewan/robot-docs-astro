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

I'm ditching the hand-built Isaac ROS container in favor of NVIDIA's full `noble-ros2_jazzy` image. The current setup is a house of cards and it's time to fix that before I go any further with G1 development.

<!-- excerpt -->

## The Problem

My `isaac_ros_dev_container` was built interactively -- `docker exec` to get a shell, install packages by hand, `docker commit` to save. No Dockerfile, completely non-reproducible. Worse, if `isaac-ros activate` runs on a stopped container, it wipes everything. I've already lost work to this once. Not again.

## What NVIDIA Gives You

NVIDIA provides three Dockerfile layers that stack into one image:

| Layer | Size | Contents |
|---|---|---|
| `Dockerfile.isaac_ros` | ~1.5 GB | Ubuntu Noble, ROS base, apt repos |
| `Dockerfile.noble` | ~15 GB | TensorRT, PyTorch, CUDA dev, VPI, Triton |
| `Dockerfile.ros2_jazzy` | ~3 GB | 50+ ROS 2 packages, nav2, rviz2, foxglove-bridge, slam_toolbox |

My current container uses only the first layer plus manually installed packages, and it's already ~19 GB. The full image is the same size but includes nav2, rviz2, slam_toolbox, and a lot more. No reason not to use it.

## Why Now

I'm starting to build toward actual Unitree G1 development, which needs both simulation (Isaac Sim) and real robot (Jetson Orin) support. The full image already includes everything I was going to need:

- **foxglove-bridge** -- WebSocket visualization
- **nav2** -- navigation stack for the G1
- **rviz2** -- 3D visualization
- **slam_toolbox** -- SLAM algorithms
- **CycloneDDS** -- matches the G1's DDS middleware

Only a handful of packages need adding on top: Isaac ROS perception (AprilTag, cuVSLAM, nvblox), H.264 NVENC transport, and custom entrypoint scripts.

## Image Pulled

```
nvcr.io/nvidia/isaac/ros:noble-ros2_jazzy_d3e84470d576702a380478a513fb3fc6-amd64
```

Most layers already existed locally from the base image, so the pull was fast.

## Next Steps

- Write the Dockerfile (~40 lines on top of the full base)
- Create docker-compose.yml
- Build, verify, and swap the container
- Update `~/bin/ros2` wrapper to use `/etc/fastdds_no_shm.xml` instead of `/tmp/`
