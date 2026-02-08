---
title: Current Progress
description: What's currently in progress and what's next.
---

## In Progress

### Isaac ROS Dockerfile
**Status:** Full NVIDIA base image pulled, plan ready, awaiting Dockerfile creation.

Replacing hand-built `isaac_ros_dev_container` (19.8 GB, `docker commit`) with a reproducible Dockerfile.

- **Base image:** `nvcr.io/nvidia/isaac/ros:noble-ros2_jazzy_d3e84470d576702a380478a513fb3fc6-amd64` (pulled Feb 8, 2026)
- **Custom additions:** Isaac ROS perception packages (AprilTag, cuVSLAM, nvblox), H.264 NVENC transport, FastDDS fix, entrypoint scripts
- **Full plan:** [Dockerfile Plan](/robot-docs/isaac-ros/dockerfile-plan/)

## Next Steps

### Isaac Sim MCP Server
Control Isaac Sim from Claude Code via MCP. Plan exists, needs path updates for pip-installed Isaac Sim 5.1.

### Performance Tuning to 60fps
- Lower viewport resolution (1280x720 -> 960x540)
- Reduce physics solver iterations (4 -> 2)
- Disable unnecessary rendering (reflections, translucency, AO)
- Set physics CPU threads to 1

### Privacy Scanner
GitHub Actions workflow to scan commits for leaked credentials before they reach the public repo.

## Completed

- Workstation OS upgrade from Ubuntu 22.04 to 24.04 (Feb 2026)
- Isaac Sim migration from standalone to pip install (Feb 2026)
- System ROS 2 removal (conflicted with Isaac Sim's Python 3.11)
- Isaac ROS container with AprilTag + Foxglove + H.264 NVENC
- Tailscale peer relay setup (tsrelay in Toronto)
- Architecture documentation for sim + real G1 workflows
- Full NVIDIA Docker image pulled for container migration
