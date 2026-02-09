---
title: "ROS 2 Bridge, Headless Isaac Sim, and Wrapper Scripts"
date: 2026-02-07
authors:
  - zeul
tags:
  - isaac-sim
  - isaac-ros
  - ros2
  - foxglove
---

Big day. I got Isaac Sim's ROS 2 bridge actually publishing topics, wrote a headless launch script, and verified the whole pipeline end-to-end with Foxglove. This is the first time I've seen camera feeds flow from simulation all the way through to a visualization tool.

<!-- excerpt -->

## Getting the ROS 2 Bridge to Work

This took longer than it should have. Isaac Sim 5.1 bundles its own ROS 2 Jazzy bridge built against Python 3.11, and it absolutely does not play nice with a system ROS 2 install. The ABI conflicts between the bundled rclpy and system libraries are fatal -- I'm talking instant segfaults.

The solution turned out to be: don't install system ROS 2 at all. Instead:

- **Isaac Sim side**: Set `ROS_DISTRO=jazzy`, `RMW_IMPLEMENTATION=rmw_fastrtps_cpp`, and add the bundled bridge's `lib/` and `rclpy/` dirs to `LD_LIBRARY_PATH` and `PYTHONPATH`
- **ROS 2 CLI**: Run `ros2` commands inside the Isaac ROS Docker container via a wrapper script

Two completely separate ROS environments that talk over DDS. It works.

## Wrapper Scripts

To make this less painful day-to-day, I wrote two scripts in `~/bin/`:

- **`isaacsim-ros`** -- launches Isaac Sim with all the ROS 2 bridge environment variables pre-configured. Just run `isaacsim-ros` and it handles the rest.
- **`ros2`** -- runs `ros2` commands inside the `isaac_ros_dev_container` Docker container with FastDDS SHM-bypass config applied. So `ros2 topic list` and `ros2 topic hz /clock` just work from my terminal like normal.

## Headless Launch Script

I created `~/Documents/isaac-sim-scripts/headless-sample-scene.sh` to run Isaac Sim without a GUI. The tricky part: headless mode doesn't auto-enable the ROS 2 bridge extension, so OmniGraph nodes like `ROS2CameraHelper` silently do nothing. You have to explicitly enable `isaacsim.ros2.bridge` via the extension manager. That one cost me a solid hour of wondering why my camera topics were empty.

The script loads the carter warehouse AprilTag scene via `get_assets_root_path()` (resolves to NVIDIA's S3 CDN) and logs cache hit/miss stats so I can track asset downloads.

## The FastDDS Shared Memory Trap

This was the sneakiest bug of the day. Cross-process DDS between Isaac Sim on the host and the Isaac ROS Docker container *looks* like it works -- `ros2 topic list` shows all the topics. But `ros2 topic hz` shows nothing. Zero data flowing.

The culprit: FastDDS shared memory transport. It can discover topics across the container boundary but can't actually move data. The fix is a FastDDS XML profile that forces UDP-only transport, set via `FASTRTPS_DEFAULT_PROFILES_FILE`. Once I figured that out, everything lit up.

## Foxglove Verification

Launched Foxglove Studio and there it was -- camera feeds, `/clock`, `/tf`, and `/tag_detections` all flowing from headless Isaac Sim through the Isaac ROS container. Genuinely satisfying to see it all come together.

## Storage Architecture Plan

Also drafted a plan to mount the 1.8 TB Seagate FireCuda 520 as `/data`, replace the oversized 121 GB swap partition with a 16 GB swapfile, and bind-mount the Isaac Sim asset cache (`~/.cache/ov/`) to the faster drive. That's a project for another day.

## Skills and Docs

Updated the Isaac Sim skill docs with the new ROS 2 bridge architecture, headless launch workflow, and all the lessons learned. Added workstation, networking, and Moonlight skills too.
