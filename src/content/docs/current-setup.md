---
title: Current Setup
description: Accurate snapshot of the workstation's Isaac Sim / ROS 2 / IsaacLab stack (February 2026).
---

## System

| Component | Value |
|-----------|-------|
| **OS** | Ubuntu 24.04.3 LTS (upgraded from 22.04) |
| **Kernel** | 6.8.0-100-generic |
| **Desktop** | GNOME (X11 session on :0) |
| **Shell** | zsh + oh-my-zsh + powerlevel10k |
| **GPU** | NVIDIA RTX 3090 (24 GB VRAM) |
| **NVIDIA Driver** | 580.126.09 |
| **CUDA** | 13.0 |
| **RAM** | 62 GB |
| **Python (system)** | 3.12 |

## Isaac Sim 5.1

| | |
|---|---|
| **Version** | 5.1.0 (pip) |
| **Install** | `isaacsim[all,extscache]==5.1.0` in conda env `isaaclab` |
| **Launch** | `conda activate isaaclab && isaacsim` |
| **Extensions** | 95+ first-party (`isaacsim.*`), 40+ deprecated shims (`omni.isaac.*`), 80+ cached Kit extensions |
| **ROS 2 Bridge** | Enabled (bundled Jazzy libs) |
| **Streaming** | Sunshine + Moonlight (NvFBC + NVENC) |

### Key Isaac Sim Extensions

- **ROS 2:** `isaacsim.ros2.bridge`, `isaacsim.ros2.sim_control`, `isaacsim.ros2.tf_viewer`, `isaacsim.ros2.urdf`
- **Sensors:** `isaacsim.sensors.camera`, `isaacsim.sensors.physics`, `isaacsim.sensors.physx`, `isaacsim.sensors.rtx`
- **Robots:** `isaacsim.robot.manipulators`, `isaacsim.robot.wheeled_robots`, `isaacsim.robot.surface_gripper`
- **Motion:** `isaacsim.robot_motion.lula`, `isaacsim.robot_motion.motion_generation`
- **Replicator:** `isaacsim.replicator.behavior`, `isaacsim.replicator.domain_randomization`, `isaacsim.replicator.synthetic_recorder`
- **Asset Import:** `isaacsim.asset.importer.urdf`, `isaacsim.asset.importer.mjcf`

### Custom modifications (pip install)

- **FPS cap:** 60 Hz (`isaacsim.exp.full.kit` -> `rateLimitFrequency = 60`)
- **Layout persistence:** Auto-save on shutdown, auto-restore on startup (hooks in `extension.py`)
- **Launch wrapper:** `~/bin/isaacsim-ros` -- sets env vars for bundled ROS 2 bridge
- **Dock shortcut:** `~/.local/share/applications/isaacsim-ros.desktop`
- **`~/bin/ros2`** -- wrapper that runs `ros2` commands inside the Docker container with FastDDS UDP fix

## Isaac Lab 2.3.2

| | |
|---|---|
| **Location** | `~/IsaacLab/` |
| **Conda Env** | `isaaclab` (Python 3.11) |
| **PyTorch** | 2.7.0+cu128 |
| **isaacsim pip** | 5.1.0.0 (all sub-packages) |

Activate: `conda activate isaaclab`

Editable packages: `isaaclab` 0.54.2, `isaaclab_assets` 0.2.4, `isaaclab_tasks` 0.11.12, `isaaclab_rl` 0.4.7, `isaaclab_mimic` 1.0.16, `isaaclab_contrib` 0.0.2.

## ROS 2

**No system ROS 2 on host.** Previous `ros-jazzy-desktop` (295 packages) removed Feb 2026 -- conflicted with Isaac Sim's bundled Python 3.11 rclpy.

| Component | Where |
|-----------|-------|
| **Isaac Sim ROS 2 bridge** | Bundled in pip package (Python 3.11 rclpy + Jazzy libs). Launch via `~/bin/isaacsim-ros` |
| **ros2 CLI, foxglove-bridge** | Inside Isaac ROS Docker container |
| **Isaac ROS packages** | Inside Isaac ROS Docker container |

## Docker / Isaac ROS

| | |
|---|---|
| **Docker** | 29.2.1 |
| **nvidia-container-toolkit** | 1.18.2 (CDI support) |
| **NVIDIA runtime** | Registered |
| **isaac-ros-cli** | v2.0.0 |
| **Container** | `isaac_ros_dev_container` (`--network host`, `--runtime nvidia`, `--ipc host`) |
| **Committed image** | `isaac-ros-apriltag:latest` (~19 GB) |

### Packages installed in container

- `ros-jazzy-isaac-ros-apriltag` -- GPU-accelerated AprilTag detection
- `ros-jazzy-foxglove-bridge` -- WebSocket bridge (port 8765)
- `ros-jazzy-foxglove-compressed-video-transport` -- H.264 NVENC video encoding for Foxglove (H.265 doesn't work with browser decoding)
- `ros-jazzy-ffmpeg-encoder-decoder` -- ffmpeg with NVENC support

### DDS cross-container fix

FastDDS SHM transport breaks between Isaac Sim (host) and container. All `ros2` commands in container must set:
```bash
export FASTRTPS_DEFAULT_PROFILES_FILE=/tmp/fastdds_no_shm.xml
```
Forces UDP-only transport (no shared memory). Must be recreated after container restart.

## Remote Access

| Method | Details |
|--------|---------|
| **Sunshine + Moonlight** | NvFBC + NVENC, hardware streaming |
| **Foxglove** | `ws://workstation:8765` (bridge runs in Isaac ROS container) |
| **Tailscale** | `workstation.tailee9084.ts.net` / `100.101.214.44` |

## Missing Tools

- **uv** -- not installed (needed for Isaac Sim MCP Server plan)
