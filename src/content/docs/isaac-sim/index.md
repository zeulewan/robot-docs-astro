---
title: Isaac Sim & Isaac Lab
description: Isaac Sim 5.1 and Isaac Lab 2.3.2 installation, configuration, ROS 2 bridge, and troubleshooting.
---

## Current Installation

| | |
|---|---|
| **Version** | 5.1.0 (pip) |
| **Conda Env** | `isaaclab` (Python 3.11) |
| **Launch** | `conda activate isaaclab && isaacsim` |

---

## Quick Start

```bash
conda activate isaaclab
isaacsim
```

Must launch from a desktop session (Sunshine/Moonlight) -- needs a display.

---

## Installation Methods

### Via pip (Current)

```bash
# Requires Python 3.11, GLIBC 2.35+
conda activate isaaclab
pip install "isaacsim[all,extscache]==5.1.0" --extra-index-url https://pypi.nvidia.com
```

### Standalone (Removed Feb 2026)

Standalone install at `~/isaac-sim/` removed to save 17 GB. Pip version has the same features (GUI, ROS 2 bridge, streaming, all extensions).

---

## Isaac Lab

Separate framework for robot learning built on Isaac Sim.

### Installation (after Isaac Sim works)

```bash
git clone https://github.com/isaac-sim/IsaacLab.git
cd IsaacLab
./isaaclab.sh --install
```

### Isaac Sim vs Isaac Lab

| | Isaac Sim | Isaac Lab |
|---|-----------|-----------|
| **Purpose** | Full simulation platform + GUI | Robot learning framework |
| **Use** | Design environments, test robots | Train RL policies at scale |
| **Mode** | Interactive GUI | Headless training |
| **Install** | pip in conda env | Separate from source |

---

## ROS 2 Bridge (bundled in Isaac Sim)

- ROS 2 Jazzy libs bundled in `isaacsim.ros2.bridge` extension (Python 3.11)
- Requires env vars to load correctly -- use `~/bin/isaacsim-ros` wrapper
- **No system ROS 2 on host** -- conflicts (Python 3.12 vs 3.11 ABI mismatch)
- DDS handles communication between Isaac Sim and external ROS 2 nodes regardless of Python version

---

## Foxglove Visualization

- foxglove-bridge on port 8765 (container, `--network host` so accessible on localhost)
- Connect from any browser: `ws://workstation:8765` or `ws://localhost:8765`
- Raw images too large for remote viewing -- use compressed video topic instead
- H.264 NVENC republisher encodes on GPU (~50KB/frame vs 2.7MB raw)
- H.265 NVENC does not work with Foxglove (browser can't decode keyframes)
- NVENC requires `gop_size` > 1 (B-frame constraint). Use `gop_size:=10`

### Republisher command

```bash
ros2 run image_transport republish raw foxglove --ros-args \
  -r in:=/front_stereo_camera/left/image_rect_color \
  -r out/foxglove:=/front_stereo_camera/left/compressed_video \
  -p out.foxglove.encoder:=h264_nvenc \
  -p out.foxglove.gop_size:=10 \
  -p out.foxglove.bit_rate:=5000000 \
  -p out.foxglove.qmax:=10 \
  -p out.foxglove.encoder_av_options:='forced-idr:1,preset:p1,tune:ll'
```

---

## Software Versions (February 2026)

| Component | Version | Notes |
|-----------|---------|-------|
| Isaac Sim | 5.1.0 | Pip install in conda env `isaaclab` |
| Isaac Sim | 6.0 | Early dev preview only |
| Isaac Lab | 2.3.2 | Separate install from source |

---

## Troubleshooting

### Isaac Sim won't start

```bash
# Check DISPLAY variable
echo $DISPLAY  # Should be :0

# Check GPU
nvidia-smi

# Check logs
ls ~/.nvidia-omniverse/logs/Kit/Isaac-Sim/*/kit_*.log
```

### Slow first launch

Normal -- shader compilation takes 5-10 minutes on first run. Subsequent launches are faster.

### GPU not being used

```bash
# Check GPU memory usage while Isaac Sim runs
watch -n 1 nvidia-smi
```

---

## File Locations

```
~/.nvidia-omniverse/logs/       # Isaac Sim logs
~/.local/share/ov/data/Kit/     # Kit config and layout files
~/IsaacLab/                     # Isaac Lab source
```

---

## References -- The Three Doc Sites

| Site | What it is | Follow for install? |
|------|-----------|----------------------|
| [Isaac Lab](https://isaac-sim.github.io/IsaacLab/main/index.html) | Robot learning framework (includes Isaac Sim pip install) | **YES** -- our Isaac Sim + Isaac Lab install source |
| [Isaac Sim](https://docs.isaacsim.omniverse.nvidia.com/5.1.0/) | Simulator docs (GUI, sensors, ROS bridge config, extensions) | **NO** for install. Reference only |
| [Isaac ROS](https://nvidia-isaac-ros.github.io/index.html) | GPU-accelerated ROS 2 perception packages | **YES** -- our Isaac ROS Docker install source |

Isaac Lab handles Isaac Sim installation. Isaac Sim docs are reference only (don't follow their install guides). Isaac ROS uses Docker (no host ROS 2).
