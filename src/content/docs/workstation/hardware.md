---
title: "Hardware & Specs"
description: "Workstation hardware, GPU, and system specifications."
---

## Access

| | |
|---|---|
| **Hostname** | workstation |
| **Tailscale DNS** | workstation.tailee9084.ts.net |
| **Tailscale IPv4** | 100.101.214.44 |
| **Local IP** | 192.168.2.239 (UniFi LAN) |

## Hardware Specs

| | |
|---|---|
| **GPU** | NVIDIA GeForce RTX 3090 (24GB VRAM) |
| **RAM** | 62GB |
| **OS** | Ubuntu 24.04.3 LTS |
| **Display Server** | X11 (Xorg) - required for Isaac Sim |
| **Desktop** | GNOME |

## NVIDIA Driver

| | |
|---|---|
| **Driver** | 580.126.09 |
| **CUDA** | 13.0 |

### Check GPU Status

```bash
nvidia-smi
```

### Check Driver Version

```bash
nvidia-smi --query-gpu=driver_version --format=csv
```

### Check DKMS Module

```bash
dkms status
```

## GPU Requirements for Isaac Sim

### Supported (RT Cores + NVENC required)

- RTX 3090 (workstation)
- RTX 4080/4090
- RTX A4000/A5000/A6000

### NOT Supported

- A100 (no RT Cores)
- H100 (no RT Cores)
- Tesla GPUs

### Minimum Specs

- 16GB VRAM (RTX 3090 has 24GB)
- 32GB RAM (workstation has 62GB)
- Driver 535+

## tmux Sessions

| Session | Purpose |
|---------|---------|
| `isaac-setup` | Setup and installation |
| `isaac` | Running Isaac Sim |
| `work` | General commands |

```bash
# Create
tmux new -s isaac-setup

# Attach
tmux attach -t isaac-setup

# List
tmux ls
```

## Troubleshooting

### NVIDIA driver issues after update

If NVIDIA package installs cause driver conflicts:

```bash
# Check what's installed
dpkg -l | grep nvidia | grep -E '^ii'

# Check DKMS status
dkms status

# Restart GDM to reload driver
sudo systemctl restart gdm

# Verify driver works
nvidia-smi
```

**Note:** Installing `libnvidia-encode-550` can pull in conflicting server packages. The newer driver takes over -- just restart GDM.
