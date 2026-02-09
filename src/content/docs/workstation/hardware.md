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

---

## Hardware Specs

| | |
|---|---|
| **GPU** | NVIDIA GeForce RTX 3090 (24GB VRAM) |
| **RAM** | 62GB |
| **OS** | Ubuntu 24.04.3 LTS |
| **Display Server** | X11 (Xorg) -- required for Isaac Sim |
| **Desktop** | GNOME |

---

## NVIDIA Driver

| | |
|---|---|
| **Driver** | 580.126.09 |
| **CUDA** | 13.0 |

```bash
# GPU status
nvidia-smi

# Driver version
nvidia-smi --query-gpu=driver_version --format=csv

# DKMS module
dkms status
```

---

## GPU Requirements for Isaac Sim

RT Cores + NVENC required.

| Supported | NOT Supported |
|---|---|
| RTX 3090 (workstation) | A100 (no RT Cores) |
| RTX 4080/4090 | H100 (no RT Cores) |
| RTX A4000/A5000/A6000 | Tesla GPUs |

### Minimum Specs

| | |
|---|---|
| **VRAM** | 16GB (RTX 3090 has 24GB) |
| **RAM** | 32GB (workstation has 62GB) |
| **Driver** | 535+ |

---

## tmux Sessions

| Session | Purpose |
|---------|---------|
| `isaac-setup` | Setup and installation |
| `isaac` | Running Isaac Sim |
| `work` | General commands |

```bash
tmux new -s isaac-setup    # Create
tmux attach -t isaac-setup # Attach
tmux ls                    # List
```

---

## Troubleshooting

### NVIDIA driver issues after update

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

Installing `libnvidia-encode-550` can pull in conflicting server packages. The newer driver takes over -- restart GDM.
