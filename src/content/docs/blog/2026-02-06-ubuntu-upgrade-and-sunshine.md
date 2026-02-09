---
title: "Ubuntu 24.04 Upgrade and Sunshine Rebuild"
date: 2026-02-06
authors:
  - zeul
tags:
  - workstation
  - ubuntu
  - sunshine
  - nvidia
---

Today I finally pulled the trigger on upgrading the workstation from Ubuntu 22.04 to 24.04. I'd been putting it off because this machine runs my entire robotics stack, but it had to happen eventually.

<!-- excerpt -->

## The OS Upgrade

Before touching anything, I backed up the configs I knew I'd regret losing:

- `/etc/X11/xorg.conf` (my custom virtual display + EDID config for headless Sunshine streaming)
- `~/.zshrc`
- `~/.config/sunshine/` and the systemd overrides

Then I ran `do-release-upgrade` and let it churn. It went surprisingly smoothly -- post-reboot health checks confirmed Docker 29.2.1, nvidia-container-toolkit 1.18.2, and all my conda environments survived intact.

## NVIDIA Driver 570 to 580

While I was at it, I upgraded to driver 580.126.09 (CUDA 13.0). Reboot, `nvidia-smi`, RTX 3090 looking healthy. Nothing exciting, which is exactly how driver upgrades should go.

## Sunshine Broke (Of Course)

This one I expected. The 22.04 Sunshine .deb immediately complained about a missing `libicuuc.so.70` -- 24.04 ships `libicuuc.so.74` instead. Grabbed the 24.04 .deb from the LizardByte releases page, installed it, restored my config backup and systemd overrides, and confirmed NvFBC capture and NVENC encoding were back.

I also wrote `~/set-resolution.sh`, a Sunshine pre-session script that matches the display resolution to whatever the client requests via `nvidia-settings`. Nice quality-of-life improvement for streaming from my MacBook.

## Toshy Venv Fix

The Toshy keyboard remapper broke too -- its Python venv was built against 3.10 and 24.04 ships 3.12. Quick fix: re-ran the Toshy setup script to rebuild the venv. One of those things you only figure out after staring at a cryptic import error for a few minutes.

## Claude Skills Cleanup

Since I was already in spring-cleaning mode, I reorganized my Claude skill docs in `~/.claude/skills/` -- overhauled the Isaac Sim skill for the pip-based install, consolidated the dock app skills into a single file, and updated the README.
