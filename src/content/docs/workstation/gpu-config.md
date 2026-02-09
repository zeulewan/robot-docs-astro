---
title: "GPU & Sunshine Config"
description: "NVIDIA persistence mode, Sunshine streaming server, and display configuration."
---

## GPU Configuration

### NVIDIA Persistence Mode

| | |
|---|---|
| **Status** | Enabled (set Feb 2026 to fix 30fps idle issue) |
| **Service** | `nvidia-persistenced` |
| **Override** | `/etc/systemd/system/nvidia-persistenced.service.d/override.conf` |
| **Verify** | `nvidia-smi -q \| grep "Persistence Mode"` -- should show `Enabled` |
| **Manual enable** | `sudo nvidia-smi -pm 1` |

The stock service at `/usr/lib/systemd/system/nvidia-persistenced.service` has `--no-persistence-mode` which defeats the purpose. Override removes that flag.

---

### Known Issue: 30fps after idle

- **Symptom**: Sunshine drops to 30fps after not streaming for a while
- **Root cause**: Without persistence mode, GPU drops to P8 (low power) when idle, throttling NVENC encoder
- **Fix applied**:
  1. Enabled persistence mode (`nvidia-smi -pm 1`)
  2. Created systemd override to persist across reboots
  3. Disabled DPMS in xorg.conf and via `xset -dpms`
- If it recurs: check `nvidia-smi` for P-state (should not be P8) and persistence mode status

---

## X11 / Display Configuration

### xorg.conf

| | |
|---|---|
| **Path** | `/etc/X11/xorg.conf` |
| **Display** | Headless virtual (no physical monitor) |
| **Connected monitor** | `DP-0` (virtual) |
| **DPMS** | Disabled (`Option "DPMS" "false"`) |
| **DPI** | 96x96 |
| **ModeValidation** | Permissive (AllowNonEdidModes, NoVirtualSizeCheck, etc.) |

### Available Resolutions (MetaModes)

| Resolution | Refresh | Use Case |
|------------|---------|----------|
| 3024x1964 | 60Hz | MacBook Pro 14" native (with notch) |
| 3024x1890 | 60Hz | MacBook Pro 14" native (no notch) |
| 3440x1440 | 60Hz | Ultrawide |
| 3840x2160 | 60Hz | 4K |
| 2560x1440 | 60Hz | 1440p |
| 1920x1080 | 120Hz | 1080p high refresh |
| 1920x1080 | 60Hz | 1080p standard |
| 1600x900 | 60Hz | Low-res fallback |

### DPMS / Screen Blanking

- DPMS disabled (both in xorg.conf and at runtime via `xset -dpms`)
- Screen saver off (`xset s off`)
- Prevents GPU from downclocking when idle

---

### GDM

| | |
|---|---|
| **Autologin** | Enabled (user: zeul) |
| **Wayland** | Disabled (X11 only for NVIDIA KMS capture) |
| **Config** | `/etc/gdm3/custom.conf` |

---

## Sunshine Configuration

### Service

| | |
|---|---|
| **Type** | systemd user service (`sunshine.service`) |
| **Auto-start** | On graphical session |
| **Override** | `~/.config/systemd/user/sunshine.service.d/override.conf` |
| **Restart** | `Restart=always`, `RestartSec=5` |

### Config (`~/.config/sunshine/sunshine.conf`)

```ini
nvenc_preset = 1
nvenc_twopass = disabled
fec_percentage = 10
origin_web_ui_allowed = wan
qp = 10
```

### Resolution Auto-Switch

| | |
|---|---|
| **Script** | `~/set-resolution.sh` |
| **Trigger** | Sunshine prep-cmd on stream start |
| **Env vars** | `SUNSHINE_CLIENT_WIDTH` / `SUNSHINE_CLIENT_HEIGHT` |
| **Method** | `nvidia-settings -a CurrentMetaMode` |
| **Log** | `/tmp/sunshine-res.log` |

### Web UI

`https://localhost:47990` (or via Tailscale IP). Remote access enabled (`origin_web_ui_allowed = wan`).

---

## Quick Reference

```bash
# GPU status
nvidia-smi

# Display config
DISPLAY=:0 xrandr

# Sunshine logs
journalctl --user -u sunshine -n 50

# Restart Sunshine
systemctl --user restart sunshine

# DPMS status
DISPLAY=:0 xset q | grep -A3 DPMS
```
