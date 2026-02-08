---
title: "GPU & Sunshine Config"
description: "NVIDIA persistence mode, Sunshine streaming server, and display configuration."
---

## GPU Configuration

### NVIDIA Persistence Mode

- **Status: Enabled** (set Feb 2026 to fix 30fps idle issue)
- Systemd service: `nvidia-persistenced`
- Override: `/etc/systemd/system/nvidia-persistenced.service.d/override.conf`
  - Removes the `--no-persistence-mode` flag from the stock service
  - Stock service at `/usr/lib/systemd/system/nvidia-persistenced.service` has `--no-persistence-mode` which defeats the purpose
- Verify: `nvidia-smi -q | grep "Persistence Mode"` should show `Enabled`
- Manual enable: `sudo nvidia-smi -pm 1`

### Known Issue: 30fps after idle

- **Symptom**: Sunshine drops to 30fps after not streaming for a while
- **Root cause**: Without persistence mode, GPU drops to P8 (low power) when idle, throttling NVENC encoder
- **Fix applied**:
  1. Enabled persistence mode (`nvidia-smi -pm 1`)
  2. Created systemd override to persist across reboots
  3. Disabled DPMS in xorg.conf and via `xset -dpms`
- If it recurs: check `nvidia-smi` for P-state (should not be P8) and `nvidia-smi -q | grep "Persistence Mode"`

## X11 / Display Configuration

### xorg.conf

- Path: `/etc/X11/xorg.conf`
- Headless virtual display (no physical monitor)
- Connected monitor: `DP-0` (virtual)
- DPMS: **Disabled** (`Option "DPMS" "false"` in Monitor section)
- DPI: 96x96
- ModeValidation: permissive (AllowNonEdidModes, NoVirtualSizeCheck, etc.)

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

- **DPMS: Disabled** (both in xorg.conf and at runtime via `xset -dpms`)
- Screen saver: Off (`xset s off`)
- Prevents GPU from downclocking when idle

### GDM

- Autologin: enabled (user: zeul)
- Wayland: disabled (X11 only for NVIDIA KMS capture)
- Config: `/etc/gdm3/custom.conf`

## Sunshine Configuration

### Service

- Type: systemd user service (`sunshine.service`)
- Auto-start: on graphical session
- Override: `~/.config/systemd/user/sunshine.service.d/override.conf`
  - `Restart=always`, `RestartSec=5`
  - Requires `graphical-session.target`

### Config (`~/.config/sunshine/sunshine.conf`)

```ini
nvenc_preset = 1
nvenc_twopass = disabled
fec_percentage = 10
origin_web_ui_allowed = wan
qp = 10
```

### Resolution Auto-Switch

- Script: `~/set-resolution.sh`
- Called via Sunshine prep-cmd on stream start
- Uses `SUNSHINE_CLIENT_WIDTH` / `SUNSHINE_CLIENT_HEIGHT` env vars
- Switches via `nvidia-settings -a CurrentMetaMode`
- Logs to `/tmp/sunshine-res.log`

### Web UI

- URL: `https://localhost:47990` (or via Tailscale IP)
- Remote access enabled (`origin_web_ui_allowed = wan`)

## Quick Reference

### Check GPU status

```bash
nvidia-smi
```

### Check display config

```bash
DISPLAY=:0 xrandr
```

### Check Sunshine logs

```bash
journalctl --user -u sunshine -n 50
```

### Restart Sunshine

```bash
systemctl --user restart sunshine
```

### Check DPMS status

```bash
DISPLAY=:0 xset q | grep -A3 DPMS
```
