---
title: "NAT Troubleshooting"
description: "NAT concepts, current NAT status, and troubleshooting guide for Tailscale connectivity."
---

## NAT Concepts

### NAT Mapping vs NAT Filtering

Two independent behaviors per NAT router:

**Mapping** (port assignment):

- **Endpoint-Independent Mapping (EIM)**: Same external port regardless of destination. `MappingVariesByDestIP: false`. Good for hole-punching.
- **Endpoint-Dependent Mapping (EDM)**: Different external port per destination. `MappingVariesByDestIP: true`. Hard to hole-punch -- the other side can't predict your port.

**Filtering** (who can send inbound):

- **Endpoint-Independent Filtering (EIF)**: Once a port opens via outbound traffic, ANY external host can send to it. Consumer routers (Bell Home Hub) typically do this.
- **Endpoint-Dependent Filtering (EDF)**: Only the original destination can send back. UniFi default. Blocks hole-punching because the remote peer differs from the STUN server that discovered the port.

`tailscale netcheck` only tests mapping, NOT filtering. Two routers can show identical netcheck results but behave completely differently for hole-punching.

### CGNAT (Carrier-Grade NAT)

- ISP adds an extra NAT layer between router and internet
- Router gets a private IP (e.g., 10.x.x.x) instead of a real public IP
- Symptom: WAN IP on router differs from `curl ifconfig.me`
- Port forwarding on router is useless -- can't control ISP's NAT
- Kingston workstation behind Bell CGNAT: router gets 10.130.37.119, internet sees <REDACTED>

### STUN (Session Traversal Utilities for NAT)

- Server that reports your public IP:port as seen from outside
- Tailscale runs STUN alongside every DERP server
- `tailscale netcheck` queries STUN servers in all regions to discover NAT behavior
- Discovered public IP:port shared with peers via Tailscale's coordination server

### UPnP / NAT-PMP / PCP

- Protocols that let LAN devices request port forwards from the router
- Tailscale requests: "open my port and let anyone send to it"
- Bypasses endpoint-dependent filtering by creating an explicit allow-any DNAT rule
- UPnP is most common, NAT-PMP is Apple's version, PCP is the newer standard

## Current NAT Status

### Toronto UniFi

- Mapping: EIM (`MappingVariesByDestIP: false`)
- Filtering: EDF (endpoint-dependent) -- strict by default
- UPnP: **Enabled** (fixes the filtering issue)
- PortMapping: UPnP, NAT-PMP, PCP (all three working)
- Result: Direct connections work

### Toronto Bell Wi-Fi

- Mapping: EIM
- Filtering: EIF (endpoint-independent) -- permissive
- UPnP: Not needed (filtering already permissive)
- Result: Direct connections work naturally

### Kingston (Workstation)

- ISP: Virgin Plus (Bell flanker brand) -- behind CGNAT (`MappingVariesByDestIP: true`)
- UPnP on Kingston UniFi is useless -- CGNAT sits above it
- Cannot accept inbound connections -- relies on hole-punching from the other side
- 10G NIC (`enp3s0`) has a PPPoE config that previously got <REDACTED> (possibly non-CGNAT)
- **CGNAT outbound throughput cap**: ~260 Mbps to public internet. Hard packet drops above this rate (52% loss at 500 Mbps). Inbound has deeper buffers -- asymmetry causes upload (434 Mbps) >> download (119 Mbps) through Tailscale.
- Ookla: 896 down / 342 up to Bell servers (stays inside Bell's network, bypasses CGNAT). Cross-network to Rogers: 99 down / 212 up -- confirms CGNAT bottleneck.

## Troubleshooting Commands

### Check connection type

```bash
tailscale ping <hostname>
# Look for: "via <IP>" = direct, "via DERP(xxx)" = relayed, "via peer-relay(...)" = peer relay
```

### Check NAT type and port mapping

```bash
tailscale netcheck
# Key fields: MappingVariesByDestIP, PortMapping
```

### Check UPnP/NAT-PMP detection

```bash
# macOS:
/Applications/Tailscale.app/Contents/MacOS/Tailscale debug portmap

# Restart Tailscale to re-probe:
# Mac: tailscale down && tailscale up
# Linux (safe over SSH): sudo systemctl restart tailscaled
```

### Check peer endpoint details

```bash
# macOS:
/Applications/Tailscale.app/Contents/MacOS/Tailscale debug netmap           # Full netmap with all peer endpoints
/Applications/Tailscale.app/Contents/MacOS/Tailscale debug peer-endpoint-changes <hostname>  # Endpoint change history
/Applications/Tailscale.app/Contents/MacOS/Tailscale debug restun           # Force STUN re-discovery
```

### Check Tailscale status and endpoints

```bash
tailscale status
tailscale status --json  # Full details with endpoints
```

### Check workstation connection from Mac

```bash
ssh workstation "tailscale netcheck"
ssh workstation "curl -s ifconfig.me"  # Should differ from UniFi WAN IP if CGNAT
```

## Known Issues and Fixes

### No direct connection from Toronto UniFi to Kingston

- **Root cause**: UniFi's endpoint-dependent NAT filtering drops hole-punch packets
- **Fix**: Enable UPnP + NAT-PMP on UniFi Default network (Settings -> Networks -> Default -> UPnP)
- **Must restart Tailscale** after enabling UPnP so it re-probes port mappings
  - Mac: `tailscale down && tailscale up`
  - Linux (over SSH): `sudo systemctl restart tailscaled` (safe -- doesn't break SSH)

### Tagged devices can't be untagged

- Tailscale error: "tagged nodes cannot be untagged without reauth"
- Workaround: Modify the ACL grants section instead of changing device tags
- Full fix: `tailscale up --force-reauth` (will re-register the device)

### ICMP ping to Kingston public IP fails

- Normal with CGNAT -- Bell doesn't forward ICMP through their NAT
- Not a firewall issue on the workstation

### Workstation PPPoE on 10G NIC causes stale endpoints

- Config exists at `/etc/ppp/peers/dsl-provider` for `enp3s0.35` (VLAN 35)
- PPPoE user: `<REDACTED>`
- **Keep PPPoE off** -- when active, Tailscale advertises extra endpoints from the 10G NIC (different CGNAT IP, IPv6, PPPoE internal IP). Stale endpoints persist in the coordination server after PPPoE shuts down, breaking hole-punching.
- When tested: got CGNAT IPv4 (10.130.54.202) and real IPv6 (<REDACTED>)
- To clear stale endpoints: `sudo systemctl restart tailscaled` on workstation

### No IPv6 on Toronto (Bell Fibe)

- Bell Canada does NOT support IPv6 on residential Fibe Internet (DSL or FTTH)
- Only wireless/mobility (cellular) has IPv6
- DHCPv6 prefix delegation (/60 or /56) won't work -- no prefix delegated
- Kingston 10G PPPoE (Virgin Mobile credentials) DID get IPv6 -- possibly different tier
- Leave IPv6 disabled on Toronto UniFi

### Peer relay not engaging

Two common causes:

**1. Grants missing from ACL**

- **Symptom**: `tailscale status --json` shows tsrelay with `Capabilities: []` and `CapMap: null`
- **Root cause**: MCP `manage_acl` tool silently drops the `grants` field -- use the Tailscale API directly.
- **Fix**: Update ACL via API with both grants:
  1. `"app": {"tailscale.com/cap/relay": []}` -- enables relay capability
  2. `"ip": ["*"]` -- allows IP access to the relay node

**2. Stale static endpoint on tsrelay**

- **Symptom**: `tailscale ping` shows DERP instead of peer-relay. tsrelay shows `Active: False`, `InEngine: False`, `LastHandshake: 0001-01-01` from client perspective.
- **Root cause**: `--relay-server-static-endpoints` pointed to a stale Bell PPPoE IP. Clients try the dead address. Bell PPPoE IPs are dynamic.
- **How it happened**: Systemd override (`/etc/systemd/system/tailscaled.service.d/relay.conf`) re-applied the stale IP on every tailscaled restart via `ExecStartPost`.
- **Fix**: Clear the static endpoint and fix the systemd override:
  ```bash
  sudo tailscale set --relay-server-static-endpoints=''
  # Then fix /etc/systemd/system/tailscaled.service.d/relay.conf to only set --relay-server-port=40000
  sudo systemctl daemon-reload
  ```
- **Never use static endpoints** -- Bell PPPoE IP is dynamic. STUN + UPnP on Toronto UniFi discovers the correct public IP automatically.
- After clearing, restart tailscaled to force re-advertisement: `sudo systemctl restart tailscaled`

**Verification**: `tailscale ping <host>` should show `via peer-relay(IP:PORT:vni:N)` instead of `via DERP(tor)`

### Mobile hotspot has symmetric NAT (no direct connections)

- **Symptom**: `tailscale netcheck` shows `MappingVariesByDestIP: true` and `PortMapping: (none)` on phone hotspot
- **Root cause**: Carrier CGNAT on mobile hotspots uses Endpoint-Dependent Mapping (symmetric NAT). No UPnP, so direct hole-punching is impossible when the remote side also has restrictive NAT.
- **Result**: Direct connections fail to all peers. Peer relay still works (zmac to tsrelay via DERP or direct, then to workstation). DERP works as fallback.
- **Tip**: Coffee shop Wi-Fi often has more permissive NAT than carrier hotspots -- test with `tailscale netcheck` and look for `MappingVariesByDestIP: false`

### Never run `tailscale down` over a Tailscale SSH session

- `tailscale down` kills the connection immediately
- Over a Tailscale SSH session, the shell dies before `tailscale up` can execute
- `--accept-risk=lose-ssh` warns but doesn't prevent it
- **Use `sudo systemctl restart tailscaled` instead** -- it restarts the daemon and auto-reconnects
- Backup access: Teleport VPN to 192.168.2.239 if Tailscale is stuck down
