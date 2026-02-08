---
title: "Tailscale VPN"
description: "Tailscale configuration, ACL policy, peer relay setup, and connection troubleshooting."
---

## ACL Policy

```json
{
  "tagOwners": {
    "tag:relay": ["autogroup:admin"],
    "tag:clients": ["autogroup:admin"]
  },
  "acls": [
    { "action": "accept", "src": ["*"], "dst": ["*:*"] }
  ],
  "grants": [
    {
      "src": ["tag:clients"],
      "dst": ["tag:relay"],
      "app": { "tailscale.com/cap/relay": [] }
    },
    {
      "src": ["tag:clients"],
      "dst": ["tag:relay"],
      "ip": ["*"]
    }
  ]
}
```

**Important**: The `grants` section must be set via the Tailscale API directly -- the MCP ACL tool (`manage_acl`) silently drops the `grants` field. Use:

```bash
curl -X POST -H "Authorization: Bearer $TAILSCALE_API_KEY" \
  -H "Content-Type: application/json" \
  "https://api.tailscale.com/api/v2/tailnet/mordasiewicz.com/acl" \
  -d '{ ... full ACL with grants ... }'
```

## Peer Relay

- **Device**: tsrelay (Toronto, 100.95.40.19)
- **Relay port**: 40000 (`tailscale set --relay-server-port=40000`)
- **Port forward**: Toronto UniFi forwards UDP 40000 -> tsrelay LAN IP (192.168.177.228)
- **Static endpoints**: CLEARED -- do NOT set `--relay-server-static-endpoints` since Bell PPPoE IP is dynamic. STUN + UPnP discovers the public IP automatically.
- **Systemd override**: `/etc/systemd/system/tailscaled.service.d/relay.conf` sets `--relay-server-port=40000` on startup. Do NOT add `--relay-server-static-endpoints` here -- it will go stale when Bell rotates the PPPoE IP.
- **Grant**: Devices tagged `tag:clients` can use devices tagged `tag:relay` as peer relays (requires BOTH the `app` grant for `tailscale.com/cap/relay` AND an `ip` grant)
- **Purpose**: When direct connections fail (school NAT, mobile hotspot symmetric NAT), devices route through tsrelay instead of Tailscale's DERP servers. Peer relay has no bandwidth throttling unlike DERP.

### Connection priority

Tailscale decides automatically:

1. **Direct** (UDP hole-punch) - fastest
2. **Peer relay** (tsrelay) - no throttling, ~15ms Toronto-Kingston
3. **DERP** (Tailscale servers, Toronto) - throttled, ~14ms

### How peer relay grant works

- `src` = who can USE the relay (must be tag:clients)
- `dst` = the relay device itself (must be tag:relay)
- Only the device initiating through the relay needs the tag, not both sides
- Removing tag:clients from a device blocks its relay access

### Tagged device limitation

- Tags can't be removed without re-authenticating (`tailscale up --force-reauth`)
- To test without relay, temporarily remove the `grants` section from ACL instead of changing device tags

## Connection Results by Location

| Mac Location | Connection Type | Latency | Notes |
|-------------|----------------|---------|-------|
| Toronto UniFi | Direct | ~17ms | Requires UPnP enabled on UniFi |
| Toronto Bell Wi-Fi | Direct | ~15ms | Works naturally (permissive NAT) |
| School (university) | Peer relay | ~45ms | EDM NAT + no UPnP, direct fails, relay kicks in |
| Mobile hotspot | Peer relay | ~50-120ms | Carrier CGNAT has symmetric NAT (EDM), no UPnP. Direct to tsrelay may work; workstation goes via peer relay |
| Anywhere (no relay) | DERP Toronto | ~14ms | Bandwidth throttled |

## Tailscale Ports

- **41641/UDP**: Default Tailscale WireGuard port (on each device)
- **40000/UDP**: Peer relay port (tsrelay only)
- **3478/UDP**: STUN port (used by Tailscale's DERP/STUN servers for NAT discovery)
