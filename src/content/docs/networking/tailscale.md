---
title: "Tailscale VPN"
description: "Tailscale ACL policy, peer relay, and connection troubleshooting."
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

The `grants` section must be set via the Tailscale API directly -- the MCP ACL tool (`manage_acl`) silently drops it:

```bash
curl -X POST -H "Authorization: Bearer $TAILSCALE_API_KEY" \
  -H "Content-Type: application/json" \
  "https://api.tailscale.com/api/v2/tailnet/mordasiewicz.com/acl" \
  -d '{ ... full ACL with grants ... }'
```

---

## Peer Relay

When direct connections fail (school NAT, mobile hotspot symmetric NAT), devices route through tsrelay instead of Tailscale's DERP servers. No bandwidth throttling unlike DERP.

| | |
|---|---|
| **Device** | tsrelay (Toronto, 100.95.40.19) |
| **Relay port** | 40000 (`tailscale set --relay-server-port=40000`) |
| **Port forward** | Toronto UniFi: UDP 40000 -> tsrelay (192.168.177.228) |
| **Systemd override** | `/etc/systemd/system/tailscaled.service.d/relay.conf` |
| **Grant** | `tag:clients` -> `tag:relay` (requires both `app` and `ip` grants) |

Do not set `--relay-server-static-endpoints` -- Bell PPPoE IP is dynamic. STUN + UPnP discovers the public IP automatically.

### Connection priority (automatic)

| Priority | Type | Latency |
|---|---|---|
| 1 | Direct (UDP hole-punch) | Fastest |
| 2 | Peer relay (tsrelay) | ~15ms Toronto-Kingston, no throttling |
| 3 | DERP (Tailscale servers) | ~14ms, throttled |

### Grant mechanics

- `src` = who can use the relay (`tag:clients`)
- `dst` = the relay device (`tag:relay`)
- Only the initiating device needs the tag, not both sides
- Tags can't be removed without `tailscale up --force-reauth`
- To test without relay, remove `grants` from ACL instead of changing device tags

---

## Connection Results by Location

| Mac Location | Connection Type | Latency | Notes |
|-------------|----------------|---------|-------|
| Toronto UniFi | Direct | ~17ms | Requires UPnP enabled on UniFi |
| Toronto Bell Wi-Fi | Direct | ~15ms | Works naturally (permissive NAT) |
| School (university) | Peer relay | ~45ms | EDM NAT + no UPnP, direct fails, relay kicks in |
| Mobile hotspot | Peer relay | ~50-120ms | Carrier CGNAT has symmetric NAT (EDM), no UPnP. Direct to tsrelay may work; workstation goes via peer relay |
| Anywhere (no relay) | DERP Toronto | ~14ms | Bandwidth throttled |

---

## Tailscale Ports

| Port | Protocol | Purpose |
|---|---|---|
| 41641 | UDP | Default WireGuard (each device) |
| 40000 | UDP | Peer relay (tsrelay only) |
| 3478 | UDP | STUN (NAT discovery) |
