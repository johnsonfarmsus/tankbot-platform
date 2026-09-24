# TankBot Platform protocol (v1)

All messages are UDP. Multi-byte numbers are little-endian. Devices advertise themselves with mDNS/Bonjour so clients never need hardcoded IP addresses.

## Lidar bridge

- mDNS host: `tanklidar.local`
- Bonjour service: `_tanklidar._udp`, port **5601**

### Subscribing

The client sends the 5-byte ASCII message `TLSUB` to port 5601. The bridge streams to that client's IP and source port. The client must resend `TLSUB` at least every **5 s** (1 s recommended); otherwise the subscription expires. Up to 3 subscribers at once.

### Scan packet `TLS1`

One full rotation is split into chunks of up to 250 points.

| Offset | Type | Field |
|---|---|---|
| 0 | char[4] | `TLS1` |
| 4 | u32 | rotation number |
| 8 | u32 | bridge `millis()` at rotation start |
| 12 | u8 | chunk index (0-based) |
| 13 | u8 | chunk count |
| 14 | u16 | n = points in this chunk |
| 16 | n x 5 bytes | points |

Each point: `u16 angle_q6` (degrees = value / 64, clockwise from the lidar's front), `u16 dist_q2` (millimetres = value / 4), `u8 quality`. Only valid (non-zero distance) points are sent.

A rotation is complete when all `chunk count` chunks with the same rotation number have arrived. Drop incomplete rotations; the next one arrives 100 ms later.

### Status packet `TLH1`

Sent once per second to each subscriber: `TLH1` followed by a JSON object, e.g.

```json
{"uptime_ms":46192,"revs":432,"hz":9.99,"pts":417,"sync_errs":0,"rssi":-49,"subs":1,"sent":193,"send_errs":0}
```

## Motion controller

To be defined: velocity commands (forward speed + turn rate), a command watchdog (stop if no command for ~300 ms), and status reports.
