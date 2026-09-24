# TankBot Platform protocol (v1)

All messages are UDP. Multi-byte numbers are little-endian. Devices advertise themselves with mDNS/Bonjour so clients never need hardcoded IP addresses.

## Lidar bridge

- mDNS host: `tankbot.local` (single host for web page, lidar and motion)
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

- Bonjour service: `_tankmotion._udp`, port **5602**
- Web control page: `http://tankbot.local/` (original TankBot page, now with hold-to-drive keep-alive)

### Drive command `TMC1` (12 bytes)

| Offset | Type | Field |
|---|---|---|
| 0 | char[4] | `TMC1` |
| 4 | f32 | forward, -1..1 |
| 8 | f32 | turn, -1..1 (positive = same as pushing the web joystick to the right) |

Mixed on the robot exactly like the web joystick: `left = forward - turn`, `right = forward + turn`, normalised to at most 1, then scaled by the current speed level and trim.

**Watchdog:** commands must repeat at least every **300 ms** (send at 10-20 Hz). If they stop, the robot stops on its own. Web page commands have a 500 ms watchdog; the page resends every 150 ms while a control is held.

### Stop `TMS1` (4 bytes)

Immediate stop.

### Motion status `TMH1`

Sent every 250 ms to whoever sent a motion command in the last 3 s: `TMH1` + JSON, e.g.

```json
{"left":0.00,"right":0.00,"src":"none","wd_trips":0,"speed":220,"trim":18,"cmds":0}
```
`src` is `udp`, `web` or `none`; `wd_trips` counts watchdog stops since boot.
