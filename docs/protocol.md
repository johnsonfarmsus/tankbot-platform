# TankBot Platform protocol

Last updated: 2026-10-07 (firmware v3.1).

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

### Clock sync `TLSYN` / `TLSY1`

The client sends `TLSYN` + `u32 seq` (9 bytes) to port 5601. The bridge replies immediately with `TLSY1` + `u32 seq` + `i64` bridge time in microseconds (17 bytes, same clock as `millis()`). The client keeps the reply with the smallest round-trip time and estimates `offset = bridge_ms - (sent + received) / 2`, which converts scan timestamps into its own clock. Send about once per second.

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

### Ping `TMP1` (4 bytes)

Keeps motion status (`TMH1`) flowing to the sender without moving the robot.

## Sensor feed and capabilities (firmware v3)

- UDP port **5603**.

### Subscribing `TSSUB`

The client sends `TSSUB` (resend every 2 s). The robot replies once with `TCAP1` + a JSON capability
announce (firmware version, name, drive type and the hardware table, the same as `GET /api/hardware`),
then streams `TSN1` + JSON at 20 Hz:

```json
{"t":905654,"sensors":[{"id":"us1","v":1113,"ok":true},{"id":"bump1","v":0,"ok":true}],
 "block":{"front":"ultrasonic"},"reflexEvents":0}
```

`block` lists the directions the robot's reflexes currently refuse to drive (front/back/left/right)
and why. Commands into a blocked direction are dropped on the robot itself.

### Brain announce `TBRN1:<port>`

A brain sends `TBRN1:8080` (ASCII) to port 5603 with each subscribe. The robot remembers the sender's
address for 10 s and uses it for `/api/brain` and the `/brain` redirect.

## Robot HTTP (port 80)

| Path | What |
|---|---|
| `/` | drive page: buttons, joystick, keyboard (arrows/WASD, Space stops), trim, speed, links |
| `/setup` | Wi-Fi, name, drive type, pins |
| `GET /api/hardware` | the sensor table (id, name, type, slot, pinA/pinB, role, enabled, yawDeg, floorTilt, stopMm, floorMm, backoffMs, placement) and `pins` (in1-in4, ena, enb, lidarRx, lidarTx). A sensor whose wiring failed the startup check carries `error` (and isn't started) |
| `POST /api/hardware` | replace the table and the pins (the robot restarts to apply) |
| `GET /api/info` | how to reach the robot: `name`, `host`, `ip`, `mode` (wifi / hotspot), `ssid`, `rssi`, `fw`, `brain` |
| `GET /api/test/motor?motor=a\|b&dir=1\|-1&ms=400` | run one motor briefly (wiring check; reflexes still apply). A = IN1/IN2/ENA, B = IN3/IN4/ENB |
| `GET /api/sensors` | live readings + directional blocks (same as `TSN1`) |
| `GET /tof/calibrate?id=X` | store the current reading of a floor-facing ToF as its floor distance |
| `GET /api/brain` | `{"url":"http://<brain>:8080/"}` if a brain announced itself in the last 10 s, else `{}` |
| `/brain` | 302 redirect to the brain; a help page if none is running |
| `GET` / `POST /api/settings` | robot-level brain settings (JSON with an `updated` time; the newer side wins). Stored in NVS, max ~3.8 KB |
| `GET /api/map/info` | `{id, name, updated, size, keyframes}` of the stored compact map, or `{}` |
| `GET /api/map` | the compact map file (`TCM1`, see below) |
| `POST /api/map/begin?size=N`, `POST /api/map/chunk` (body: base64, ~6 KB decoded), `POST /api/map/end` (body: info JSON) | chunked upload into LittleFS; the stored map is replaced only when all N bytes arrived. Send bodies as `text/plain` / JSON, not form-encoded |

### Compact map `TCM1`

`TCM1` + u32 header length (little-endian) + header JSON + zlib(cells). Cells: one byte per grid cell
over the map's bounding box, 0 unexplored, 1 open floor, 2 wall. Header: `id`, `name`, `created`,
`updated`, `res`, `x0`/`y0` (cell index relative to the grid centre), `w`, `h`, `lidarFwdM`,
`lidarLeftM`, `lastPose`, `nextEditId`, `edits`. Raw keyframes are not included.

## Controller WebSocket (brain, port 8080, path `/ws`)

JSON messages. The brain sends:

| type | when | content |
|---|---|---|
| `telem` | 5 Hz | pose, live scan, settings, stats, tracking, loc, nav, explore, quality, robot, guard, depth, rangers, marks, no-go lines, log, flash |
| `map` | when the map image changes | PNG (base64) + `left`, `top` (m) and `res` (m/pixel) |
| `maps` | on request / change | saved maps + the active map (with its restore points) |
| `bot` | on request / change | the bot profile |

Controllers send (`type` plus fields):

| type | fields | effect |
|---|---|---|
| `drive` | `f`, `t` (-1..1) | manual drive; cancels Go To and exploring |
| `stop` | | stop everything |
| `set` | any of `maxSpeed`, `obstacleStop`, `mappingMode`, `trackingMode`, `wallAlign`, `gpsMaxAccM`, `stopDistMm`, `passDistMm`, `depthStopMm`, `depthMinHeightMm`, `trim`, `minPower`, `cruisePower` | change settings (saved) |
| `nav.goto` / `nav.cancel` | `x`, `y` | Go to / stop |
| `explore.start` | `ackNoCliff` | start exploring (needs Explore mode; acknowledgment without a cliff sensor) |
| `explore.stop` / `explore.preview` | | stop / compute the next target without moving |
| `loc.set` | `x`, `y`, `h` | Set position (fine-tuned by the lidar) |
| `reloc` / `atHome` | | Try again (whole map) / I'm at home |
| `clearMap` | | new map here (saves the current one) |
| `maps.list`, `maps.save`, `maps.load`, `maps.delete` | `name` / `id` | map management |
| `map.restore` | `id` | put the map back to a restore point |
| `map.erase` | `x`, `y`, `r`, `stroke` | eraser (also removes marks under it) |
| `map.nogo` / `map.nogoDelete` | `x1..y2` / `id` | no-go lines |
| `map.undo` | | undo the last user edit |
| `map.clearDropoffs` | | remove remembered drop-offs |
| `bot.get` / `bot.set` / `bot.calibrate` | `profile` / `id` | Bot page (the profile's `pins` carry the motor driver and lidar wiring) |
| `wiring.testMotor` | `motor` (a / b), `dir` (1 / -1) | run one motor for half a second |
| `log.start` / `log.stop` | | sensor log (GPS, compass, pose) |
