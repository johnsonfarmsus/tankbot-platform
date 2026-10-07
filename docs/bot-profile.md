# Bot profile

The bot profile describes one robot: its size, how it drives, where every sensor sits, and the
settings that belong to that robot. All software (tracking, mapping, navigation, safety, the
controller's drawings) reads from it. Nothing about a specific robot is hard-coded.

Edit it on the controller's **Bot** page; **Save to robot** stores it on the brain and sends the
robot-side sensor table to the ESP32 (which restarts once to apply it).

## Coordinates

- Positions on the platform: **from the front edge** (`fromFrontMm`, negative = sticking out ahead)
  and **from the left edge** (`fromLeftMm`), seen from above with the front at the top.
- Heights (`heightMm`): **above the platform top** (negative = below it). The platform's own height
  above the floor is `platform.heightMm`.
- Internally: robot frame **x forward, y left**, origin at the platform centre.
- `yawDeg`: facing direction, 0 = forward, positive = turned left.

## Format (version 2)

```json
{
  "version": 2,
  "name": "TankBot",
  "drive": "tank",                                  // tank | wheelchair | mecanum
  "phoneMount": "portrait",                         // portrait | landscape
  "platform": { "widthMm": 185, "lengthMm": 170, "heightMm": 66 },
  "minPower": 0.8, "cruisePower": 0.9,              // the lowest power that moves it; its best driving power
  "mapping": { "mode": "maintain", "tracking": "auto", "wallAlign": true, "gpsMaxAccM": 5 },
  "sensors": [
    { "id": "lidar", "type": "lidar",      "fromLeftMm": 98,   "fromFrontMm": 65,    "heightMm": 200,  "slot": "LIDAR", "role": "mapping" },
    { "id": "cam",   "type": "camera",     "fromLeftMm": 110,  "fromFrontMm": 38,    "heightMm": 135 },
    { "id": "us1",   "type": "ultrasonic", "fromLeftMm": 70,   "fromFrontMm": 3.5,   "heightMm": 13.5, "slot": "US1",   "role": "obstacle", "stopMm": 150 },
    { "id": "bump1", "type": "bumper",     "fromLeftMm": 92.5, "fromFrontMm": -17.5, "heightMm": -28,  "slot": "BUMP1", "role": "bump", "widthMm": 160, "backoffMs": 150 }
  ]
}
```

Also stored: safety distances (stop, pass, depth stop, depth height), `hardwareDirty` (local sensor
edits the robot hasn't confirmed yet) and `hardwareMigrated` (the one-time carry-over of old measured
positions onto the robot's table has happened).

## Sensors

Types: `lidar`, `camera` (the phone camera: where camera tracking sits), `depth`, `bumper`, `tof`,
`ultrasonic`, `imu`.

Robot-side fields (they live in the ESP32's table too): `slot` (BUMP1/BUMP2/TOF/US1/US2/I2C/LIDAR),
`role` (`obstacle`, `cliff`, `bump`, `orientation`, `mapping`, `none`), `enabled`, `yawDeg`,
`floorTilt` (aimed at the floor), `stopMm` (obstacle role), `floorMm` (calibrated floor distance,
cliff role), `backoffMs` (bump role: reverse this long after a hit, 0 = just stop).

A sensor with role `cliff` counts as a drop-off sensor (exploring then needs no acknowledgment).

## Merging with the robot

On connect the brain reads the robot's table, keeps its own measured positions for those sensors,
and pushes back any difference. The brain never merges into, or saves, a profile before the saved one
has loaded from storage.

## Derived

- **Body radius** (half the platform diagonal) and **inflation** (body radius + pass distance) for
  planning and exploration reachability.
- **Front edge and half-width** for the guardian's forward-clear decision.
- **Lidar offset** from the tracked point, and every sensor's frame for live readings.
