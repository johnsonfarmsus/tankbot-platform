# Bot profile

The bot profile describes one robot: its size, how it drives, and where every sensor sits. All
software (mapping, navigation, safety, the controller's drawings) reads from it. Nothing about a
specific robot is hard-coded.

## Coordinates

- Positions on the platform are measured **from the front edge** (`fromFrontMm`) and **from the left
  edge** (`fromLeftMm`), looking down from above with the front at the top. Heights (`heightMm`) are
  above the floor.
- Internally these become the robot frame used everywhere else: **x forward, y left**, origin at the
  platform's centre.
- `yawDeg` is the sensor's facing direction: 0 = forward, positive = turned to the left.

## Format (JSON)

```json
{
  "version": 1,
  "name": "TankBot",
  "drive": "tank",                    // tank | wheelchair | mecanum
  "platform": { "widthMm": 185, "lengthMm": 170 },   // left-right, front-back
  "wheels": { "trackMm": 160, "axleFromFrontMm": 85 }, // rotation centre for tank/wheelchair
  "sensors": [
    { "id": "lidar",  "type": "lidar",  "name": "RPLidar C1", "fromLeftMm": 92, "fromFrontMm": 40, "heightMm": 300, "yawDeg": 0 },
    { "id": "cam",    "type": "camera", "name": "Phone camera", "fromLeftMm": 108, "fromFrontMm": 40, "heightMm": 200, "yawDeg": 0 },
    { "id": "bump1",  "type": "bumper", "name": "Front bumper", "fromLeftMm": 92, "fromFrontMm": 0, "heightMm": 30, "yawDeg": 0 },
    { "id": "tof1",   "type": "tof",    "name": "Front ToF",    "fromLeftMm": 92, "fromFrontMm": 5, "heightMm": 40, "yawDeg": 0 }
  ]
}
```

Sensor types: `lidar`, `camera` (the phone's camera, i.e. where the brain's position tracking sits),
`bumper`, `tof`, `imu`, `depth` (phone depth camera, same position as `camera` on a phone).

## Where it lives

- Phase 1: the brain stores the whole profile (`Documents/bot_profile.json`) and the controller's
  **Bot** tab edits it.
- Phase 2: the ESP32 stores the hardware part (pins, drive type, chassis basics) in its flash and
  announces it; the brain merges that with sensor positions.

## What is derived from it

- **Lidar offset** relative to the camera: where scans sit relative to the tracked pose.
- **Footprint** for planning: the platform rectangle around the tracked point (the camera), padded by
  a safety margin.
- **Sensor frames** for the guardian: each sensor's readings placed in the robot frame using its
  position and yaw.
