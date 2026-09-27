# App architecture (Phase 3)

One Flutter app, three roles. The web controller served by the brain stays as-is for laptops.

## Roles

| Role | Runs on | What it does |
|---|---|---|
| **Mounted brain** | the phone on the robot | camera tracking (ARKit / ARCore), mount detection, keep-awake status face, hosts the control server |
| **Brain in hand** | any phone/tablet on the network | same brain without the phone sensors: lidar-only tracking, mapping, relocalisation, tap-to-go; hosts the control server |
| **Controller** | any phone/tablet | connects to a brain and shows its control pages (Drive / Maps / Bot / Settings) |

The role is chosen on first launch and can be changed from the app's menu. A brain never drives
from the background: leaving the foreground stops the robot.

## Layers inside the brain

```
 ESP32 (motion, reflexes, sensor feed)  ---UDP--->  Brain
 Lidar bridge (ESP32)                   ---UDP--->    |
 Phone sensors (camera tracking, depth) ------------> |
                                                      v
   Tracking  (camera + lidar scan matching, or lidar alone)
   Map       (keyframes -> occupancy grid, loop closing, edits)
   Guardian  (forward clear / blocked + why: lidar front cone from the profile's body
              front, ESP32 reflexes, ultrasonic, later depth camera)
   Navigator (planner + follower; asks the guardian before any forward motion)
   Control server (web pages + WebSocket for controllers)
```

Every module reads geometry from the bot profile; nothing about a specific robot is hard-coded.

## Capability tiers

Computed from the robot's capability announce plus the phone's capabilities, shown in Settings:
Drive -> Reflexes (bumpers/ToF/ultrasonic) -> Brain (a phone) -> Mapping (lidar) -> 3D (depth camera).
Each tier lists what would unlock the next.

## Storage

Per robot (keyed by the robot's name from its capability announce):
`Documents/robots/<name>/bot_profile.json` and `Documents/robots/<name>/maps/...`.
The app remembers the last role and the last brain address (controller role).
