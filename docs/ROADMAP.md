# TankBot Platform roadmap

Last updated: 2026-10-06

## Vision

A robot platform that scales with what is plugged into it. The bare minimum (ESP32, motor driver,
chassis) is a drivable robot. Everything added on top, from bumper switches to a smartphone brain
and a lidar, is recognised and used automatically. Any chassis, any sensor layout, one setup flow
that a newcomer can follow.

## Capability tiers

| Tier | Hardware | What you get |
|---|---|---|
| Drive | ESP32 + motor driver + chassis | Manual driving from the ESP32's own web page (buttons, joystick, keyboard) |
| Reflexes | + bumpers, ToF, ultrasonic | Bump back-off, close-obstacle stop, cliff stop, directional blocks. Runs on the ESP32, no phone needed |
| Brain | + a phone or tablet running the app (mounted or in hand) | Full controller, position tracking, the robot's status face |
| Mapping | + lidar | Maps, map modes, relocalisation, Set position, Go to, autonomous exploration |
| 3D awareness | + depth camera (LiDAR iPhones) | Low obstacles and drop-offs the 2D lidar misses |

## Decisions

- **Bot profile is the single source of truth** for geometry (see `bot-profile.md`). The ESP32 keeps
  its own hardware table (pins, roles, reflex settings); the brain merges it with measured positions.
- **The brain is a role, not a device.** One app: Mounted brain, Brain in hand, Controller.
- **Safety chain never depends on the brain:** ESP32 watchdog + on-board reflexes always apply.
  Every autonomous behaviour runs its own obstacle checks regardless of manual settings.
- **The map is precious.** It changes deliberately (Explore), slowly and only for persistent changes
  (Maintain), or not at all (Off). Doubtful positions never write to it; restore points make any
  mistake undoable.
- **Measure, don't guess.** Trip logs, position events and sensor logs exist so behaviour problems are
  diagnosed from data (see `diagnostics.md`).
- **The user supervises autonomy.** Without a cliff sensor, exploring needs an explicit
  acknowledgment each time; a controller must stay connected for any self-driving.

## Done

- **Phase 1: Bot profile + Bot page.** Profile v2 (platform height, platform-relative sensor
  heights), one editor for every sensor (slot, role, facing, thresholds, placement, live reading).
- **Phase 2: Modular firmware v3.** Generic sensor table in flash, directional reflexes, configurable
  bumper back-off, sensor feed, capability announce, brain announce, `tankbot.local/brain` redirect.
- **Phase 3: App around roles.** Mounted brain / Brain in hand / Controller; per-robot storage;
  guardian fusing every sensor from its profile position; capability tiers; landscape or portrait
  phone mounting; brains on phones without depth sensors (iPhone SE) with automatic fallbacks.
- **Controller redesign.** One consistent shell, pan/zoom/follow map, legend, status cards and toasts,
  Set position, restore points, Settings that save as you go.
- **Mapping.** Pose-graph map straightening (driving links, loop links, wall alignment, GPS when
  good), mapping modes Explore / Maintain / Off, self-cleaning bump and drop-off marks, restore
  points, trust gating (only confident scans are drawn).
- **Navigation.** Planner + follower with steering while driving, pulsed turns with settling,
  recovery when blocked (mark, back up, re-plan), stall detection, standing still when unsure.
- **Autonomous exploration** (first version): frontiers with gap closing, room-first ordering,
  reachability, previews, settle pauses in new territory, automatic stop and rollback when lost.
- **Tracking robustness.** Camera glitch filter, distrust of a drifting camera (lidar-only fallback),
  wide lidar re-search, no relocalisation jumps while self-driving, no lidar corrections from scans
  smeared by fast spins, Settings choice of camera + lidar or lidar only.
- **Diagnostics.** Trip recorder, position events log, sensor/GPS/compass log, analysis tools.
- **Interchangeable brains.** The robot keeps its robot-level settings (newer side wins) and a compact
  copy of the most recent map (~40 KB for a whole house); a new brain picks both up on connect.

## Next (in priority order)

1. **Battery monitoring.** Voltage divider into an ESP32 ADC pin, battery level in telemetry, a
   low-battery warning, and self-driving heading home before it browns out (2026-10-06 ended on a
   flat battery).
2. **Exploration robustness on the SE.** Compare camera + lidar against lidar only on the same runs
   (trip logs + events); make the better one the default for phones without a depth sensor.
3. **Phone torch as headlights** (Auto / On / Off from the phone's light estimate and camera drift).
4. **Help tab in the controller** (the user guide, with drawn examples), linked from every page.
5. **Cliff sensor.** Wire the TOFSense-F2 Mini (T to GPIO32, R from GPIO33, 5 V), aim it at the floor,
   calibrate; removes the exploration acknowledgment.
6. **Rear sensing.** A rear bumper or ultrasonic so recovery back-ups can't nudge something behind.
7. **IMU + encoder fusion** for tracking without a camera and better stall detection.
8. **ARKit place memory** (world maps) as a relocalisation booster on iPhones.
9. **Second robot:** wheelchair-base profile; prove everything carries over.
10. **Wired brain option** (iPhone 12 Pro via USB Ethernet + an on-board travel router).

## Always

- Test-drive between steps; nothing regresses. `flutter test` must pass before every install.
- Commit each working change with a message that says why, push, and keep these docs current.
