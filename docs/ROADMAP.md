# TankBot Platform roadmap

Last updated: 2026-11-12

## Vision

A robot platform that scales with what is plugged into it, built **cheap first**. The bare minimum
(ESP32, motor driver, chassis) is a drivable robot. Everything added on top, from bumper switches to
a smartphone brain and a lidar, is recognised and used automatically. Any chassis and size, any
sensor layout, one setup flow that a newcomer can follow.

Three ways to build, one software stack (see [hardware-guide.md](hardware-guide.md)):

- **Phone on the robot:** the phone brings the sensor suite (camera tracking, gyro, depth on LiDAR
  iPhones).
- **Sensors on the robot, brain in hand:** the robot carries the sensors; any phone, tablet or
  laptop does the thinking.
- **Both:** independent sources cross-check each other.

The long-term goal is **autonomous operation**, indoors and out: deliveries, carrying tools (brooms,
claws), on bases from TankBot up to a wheelchair (about $250, 250 lb payload).

## Capability tiers

| Tier | Hardware | What you get |
|---|---|---|
| Drive | ESP32 + motor driver + chassis | Manual driving from the ESP32's own web page (buttons, joystick, keyboard) |
| Reflexes | + bumpers, ToF, ultrasonic, IMU | Bump back-off, close-obstacle stop, cliff stop, tilt stop, directional blocks. Runs on the ESP32, no phone needed |
| Brain | + a phone or tablet running the app (mounted or in hand) | Full controller, position tracking, the robot's status face |
| Mapping | + lidar | Maps, map modes, relocalisation, Set position, Go to, autonomous exploration |
| 3D awareness | + depth camera (LiDAR iPhones) or a tilted lidar | Low obstacles and drop-offs the 2D lidar misses |
| Outdoors (planned) | + GPS, look-ahead cliff sensor, long-range link | Routes over GPS-anchored maps, curbs, remote stop and status |

## Decisions

- **Cheap first.** Every part is weighed for capability per dollar; expensive parts are optional
  upgrades, never requirements. One lidar per robot.
- **Bot profile is the single source of truth** for geometry (see `bot-profile.md`). The ESP32 keeps
  its own hardware table (pins, roles, reflex settings); the brain merges it with measured positions.
- **The brain is a role, not a device.** One app: Mounted brain, Brain in hand, Controller.
- **The phone stays the brain.** New pieces (state estimator, mission layer, message formats) are
  designed so a ROS 2 bridge could plug in later, without paying for it now.
- **Use whatever sensors a robot has.** Tracking fuses every available source, each weighted by how
  trustworthy it currently is; no single sensor is required beyond the lidar for mapping.
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
- **Wiring tools.** Bot > Wiring: pin map, default or custom pins, checks in the controller and the
  firmware, motor tests and live readings. The robot page shows how to reach the robot.
- **Lidar + gyro tracking** for phones whose camera struggles; a broken camera is benched
  automatically; Set position searches every heading.
- **The app** asks for its role on every launch and keeps one brain per robot.
- **Interchangeable brains.** The robot keeps its robot-level settings (newer side wins) and a compact
  copy of the most recent map (~40 KB for a whole house); a new brain picks both up on connect.

## Next

### 1. Tracking and the indoor platform (in priority order)

1. **State estimator.** One estimate fused from lidar matches, the camera's step-by-step movement
   (never its absolute position), the phone gyro and (later) the robot IMU, optical flow and GPS, each
   weighted by current trust. Camera steps the lidar disagrees with are down-weighted at once; ARKit's
   own warnings (tracking reason, visual-point count) lower its weight before it fails. No new
   hardware needed; it is what the rest builds on.
2. **IMU (GY-521).** Driver with chip auto-detection, re-zeroing when idle, turn rate and tilt to the
   brain, tilt / bump / lifted / stuck reflexes on the ESP32, automatic mounting-orientation
   calibration; fused with the phone gyro.
3. **Battery monitoring.** Voltage divider into an ESP32 ADC pin, battery level in telemetry, a
   low-battery warning, and self-driving heading home before it browns out.
4. **12 V to 5 V buck converter** as standard on the 5 V rail (see wiring.md).
5. **Look-ahead cliff sensor (TF-Luna)** on the lidar tower, aimed 30-40 cm ahead, with floor
   calibration, the three-band reading and IMU slope correction; removes the exploration
   acknowledgment. (Replaces the earlier straight-down TOFSense plan.)
6. **Phone torch as headlights** (Auto / On / Off from the phone's light estimate and camera drift).
7. **Help tab in the controller** (the user guide, with drawn examples), linked from every page.
8. **Rear sensing.** A rear bumper or ultrasonic so recovery back-ups can't nudge something behind.
9. **ARKit place memory** (world maps) as a relocalisation booster on iPhones.

### 2. Brains and connectivity

1. **Wired brain** for phones with dead radios: USB Ethernet + a GL.iNet Mango in bridged repeater
   mode on the robot (no code changes; hardware-guide.md).
2. **Android brain in hand** (networking and permissions; any reasonable Android phone).
3. **Android mounted brain** (ARCore tracking and depth; ARCore-certified phones).
4. **Communication layer:** one set of messages over Wi-Fi, cellular (the brain phone's own, via a
   relay) or LoRa (compact versions).
5. **LoRa (Meshtastic-class) link:** status beacon, alerts and an authenticated remote stop that work
   without Wi-Fi, cellular or the phone.

### 3. Outdoor autonomy

1. **Second robot: wheelchair base** (cheap electric wheelchair); prove everything carries over.
2. **Cheap odometry inputs** (optional): hall-sensor taps on brushless hub motors, magnet + hall or
   AS5600 sensors, an optical-flow / mouse sensor.
3. **GPS-anchored, tiled maps** beyond the current single ~40 m grid.
4. **Teach and repeat:** drive a route once (lidar map + GPS track), then repeat it autonomously;
   **OpenStreetMap** (and Overture) for route planning and geofencing, not for precise steering.
5. **Mission layer:** routes, waypoints, pickup / drop-off, return to base, pause and resume.
6. **Safety case:** hardware e-stop, remote stop, context speed limits, a defined behaviour for every
   failure.
7. **Outdoor perception:** look-ahead curb detection, moving people and pets (lidar tracking, the
   phone camera), and later a tilted lidar for a height map.
8. **Drivers for cheaper lidars** (LD06 / LD19, YDLIDAR, salvaged vacuum lidars).
9. **Log replay as regression tests** for tracking and navigation.

## Always

- Test-drive between steps; nothing regresses. `flutter test` must pass before every install.
- Commit each working change with a message that says why, push, and keep these docs current.
