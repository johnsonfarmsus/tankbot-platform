# TankBot Platform roadmap

Last updated: 2026-09-27

## Vision

A robot platform that scales with what is plugged into it. The bare minimum (ESP32, motor driver,
chassis) is a drivable robot. Everything added on top, from bumper switches to a smartphone brain
and a lidar, is recognised and used automatically. Any chassis, any sensor layout, one setup flow
that a newcomer can follow.

## Capability tiers

| Tier | Hardware | What you get |
|---|---|---|
| Drive | ESP32 + motor driver + chassis | Manual driving from the ESP32's own web page |
| Reflexes | + bumpers, ToF, IMU, encoders | Bump stop, cliff/drop-off stop, straighter driving, stuck detection. Runs on the ESP32, no phone needed |
| Brain | + a phone or tablet running the app (mounted or in hand) | Full controller, position tracking (mounted), bump/tilt sensing, the robot's face |
| Mapping | + lidar (or depth camera) | Maps, remembering places, relocalisation, tap-to-go |
| 3D awareness | + depth camera (LiDAR iPhones, ARCore depth) | Low obstacles and overhangs the 2D lidar misses |

## Decisions

- **Bot profile is the single source of truth** (dimensions, chassis outline, drive type and wheel
  geometry, every sensor's position and height). All software reads from it. See `bot-profile.md`.
- **Profile split:** the ESP32 stores its own hardware setup (pins, drive type, chassis basics) so a
  phoneless robot always knows itself; the brain stores sensor positions and maps.
- **Drive types:** tank, wheelchair (two powered wheels + free casters), mecanum. Motion commands are
  forward / sideways / turn; tank and wheelchair ignore sideways.
- **Standard starter wiring:** ESP32 + L298N + one front bumper + one front ToF, fixed default pins,
  with the RPLidar as the standard add-on. One diagram that just works.
- **The brain is a role, not a device.** One app with modes: Controller, Brain (any device on the
  network; lidar-only tracking), Mounted brain (adds camera tracking, depth, keep-awake face).
- **Safety chain never depends on the brain:** ESP32 watchdog + on-board reflexes always apply.

## Phases

### Phase 1: Bot profile + Bot tab  (done, 3D preview pending)
- Profile format and storage on the brain; default TankBot profile.
- Bot tab in the controller: platform size, drag-and-drop sensor placement (top-down), side view for
  heights, 3D preview later.
- Migrate hard-coded values (lidar offset, footprint, phone position) to the profile.
- Deliverable: the TankBot profile, entered through the tab, driving mapping and navigation.

### Phase 2: Modular firmware  (done; awaiting sensors to test)
- ESP32 config page: pins, drive type, attached sensors, stored in flash; starter wiring as default.
- Capability announce + generic sensor feed (bumpers, ToF, IMU, encoders).
- On-board reflexes: bumper stop/back-off, ToF close-range stop, cliff stop.
- Home Wi-Fi retry while in fallback access-point mode.
- Deliverable: bumper and ToF wired on the TankBot and protecting it with no phone attached.

### Phase 3: App redesign around roles  (core done)
- Controller / Brain / Mounted-brain modes in one app; web controller kept for laptops.
- Guardian: one safety layer fusing every present sensor using its profile position.
- Planner uses the real footprint and rotation centre from the profile.
- Capability tiers shown plainly ("add a lidar to unlock mapping").
- Multiple robots: profile and maps per robot.

### Phase 4: Boosters  (depth camera done; IMU/encoders and ARKit place memory pending)
- Phone depth camera for low obstacles and drop-offs.
- IMU + encoder fusion; ARKit place memory as a relocalisation speed-up.
- Map polish: multi-scan relocalisation, better loop closing.

### Phase 5: Second robot
- Wheelchair-base profile; prove everything carries over with no code changes.
- Documentation and example profiles for new users.

## Always
- Keep test-driving the TankBot between steps; nothing regresses.
- Every autonomous behaviour keeps its own obstacle checks regardless of manual settings.
