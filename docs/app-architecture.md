# App architecture

Last updated: 2026-10-06. One Flutter app (`app/tankbot_brain`), three roles. The controller UI is a
web page served by the brain, so any browser (and the app's Controller role) uses the same pages.

## Roles

| Role | Runs on | What it does |
|---|---|---|
| **Mounted brain** | the phone on the robot | camera tracking (ARKit) + lidar, mount detection, orientation lock, keep-awake status face, hosts the control server |
| **Brain in hand** | any phone/tablet on the network | the same brain tracking with the lidar alone |
| **Controller** | any phone/tablet | a WebView of a brain's control pages |

Leaving the foreground stops the robot. Any self-driving needs a controller connected.

## Data flow

```
 ESP32: motion (UDP 5602), lidar bridge (5601), sensor feed + reflexes (5603), web page + /setup
   |                                   ^
   v                                   | drive commands at 20 Hz (robot watchdog: 300 ms)
 Brain (main.dart)
   Robot link      find tankbot.local, fall back to the last address, reconnect after silence
   Tracking        camera pose (pose_client: glitch filter) corrected by lidar scan matching,
                   or lidar alone; trust gating; drift / spin / wide-search handling
   Mapping         mode: explore | maintain | off; keyframes -> occupancy grid; edits; pose graph
   Guardian        one forward-clear decision from body geometry, lidar, reflexes, rangers, depth
   Navigator       planner + follower; recovery, stall detection, settle; exploration on top
   Control server  web page + WebSocket (brain_server.dart, assets/controller.html)
   Logs            trip recorder, position events, sensor log
```

## Modules (`lib/`)

| File | Role |
|---|---|
| `main.dart` | the brain: link, tracking, mapping modes, guardian use, navigation, exploration, recovery, telemetry, logs |
| `brain_server.dart` | HTTP + WebSocket server; serves `assets/controller.html` |
| `bot_profile.dart` | profile model, hardware-table merge, settings that travel with the robot |
| `lidar_client.dart`, `motion_client.dart`, `sensor_client.dart` | UDP links to the ESP32 |
| `pose_client.dart` | ARKit pose + depth stream; smooths over impossible camera leaps |
| `scan_matcher.dart` | likelihood-field matching (local, coarse-fine, global) |
| `occupancy_grid.dart` | log-odds grid, rendering, erase/mark, likelihood field |
| `map_store.dart` | per-robot maps, keyframes (TKF1), edits, graph, restore points |
| `pose_graph.dart` | Levenberg-Marquardt pose graph with a skyline solver; wall direction |
| `loop_closer.dart` | loop detection against first visits |
| `frontier.dart` | exploration targets: gap closing, rooms, reachability, stats |
| `planner.dart` | A* on the grid inflated by the body radius + pass distance |
| `guardian.dart` | forward-clear decision from the profile's body geometry |
| `depth_obstacles.dart` | depth camera -> low obstacles and drop-offs (floor-missing test) |
| `sensor_log.dart` | GPS / compass / pose recorder |
| `app_settings.dart`, `role_screens.dart` | role, last robot and address, role UI |

## Tracking

Each lidar scan is matched against the map around the predicted pose (camera, or the last lidar pose).

- **Trust:** a scan is drawn into the map only if it agrees with the parts of the map it lands on;
  in new territory (little known) the camera carries the pose.
- **Doubt:** after 2 s of untrusted scans, self-driving stands still; after 15-17 s it stops (exploring
  also rolls back the keyframes from just before).
- **Drift:** repeated misses widen the window (35 cm); still lost, a Set-position-style wide search
  (60 cm, 20 deg) runs every 1.5 s. A confident match more than 10 cm from the camera snaps to the
  lidar, only when at least half the scan lies on known map.
- **Camera health:** leaps beyond physical limits between frames are smoothed over in the pose stream;
  a camera that reports motion while the motors are idle is distrusted for 30 s (lidar only). Settings
  can force lidar only.
- **Spins:** scans taken while turning faster than 60 deg/s are not matched or drawn (they are smeared).
- **Lidar only** (by choice, or while the camera is benched): turning between scans comes from the
  phone's raw gyro (CoreMotion, independent of the camera, direction checked against the lidar), and
  the robot carries on at its recent speed through missed scans instead of freezing.
- **A broken camera is benched:** 3+ impossible jumps within 5 s, or motion reported while the motors
  are idle, switch to lidar only for a while and restart ARKit; autofocus is locked; ARKit's
  tracking-state reason and visual-feature count are logged.
- **Set position** fine-tunes within 50 cm / 25 deg, and searches every heading around the pressed
  spot when the dragged direction doesn't fit.
- **Relocalisation:** quick checks (last pose, home spot), then a whole-map search; while self-driving
  only near the current pose (never a jump across the map).

## Wiring

The robot's hardware table holds each sensor's pins (a default slot or custom pins) and the motor
driver and lidar pins; the brain carries them in the profile. The controller's Bot > Wiring view edits
them against a pin rule-book (usable, input-only, boot, flash and USB-serial pins) and blocks saving
on conflicts; the firmware runs the same checks at startup and doesn't start a sensor whose wiring
can't work (reporting why). `tools/wiring_check_test.js` runs the controller's checker against known
scenarios.

## Mapping

- **Modes:** Explore (every trusted scan, keyframes, loop closing, straightening), Maintain (default;
  extends into unmapped ground; elsewhere a spot changes only after 3 separate passes over 10+ min
  that clearly disagree with the map), Off (nothing written). Self-driving caps Explore to Maintain,
  except autonomous exploration.
- **Pose graph:** driving links between consecutive keyframes, loop links, wall-alignment heading priors
  (houses are square), GPS fixes better than the threshold once they span 15 m. Runs every 40 keyframes
  and on loops; redraws only if something moved > 3 cm / 0.6 deg.
- **Marks:** bumps, stalls and drop-offs are temporary (15 min) unless they recur 10+ min later.
- **Restore points:** before switching to Explore and before exploring; the newest 6 are kept.

## Navigation

Planner (A*, live obstacles) -> follower: turn on the spot only beyond 30 deg, in pulses, ending at
12 deg or on overshoot, then a 350 ms settle; steering while driving within the cruise - minimum power
margin. Re-plans every 8 s or after 3 consecutive sightings of something on the route. Blocked > 1.2 s
or stalled ~2 s: mark the spot, back up 25 cm if the rear is clear, settle, re-plan; give up after 4.

## Exploration

`frontier.dart` picks the next target: open floor next to unexplored space at least 30 cm from walls
(gaps under ~60 cm count as closed), leading to real unexplored space, reachable by the body, in the
robot's own room first (doorways under ~1 m split rooms). Pauses 1 s every 40 cm in new territory.
"Nothing left" must repeat 3 times before it drives home.

## Storage (per robot)

```
Documents/
  app_role.json                         role, brainUrl, lastRobot, lastRobotIp
  robots/<name>/bot_profile.json
  robots/<name>/maps/last.json
  robots/<name>/maps/<id>/meta.json, keyframes.bin, edits.json, graph.json
  robots/<name>/maps/<id>/restore/<time>/...   restore points
  logs/nav_*.csv  logs/events_*.csv  logs/log_*.csv
```
