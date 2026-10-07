# TankBot Platform

A robot platform that scales with what you plug into it. The bare minimum (an ESP32, a motor
driver and a chassis) is a drivable robot. Add bumpers and distance sensors and it protects itself.
Add a phone and it gains a brain. Add a lidar and it maps, remembers your house and drives itself to
where you tap. Any chassis, any sensor layout, one setup flow.

Developed on the [TankBot](https://github.com/johnsonfarmsus/tank-bot-esp32) (TP101 tracked chassis);
designed to move to a larger wheelchair-based robot next.

## How it fits together

```
 Robot (ESP32)                         Brain (phone/tablet running the app)         Controllers
 motors + watchdog                 --->  tracking: camera + lidar scan matching  <---  web browser
 reflexes: bumper / cliff / range        map: keyframes, loop closing, edits           (any laptop)
 lidar bridge                            guardian: is forward clear, and why?    <---  the app in
 sensor feed + capability announce       navigator: plan a route, drive it             Controller role
 own web page (drive + setup)            control server (web pages + live data)
```

Every part reads the robot's geometry from a **bot profile** (size, drive type, where each sensor
sits). Nothing about a specific robot is hard-coded.

## Capability tiers

| Tier | Hardware | You get |
|---|---|---|
| Drive | ESP32 + motor driver + chassis | manual driving from the ESP32's own web page |
| Reflexes | + bumpers, ToF, ultrasonic | bump stop and back-off, cliff stop, close-obstacle stop, on the ESP32 itself |
| Brain | + a phone or tablet running the app | full controller, position tracking (mounted), the robot's status face |
| Mapping | + RPLidar | maps, remembering places, relocalisation, tap-to-go |
| 3D awareness | + a phone with a depth camera (LiDAR iPhone) | low obstacles and drop-offs the lidar can't see |

The controller's Settings page shows the current tier and what would unlock the next.

## Repository layout

| Folder | Contents |
|---|---|
| `firmware/tankbot/` | ESP32 firmware v3: sensor table, directional reflexes, lidar bridge, sensor feed, web page, OTA |
| `firmware/motion/` | the original TankBot firmware, kept for reference |
| `app/tankbot_brain/` | the **TankBot** app (Flutter): Mounted brain / Brain in hand / Controller roles |
| `docs/` | [user guide](docs/user-guide.md), [wiring](docs/wiring.md), [protocol](docs/protocol.md), [bot profile](docs/bot-profile.md), [app architecture](docs/app-architecture.md), [diagnostics](docs/diagnostics.md), [roadmap](docs/ROADMAP.md), [changelog](CHANGELOG.md) |
| `tools/` | desktop helpers: log analysis, brain commands, controller checks ([tools/README](tools/README.md)) |

## Setting up a robot

1. **Wire it** following [docs/wiring.md](docs/wiring.md). Start with the motor driver; add sensors
   any time.
2. **Flash the firmware.** Copy `firmware/tankbot/src/secrets.example.h` to `secrets.h`, enter your
   2.4 GHz Wi-Fi and an OTA password, then `cd firmware/tankbot && pio run -t upload` over USB once.
   From then on `pio run -e ota -t upload` updates it over Wi-Fi.
3. **Drive it.** Open `http://tankbot.local/` on any device on your Wi-Fi (buttons, joystick, or the
   arrow keys / W A S D on a computer) (away from home the robot
   broadcasts its own `TankBot` network instead). Speed levels and steering trim live here too.
4. **Tell it what's attached** at `http://tankbot.local/setup`: name, drive type, sensors, pins.
   If you followed the standard wiring the pins are already right. Pointing the ToF at the floor?
   Press "Calibrate ToF floor" once.

## Adding a brain

1. Install the **TankBot** app on a phone (iOS today; Android when ARCore support lands). Each time it
   opens it asks what the device is doing:
   - **Mounted brain**: the phone rides on the robot; camera tracking, mount detection, status face.
   - **Brain in hand**: same brain off the robot, tracking with the lidar alone.
   - **Controller**: a remote for a brain on the network.
2. Open **`http://tankbot.local/brain`** in any browser on the same Wi-Fi: the robot sends you to
   wherever the brain is (bookmark it). The robot's own page (`tankbot.local`) also shows an
   "Open full controls" button whenever a brain is running. The app's Controller role works too.
3. Controller pages: **Drive** (map you can drag, zoom and pinch, follow-the-robot, a colour key,
   joystick, arrow keys, radar view, Go to... anywhere on the map, status cards with actions), **Maps**
   (save, load, edit, no-go lines, map quality), **Bot** (the profile: platform size, sensor
   positions and heights), **Settings** (trim, power levels, obstacle stop and pass distances,
   what the robot and the phone can do).

### Which phone, and mounting it

Any iPhone with ARKit works as a brain. A LiDAR iPhone (12 Pro and later Pros) adds the depth camera
(low obstacles, drop-offs); without one (e.g. iPhone SE) the brain runs one tier down and falls back to
lidar-only tracking when the camera drifts (dim rooms, plain walls).

Mount it upright or on its side (set "Brain phone mounted" on the Bot page; the screen locks to it)
with the back camera facing forward. Enter the camera's position and height in the Bot page.
In Robot mode the brain waits until the phone has sat still in its cradle for 3 s before mapping, so
handling the phone never smears a map. If the phone gets knocked, mapping pauses until it settles.

## Mapping and driving

See the [user guide](docs/user-guide.md) for every page, button and map symbol.

- **Mapping modes:** Explore (build the map deliberately), Maintain (default: grows into new areas and
  only changes for persistent changes), Off. Bumps and drop-offs are temporary unless they recur.
- **Go to...:** click anywhere on the map. It plans around walls, no-go lines and live obstacles,
  backs away and re-plans when blocked or stalled, and stands still when unsure where it is.
- **Explore on its own** (Explore mode): drives to unexplored edges room by room and comes home.
- **Set position** when it's lost; **restore points** undo bad mapping (Maps page).
- Map straightening keeps long maps square (wall alignment, loop closing, good GPS outdoors).
- Safety is layered: ESP32 watchdog -> on-board reflexes -> the brain's guardian -> the controller's
  watchdog. Autonomous driving always runs its own obstacle checks.
- Trip, position-event and sensor logs record what happened; see [diagnostics](docs/diagnostics.md).

## Developing

- Firmware: PlatformIO. `pio run -e esp32dev` builds; `pio run -e ota -t upload` deploys over Wi-Fi.
- App: Flutter 3.32+. `flutter test` runs the mapping, matching, planning, guardian, pose graph and
  exploration tests (must pass before every install);
  `flutter build ios` / Xcode for the phone.
- The wire protocols are in [docs/protocol.md](docs/protocol.md); `tools/lidar_client.py` is a
  minimal reference client.

## License

GNU AGPL 3.0 (inherited from tank-bot-esp32).
