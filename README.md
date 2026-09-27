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
| `firmware/tankbot/` | ESP32 firmware v2: configurable pins and sensors, reflexes, lidar bridge, web page, OTA |
| `firmware/motion/` | the original TankBot firmware, kept for reference |
| `app/tankbot_brain/` | the Flutter app: Mounted brain / Brain in hand / Controller roles |
| `docs/` | [wiring](docs/wiring.md), [protocol](docs/protocol.md), [bot profile](docs/bot-profile.md), [app architecture](docs/app-architecture.md), [roadmap](docs/ROADMAP.md) |
| `tools/` | desktop helpers (`lidar_client.py` live lidar plot) |

## Setting up a robot

1. **Wire it** following [docs/wiring.md](docs/wiring.md). Start with the motor driver; add sensors
   any time.
2. **Flash the firmware.** Copy `firmware/tankbot/src/secrets.example.h` to `secrets.h`, enter your
   2.4 GHz Wi-Fi and an OTA password, then `cd firmware/tankbot && pio run -t upload` over USB once.
   From then on `pio run -e ota -t upload` updates it over Wi-Fi.
3. **Drive it.** Open `http://tankbot.local/` on any device on your Wi-Fi (away from home the robot
   broadcasts its own `TankBot` network instead). Speed levels and steering trim live here too.
4. **Tell it what's attached** at `http://tankbot.local/setup`: name, drive type, sensors, pins.
   If you followed the standard wiring the pins are already right. Pointing the ToF at the floor?
   Press "Calibrate ToF floor" once.

## Adding a brain

1. Install the app on a phone (iOS today; Android when ARCore support lands) and choose a role:
   - **Mounted brain**: the phone rides on the robot; camera tracking, mount detection, status face.
   - **Brain in hand**: same brain off the robot, tracking with the lidar alone.
   - **Controller**: a remote for a brain on the network.
2. On the brain phone's screen, note the **controller address** (e.g. `http://192.168.1.199:8080`)
   and open it in a browser, or in the app in Controller role.
3. Controller pages: **Drive** (joystick, arrow keys, radar and map views, Go to...), **Maps**
   (save, load, edit, no-go lines, map quality), **Bot** (the profile: platform size, sensor
   positions and heights), **Settings** (trim, power levels, obstacle stop and pass distances,
   what the robot and the phone can do).

### Mounting the phone

Mount it upright with the back camera facing forward. Enter its position and height in the Bot page.
In Robot mode the brain waits until the phone has sat still in its cradle for 3 s before mapping, so
handling the phone never smears a map. If the phone gets knocked, mapping pauses until it settles.

## Mapping and driving

- Drive a room with the joystick or arrow keys; the map builds live. Turns are fine; the brain skips
  scans taken while spinning fast and corrects camera drift against the map ten times a second.
- Loops close automatically when the robot returns somewhere it mapped earlier.
- Maps autosave every 20 s and reload on startup; the robot finds itself on the saved map with the
  lidar (checking where it last was and the home spot first).
- **Go to...**: tap a spot on the map. The robot plans around walls, no-go lines and anything it sees
  live, drives there, and re-plans when something is in the way. Space, an arrow key, the joystick,
  the Stop button, or leaving the page stops it immediately.
- Safety is layered: ESP32 watchdog (commands must repeat every 300 ms) -> on-board reflexes -> the
  brain's guardian (lidar around the body's front, depth camera, reflexes) -> the controller's own
  watchdog. Autonomous driving always runs its own obstacle checks regardless of manual settings.

## Developing

- Firmware: PlatformIO. `pio run -e esp32dev` builds; `pio run -e ota -t upload` deploys over Wi-Fi.
- App: Flutter 3.32+. `flutter test` runs the mapping, matching, planning and guardian tests;
  `flutter build ios` / Xcode for the phone.
- The wire protocols are in [docs/protocol.md](docs/protocol.md); `tools/lidar_client.py` is a
  minimal reference client.

## License

GNU AGPL 3.0 (inherited from tank-bot-esp32).
