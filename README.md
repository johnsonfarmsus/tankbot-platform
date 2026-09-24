# TankBot Platform

A phone-brained robot platform, developed at small scale on the TankBot and designed to move up to a larger sidewalk robot (a wheelchair-based base with two arms) later.

The phone (iPhone or Android, via a Flutter app) is the brain. Small ESP32 boards handle hardware: motors, safety, and sensors. Everything talks over Wi-Fi using simple UDP messages defined in [`docs/protocol.md`](docs/protocol.md).

## Layout

| Folder | What it is |
|---|---|
| `firmware/motion/` | Motor control ESP32. Started from [tank-bot-esp32](https://github.com/johnsonfarmsus/tank-bot-esp32); gaining speed commands, a command watchdog, and a UDP API |
| `firmware/lidar-bridge/` | ESP32 that reads a Slamtec RPLidar C1 and streams full rotations over UDP |
| `app/` | Flutter brain app (coming next) |
| `tools/` | Desktop helpers, e.g. `lidar_client.py` live plot |
| `docs/` | Protocol and architecture notes |

## Current hardware (TankBot scale)

- ESP32 DevKit (38-pin, CP2102) + L298N + TP101 tank chassis
- Slamtec RPLidar C1 on UART2: lidar TX -> GPIO4, lidar RX -> GPIO27, 460800 baud, 5 V power
- iPhone 12 Pro as the brain

Note: on the TankBot a single ESP32 currently carries both the motors and the lidar. The firmware keeps them as separate modules so they can move to separate boards later.

## Quick start: lidar bridge

1. `cp firmware/lidar-bridge/src/secrets.example.h firmware/lidar-bridge/src/secrets.h` and fill in your 2.4 GHz Wi-Fi.
2. `cd firmware/lidar-bridge && pio run -t upload`
3. Live plot from a computer on the same network:
   ```
   python3 -m venv venv && venv/bin/pip install matplotlib
   venv/bin/python tools/lidar_client.py --plot
   ```

## Safety

The motion firmware must stop the motors on its own if commands stop arriving (watchdog). Keep the robot where it cannot drive off a table while testing.

## License

GNU AGPL 3.0 (inherited from tank-bot-esp32).
