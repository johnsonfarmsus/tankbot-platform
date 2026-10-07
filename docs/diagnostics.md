# Diagnostics

When the robot misbehaves, look at data before changing code. The brain keeps three kinds of log in
its app container (`Documents/logs/`).

| Log | Written | Contents |
|---|---|---|
| `nav_YYYYMMDD_HHMMSS.csv` | automatically, one per Go To / exploration leg | every driving step (10 Hz): decision and why, pose, target, heading error, motor command, guardian verdict, re-plans, the robot's motor state and watchdog count, scan age, tracking mode, event-loop lag |
| `events_YYYYMMDD.csv` | automatically, whenever the position changes by anything other than driving | relocalisations, Set position, tracking switches (camera + lidar / lidar only), camera jumps and drift, lidar snaps, wide-search fixes, map straightening > 20 cm, rollbacks |
| `log_YYYYMMDD_HHMMSS.csv` | Settings > Sensor log | GPS fixes, compass readings, pose at 5 Hz |

## Getting the logs onto the Mac

```bash
xcrun devicectl device copy from --device <device id> \
  --domain-type appDataContainer --domain-identifier com.johnsonfarms.tankbotBrain \
  --source Documents/logs --destination ~/tankbot-work/logs/<phone>
```

(`xcrun devicectl list devices` shows the ids.) The same command with `--source Documents/robots`
backs up profiles and maps; `copy to` puts files back.

## Tools (`tools/`)

| Tool | Use |
|---|---|
| `analyze_nav.py <nav.csv>` | where the time went, motor starts and bursts, what interrupted driving, re-plans, turn reversals, timing health, watchdog stops |
| `analyze_sensor_log.py <log.csv>` | GPS accuracy and drift against the tracked path; compass error by place |
| `brain_send.py '<json>'` | send one controller message to the brain and print the result (e.g. `{"type":"explore.preview"}`) |
| `nopath_check.py` | rebuild what the planner sees and show what boxes the robot in |
| `cdp_check.py <url> [seconds] [js]` | load the controller in headless Chrome and query it (layout and error checks) |
| `lidar_client.py` | live lidar plot from the robot |

## What we learned the hard way (2026-10)

- **Manual drive re-apply fought Go To** (commanded stop 10x a second): found by the trip recorder
  showing motors off 70 % of the time while commanded forward.
- **Pulsed full-power turning overshot and reversed** (the left-right wave): turn reversals in the log.
- **The robot spins at 200+ deg/s**; scans taken while spinning are smeared and can match 90/180 deg off.
- **iPhone SE camera tracking drifts in dim rooms**, even standing still: events log shows camera drift
  with idle motors. Lights on helps; the lidar-only fallback covers it.
- **Indoors GPS is useless** (2.6 m typical, 10 m worst) and the compass is 20-40 deg off near the
  motors: `analyze_sensor_log.py` on a recorded drive.
- **A flat battery looks like a software hang**: robot unreachable, lidar stale, no reflex events.
