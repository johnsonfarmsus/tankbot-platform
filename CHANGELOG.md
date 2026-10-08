# Changelog

## Unreleased

- **Interchangeable brains:** firmware stores robot-level settings (`/api/settings`) and a compact
  map (`/api/map`, LittleFS, chunked all-or-nothing uploads); the brain syncs settings (newer wins),
  keeps the robot's map copy fresh and starts from it when it has no map. Maps can start from a grid.
- Defaults that drive well: stop 100 mm, pass 50 mm, depth 100/40 mm, platform height 66 mm.
- Fixed the flickering Robot not responding card.
- iPhone 11 Pro as a brain; new app icon.
- The app asks for its role on every launch (last choice highlighted), checks whether the robot
  already has a brain and suggests Controller; a running brain that sees a second brain warns and
  won't self-drive.
- **Wiring view** (Bot > Wiring): the expansion board's pin map, a pin budget, Default or Custom pins
  for the motor driver, lidar and each sensor, checks that block impossible wiring, motor tests,
  swap / reverse helpers and live readings. Firmware 3.1 validates the wiring at startup (miswired
  sensors aren't started and say why) and adds `/api/test/motor`.
- The robot page's gear panel shows the robot's address, name, Wi-Fi and signal, brain and firmware
  (`/api/info`).
- **Lidar + gyro tracking:** in lidar-only mode turning comes from the phone's raw gyro; no freezing
  on missed scans; a camera making impossible jumps is benched for a minute and ARKit restarted;
  autofocus locked; Set position searches every heading.
- The app is called **TankBot**; the README has media, a parts list and build steps; the model files
  are a backup zip next to the Printables link.
- Security: hotspot password in `secrets.h`, a pre-commit secret check, `SECURITY.md`; history
  cleaned of an upload log and an old key before going public.

## v0.3 (2026-10-06)

Firmware v3 and a brain that maps, navigates and explores on its own.

**Robot (firmware v3)**
- Generic sensor table in flash (any number of bumpers and rangers in any direction), directional
  reflexes, per-bumper back-off duration (default 150 ms, 0 = just stop).
- Sensor feed (`TSSUB`/`TCAP1`/`TSN1`), brain announce (`TBRN1`), `/api/brain` and the
  `tankbot.local/brain` redirect; keyboard driving and links on the robot's own page.

**Controller**
- Redesign: one shell with status lights, pan/zoom/pinch/follow map, colour key, scale bar, status
  cards and toasts, phone layout. Moved into `assets/controller.html`.
- Set position, restore points, mapping mode selector, exploration controls, tracking choice,
  Saved confirmations, Robot not responding card.

**Mapping**
- Pose graph straightening (Levenberg-Marquardt, skyline solver): driving and loop links, wall
  alignment, GPS only when better than a threshold and spread over 15 m.
- Mapping modes Explore / Maintain (default) / Off; Maintain writes only persistent changes and
  extends into unmapped ground.
- Bump, stall and drop-off marks are temporary unless they recur; drop-off detection rejects glossy
  floor reflections; eraser removes marks; undo only undoes user edits.
- Trust gating: doubtful scans are never drawn. Restore points before Explore / exploring.

**Navigation and exploration**
- Fixed the Go To stutter (manual drive re-apply commanded stop 10x/s during autonomous driving).
- Steering while driving, pulsed turns with settling, calmer re-planning.
- Recovery when blocked (mark, back up, re-plan), stall detection, standing still when unsure.
- Autonomous exploration: frontiers with gap closing, rooms first, reachability, previews, settle
  pauses, rollback when lost, confirmed "nothing left", reasons on finish; cliff-sensor acknowledgment.

**Tracking**
- Robot link finds and re-finds the robot (retry, last address, reconnect after silence).
- Drift handling: wider windows, wide re-search, lidar snap on solid ground, no whole-map jumps while
  self-driving, camera glitch filter, drifting-camera fallback to lidar only, no matching on spin-smeared
  scans; Settings choice camera + lidar / lidar only.
- iPhone SE support (no depth sensor); landscape phone mounting with orientation lock.

**Diagnostics**
- Trip recorder, position events log, GPS/compass sensor log; analysis tools in `tools/`.

**Fixes**
- Startup race that replaced the saved profile with defaults and pushed a default lidar position.
- Mode switch to lidar-only starting from a stale pose (5 m jumps).

## v0.2 (2025-12-23) and earlier

The original TankBot firmware, lidar bridge and first brain app; see the git history.
