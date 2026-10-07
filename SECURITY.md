# Security and privacy

TankBot is a home robot platform. This page says what is protected, what isn't, and what data the
app collects. Last reviewed: 2026-10-07.

## Secrets stay out of the repository

- Wi-Fi, OTA-update and hotspot passwords live in `firmware/*/src/secrets.h`, which is git-ignored.
  Copy `secrets.example.h` to start. The OTA upload script reads the password from there; it is never
  typed or stored anywhere else.
- **Pre-commit check:** run `git config core.hooksPath .githooks` once per clone. Every commit is then
  scanned for the actual values in your `secrets.h`, private keys, API tokens and files that should
  never be committed (secrets, keys, upload logs, which can contain the OTA password).
- Use **different** passwords for your Wi-Fi, OTA updates and the hotspot.

## Network: trusted home network only

The robot and the brain are designed for a home Wi-Fi you trust, like most hobby robots:

- The robot's web page and APIs (`tankbot.local`), the brain's controls (port 8080) and the UDP
  links (lidar, motion, sensors) have **no login**. Anyone on the same network can drive the robot,
  change its sensor setup and maps, and see its lidar and map.
- Firmware updates over Wi-Fi require the OTA password.
- When the robot can't reach your Wi-Fi it opens its own **"TankBot" hotspot**. Set `AP_PASS` in
  `secrets.h` (8+ characters); the built-in default is published in this repository.
- **Never** forward these ports to the internet or put the robot on a public or shared network.
- Self-driving always needs a controller page open and stops if it closes; the robot's watchdog and
  reflexes stop it if the brain goes silent.

## Privacy: what the app collects

Everything stays on your own devices and robot. There is no account, cloud service, analytics or
telemetry, and nothing is sent to us or anyone else.

| Data | Why | Where it stays |
|---|---|---|
| Camera (ARKit tracking, depth) | knowing where the robot is, seeing low obstacles | processed live on the phone; images are not stored or sent |
| Location (GPS) and compass | outdoor map accuracy; sensor logs you start | GPS fixes are tagged into maps on the phone; sensor logs only when you press Record |
| Maps of your home | navigation | the brain phone, a compact copy on the robot, and backups you make |
| Trip and position logs | diagnosing driving problems | the brain phone (`Documents/logs`); copied off only if you do it |
| Local network access | talking to the robot and serving the controls | your home network |

Maps and logs describe the inside of your home (and GPS-tagged maps where it is), so treat them like
photos of your house: think before sharing them, and don't commit them to a public repository.

## Reporting a problem

Open an issue on GitHub, or for anything sensitive, contact the maintainer privately first.
