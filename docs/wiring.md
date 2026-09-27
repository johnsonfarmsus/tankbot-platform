# Standard wiring (ESP32 DevKit, 38-pin)

The default pin map baked into the firmware. Change it on the robot's setup page
(`http://tankbot.local/setup`) if your build differs.

| Function | ESP32 pin | Notes |
|---|---|---|
| Motor driver (L298N) | IN1 16, IN2 17, IN3 18, IN4 19, ENA 25, ENB 26 | ENA/ENB jumpers removed |
| Lidar (RPLidar C1, UART2) | lidar TX -> 4, lidar RX <- 27 | 5 V power, 3.3 V logic |
| ToF (TOFSense-F2 Mini, UART1) | ToF T -> 32, ToF R <- 33 | 5 V power (4.3-5.2 V), 3.3 V logic, 921600 baud. JST GH 1.25 mm connector |
| Ultrasonic (HC-SR04P / RCWL-1601) | TRIG 14, ECHO 34 | power from 3.3 V so ECHO is 3.3 V. A plain 5 V HC-SR04 needs a divider on ECHO (1 k / 2 k) |
| Bumper left | 13 to COM, NC to GND | internal pull-up; normally-closed = fail-safe |
| Bumper right | 23 to COM, NC to GND | internal pull-up; normally-closed = fail-safe |
| I2C (future IMU) | SDA 21, SCL 22 | reserved |
| Status LED | 2 (onboard) | |

Pins 0, 2, 5, 12 and 15 affect booting and are not used for anything that could be held at power-up.
Pins 34-39 are input-only (fine for ECHO).

## Power

The lidar, the ToF and the ultrasonic all share the 5 V rail. Feed it from a proper 5 V buck
converter (2 A or more) off the battery; the L298N's small onboard regulator is marginal for the
lidar alone.

## Sensor jobs

- **Bumpers:** last line of defence. A hit stops the motors on the ESP32 itself, backs off briefly,
  and blocks forward motion until released.
- **ToF (narrow beam):** cliff / drop-off sensor. Mount low at the front, angled down to look at the
  floor 15-25 cm ahead, then press "Calibrate ToF floor" on the setup page. If the floor reading
  jumps much longer (or disappears), forward motion is blocked.
- **Ultrasonic (wide cone):** low obstacles ahead that the lidar's scan plane misses. Forward motion
  is blocked closer than the stop distance (default 150 mm).

## Robot-side data

- `http://tankbot.local/api/capabilities` : what is attached and how it is wired (JSON)
- `http://tankbot.local/api/sensors` : live readings (JSON)
- UDP 5603: send `TSSUB` (repeat every few seconds); receive `TCAP1`+JSON once, then `TSN1`+JSON at 20 Hz
