# Standard wiring (ESP32 DevKit, 38-pin)

The default pin map baked into the firmware. If your build differs, change it in the controller's
**Bot > Wiring** view (see *The Wiring view* below).

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

**Power as built:** the 3S battery (9-12.6 V) goes through the rocker switch to the expansion board's
DC input and to the L298N's 12 V input (the battery kit's lever connectors and barrel-to-terminal adapters
make the joins). The expansion board's 5 V powers the lidar; the ultrasonic runs from 3.3 V.

**Recommended:** feed the 5 V rail (lidar, ToF) from a 12 V to 5 V buck converter (2 A or more) instead of
the expansion board's onboard 5 V. That is a small linear regulator: from a 12 V battery it turns several
volts into heat, runs hot with the lidar and the ESP32 on it, and an overheating regulator browns out the
ESP32.

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

## Defaults for more sensors

Each sensor type has default pins, given in order as you add sensors on the Bot page:

| Sensor | 1st | 2nd |
|---|---|---|
| Bumper | P13 | P23 |
| Ultrasonic | TRIG P14, ECHO P34 | TRIG P2, ECHO P35 (P2 is a boot pin: fine for a trigger) |
| ToF (serial) | T P32, R P33 | (one serial port: one ToF) |
| I2C (IMU) | SDA P21, SCL P22 (shared by all I2C parts) | |

Beyond these, choose **Custom** pins.

## The Wiring view

Controller > **Bot** > **Wiring**: how every part connects to the ESP32.

- **Pin map:** the expansion board's two header columns as printed, each used pin labelled with
  what's on it; flash, USB-serial and power pins greyed out; conflicts red, boot pins amber.
- **Pin budget:** how many of the 24 usable pins are in use.
- **Default or Custom** for the motor driver, the lidar and each sensor. Custom pickers only offer pins
  that can do the job (34, 35, 36 and 39 are inputs only).
- **Checks** before saving: two parts on one pin, an output on an input-only pin, flash or USB-serial
  pins, more than one ToF, a pin left unchosen. Boot pins (0, 2, 5, 12, 15) get a caution.
- **The robot checks too:** at startup it refuses to start a sensor whose wiring can't work, and the
  view shows why ("The robot didn't start it: pin 34 is input-only"). Everything else keeps running.
- **Tests:** motor A / B forward and back for half a second (A forward should drive the left track
  forward; **Swap A and B** and **Reverse A / B** fix it if not), and live readings for each sensor
  (press the bumper, wave a hand in front of the ultrasonic).

Save sends the wiring to the robot, which restarts once to apply it.
