# Hardware guide

What to buy for a TankBot-platform robot, and why. The platform's rule is **cheap first**: every
choice here was weighed for capability per dollar, and anything optional is marked so. Prices are
rough US figures from late 2026; they move, so check current listings.

Last updated: 2026-11-12.

## Three ways to build

The software uses whatever a robot has, so the same platform supports three kinds of build:

| | Phone on the robot | Sensors on the robot, brain in hand | Both |
|---|---|---|---|
| Idea | cheapest robot hardware; the phone brings the sensor suite | no spare phone needed; any phone, tablet or laptop does the thinking | independent sources cross-check each other |
| Turning | phone gyro | robot IMU | robot IMU + phone gyro |
| Position | camera tracking, plus the lidar if fitted | the lidar | lidar + camera |
| Low obstacles, drop-offs | the phone's depth camera (LiDAR iPhones) | ultrasonic / ToF / bumper, look-ahead sensor | all of them |

## Core electronics

See the parts list in the [README](../README.md#parts-list) for the full bill of materials.

**5 V supply.** The expansion board's onboard 5 V is a small linear regulator that runs hot feeding
the lidar and ESP32 from a 12 V battery. Use a **12 V to 5 V buck converter**: a fixed-5 V mini module
(5.5-30 V in, about 1.8 A, wired leads) is plenty, since TankBot's whole 5 V side draws under about
0.7 A. Wiring is in [wiring.md](wiring.md). A second module can run a travel router and phone
charging (below); the two loads together would overload one.

## IMU

**Default: GY-521 (MPU-6050)**, about $3, pins pre-soldered, I2C on SDA P21 / SCL P22, 3.3 V.

- It has a **gyro** (turning rate) and an **accelerometer** (tilt, bumps, being lifted). The gyro is
  the important half: it keeps the heading through fast turns between lidar scans, and it is what a
  brain-in-hand build turns with.
- Drift is handled in software: the gyro is re-zeroed whenever the robot sits still, and checked
  against the lidar and the phone's gyro while driving.
- Many GY-521s carry clone or related chips; the driver should accept them by chip ID.
- Mount it flat, rigid and near the middle, away from the motors.
- **Not an ADXL345:** accelerometer only, no gyro. **BNO055 / BNO085** (onboard fusion, $18-35) only
  pay off on robots without a lidar; the BNO055 also needs soldering and has I2C quirks on the ESP32.

With a phone also on the robot, the brain fuses both gyros, weighting each by how much it can be
trusted, with the lidar as the referee.

## Cliffs and drop-offs

Missing an edge can destroy the robot, and every sensor has a way to be fooled, so use layers.

**Look ahead, at an angle.** A downward sensor at the nose either sticks out (a new obstacle) or
sees the edge too late at speed. Aim a **narrow-beam** distance sensor **down and forward** from up
high (the front of the lidar tower) at a spot on the floor 30-40 cm ahead (50-80 cm for a faster
outdoor robot). Mounting high gives a steeper angle, which returns more light from glossy floors
like vinyl.

- **Recommended: TF-Luna** (about $21-26): about a 2 degree beam (a crisp spot), rated for 70 klux of
  sunlight, up to 250 readings a second, UART or I2C (share the IMU's bus). Its 20 cm minimum range
  doesn't matter in a look-ahead mount.
- **TOFSense** (F-series, $40+): also sun-tolerant; check its beam angle and minimum range before
  using it this way.
- The three readings: about the calibrated floor = normal; much longer or no return = drop-off
  (stop, back up); much shorter = something on the floor ahead. The IMU's pitch adjusts the expected
  reading on ramps.
- A single spot guards forward driving only; turning near an edge needs a corner sensor, slowing
  near remembered drop-offs, or a tilted lidar.

**Backups.** The **IMU** notices the nose dipping and triggers stop-and-reverse. Drop-offs found are
**remembered on the map**.

**Phones.** A LiDAR iPhone mounted level sees the floor from about 30 cm ahead to several metres,
about 45-50 cm wide: a built-in look-ahead cliff sensor indoors (weaker in bright sun). About 10
degrees of downward tilt brings that in to about 20 cm; much more hurts camera tracking.

**Not recommended as the cliff sensor:**
- **Ultrasonic pointed down:** sun-proof, but sound skips off smooth floors at shallow angles (vinyl
  reads as a false cliff), and a steep mount needs a protruding nose.
- **IR reflective modules** (LM393, TCRT5000): fooled by dark floors and sunlight.
- **Wide-cone ToF** (VL53L0X, TOF10120): the cone smears over the floor at a distance. A VL53L4CD is
  a fine budget **indoor** sensor for other jobs; none of them like sunlight.

## Lidar

**RPLidar C1** (Slamtec spec): 0.05-12 m on white, 0.05-6 m on black; accuracy plus or minus 30 mm,
resolution 15 mm; 5,000 readings a second, 8-12 turns a second, one reading every 0.72 degrees; up to
1.5 degrees of scan tilt; 40 klux. Single readings are only good to 3 cm; matching whole scans of
about 500 points pins position down far better.

**One per robot** is the budget. Cheaper alternatives (each needs a small driver): LDRobot LD06 /
LD19, YDLIDAR X2 / X3, and salvaged robot-vacuum lidars.

**A second, tilted lidar** (a later 3D project): aimed down, its slice hits the floor across the whole
width ahead, sees low obstacles, and as the robot drives it sweeps out a 3D point cloud (a height
map). The level lidar keeps doing the tracking. It needs a precise tilt calibration, the IMU for
pitch and roll, the ESP32's last serial port, and the buck converter.

## Odometry without expensive encoders

Wheel or track odometry is optional and cheap to add:

- **Brushless hub motors** (many cheap wheelchairs) already contain hall sensors: tap their signals.
- **Brushed geared motors:** a magnet plus a hall sensor (about $1-3 a wheel) or an AS5600 magnetic
  angle sensor (about $2).
- **Optical-flow sensor** looking at the ground (drone flow sensors, or a cheap optical-mouse
  sensor): measures real movement, immune to wheel slip.
- **Or none:** lidar matching, the IMU and the phone's tracking already provide it.

## Phones as brains

**iPhone, with LiDAR depth:** Pro models only: 12 Pro / Pro Max through 17 Pro / Pro Max, and the
18 Pro (2026); also iPad Pro (2020 on). No standard, mini, Plus, SE, e or Air model has it. The
cheapest is a **refurbished 12 Pro** (from about $274 on Back Market in Aug 2026). For a robot: the
lowest cosmetic grade is fine, carrier-locked is fine (Wi-Fi only), check battery health and that it
isn't Activation Locked.

**iPhone without LiDAR** (SE, 11 Pro, any non-Pro): a full brain with camera + lidar tracking, no depth.

**Android** (not yet supported). The brain-in-hand role is the easy port (lidar + robot IMU do the
sensing); the mounted role needs ARCore. Only phones on Google's
[ARCore supported list](https://developers.google.com/ar/devices) can be mounted brains; most give
depth estimated from camera motion, not a sensor. With a hardware ToF depth sensor: **LG V60 ThinQ**
(listed by Google as supporting ToF hardware), and Samsung's **Galaxy S20 Ultra** and **Note10+**.
Cheap certified picks: moto g 5G-series, used Pixel 6a / 7a. Budget models not on the list (moto g
pure, BLU, unbranded "Ultra" phones) can at most be brains in hand.

**The controller** is a web page: any phone, tablet or laptop with a browser can use it today.

## Wired brain (a phone with dead radios)

A phone with broken Wi-Fi can still be a mounted brain over a cable: **phone, Apple Lightning to USB
3 Camera Adapter (with its charging port), USB Ethernet adapter, a pocket travel router on the
robot.** The router joins the home Wi-Fi for the phone.

- Router: **GL.iNet GL-MT300N-V2 (Mango)**, about $30, 5 V USB powered, two Ethernet ports.
- Set it as a **bridged repeater** (labelled relay / relayd / WDS / extender), so the phone gets a
  normal home-network address. **Not WISP / router mode**, which hides the brain behind a separate
  network: the robot and controller couldn't reach it.
- Reserve the robot's address in the home router: bridged mode may not pass `.local` names.
- Power the router and the phone's charging from their own buck converter.
- Test on the desk first: the phone's Settings shows an Ethernet entry with a home-network address.

## Long range: LoRa / Meshtastic (planned)

A slow (around a kilobit a second, messages of a couple of hundred bytes), long-range (kilometres)
channel for **status, alerts and commands** (stop, pause, return home, go to waypoint), not for
driving, maps or video. Its value is a **remote stop and a beacon that work without Wi-Fi, cellular
or the phone**. Attach it to the robot, not the phone. Use a private channel with its own key, and
authenticate commands. Cellular (the brain phone's own) is the main long-range link; LoRa is the
independent backup.
