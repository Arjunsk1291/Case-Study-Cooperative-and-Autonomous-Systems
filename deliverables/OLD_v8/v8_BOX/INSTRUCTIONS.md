# Mission v8.1 (kit 04, after the daytime runs test1/test2) - box only, no camera

## What went wrong in daylight (checked record by record, campus_6/New_log)
- **"Obstacles" when nothing was in front:** LiDAR points at **20-34 cm** that are not there. Scans with such a point:
  day 15-21 %, night 0-5 % (campus_6 night 0.0 %). While driving they sit mostly on the rover's RIGHT side, then front;
  never a fixed sun direction. -> LD20 near-range noise in daylight. **Please look what sits on the right side of the
  box within ~35 cm of the LiDAR** (power bank, cable, mount) - a bright surface there makes it worse.
- **test1 turned right in the open sea:** at 26.6 m two ghost points (22-24 cm, front-left) -> "something in front,
  wait" -> 0.25 s later "end wall" because the end-wall check while waiting did not test for a flat wall. Straight ahead
  the LiDAR saw 11 m (nothing). The ramp did not cause it.
- **Many Resume presses:** the phone-heartbeat stop (screen off).
- **Manual climb onto the interlock failed:** short pad taps only reached 0.12 power (soft start), the rover never moved.

## v8 changes
| change | result (replay of all logs) |
|---|---|
| LiDAR ignores points closer than 35 cm (was 20) | ultrasonics cover that range |
| **v8.1: every near LiDAR obstacle (< 0.6 m) is VERIFIED by the front ultrasonic**: median of its last 3 readings below 50 cm and not further than the LiDAR + 15 cm (its floor echo is 50-90 cm - 40 % of readings with nothing in front, so "< 50 cm" alone is not proof). Only WIDE things (>= 4 neighbouring beams: a person, a wall at an angle) need no ultrasonic confirmation. The 45-deg-down ultrasonic is never used for this (it sees the ground) | holds: test1 7 -> 0, test2 21 -> 0; still stops for the campus_2 person, the campus5 tree stake (LiDAR 36 / ultrasonic 34 cm) and the campus_3 wall it drove into |
| ultrasonic pings 60 ms apart (was 50; common HC-SR04 guidance against cross-talk between several sensors) | |
| end wall while waiting also needs a flat wall | test1's false turn cannot happen |
| **no phone heartbeat**; rover link lost -> waits and goes on; heading failsafe -> turns parallel and goes on; blocked 20 s -> lines up and tries again (3x); turn not done in 2 min -> tries again | **only STOP and a real tilt need Resume** (and "IMU lost", where it cannot steer at all) |
| leg 3 (wall after the open sea) closes in to **0.6 m** while driving | simulated: 0.84 -> 0.64 m after 5 m, 0.61 m after 8 m, no overshoot |
| **CLIMB** button (manual pad): hold = 0.35 for max 1.5 s (ramped), only if nothing is in front | for the cobble -> interlock edge |
| **Record a manual drive** button | for the leg-4 drive along the interlock (see below) |
| LiDAR intensity: near points can be filtered by intensity (setting, 0 = off); the lowest near intensity is logged | to set that filter from the next daytime log |
Kept from v7: landmark corridor + side PID in the open sea, last-leg wall detection (close in to 1.0 m), leg 1 / turns.

## Please do on the next run
1. Flash v8 (below). Daylight run as usual: Start mission. Stay in phone range for STOP / manual - it no longer
   stops when the phone sleeps.
2. **Leg 4 recording:** after the T3 left turn (or anytime), press **Record a manual drive** and drive along the
   interlock (light pavers, tactile strip on the left, dark cobbles on the right) up to the ramp with the railings.
   Use **CLIMB** at the edge. Then **Finish**. Send that log - I measure what the LiDAR sees along it and build leg 4.
3. **30 s sun test:** rover standing in the sun, log on, then turn it by hand 90 deg, another 30 s. Tells whether the
   near noise comes from the sun or from something on the box.

## Flash (box only)
Arduino IDE: `deliverables/v8_BOX/cas_box_mission/cas_box_mission.ino` - ESP32 Dev Module, Upload 115200, CPU 240 MHz,
Flash Frequency 40 MHz, Flash Mode DIO, Flash Size 4MB, Partition Default 4MB with spiffs, PSRAM Disabled,
Erase All Flash Disabled. Close the Serial Monitor first.
Or browser (espressif.github.io/esptool-js, 115200): `v8_BOX/BOX_v8.1_cas_box_mission_0x0.bin` at 0x0, no Erase.
Program size: 95 % of the app space.
