# Mission v6 (kit 04, after campus5) - box only, no camera

## Why the open sea still went left (campus5 log + run.mp4, video time = log time + 43 s)
- The right path is the straight tactile strip. The rover started on it (video 152 s), drifted left and ended at the
  tree grate with the yellow sign (video 248 s; the log's "something 0.43 m in front" was a tree stake).
- Cause: v5's drift correction (+3.8 deg/min, learned on the B wall) walked the target from -84 to -76 deg. The end
  wall proves the gyro did NOT drift that way in the open sea (the B facade is probably slightly curved).
- Not a constant pull either: checked against the landmark row in 4 runs, the extra sideways pull is -0.9 +- 2.5 deg
  (both directions). Every deviation came from a wrong heading target -> no fixed "-3 deg" trim.

## v6: landmark corridor (permanent landmarks from 4 logs + the video, no teach drive)
| along the open sea | reference | gives |
|---|---|---|
| 0-14 m | B wall behind (perpendicular) | heading |
| 10-26 m | **row on the LEFT: trees in grates + stakes + sign/lamp posts**, strip **6.5 m** to its right (measured 6.2-6.8 m in the runs that were on the strip) | heading **and side position** |
| 26-40 m | left building front, ~3 m (when seen) | heading + side |
| last ~10 m | end wall ahead (perpendicular) | heading |
| in between | hold the heading (no drift correction) | - |
Picture `opensea_landmark_corridor.png`: green = the row the firmware locks on, red = the strip line it steers to
(campus_2 left, campus5 right). The track in the picture is odometry + gyro (approximate).

## Tested
- Replay of all 4 runs: the row locks only where it really is (10-25 m), corrections always point the right way
  (campus5/campus_2/campus4 drifted left -> steers right; campus_3 was right of the strip -> steers left).
- Simulation (calibrated to the real row detection rate, people, missing trees, gyro bias +-6/min, turn error 8 deg,
  start 0.7 m off, 3 % slide): end error 0.0-0.1 m typical, 0.5 m worst combination. v5 in the campus5 situation: 3.3 m.

## On site (phone, Settings) - no reflash needed
- "strip to left tree/post row (m)" 6.5: Live shows "LEFT ROW x m (want 6.5)". If the rover runs on the strip but Live
  says e.g. 6.2, set 6.2.
- "open sea: follow the landmark rows" can be switched off (then v5 walls only, without drift correction).
- "open sea heading trim" stays 0.

## Flash (box only)
Arduino IDE: `deliverables/v6_BOX/cas_box_mission/cas_box_mission.ino` - ESP32 Dev Module, Upload 115200, CPU 240 MHz,
Flash Frequency 40 MHz, Flash Mode DIO, Flash Size 4MB, Partition Default 4MB with spiffs, PSRAM Disabled,
Erase All Flash Disabled. Close the Serial Monitor first.
Or browser (espressif.github.io/esptool-js, 115200): `v6_BOX/BOX_v6_cas_box_mission_0x0.bin` at 0x0, no Erase.
Start with **Start mission**. Send the log + video like this time.
