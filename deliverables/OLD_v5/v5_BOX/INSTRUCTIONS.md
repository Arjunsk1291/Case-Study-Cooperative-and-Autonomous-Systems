# Mission v5 (kit 04, after campus4) - box only, no camera

## Why the open sea drifted (campus4 log + video)
1. **Wrong reference:** v4 locked the heading on the entrance signboard. That board line was 9 deg off the
   B-perpendicular (campus_3: 5 deg the OTHER way) -> the video shows it leaving the slab path to the left.
2. **IMU drift:** the BNO055 runs in IMUPLUS mode (gyro + accelerometer). Standing still it does not drift (0.00 deg in
   every log), but WHILE DRIVING it drifts **+3.6 to +6.0 deg per minute** (measured on straight walls in campus_2/3/4,
   scatter 0.3 deg). The open sea takes ~2.5 min -> 8-15 deg -> metres off at the far end.

## What v5 does in the open sea (your order: wall perpendicular > board parallel > IMU)
| stretch | reference |
|---|---|
| first ~8-10 m | the **B wall BEHIND** the rover: heading kept at right angles to it (robust line fit, +-12 deg gate) |
| middle ~10-15 m | gyro + **drift correction** measured on the leg-1 B wall (each >= 15 s straight stretch, outliers rejected, default +5 deg/min) |
| last ~10 m | the **end wall AHEAD**: right angles to it (only if it is where the end wall must be; may correct max 8 deg) |
| first 6 m, no wall seen | the board, only within 5 deg (weak fallback) |
No board line-up stops any more.

**Closer to the last pillar (v5c, against the cobble drift):** it now turns when it is **0.40 m** past the last pillar
(was 0.60; setting "turn when the last pillar is (m) behind"). The pillar flickers in the LiDAR when it is beside the
rover, so every sighting is turned into the pillar's position along the wall and the rover's distance count says
when it is 0.40 m past it (campus_2: turn at 19.03 m instead of ~19.35 m). If the pillar is only seen later (campus_3/4:
the rover was further from it), the rover moves right **while driving** at the start of the open sea: 10 deg right for
exactly the extra distance, then straight - no stop. Simulated: it is on the intended line within 3 cm by 10 m
(start 0-1 m off) and stays on it to the end wall. Leg 1 / arcade / T3 / leg 4 as in v4.

## Tested
- Replay of the real scans (detection only, nothing tuned to old paths): B wall behind found in campus_2 and campus4
  for the first 7-9 m; drift estimate from leg 1: +5.2 / +5.0 (outlier rejected) / +5.5 deg/min.
- Simulation (36 m crossing, LiDAR with noise and dropouts, drifting gyro), lateral error at the end wall:
  locked gyro heading 3.5 m median / 6 m worst; board lock (campus4) 5-11 m; **v5: 0.14 m median, 0.3 m worst**
  (typical drift), 0.4 m if the drift is only +3/min. Limits: a skew end wall (8 deg -> 1.6 m), sideways sliding on
  slanted cobbles (3 % -> 1.1 m: no heading reference can see that).
- Also fixed: "Resume" after manual driving before Start no longer runs leg 1 without its start-up (campus4 log).

## Flash (box only)
Arduino IDE: `deliverables/v5_BOX/cas_box_mission/cas_box_mission.ino` - ESP32 Dev Module, Upload 115200, CPU 240 MHz,
Flash Frequency 40 MHz, Flash Mode DIO, Flash Size 4MB, Partition Default 4MB with spiffs, PSRAM Disabled,
Erase All Flash Disabled. Close the Serial Monitor first.
Or browser (espressif.github.io/esptool-js, 115200): `v5_BOX/BOX_v5_cas_box_mission_0x0.bin` at 0x0, no Erase.

## Run
Press **Start mission** (not Manual -> Resume). Drive leg 1 normally (it measures the drift on the B wall).
On the phone, Live shows "open sea ref: WALL BEHIND / gyro (+drift x deg/min) / WALL AHEAD".
Send the log + video like this time.
