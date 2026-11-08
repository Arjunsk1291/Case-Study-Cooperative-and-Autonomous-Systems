# Mission v7 (kit 04, after campus_6) - box only, no camera

campus_6 = the first fully good open-sea crossing (video: on/along the tactile strip the whole way). It is used as
the **teaching run**: its landmark measurements set the values (tools/landmark_db.json, run marked "teach").

## What the rover thought in campus_6 (full story: campus_6_what_the_rover_thought.md)
- Leg 1: wall 1.07 m, gap at 7 m, wall found again, turn landmark at 18.4 m -> right turn.
- Open sea: B wall behind first; at 0-2 m it fitted the arcade corner and wanted -95 deg (10 deg too far right, fixed
  itself by 4 m). Tree/post row at 11.5 m: **6.55-6.87 m** (median 6.67) - on the strip. The row DIRECTION wobbled
  +-6 deg and the rover followed that wobble (wanted heading -74..-86). End wall from 25 m. Stopped at 1.1 m.
- Leg 3 (arcade): wall 1.08 m, 8.3 m long (8.4 in campus5 - consistent), T3 left turn.
- **Leg 4 failed**: the real wall after T3 runs at -78..-85 deg (= arcade + 90), 2.1 m away. The old wall fit looked
  at the whole left side, caught the corner behind and "saw" the wall at -62 then -45 deg. The rover chased it, the
  heading failsafe fired 5 times (the stop-and-turn waited 3 s after Resume, the failsafe only 2 s), you took over.
- 6 pauses: the log could not say why (v7 now records the reason: STOP / phone / rover / tilt / failsafe / blocked).

## v7 changes (each one simulated first: tools/sim_open_sea.py, V6 vs V7)
| | change | why / result |
|---|---|---|
| last leg | wall = straight line **ahead-left** only, accepted within **15 deg** of the expected direction (arcade + 90) | replay of campus_6: sees the real wall the whole time (-79..-86) instead of -39 |
| last leg | **closes in to 1.0 m while driving ahead** (setting "last leg: distance to the wall"), target never more than 10 deg ahead of the heading | simulated: 2.2 m -> 1.3 m after 5 m, 1.1 m after 8 m, 1.0 m at 12 m, no overshoot, failsafe never close |
| all wall legs | stop-and-turn-parallel runs **before** the heading hold (1 s after Resume, not 3 s) | the campus_6 failsafe loop cannot happen |
| open sea | row distance **6.65 m** (teach run median 6.67) | |
| open sea | **cross-track PID** on the row distance: P 10 deg/m, I 1.5 (kept after the row: a constant sideways pull), D 20 (damping from the heading vs the row) | 5 % slide: 0.79 -> 0.30 m at the end; start 0.7 m off: corrected by 24 m |
| open sea | the row corrects the SIDE position, not the direction (its direction wobbled +-6 deg) | the worst simulated V6 failure (wrong direction kept after the row) is gone |
| open sea | row window 10-24.5 m, 4.0-9.5 m away, fit range 11 m; building front 2-6 m | a rover 1.3 m off now still sees the row; replay: row locks only at 10-24.4 m in all 5 runs |
| open sea | B wall behind not in the first 2 m / closer than 2.5 m | the campus_6 -95 deg glitch |
| open sea | the end wall clears the kept integral | no double correction |
Simulation, 90 % end error summed over 11 cases: V6 11.9 m -> V7 ~8 m; realistic worst case 1.9 -> 0.9 m; nominal 0.01 m.
Still hard (both versions): strip rotated 4 deg against the B wall + 3 % slide (1.3-1.5 m).

## IMU: accelerometer and magnetometer (checked, not used for steering)
- The heading loop already is a PID (Kethan's PD + the learned trim = integral). New: the cross-track PID above.
- Accelerometer (roll = side slope): roll was +1.5..+3.3 deg in every run (mostly the mounting); the sideways pull
  against the row does not follow it consistently (5 runs, one disturbed). A roll feed-forward would fit noise -> the
  PID integral handles any constant pull instead. Roll stays in the log; the benchmark re-checks it with every run.
- Magnetometer: the BNO055 runs in IMUPLUS mode (gyro + accelerometer); the magnetometer is switched off in that
  mode, so there is no compass data to use or log. Compass mode (NDOF) would change the heading source of the whole
  rover and is disturbed near steel buildings and the motors - not changed without a separate test.
- Gyro drift in the open sea: campus5 -6.3 deg, campus_6 +5.3 deg over ~3 min (opposite signs): ~+-2 deg/min. The
  walls behind/ahead and the row correct it.

## Improving with every run (laptop, in cas_box_mission/tools)
```
python3 narrate_run.py mis_NNN.bin                       # what the rover thought + problems
python3 landmark_benchmark.py mis_NNN.bin --name campus_7 [--teach]   # --teach if it drove ON the strip
python3 landmark_benchmark.py --report                   # all runs side by side + recommended settings
python3 sim_open_sea.py                                  # V6 vs V7 simulation (after parameter changes)
```
Send me the log + video after each run; I run these and adjust the settings / code from the evidence.

## Flash (box only)
Arduino IDE: `deliverables/v7_BOX/cas_box_mission/cas_box_mission.ino` - ESP32 Dev Module, Upload 115200, CPU 240 MHz,
Flash Frequency 40 MHz, Flash Mode DIO, Flash Size 4MB, Partition Default 4MB with spiffs, PSRAM Disabled,
Erase All Flash Disabled. Close the Serial Monitor first.
Or browser (espressif.github.io/esptool-js, 115200): `v7_BOX/BOX_v7_cas_box_mission_0x0.bin` at 0x0, no Erase.
Start with **Start mission**. On the phone (Settings, no reflash): "last leg: distance to the wall" (1.0),
"strip to left tree/post row" (6.65), side PID kp/ki/kd (10 / 1.5 / 20).
