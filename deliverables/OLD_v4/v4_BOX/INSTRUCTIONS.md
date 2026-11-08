# Mission v4 (kit 04, after campus_3) - box only, no camera

## What changed after campus_3
| seen | cause (from the log) | v4 |
|---|---|---|
| leg 1 drove into the wall, then stopped "for a person" | **bug**: the wall timer was reset every scan, so "wall lost" never fired. After the gap at 6.3 m the follower stayed glued to the old wall line (1.20 m) and never accepted the next wall (0.90 m, converging 5-7 deg). The gyro held a straight line perfectly - straight into the converging wall | wall lost after 1 s, new wall taken after 3 consistent fits, noise rejected (median of 5; a jump only counts if 3 fits in a row agree). Stays **parallel** + keeps its distance (replayed: new wall found at 12.6 m, steers 8 deg away) |
| rover at an angle to the wall (hand-placed, after a stop) | rolling correction too weak for big angles; failsafe at 12 deg | more than 10 deg off the wall line -> **stop + pivot parallel**, then go on |
| at 45 deg to the wall it thought someone was standing there | front blocker = the wall itself | front points on the followed wall's line = **wall, not a person** -> turn parallel instead of waiting. Anything else in front: stop and wait as before |
| open sea: you had to steer by hand | the gyro held its heading within 1 deg - the reference heading was the weak part; positioning chased the wrong object (10 deg off) | **entrance signboard** = the reference: rolls to it, stops, moves sideways to **60 cm** from it, turns **parallel**, locks that heading for the crossing. Near the end it follows the **end board** on the right. No board -> B-perpendicular as before |
| after the open sea | route map: arcade wall, then T3 LEFT at the building corner | leg 3 (arcade) -> **T3 left** -> **leg 4**: next left wall until it ends -> stop (open space 2) |

Kept: Kethan's pivot (full power at once, stop ~7 deg early), Kethan's landmark for the first turn, his heading-hold numbers, flat-wall end check, level gate, people rule, STOP / manual take-over, gentle drive ramps.
Nothing from earlier runs is "learned" into the path: every heading is measured live on the run.

## Flash (box only)
Arduino IDE: open `deliverables/v4_BOX/cas_box_mission/cas_box_mission.ino` - ESP32 Dev Module, Upload 115200, CPU 240 MHz,
Flash Frequency 40 MHz, Flash Mode DIO, Flash Size 4MB, Partition Default 4MB with spiffs, PSRAM Disabled,
Erase All Flash Disabled. Close the Serial Monitor first.
Or browser (espressif.github.io/esptool-js, 115200): `v4_BOX/BOX_v4_cas_box_mission_0x0.bin` at **0x0**, no Erase.

## Run
1. Phone: `CAS-Box-04` / `CASbox04` -> http://192.168.14.1 -> title "Mission v4".
2. Start of B, wall on the LEFT, roughly parallel. **Start mission**. Stay behind it.
3. It follows the wall (also across gaps and onto the next wall), turns right at the landmark,
   lines up on the board (step 3 shows what it does), crosses, follows the end board, turns right at the end wall,
   follows the arcade, turns LEFT at the corner, follows the next wall, stops when it ends -> **Finish (save log)**.
4. Send `mis_NNN.bin` + photos/videos like this time (the board / corner moments are the interesting ones).
