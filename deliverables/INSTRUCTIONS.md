# Mission v10 (kit 04, after campus_9): box only, no camera

## What went wrong in campus_9 (v9), checked record by record
1. **Early turn, one pillar before the THD sign.** The run started 7.7 m earlier on B. The left wall was lost at 14.7 m and came back at ~22.5 m, so the sign pillar was at ~28 m. v9 looked for the structure only between 15 and 25 m, found nothing, and its blind fallback (`turnAt + 2.5`) turned at 22.6 m, exactly one pillar early. The turn target was also taken from the rover's own heading, because the wall was not seen at that moment.
2. **No stop at Block I.** At 619.5 s it held for the bollard (LiDAR 0.42 m). Then the bollard came closer than 35 cm: the LiDAR minimum range (daylight ghost filter) drops it, the front ultrasonic read its floor echo (59-68 cm), the front looked free and after 2 s the hold ended and it drove into it. The railing corridor was never recognised (right railing 0.9 m, v9 wanted <= 0.85 m), so it only stopped at the 60 m leg limit. The edge assist pushed twice against the bollard (trigger: a -3.9 deg pitch spike).

## v10 changes
| change | evidence |
|---|---|
| **Turn independent of the start position:** structure window = B-gap end + 5.35 +- 1.2 m (anchor at any distance >= 5 m). Without a gap: 15 m on. **No blind turn any more:** no sign found -> stops and asks (Turn RIGHT here). Turn target = last good B-wall direction - 90 deg and the moment = pillar edge + 0.40 m, exactly as v5 | `tools/replay_turn.py`, `tools/sweep_start.py`: shifting the start by -8...+8 m moves the turn by 0.00-0.13 m (campus2/3/5/6, test1/2, log8) |
| **HOLD release:** an obstacle that vanishes from < 0.45 m without being seen moving away is still there. Release only with >= 4 valid front beams beyond 1 m, or ultrasonic >= 1 m. Ultrasonic stop distance 25 -> 35 cm | campus_9 records replayed with the v10 rule: stays in HOLD from 619.8 s to the end of the log (v9 released after 2 s and drove in) |
| **Edge assist** only when nothing was < 1.2 m ahead for 10 s, pitch change 3-12 deg, aborts when the front distance shrinks while pushing | |
| **Leg 4 end:** railing corridor = right line 0.35-1.3 m + left facade 1.4-2.6 m, total width 2.4-3.3 m; then the **KARL entrance** (facade steps back, glass drops beams) -> stop 1.5 m later. Any HOLD after the corridor ends the route | campus_9 replay: corridor at 35.6 m, entrance at 47.0 m, stop 48.5 m (bollard 56 m). log8: corridor at 40.0 m, nothing earlier |
| **End wall must be WIDE** (>= 1.1 m to each side of the NEAREST surface), only after 85 % of the crossing (was 70 %), leaky counter (+2 good / -1 bad, trigger 6). Fixed a bug found in the sim: the median front distance could land on the far wall behind a person | `tools/sim_open_sea.py --crowd`: 1 person in the path false end wall 18 -> 0 of 20; group of 3 20 -> 0; walking crowd of 20 13 -> 0. All real end walls of the logs still pass |
| People simulation on the turn (`tools/sim_people.py`, 5,670 cases) | within +-0.5 m of the clean turn 93.5 %; wrong pillar 66 cases (v8: 421). Still open: groups of 3-5 at the pillar before the sign (6 % wrong) |
| Weather (`tools/sim_open_sea.py --env`): daylight ghosts, low sun, light rain 12/12 reach the end wall, heavy rain / snow 10/12, no false stop. Worst combined case (snow + slide + drift + sun): 7/12 | |

Kept: turns full power, forward 0.22 (leg 4: 0.30), every stop a gentle ramp, people = stop and wait, only STOP / tilt pause, speedometer and speed override from v9. The rover firmware is not touched.

## Please check outdoors
1. Page line **"turn structure"**: "B gap end ~13-23 m", then LOCKED, the rover turns ~0.6 m past the sign pillar. If the sign is not found it stops and asks (button).
2. Leg 4: walkway -> RAILINGS -> stops at the KARL entrance with "ARRIVED". If anything is in the way it holds and finishes.

## Flash (box only)
Arduino IDE: `deliverables/v10_BOX/cas_box_mission/cas_box_mission.ino` - ESP32 Dev Module, Upload 115200, CPU 240 MHz, Flash Frequency 40 MHz, Flash Mode DIO, Flash Size 4MB, Partition Default 4MB with spiffs, PSRAM Disabled, **Erase All Flash Disabled**. Close the Serial Monitor first.
Or browser (esptool-js, 115200): `BOX_v10_cas_box_mission_0x0.bin` at 0x0, no Erase. Program size 96 %.
