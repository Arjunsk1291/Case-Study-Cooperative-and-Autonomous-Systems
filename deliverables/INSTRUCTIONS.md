# Mission v10.2 (after campus_10, 2026-11-09): box only, no camera

## What went wrong in campus_10 (v10), checked record by record
1. **Turn too early.** The structure (THD sign) was locked correctly at 24.7 m (B gap end 18.9 m + 5.8 m). But the pillar edge was seen more than 0.8 m after the lock, so v10 used its fallback timing "lock + 0.6 m" and turned at 25.3 m **with the pillar still 0.3-0.6 m ahead, 1.0 m to the right** (the rover drove 1.5 m from the wall, v6 drove 1.0 m). After the turn the pillar was 0.8 m in front of it.
   In the v6 success run (campus_6) the pillar was **0.9-1.25 m behind** when the turn started, in test2 0.4-0.8 m behind.
2. **Open Sea.** It was not really a crossing.
   - For the first 3 m of wheel command the rover stood in front of the pillar: front 0.80 m, nose up 3.75°.
   - At 137 s it tilted to 12° roll and both side ultrasonics read 2-4 cm (it was lifted or handled).
   - After the manual takeovers, the B wall behind stayed at the same 1.5 m for 7 m of commanded driving: the rover was not moving forward.
   - Each manual takeover also resets the Open Sea heading to where the rover was left (62.6 -> 64.1 -> 66.6 -> 65.4 deg). This is the same as v6.
   - The Open Sea logic itself is unchanged since v7.

## v10.2 changes (turn only)
| change | evidence (replay, `tools/replay_turn.py`, `sweep_start.py`) |
|---|---|
| **v6 timing enforced:** no turn while anything is 0.3-2.0 m to the right between 0.40 m behind and 1.5 m ahead of the rover (`_rightAhead`) | every good turn (campus2/3/5/6, test1, test2) had nothing there; every bad one (campus_9 one pillar early, campus_10 too early, log8 early) had a pillar there |
| pillar edge accepted up to 2.0 m after the lock (was 0.8 m) | campus_10 |
| no sign sightings while the left wall is lost (inside the B gap) | campus_10 started 4 m earlier would have locked inside the gap at 13.8 m; now 25.4 m for every start shift |
| locked but the pillar never passed 3.5 m after the lock -> stop and ask | safety |
Unchanged: Open Sea, end wall, people/hold rules, leg 3, leg 4.

## Please check outdoors
- Start parallel to the B wall, **about 1.0 m from it** (v6 distance), at the usual start.
- The page should show `turn structure: LOCKED`. The turn must start only when the sign pillar is about 0.5-1 m behind the rover.
- If someone stands at the pillar on the right, it waits and then asks (Turn RIGHT button).

## Flash
Arduino IDE settings:
- Board: ESP32 Dev Module
- Upload speed: 115200
- CPU frequency: 240 MHz
- Flash frequency: 40 MHz
- Flash mode: DIO
- Flash size: 4MB
- Partition scheme: Default 4MB with spiffs
- PSRAM: Disabled
- **Erase All Flash: Disabled**

Or esptool-js: `BOX_v10_2_cas_box_mission_0x0.bin` at 0x0, no erase. Program size 96 %.
