# BOX firmware v10.3: fixes from the campus_10.2 runs (9 Nov 2026)

Flash `BOX_v10_3_cas_box_mission_0x0.bin` at **0x0**, 115200 baud, DIO / 40 MHz / 4 MB. **Never "Erase All Flash".**
Check first that the port is the box: the serial output shows `[imu]` and `[rover] searching`. The rover firmware is untouched.

```
esptool --chip esp32 --port /dev/ttyUSB0 --baud 115200 write-flash -z --flash-mode dio --flash-freq 40m --flash-size 4MB 0x0 BOX_v10_3_cas_box_mission_0x0.bin
```

## What the 3 runs showed (logs `Log_run/campus_10.2/`)
| Run | Result | Cause found in the log |
|---|---|---|
| 2 | reached Block I | Started 1.6 m from the B wall. Turned 1.9 m after the sign (manual help). Pushed on the cobblestones. |
| 3 | reached Block I (KARL stop 43.2 m) | Turned 0.9 m after the sign, so the store row read 7.6 m instead of 6.65 m (1 m off). |
| 4 | **drove onto the stairs** at 35 m in leg 4 | Arrived 1.9 m right of the ramp opening. The steps are below the LiDAR. The sign was not locked (4-cell face, 6.9 m after the B gap), so it asked at the turn. |

The end-wall turn stalled for 35–42 s in runs 3 and 4. Only the pivot was ever tried: the front was 0.7–0.9 m away, so swing, arc and shift were all skipped.

## Changes
1. **Leg 4 ramp opening** (`_l4FindGap` / `_l4Gap`):
   - **What it looks for.** A cross line (step edge, lights) with a 2.2–4 m opening that has the KARL facade (or the railing) running on from its edge. It takes the leftmost one, because the stairs look open too.
   - **When it is off by more than 0.3 m:** the rover aims 1 m before the opening, with a ramped stop and a full-power pivot if the angle is over 18°. When it is lined up, it goes straight through.
   - **When the line is ≤1.6 m away and the rover is not inside the opening:** it **waits**, never enters. After 20 s it asks you.
   - **Results:** replay on c102_2/3/4 and campus_9 gave 0 false openings, and run 4's opening was found at −1.8 m. Simulation (`tools/sim_l4_gap.py`): 180 approaches, start −2…+2.5 m, ±5°, step lights, people; 0 hits on the steps.
2. **First turn** (structure first):
   - 4-cell sign face (was 5).
   - B-gap window ±2.0 m (was ±1.2).
   - New hard failsafe: **no turn earlier than 4.5 m after the B gap (left open space) ends**. All 12 logs: the sign is 5.2–6.9 m after it.
   - Replay `tools/replay_turn.py`: 11 old logs unchanged, and run 4 now turns on the sign at 26.8 m. People sim: 5,670 cases, 93.3 % on target, 0 wrong pillars, fewer late turns (198 vs 224).
3. **Position first, then the Open Sea** (`_afterTurn` + the existing `L2_ALIGN` sideways move):
   - **Lateness.** Measured as turn − sign lock − 0.55 m. Good runs: −0.14…+0.11. test1 (failed in the Open Sea): +0.44. c102_3: +0.37. c102_2: +1.34.
   - **Move.** If it is >0.15 m, the rover stops, pivots 30° right at full power, rolls 2 × lateness at 0.18 (max 1.2 m sideways), pivots back to B-perpendicular, and only then starts the Open Sea. This replaces the blind 10° shift.
   - **Why.** In the simulation with realistic row visibility (seen on only 15–25 % of scans, as in the logs) plus slip or gyro drift, a centred start never ended >1 m off (0/20). Starts 0.5–1.5 m off ended >1 m off in 4–8 of 20. Lined up within ±0.25 m: 1–2 of 20.
   - **Open Sea logic (`L2_GO`) is unchanged.** A stronger correction (±12°) or an earlier row (from 6 m) did not help in the simulation.
4. **End-wall turn:** with the front ≤1.0 m, SHIFT goes **backwards** 0.3 m at 0.20, but only if nothing is within 0.8 m behind. That gives SWING and ARC room again.
5. The phone page now says "Mission v10.3".

Compile: 1,262,352 B (96 %), no new warnings.

## Field test checklist
- Start ~1.0 m from the B wall, parallel to it.
- Page messages to watch:
  - `turn structure locked`;
  - after the turn, possibly `turned X m late: moving X m right before the open sea`, followed by `lined up after the late turn`;
  - in leg 4, `ramp opening … ahead, centre … left/right`, then `inside the ramp opening - straight on`.
- If it shows `step line ahead, not lined up … waiting`: wait 20 s, or take over, line it up and press GO.
- Avoid manual take-overs in the Open Sea, because each one resets the heading base.
- Send the new `mis_*.bin` plus the videos.
