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
3. **Position first, then the Open Sea (start alignment, after EVERY right turn)** (`_alignStart`, uses the existing `L2_ALIGN` state):
   - **Measure.** The rover stops (ramp) and measures the nearest object on the right, abeam: the THD sign pillar. Median of 5 scans, up to 8 s.
   - **Good turn: nothing.** 0.85–1.75 m (every good run read 0.9–1.6 m): it only logs "aligned" and goes on.
   - **Otherwise: take the time.** Pivot 30° (full power) → roll slowly (max 1.6 m, 0.18) → pivot back to the B-perpendicular heading → measure again, up to 3 rounds / 60 s, target 1.3 m. It follows the pillar by odometry while it moves. Sign to the right too far (late turn) → it moves right; too near → left.
   - **No sign seen in 8 s:** the earlier lateness-based move, otherwise it goes on. Front closer than 1.5 m: it goes on. People: the normal hold rule.
   - **Measuring.** The rover stands still, so a real object is at the same place in every scan. It uses the nearest point that appears within 0.15 m in at least 4 of the last 8 scans. Snow flakes, ghosts and a person walking by do not. (The first version used the plain nearest point and failed in falling snow: 13 of 144 late starts ended worse. The vote fixed that.)
   - **Real logs** (`tools/replay_start_align.py`): the 7 good turns (campus_3, 5, 6, test2, log8, campus_9, c102_4) are untouched; campus_10 (2.08 m), c102_3 (2.94 m) and c102_2 (0.83 m) are moved. test1 (0.95 m) is left alone.
   - **Simulation** (`tools/sim_start_align.py`: pillar, board, a second pole, people, then the unchanged Open Sea): 100 of 100 in-band starts untouched. Late starts +0.7 m / +1.4 m / +2.2 m end 0.06 / 0.19 / 0.18 m (median) off the strip, using 1–3 rounds and 17–46 s. A start 2.8 m late (never seen: the worst log is 1.6 m) ends within 0.5 m in 10 of 16.
   - **Harsh conditions** (falling-snow noise in every scan, 25 % dropouts, 20 % ghosts, pivot error 6°, slip 8 %, a person beside the sign in half of the runs): in-band starts untouched 120 of 120; late starts within 0.5 m 131 of 144; ended worse than before 0 of 144.
   - **Full table (end-wall error, median, late starts +0.7…+1.8 m, v10.2 → v10.3):** clean 0.1–0.15 → 0.01–0.04 m; sparse row 0.3–1.6 → 0.02–0.07 m; gyro −3°/min 1.3–2.4 → 0.2–0.35 m. **Exception, a constant 3 % sideways slide (slanted cobbles) in one direction:** v10.3 ends 0.5–0.8 m, v10.2 0.1–0.5 m, because that slide happens to push a left start back onto the strip; aligned starts get no such help. In that case alignment is not an advantage. Starts inside the band are never moved (100/100 in every table).
   - **The Open Sea logic (`L2_GO`) is unchanged.** A stronger correction (±12°) or an earlier row (from 6 m) did not help in the simulation.
3b. **Start pose: any place, any heading ("Start open sea HERE" button and after every turn)** (`_beginStartPose`, `_wallLine`, orient phases 10/11):
   - **Orient.** The rover stands still and takes 8 scans. The B wall (the cement structure behind the start) is the dominant straight line within 0.6–4 m (≥8 points, ≥2 m long). The sign pillar, the board and the structure ahead-left are too short to be mistaken for it. It then pivots in place (full power) to face **away** from the wall (max 2 pivots), then measures again.
   - **After the autonomous turn** it only checks: heading within 8° of "wall exactly behind" → nothing; 8–25° off → corrects; more → ignores it (the wall check never overrules a good turn). On the real post-turn scans of all 11 logs the wall was found every time and the heading was within 7°, so no good turn would be moved.
   - **From the button (placed anywhere, facing the wall, left, right or front):** it turns to the wall-normal heading, which becomes the Open Sea heading, then looks for the sign pillar in a wider window (up to 2.6 m ahead or behind, everything at or behind the wall excluded), then the alignment from 3b above (up to 4 rounds, 100 s).
   - **No hold during orient.** The rover only turns in place there, so a wall closer than 0.45 m does not trigger a hold (people hold works again once it drives).
   - **Simulation** (`tools/sim_start_pose.py`: pose x −1…+2 m, 1–3 m from the wall, any heading 0–360°, B wall ending 3.5 m right of the strip, pillar + board, a second pole, the structure ahead-left, people): 260 of 300 end with heading within 8°, lateral offset within 0.5 m and no contact. Heading after orient: median 2.3°, 90 % within 5.0°. Lateral offset: median 0.17 m. Time 26 s median.
   - **Harsh** (snow noise, 25 % dropouts, pivot error 6°, slip 8 %): 231 of 300; heading 90 % within 7.1°.
   - **Where it fails (honest):** mainly starts 2.5 m or more from the pillar with a person standing near it. A standing person looks exactly like the pillar to the 2D LiDAR, so the rover can line up on the person. Those placements: 6 of 15 succeed. A few contacts (4 of 300) with the pillar or the wall. **Place the rover in the 3×3 block area (pillar within about 2 m), keep people away from the pillar during the start.**
   - **Wall end as a reference:** not used. The B wall ends 2.3–4.2 m right of the strip in the logs, too variable; the sign pillar (0.9–1.6 m) is the lateral reference.
   - **Pillar counting and the Open Sea landmarks are unchanged** (structure first; row, building, end wall as in v10.2).
4. **End-wall turn:** with the front ≤1.0 m, SHIFT goes **backwards** 0.3 m at 0.20, but only if nothing is within 0.8 m behind. That gives SWING and ARC room again.
5. The phone page now says "Mission v10.3".

Compile: 1,267,776 B (96 %), no new warnings.

## NEW ROVER (number 7): not driven yet
- **Drift is learned automatically** (`DriveStraight.h`). The heading hold moves its steady correction into a learned wheel trim. Now: the first 40 s of driving learn fast (time constant 6 s, then 15 s), and the limit is 0.40 (was 0.20). `tools/sim_drive_trim.py` (the kit-04 calibration model, 5 seeds each): natural drift of ±3°/s: heading error under 1° (0.20 limit: fine); ±4°/s: the old limit failed (5 of 5 at −4°/s, 3 of 5 at +4°/s), the new one is fine at +4°/s and fails 2 of 5 at −4°/s; ±5°/s and more: fails both. Beyond that the rover pauses with "correction saturated" instead of driving off.
- **The fixed starting trim (−0.046) is from kit 04.** It is only a starting point; the learning removes any difference within about 40 s of driving, i.e. during leg 1.
- **Not corrected automatically (be aware):** the distance. The box counts commanded speed × time, not wheel turns. If rover 7 is faster or slower than rover 1–6 at the same command, the distance windows (turn window, Open Sea row 10–24.5 m, end wall at 85 % of 36 m) shift. The landmarks (sign structure, store-front row, end wall) are found by LiDAR, so a few percent is absorbed, but a large difference is not.
- **First test on rover 7, what to look at and send me:** the page shows "trim learned" under the heading line: write down its value at the end of leg 1 and at the end of the Open Sea. Put a tape measure on the floor for the first 10 m of leg 1 and tell me how far it really went. Send the `mis_*.bin` and the video as usual. With that I can fit the trim and the distance scale to this rover (and compare with rover 1–6).


## Field test checklist
- Start ~1.0 m from the B wall, parallel to it.
- Page messages to watch:
  - `turn structure locked`;
  - after the turn: `sign X m right: aligned - going on` (good turn) or `sign X m off the target: moving ... (round N)`;
  - in leg 4, `ramp opening … ahead, centre … left/right`, then `inside the ramp opening - straight on`.
- If it shows `step line ahead, not lined up … waiting`: wait 20 s, or take over, line it up and press GO.
- Avoid manual take-overs in the Open Sea, because each one resets the heading base.
- Send the new `mis_*.bin` plus the videos.

## The log carries everything for rover 7 (log version 5)
Every 250 ms record now also has: **learned wheel trim, heading-hold correction, yaw rate, crab estimate, speed boost, seconds driven** (decoder: `trim_learned`, `hold_corr`, `yaw_rate`, `crab_est`, `boost`, `drive_s`). Together with what was already there: the 90-beam LiDAR, 5 ultrasonics, gyro yaw / pitch / roll, GPS, battery voltage, the pad command and the real wheel output (`cmd_*`, `out_*`, the manual flag), the state, the references, the end-wall / structure / alignment / ramp-opening events. Old v4 logs still decode.
- **How to drive the test:** start autonomously, drive by hand whenever you want (the log keeps running in MANUAL, the pad command is recorded), give it back when you like. Press the log's **MARK** button at interesting moments (a turn, a bump, a person).
- **Start the video before the log** and say "go" aloud (or show the phone clock) so I can sync them.
- **Storage:** about 1 kB per second. Delete old logs on the page if it says "flash full".
- **Distance:** please also note with a tape measure the real length of one stretch you drove straight (for example the first 10 m of leg 1), because the box has no wheel encoders.
