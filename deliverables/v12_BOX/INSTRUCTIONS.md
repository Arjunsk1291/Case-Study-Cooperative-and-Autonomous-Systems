# BOX firmware v12: fixes from the v11 run (campus_11, 10 Nov 2026)

Flash `BOX_v12_cas_box_mission_0x0.bin` at **0x0**, 115200 baud, DIO / 40 MHz / 4 MB. **Never "Erase All Flash".**
Check first that the port is the box: the serial output shows `[imu]` and `[rover] searching`. The rover firmware is untouched.

```
esptool --chip esp32 --port /dev/ttyUSB0 --baud 115200 write-flash -z --flash-mode dio --flash-freq 40m --flash-size 4MB 0x0 BOX_v12_cas_box_mission_0x0.bin
```
The page title reads **Mission v12**. The sketch is 1,269,648 bytes (96 %). The source is in `cas_box_mission/`.

## What the v11 run showed (`Log_run/campus_11/mis_001 (6).bin` + `test_run.MOV`; video time = log time − 23 s)
| Where | Log | Video | Cause |
|---|---|---|---|
| **End of the Open Sea** (371–381 s) | The target jumps from −98° to +102…+174° (13 jumps of 15–160°). The yaw turns 18° right. | The rover is on the cobbles 1–1.5 m left of the tactile strip, then crosses onto the tiles just before the end wall. | **A v11 bug.** The end-board offset was subtracted on every 20 ms control tick, but the target is rebuilt only every 250 ms, so the offset was applied about 12 times. The sims ran once per scan and could not see it (`figures/v12_end_board_bug.png`). |
| End board | It was seen at 1.74 m (target 1.2). | The rover was wide, on the cobbles. | Too far left already during the crossing. The row read 7.2 m. |
| **Leg 4** (570–572 s) | At 29.1 m a "ramp opening" was confirmed **3.4–3.6 m to the right**. The target went −81° → −131°, then a realign pivot (the "90° turn"), then STOP. | The rover drives on the cobbles about 1.5 m left of the strip, then pivots right. **Beyond the drain grate** lies the real corridor: the facade on the left, a **concrete parapet with a railing** on the right. The "opening" was **beyond the parapet** (the steps). | The opening finder tracked at close range and jumped to the steps. The real opening was almost centred (+0.3 m). The corridor (parapet 1.1–1.3 m right, facade 1.7–1.8 m left) was in the scans from 16 m, but the rule needed 4 consecutive scans. |
| Not changed | The T1 start alignment took 42 s (3 pivot rounds). The T3 turn stalled for 21 s. | Cobbles. | Outside this change. |

Video frames synced to the log: `figures/v12_video_frames.jpg`. Leg-4 landmarks in 6 logs: `figures/v12_leg4_landmarks.png`.

## Changes in v12
1. **End-board bug fixed:** the offset is now applied **once**, in the 250 ms target refresh. The end-wall turn still uses `target + offset − 90°`, so leg 3 keeps its heading. All other `_tgt` updates were checked for the same pattern.
2. **Open Sea, more distance from the poles and trees, and closer to the end board** (all are page settings, so they can be trimmed on site):
   - Row target `rowoff`: 6.65 → **7.3 m** (building 3.65 m).
   - End board `ebt`: 1.2 → **0.9 m**.
   - End-wall stop: 1.5 → **1.8 m** from the wall.
3. **Leg 4: landmark steering to the KARL glass.**
   - **Walkway:** keep the left line (benches, planter, building base) at **2.1 m**. This value was taught from c102_3, which reached I. Before, it kept the first distance it saw. The 0.9 m clearance and the median walkway heading from v11 stay.
   - **Corridor:** it is accepted when the right parapet line is 0.35–1.5 m away and the left side is open or the facade, on **3 of the last 5 scans** (was 4 in a row). From then on the rover centres on it.
   - **Ramp-opening finder, now only a guard:**
     - It only counts an opening 2.2–4.0 m wide, with its centre between −2.5 m and +1.5 m.
     - It rejects any opening on the far side of the parapet line (the steps).
     - It does not let the tracked opening jump by more than 0.6 m.
     - It steers at most 12° right or 25° left and moves the target by at most 2° per scan.
     - **It never makes a realign pivot in leg 4.**
4. **Arrival at the KARL glass (Block I):** the rover stops with a gentle ramp. The page asks: *"ARRIVED at the KARL glass (Block I). Drive MANUALLY to H now – the log keeps recording (Finish saves it)."* It shows the buttons **Manual** and **Finish**. There is no automatic turn. Drive to H in Manual mode, then press **Finish** to close the log.

## Simulation and replay (before flashing)
| Test | v11 | v12 |
|---|---|---|
| Loop timing, 20 ms ticks, board at 1.74 m (`sim_loop_timing.py`) | target jump 22°, heading error 42° | jump 2° (3° at 40 ms ticks), error 1.5° |
| campus_11 leg 4, stateful replay (`replay_l4gap_state.py`) | jumps to the opening 3.58 m right, −50° | stays on the real opening (centre 0.41 m), at most −12° |
| Ramp entry, closed loop (`sim_l4_gap.py`) | – | PASS for starts −1.5…+2.5 m off. STOP (waits) at −2.0 m. 0 hits. People: stop, then pass. |
| Walkway, keep 2.1 m, heading bias ±12° (`sim_l4_walk.py`) | 0 mud, min clearance 0.52 m | 0 mud, min 0.52 m, median clearance 1.87 m |
| Open Sea end (`sim_end_board.py`) | – | ends 0.5–0.75 m further right (toward the strip) |
| End wall at 1.8 m (`replay_end_wall.py`) | – | triggers 0.2–0.4 m earlier; now also catches campus_3 and campus_6 test2 (missed at 1.5 m); no false triggers (campus_2, campus_6 test1 stay None) |
| Regression: first turn, sea gate, start align, start leg 1, end board | – | unchanged on all logs |

The landmark table is in `tools/landmark_leg4.py` → `landmark_leg4_db.json` (6 logs, measured from the end of the left wall). The parapet appears at 16–18 m in campus_11, c102_2 and c102_3, and at 20 m in campus_9. The left line is 1.6–2.3 m in the good runs.

## Field checklist
- The Open Sea should end on the **tiles** near the tactile strip, not on the cobbles. If it is still left, raise `rowoff` by 0.2 m. If it comes too close to the strip, lower it.
- Leg 4: no pivot after the grate. The rover enters between the facade and the parapet, then stops at the KARL glass and shows the prompt.
- Please film again (landscape and upright if possible) and note the tape distance of one straight stretch.

## Honest limits
- The steps right of the parapet are below the LiDAR plane. Rejecting them is a geometric rule tied to the parapet line, not a sensor that sees the steps.
- The landmark distances come from 6 runs and one video. Odometry is commanded (no encoders), so distances along the route vary per rover by a few metres.
- The cobble pivot stalls (T1 alignment, T3) are not changed.
- A rover more than about 1.5 m off at the end of the Open Sea cannot be fully corrected in the last 3 m.
