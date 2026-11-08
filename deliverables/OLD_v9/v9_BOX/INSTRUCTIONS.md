# Mission v9 (kit 04, after log8): box only, no camera

## Why the turn failed in log8 (checked record by record)
- The rover turned right at **18.2 m, one pillar too early**. In video 1 at 77–80 s there are still pillars ahead, and the dark structure is one pillar further on.
- Cause: the v5c turn landmark looks right-behind (90–162°) for a cluster that is ≥ 0.45 m wide.
  - From 15.9 m on, a **person walked 1.2–2.2 m behind the rover, 0.6–1.0 m to its right**, moving with it (beams 142–162°). That was most likely the person filming.
  - That person merged with an ordinary, thin pillar, so the pair looked wide → "turn pillar".
- The pillar code itself was unchanged in v8. **People beside or behind a pillar can fake the turn landmark.**

## v9 changes
| change | evidence / replay |
|---|---|
| **The turn is decided by the STRUCTURE beside the turn pillar.** It is a face running from the pillar line to ≥ 2.3 m right; ordinary pillars have nothing behind them. It is seen 4–4.5 m ahead. Only beams ahead/abeam are used (2–134°), so a person behind is never in it. It must be seen ≥ 3 times from positions ≥ 0.8 m apart, so a person walking along with the rover never counts. | Found in all 7 runs (campus2/3/5/6, test1, test2, log8). |
| **Window from the B gap:** where the left wall comes back after the gap in the B facade, the structure follows 5.2–5.5 m later in every run (log8 15.7→21.2, test2 14.7→20.0, campus6 12.9→18.1, campus5 13.7→19.2, test1 15.2→20.5). Without that anchor the window is 15–25 m. | The pillar before is outside the window. |
| Turn **0.6 m past the structure**. If the live pillar edge agrees with the structure (±0.8 m), the exact v8 timing is used (pillar edge + 0.40). | Replay (`tools/replay_turn.py`): campus6 18.7 (v8 18.51), test2 20.6 (20.66), test1 21.2 (21.49), campus5 19.8 (19.82); **log8 21.8 instead of 18.2**. |
| No structure (hidden): the old pillar landmark only from 20 m on, without the behind-right beams (follower); fallback after the window. | |
| **People simulation** (`tools/sim_people.py`, people inserted into the real scans of the logs, standing / groups / walking along / follower / crowd of 8). | Before the gap anchor: wrong-pillar turns 36 of 1530 cases (v8: 145). With the anchor: **not simulated yet** – that is the people simulation still to run after this test. |
| **Open-sea end wall must be WIDE** (≥ 1.1 m to each side; real end walls are flat over ±48°). | Crowd simulation: 1 person standing in the path → false end wall 19/20 → 1/20; group of 3 → 20/20 → 0/20; walking crowd → 4/20 → 0/20. All real end walls in the logs still accepted, the test1 false one is rejected. Not covered: 8 people shoulder to shoulder across the path. |
| **Leg 4 to Block I.** After the wall: interlock walkway (heading of the wall, left edge if seen) → **railing corridor** (right railing at 0.35–0.85 m and something left at 1.6–2.9 m for 1 s → keep that distance to the right railing, 0.5–0.7 m) → **ramp** (pitch +3° for 1 s, power ≥ 0.35) → top (level 1.5 s) → **stop at the door / wall** (or after 6 m). | Railing detector on log8: from 641 s, never before on the walkway. |
| **Leg 4 power 0.30** (Settings). | log8: at 0.22 the rover stood **60 s** at the cobble→interlock edge (LiDAR: front 3.8–4.1 m all the time, pitch +3→+6°). Its "16 m" were commanded, not driven. |
| **Edge assist** (leg 4): pitch stays ≥ 3° above its recent minimum for 3 s while driving → climb push 0.40 for 1.5 s (3×) → back up 0.25 m at 0.20 (only if nothing is behind within 0.6 m; never on the ramp) → push 0.45 for 2 s → still stuck: waits 30 s ("please help"), then tries again. All with wheel ramps. | General "not moving" detection from the LiDAR was tested on all logs and is **not reliable** (walls parallel to the path do not change range; people following). So this is only the pitch rule. |
| **Speedometer + speed override** (− slower / normal / + faster, ±0.02 per press, −0.10…+0.15). It applies to forward driving in every leg; turns stay full power. The automatic speed-up is removed: it stays slow while following a wall. | The gauge shows the wheel command in m/s (the rover has no wheel encoders). |

Kept: obstacle stops are decelerated ramps (GentleDrive, never a jump to 0), people in front = stop and wait (never swerve), only STOP and tilt need Resume, LiDAR ghost check from v8.1, the open-sea corridor, leg 3 at 0.6 m.

## Outdoor test now (no people around)
1. Flash v9 (below). Start as usual.
2. Watch the page line **"turn structure"**. It should show "B gap end ~13–16 m", then **LOCKED at ~18–22 m**, and the rover turns ~0.6 m after it, **beside the structure**. If it turns anywhere else, press STOP and send the log.
3. Leg 4: watch "leg 4: walkway → RAILINGS → RAMP → top". If it hangs at an edge, the edge assist pushes (you can still use CLIMB). Use **− / +** if it is too slow or too fast.
4. Send the log (and videos). After that I run the people simulation with the anchor version.

## Flash (box only)
Arduino IDE: `deliverables/v9_BOX/cas_box_mission/cas_box_mission.ino`. Settings: ESP32 Dev Module, Upload 115200, CPU 240 MHz, Flash Frequency 40 MHz, Flash Mode DIO, Flash Size 4MB, Partition Default 4MB with spiffs, PSRAM Disabled, **Erase All Flash Disabled**. Close the Serial Monitor first.

Or in the browser (espressif.github.io/esptool-js, 115200): `v9_BOX/BOX_v9_cas_box_mission_0x0.bin` at 0x0, no Erase. Program size: 95 %.
