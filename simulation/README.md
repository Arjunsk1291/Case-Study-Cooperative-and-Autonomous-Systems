# Strip simulation - "position once, then drive straight" (2026-11-07)

`strip_sim.py` = model + strategies + policy search. `plot_paths.py` -> `paths.png`.
Results: `sim_output.txt`, `sim_results.json`, `sim_good_start.json`, `sim_mount_cal.json`.

## Model (numbers from our measurements)
| part | value | source |
|---|---|---|
| heading hold | Kethan PD, max turn 2.8 deg/s at v 0.20, ~1 s response | log3 (34.4 deg/s per unit wheel difference) |
| crab (slides sideways while pointing straight) | outdoor -6.5 +-1.5 deg, indoor 0 +-1.5 (guess) | logs 026-028 |
| gyro bias drift | 0.02-0.03 deg/s | BNO055 standing tests |
| camera snapshot | detect 90 % indoor / 80 % dark strip / 20 % grooved; 5-10 % false alarms; 3 px / 2 deg noise; scale 150-600 px/m (unknown); mounting angle +-2 deg; 5 % link loss | route video, bench tests |
| pivot (align turn) | +-2 deg | trim runs |
| view | camera sees only a narrow floor band: strip must be within ~10-40 cm sideways | 120 px wide image |

Scenarios: indoor 2 m belt to a wall; outdoor plaza 1 (36 m, dark strip); final straight (24 m, grooved strip).
"On strip" = within 10 cm (indoor) / 50 cm (outdoor) at the end. The rover NEVER stops because of the camera.

## Results (1000 random episodes each)
Good start = strip in view (<= 6 cm off) and < 10 deg off (the page can require "Strip FOUND" before a start).

| strategy | indoor 2 m on strip | plaza 36 m on strip | plaza: reach facade within 2 m | snapshots |
|---|---|---|---|---|
| A gyro only (camera dead) | 36 % | 8 % | 29 % | 0 |
| B align once (1 snapshot) + straight | 61 % | 5 % | 21 % | 1 |
| F align 3 snapshots + verify + outdoor crab prior | 80 % | 15 % | 59 % | 6 |
| BEST (policy search) | 82 % (median 4.9 cm) | 45 % (median 81 cm) | 69 % | 8 indoor / 34 plaza |
| BEST + camera angle calibrated | **88 % (3.8 cm)** | **52 % (39 cm)** | - | same |

BEST policy (cross-entropy method, 12 x 30 policies x 450 episodes): align with 4 snapshots + verify, merge to the
strip over 1.4 m, outdoor crab prior on, check every 2.6 m with 2 snapshots, small corrections (angle gain 0.1,
offset gain 0.4, max 7.8 deg per check), ignore implausible snapshots (angle > 18 deg, offset > 0.7 m),
crab learning 0.35.

Stress tests of BEST (normal start): camera dead -> indoor 15 % / plaza 4 %; link 50 % lost -> 46 % / 19 %;
pixel scale 2.5x wrong -> 51 % / 18 %; camera mounted 8 deg off -> 12 % / 5 %; crab x2 -> 50 % / 15 %;
**camera sign reversed -> 4 % / 1 %** (corrections push the wrong way).

## What this means
1. **Biggest failure = the start**: if the strip is not in view (or a false detection is trusted), the align never
   happens and the rover leaves at its start angle (the steep red lines in paths.png).
   -> start only with "Strip FOUND" in 2 of 3 snapshots; otherwise search (small pivots) or refuse. Never trust 1 snapshot.
2. **Indoors (2 m) "align once + straight" works** (80-88 %). No checks needed: 1 deg error = 3.5 cm over 2 m.
3. **Outdoors (36 m) one alignment is not enough**: 1 deg = 0.63 m, a snapshot is +-2 deg, and the crab (6.5 deg,
   invisible to the gyro) adds metres. Needed: measured crab prior + a few check snapshots (every ~2.5 m, 2 each).
   Even then the strip itself is reached in ~half the runs; the facade (big target) within 2 m in ~70 %.
4. **Calibrate the camera once**: "Set centre" (sideways offset) AND "Set straight" (angle zero) - halves outdoor error.
5. **Verify the direction once** (strip to the right -> page says RIGHT): a reversed sign is the worst failure.
6. **Grooved strip (final straight)**: the camera finds it only ~20 % -> do not rely on it; use the left wall / LiDAR there.
7. Camera on demand: each snapshot reuses the same frame buffer (nothing piles up); idle otherwise.

Limits: the scale (px/m), look distance and indoor crab are guesses until measured (test step 2 of
cas_box_drive/README.md); the outdoor detection rates come from a phone video, not the rover camera.
