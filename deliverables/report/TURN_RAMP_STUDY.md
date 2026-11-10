# Smooth curve turns, gentle ramp climb, speed + acceleration: simulation study (nothing implemented)

*Rover 7, v11. Code: `cas_box_mission/tools/sim_turn_curve.py`, `sim_ramp_climb.py`, `make_study_videos.py`. Videos: `~/Downloads/v11_study_videos/` (not on GitHub).*
**Status: simulation and insights only. No firmware file was changed.**

## 1. The rover model and how it was calibrated
- **Command scale:** command ≈ m/s at cruise (the commanded odometry matches the landmarks within about ±5 % over 40 m).
- **Yaw:** yaw rate = 0.75 rad/s per unit of (right − left wheel command), i.e. 43°/s for a full pivot. Both wheels below the motor deadband (0.08) do not turn.
- **Ground:** the wheels lock, not the motors (code comment, 400 teaching pivots). A lock is drawn once per motion attempt and lasts until the motion type changes (pivot → swing → shift, as the box does). Lock probability pivot / swing / arc: tile 0.15 / 0.2 / 0.02, gravel 0.4 / 0.4 / 0.05, cobble 0.65 / 0.55 / 0.06. A gentler wheel ramp locks more often (teaching logs: 13 % worked with a slow ramp, 36 % with a quick full-power step).
- **Calibration against the logs (6 logs, 250 ms records):**

| Motion | Simulation | Logs |
|---|---|---|
| Pivot on tiles | 43°/s median, 12 % stalled | 35–43°/s, 4–30 % stalled |
| Pivot on cobbles | 0°/s median, 72 % stalled | 0°/s median, 66–76 % stalled |
| Swing on cobbles | 0°/s median, 60 % stalled | 0–7°/s, 29–81 % stalled |
| Rolling arc on cobbles | 15°/s, 2 % stalled | 5.8–16°/s, 5 % stalled |

- **Gearbox limits (no measured value exists, conservative):** wheel-command change ≤ 1.0 per s (today's pivot step: 5.0 per s), reversal only through 0 with a ramp, stall at command ≥ 0.4 for ≤ 2 s. These rank the options; they are not guarantees.
- **Sensitivity:** yaw gain ±20 % and deadband 0.10 do not change any conclusion (curve at v 0.30 / R 1.0: 100 % on cobble in every case, 0–1 of 20 fall back to the pivot).

## 2. Turn results (real LiDAR maps of the sites, 6 seeds, footprint 0.30 × 0.26 m + 0.15 m margin)
| Option | T1 (sign pillar) tile / gravel / cobble | End wall (1.3 m ahead) | T3 left turn | Time | Peak wheel change |
|---|---|---|---|---|---|
| **Pivot today** (step 5.0/s, tactics) | 100 / 100 / 100 % | 100 / 100 / 100 % | 67 % (map limited) | 4.4 s | **5.0 /s** |
| Pivot, wheel change ≤ 1.0 /s, power 0.35 | 100 / 100 / 75 % | 100 / 100 / 75 % | 67 / 67 / 50 % | 5.7 s | **1.0 /s** |
| Curve R 1.0, v 0.30 (inner wheel 0.10) | 100 % all grounds | does not fit | 67 % (= pivot) | 7.3 s | **1.0 /s** |
| Curve R 1.2, v 0.22 | 100 % all | 0 % (needs about 1.7 m forward) | 67 % | 10.6 s | 1.0 /s |
| Curve R 0.7, v 0.30 | 100 % | 62–75 % | 67 % | 6.8 s | 1.0 /s |

**Insights**
1. **A smooth curve works where there is forward room, and it removes the gearbox shock:** peak wheel change 1.0/s instead of 5.0/s, no reversal at all, and no locked-wheel stall on cobbles (the wheels keep rolling), at the cost of 2–6 s more and 1.3–2 m of forward travel.
2. **The inner wheel must stay at or above 0.10.** Below that it falls into the deadband, the curve becomes a swing and locks as often as a swing does (29–81 % in the logs). Rule: v·(1 − 1/(1.5·R)) ≥ 0.10. For R = 1.0 that needs v ≥ 0.30, for R = 1.2 v ≥ 0.22, for R = 2.0 v ≥ 0.15.
3. **T1 (first right turn):** feasible with R 1.0–1.2 (the pillar side has room).
4. **End wall (front wall 1.3 m):** no both-wheels-rolling curve fits; keep the pivot there. The only gearbox gain is a gentler wheel ramp (peak 1.0/s), which costs 1.3 s on tiles and, in the model, cobble success 100 → 75 % (the slow-ramp lock effect).
5. **T3:** the sim result equals the pivot (67 %: one real map is tight even for a pivot with this margin); a curve there is not more constrained than the pivot.
6. **Wheel ramp limit sweep (T1, 30 seeds):** pivot soft with limit 2.0 / 1.0 /s: 100 % on tile and gravel (+1.4 s), 93 / 90 % on cobble; limit **0.5 /s fails** (17 % tile): too slow to turn.
7. **Do not trade the full-power step away on cobbles without testing:** in the model the quick step is what wins on bad ground. The curve avoids the problem instead of weakening the pivot.

**Recommendation (to try, not implemented):** (a) at T1 try the curve R 1.0 / v 0.30, with the pivot as the fallback when it makes no progress for 2 s (the same tactic cycle as today); (b) keep the pivot at the end wall and the start; (c) lower the pivot's peak wheel change from 5.0 to 2.0/s only after a field test (2.0/s was within 3 % of today's success in the model).

## 3. Ramp: gradual speed increase only if needed
Model: steady speed on a slope v = min(command, 3·(command − c_req)), c_req = 0.08 + 1.84·sin(pitch) + surface (wet +0.05, cobble edge +0.10), battery 1.0 / 0.85, motor lag 0.5 s; the speed estimate comes from LiDAR range-rate only (70 % of the time, noise 0.03 m/s).
- **Today:** 0.30, then 0.35 once the ramp is seen, edge push 0.40 for 1.5 s (3 times), then 0.45, then a 30 s wait.
- **Gradual:** 0.30; slow/stuck = pitch ≥ 2.5° and the estimated speed < 40 % of the command for 1.5 s → +0.02 per second up to 0.45, hold when it moves at ≥ 70 % of the command, back to 0.30 on the level. No push, no jump.

| Ramp pitch | Climbed, today | Climbed, gradual |
|---|---|---|
| 3° | 100 % | 100 % |
| 5° | 50 % | 100 % |
| 7° | 17 % | 83 % |
| 9° | 0 % | 50 % |
| 11° | 0 % | 17 % |
| 13° | 0 % | 0 % |

(mean over dry / wet / cobble edge and battery 100 / 85 %, 20 seeds each, total 28 % → 58 % of 720 runs). A slow climb is accepted: for example 7°, wet, battery 85 %: 0.08 m/s, 55 s, climbs (video 5); today's rule stays stuck.
**Limits:** the model has no momentum for short edges (campus_9 climbed 13.8° edges at 0.34); above about 11° nothing climbs at the 0.45 cap. The rover must never be asked for more than the cap or for a jump in command (gearbox rule).

## 4. Speed and acceleration in the UI (mock-up only: `ui_speed_acceleration_mockup.png`)
- **Acceleration:** the BNO055 runs in IMUPLUS mode (gyro + accelerometer); `ImuManager` reads only orientation today. A linear-acceleration read at about 20 Hz would be added (small, but a new I2C transfer: it must not disturb the checked reads).
- **Velocity:** no encoders. Command × LiDAR range-rate (works only with a wall ahead or behind), with the accelerometer integrated between fixes and reset at every stop.
- **Log:** two more fields (log version 6); the page card shows both numbers and two traces.

## 5. Videos (`~/Downloads/v11_study_videos/`)
1. `1_T1_sign_pillar_turn.mp4`: today's pivot (a cobble run that stalls, so it shows the tactic cycle) against the curve R 1.0 / v 0.30.
2. `2_end_wall_turn.mp4`: today's pivot against the gentler pivot (no curve fits).
3. `3_T3_left_turn.mp4`: pivot against the curve.
4. `4_speed_comparison_T1.mp4`: four speed / radius pairs with the inner wheel at 0.10 or more.
5. `5_ramp_climb_today_vs_gradual.mp4`: 7°, wet, battery 85 %.
Each shows the real LiDAR map of the site, the footprint and trail, both wheel commands, velocity and acceleration.

## 6. Honest limits
No measured gearbox limit and no encoders: the model is fitted to yaw rates from logs, so the stress numbers are rankings, not guarantees. The pivot shown in the videos is a stalled cobble example. The real test on rover 7 decides.
