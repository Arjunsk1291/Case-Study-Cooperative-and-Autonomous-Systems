# How the four slide topics really work (read from the code, v10)

Source: `cas_box_mission/RouteMission.cpp` (case `L2_GO` ≈ lines 1071-1190), `LineRef.h`, `DriveStraight.h`.
Rule of this file: it says what the **code does**. Where the slides use a name (store front, poles, tree) that the code does not know, it says so.

---------------------------------------------------------------------------------------------------
## 1. Landmark corridor in the Open Sea (case `L2_GO`)

### 1.1 Short answer to your understanding
Mostly right, with three corrections:
1. **We do not follow the wall behind.** The rover drives away from the B wall. The wall behind is only *looked at* (rear LiDAR sector) during the first ~14 m to check that the heading is still perpendicular to it.
2. **The camera / poles / tree are not known to the code.** The code never identifies a "pole", "tree" or "store". It takes the 90 LiDAR points on the left side, finds the **best straight line** (RANSAC) that is **4.0-9.5 m away and parallel to the path (±8°)**, and measures the distance to that line. Which real object produces the line, we know only from the LiDAR map and the video: in campus_9 it is the long U-shaped line at ~7 m (the store front); the poles and trees stand in front of it (2-4 m from the track) and are *not* used.
3. **PID is not the heading controller.** It is a small outer loop that only produces a *heading offset* (≤ ±8°). The real heading control is the inner `DriveStraight` loop (Kethan's PD heading hold, gyro based). So "cross-track PID that adjusts the heading target" is the correct name; "PID reference" is loose wording.

### 1.2 What the rover is trying to do
Drive ~36-40 m in a straight line along the tactile strip, from the turn at the end of the B facade to the end wall, **using the gyro for the actual steering** and the LiDAR only to correct the gyro's slow drift (gyro ≈ ±2°/min random, GPS was 6-15 m off in log_010).

### 1.3 The layers, from bottom to top
| layer | what | where |
|---|---|---|
| Motors | wheel commands, gentle ramps | `GentleDrive.h`, `RoverLink` |
| Heading hold | `DriveStraight::update(imuOk, yaw, speed, ...)`: PD on (target − yaw), 12° failsafe → "turning back" | `DriveStraight.h` |
| Target `_tgt` | `_tgt = _base − _shiftOff + _latOff + seaTrimDeg` (every 250 ms) | `L2_GO` end of the reference block |
| `_base` | the **path direction**: starts as the heading after the turn = B-wall direction − 90° (perpendicular to the B wall, as v5). Moved by direction references by at most 0.5°/scan (median of 5) | reference block |
| `_latOff` | the **side correction** (degrees), from the cross-track PID while a left line is seen; afterwards only its integral part stays | reference block |
| `_shiftOff` | an optional 10° sideways shift at the start if the rover begins off the strip (`_shiftEndM`), ramped 1°/scan | `_startLeg2` |

### 1.4 The references, in priority order (first one that fits wins, every 250 ms)
Distance `_dist2` = metres driven in the Open Sea (**commanded odometry, no encoders**).
1. **Type 4, left line ("row"), 10-24.5 m.** `LineRef::rowFit(s, 270°, ±70°, 1.5-11 m, tol 0.25 m, ≥5 points, ≥4 m long, point pairs ≥1 m)`. Accepted if the line direction is within 8° of `_base` and its distance `off` is **4.0-9.5 m**. Then:
   `e = off − rowOffsetM (6.65 m)`; `_seaEi += e·0.25` (anti-windup limit 4/ki);
   `eDot = −0.22 · sin(yaw − lineDirection)` (rate of the side error from the heading relative to the line);
   `lat = clamp(10·e + 1.5·∫e + 20·eDot, ±8°)`.
   The line direction is **not** used to turn `_base` (it wobbled ±6° in campus_6) - the line gives side position only.
2. **Type 5, left line, 26-40 m** (fit range ≤ 7 m, distance 2-6 m): `lat = clamp(10·(off − 3.0 m), ±6°)`. Again side only.
3. **Type 1, B wall behind, 2-14 m driven:** `sectorFit(180°, ±50°, 1-12 m)`, accepted if its distance ≥ 2.5 m and ≤ `_dist2`+6 m (so it really is the B wall, not the arcade corner). Perpendicular heading `perpHeading`, accepted if within 12° of `_base`. Moves `_base` ≤ 0.5°/scan.
4. **Type 2, end wall ahead, after 50 %:** `sectorFit(0°, ±40°, 2.5-12 m)`; distance must match what is left of the crossing (remain −6 … +8 m). Moves `_base`, but never more than 8° away from where it was first used.
5. **Type 3, board (<6 m):** weak, ±5°.
6. **Nothing seen:** hold `_base` (gyro only). A lost row leaves no sideways bias (`_latOff` fades at 0.5°/scan; the integral is kept).

So the sequence in the field is: *turn → perpendicular to the B wall (type 1 checks it for 14 m) → at 10 m the left line appears and the cross-track PID holds 6.65 m from it until 24.5 m → the building front takes over 26-40 m → the end wall (type 2) straightens the heading and ends the crossing.*

### 1.5 How the crossing ends
End wall = `_frontM ≤ 1.5 m`, rover level, after 85 % of the expected 36 m, `_frontWallWide()` (flat and ≥ 1.1 m wide each side of the nearest surface) for a leaky counter (+2/−1, trigger 6 = about 0.75-1.5 s). Then a gentle stop and the right turn. People: something within 0.6 m (0.36 m after 85 %) for 0.5 s → HOLD (stop and wait, never swerve).

### 1.6 What changed in v10 in this part, and what did NOT (honest list)
* **Changed (v10, simulated and replayed on the real logs):** the end-wall test (wide, nearest surface, 85 % gate, leaky counter). Reason: people / crowds made false end walls in simulation (18 of 20 runs with one person).
* **Not changed:** the references, the PID (10 / 1.5 / 20, ±8°), the gates, the 6.65 m, the 3.0 m. This is the v7 corridor that crossed in log8 and campus_9 without hold or failsafe.
* **I tried one change and took it back:** a tighter row gate (±1.0, then ±1.5 m around 6.65 m instead of 4.0-9.5 m). In simulation it removed a real weakness (a false straight line such as a bike rack at 4.5 m or a hoarding at 8.5 m pulls the rover 1.06-1.95 m off). On the **real logs** it changed the row use (log8: 72 scans differ, max 1.3°; campus_3: 22 scans; at ±1.0 m it rejected the real row in log8 / campus_3 because the drifted rover saw it at 7.3-8.1 m). Because you do not want the working crossing touched, **the code is back to the original gate** (`git diff 3d23e98 -- RouteMission.cpp` is empty for that line). The finding stays in the slides as "found, not applied".
* **The words "store front", "poles", "tree" are slide labels**, not code. "PID reference" in the slide is the 6.65 m taught distance `rowOffsetM` (median of the campus_6 teaching run, 6.55-6.87 m).

### 1.7 Evidence
* Real logs (`tools/landmark_benchmark.py`, 9 logs): left-line distance 6.5-6.8 m in 5 runs, seen on 39-56 % of the scans, B wall ⟂ within ±1.3° in 6 runs. Real runs: log8 and campus_9 crossed fully autonomously.
* Simulation (`tools/sim_open_sea.py`): V7 vs V6 worst mix 0.41 vs 1.02 m at 24 m; faults, crowds, weather: see slide 2.
* Known weaknesses: gyro ±6°/min with no line → 0.4-0.75 m end error; a −15° heading glitch after 24.5 m → 8 m error (nothing corrects it); the left line is only a *distance* reference (it cannot tell a real line from a bike rack).

---------------------------------------------------------------------------------------------------
## 2. Open Sea simulation (slide 2) - what it is and how it was built
Files: `tools/sim_open_sea.py` (world, scanner, rover model, crowds, weather, faults), `tools/fwmirror.py` class `OpenSea` (a Python copy of the `L2_GO` logic above).
1. **World** (`world()`): line segments in metres: B wall with a gap, a left line of segments every 4 m with stakes (offset 6.65 m), building front at 3.0 m for 28-37 m, end wall at 40 m, random people (0.4 m squares).
2. **Scanner** (`scan()`): 90 rays of 4°, ray-vs-segment intersection, 2 cm noise, 12 % dropouts, real 35 cm minimum range; optional daylight ghosts (20-34 cm), a 40° blind wedge (low sun), random near returns (rain/snow).
3. **Rover** (`run()`): 0.22 m/s, turn rate ≤ 3°/s, P-heading, sideways slide %, gyro bias °/min, heading glitches.
4. **Logic:** `fwmirror.OpenSea.step()` every 0.25 s (same rules as 1.4) and the end-wall rule + people rule (hold 0.6 m, ±16°, 0.5 s).
5. **Metrics:** side error at 24 m and at the end wall, reached the end wall, false end walls, false holds. 20 random starts per case (12 for weather). Run: `python3 sim_open_sea.py --crowd | --env | --faults | --cases`.
Limits: our own world model, not the real campus; commanded odometry; the rover model is simple.

---------------------------------------------------------------------------------------------------
## 3. Last leg to Block I (slide 3) - `_leg4Wall`, `_l4Walk`, `_edgeAssist`
* **Phase 0 wall:** follow the left wall at 1.0 m, power 0.30 (log8: at 0.22 the gearbox stalled at the cobble-to-interlock edge).
* **Phase 1 walkway (centering):** `rowFit(270°, ±40°, 1-3 m, tol 0.10, ≥5 pts, ≥0.8 m)` finds a line on the left (grass edge, benches, strip); parallel within ±12° of `_l4Dir`; first distance seen is kept (clamped 1.3-2.4 m): `want = _l4Dir + clamp(10·(dist − keep), ±6°)`. Limit measured by replay: the line is found on only ~10 % of the walkway scans in campus_9 (35 % in log8); otherwise the heading of the wall is held. A stronger centering (nearest left object) is **not** implemented.
* **Phase 2 railing corridor:** right line (`rowFit(90°, ±40°, 0.3-1.8 m, tol 0.08, ≥4 pts)`) at 0.35-1.3 m + left facade at 1.4-2.6 m, total width 2.4-3.3 m, 4 scans in a row. Then it follows the right railing at 0.5-0.7 m.
* **KARL entrance:** after ≥3 m in the corridor, the left facade steps back and the glass drops beams (≥3 missing and ≥4 valid beams in bins 60-74, 3 scans) → stop 1.5 m later (or 25 m after the corridor start).
* **Hold rule (collision fix):** an obstacle that vanishes from closer than 0.45 m without being seen moving away is still there; release only with ≥4 valid beams > 1 m or ultrasonic ≥ 1 m; ultrasonic stop at 35 cm; any hold after the corridor ends the route.
* **Edge assist (`_edgeAssist`):** pitch 3-12° above its recent minimum for 3 s and nothing < 1.2 m ahead for 10 s → push with climb power 0.40, back up 0.25 m if the rear is free, push 0.45; aborts if the front distance shrinks.
* Evidence: replay of campus_9 → corridor at 35.6 m, entrance 47.0 m, stop 48.5 m (bollard at 56 m); log8 → corridor 40.0 m. **Not driven yet.**

---------------------------------------------------------------------------------------------------
## 4. Daylight ghost filter (slide 4)
* The LD20/LD19 returns false points at 20-34 cm in sunlight (1-8 % of the scans in the logs recorded before the filter).
* **Layer 1** `LidarScan` minimum range 350 mm. **Layer 2** a near obstacle counts only if ≥4 valid beams agree, or the front ultrasonic median is < 50 cm and ≤ LiDAR+15 cm (`_front()`, `_usMed()`). **Layer 3** the front ultrasonic angled 45° down sees the floor at 50-90 cm: masked.
* Datasheets (checked online): LD19 0.02-12 m, 4,500 Hz, 30 klux (LD20 datasheet not found); HC-SR04 2-400 cm, ~15°, ≥60 ms per ping.
* Price: the LiDAR is blind below 35 cm (the bollard in campus_9) → the ultrasonic stop at 35 cm and the hold rule.
* Evidence: replay 27 of 28 ghost events dropped; Open Sea simulation with ghosts in 20 % of scans: 12/12 reach the end wall.
