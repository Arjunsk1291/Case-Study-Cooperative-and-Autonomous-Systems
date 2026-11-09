# Obstacle detection and a 20 cm stop: what we have, the limits, what we could do

Status: **research only. Nothing in the firmware was changed for this.** The numbers are from the real logs (13 holds while driving, all logs in `Log_run/`, decoded with `tools/decode_mission_log.py`).

## What the box does today
| Item | Value | Where |
|---|---|---|
| People rule: something in front closer than | 0.60 m for 0.5 s → stop and wait (never swerve) | `holdM`, `_near()` |
| Emergency: front closer than | 0.45 m → harder ramp | `_frontM < 0.45` |
| Ultrasonic stop | 35 cm | `_front()` |
| Stop ramps | normal 0.30 per second (0.25 → 0 in about 0.8 s), emergency 0.80 per second (about 0.3 s). Always a ramp, never a jump | `GentleDrive` |
| LiDAR minimum range | 35 cm; a single near point is only believed if 4 beams or the ultrasonic confirm it (daylight ghost filter) | `LidarScan.h`, `_front()` |
| After a stop | released when ≥4 beams see more than 1 m or the ultrasonic more than 1 m for 2 s | case `HOLD` |

So detection from far away already exists: the LiDAR sees 12 m, and the slow speed (0.22, leg 4 0.30) means the rover has seconds to react. What it does not have is a **slow-down before the stop**: it drives at full cruise speed until 0.6 m and then ramps to zero.

## What really happens (13 holds, rover moving at ≥0.1)
| | Median | Range |
|---|---|---|
| Front distance when the hold state starts | 0.42 m | 0.34–0.53 m |
| Closest LiDAR reading afterwards (only clean ones, i.e. not an end wall or bollard contact) | 0.36 m | 0.21–0.49 m |
| Distance still driven after the hold starts | about 0.07 m | 0–0.22 m |
| Time until the wheel output is zero | 0.8 s | 0.5–1.3 s |

Three readings were 0.03–0.05 m (campus_2 end wall at 471 s, c102_4 at 517 s). Those are contact with the wall / the cement block, below the LiDAR minimum range. They are the real "too close" cases, and the LiDAR could not see them.

**Today the rover stops about 0.35 m from the obstacle (worst clean case 0.21 m).** The 0.5 s debounce is why the trigger is at 0.42 m, not 0.6 m: at 0.2–0.3 m/s the rover covers 0.1–0.15 m while the debounce runs.

## Can it stop at 20 cm? Yes, but only in a narrow window. The limits:
1. **35 cm LiDAR blind zone.** Below 0.35 m the LiDAR returns nothing. A 20 cm stop has to be decided while the obstacle is still at 0.35–0.45 m, and it only works if the rover is slow by then (creep 0.10–0.12 or less). At cruise speed the ramp alone takes 0.07–0.22 m, plus 0.05 m for the 4 Hz scan delay, so a stop at 20 cm from cruise speed is a coin flip.
2. **Ultrasonic can help below 35 cm** (it read 14–17 cm in the logs), but only in front, a narrow cone, and it ghosts on slopes and ground; so it can confirm a stop, not start one.
3. **Ghost filter delay.** A close point needs 4 beams or the ultrasonic. That is correct (it removed the daylight ghosts of campus_6), but it adds 0.25–0.5 s.
4. **2D LiDAR does not see low things.** The campus_9 bollard (hit at 56 m), the cement block and the stairs in run 4 are below the beam. A smarter stop cannot fix that; only the ramp opening rule (v10.3) and the tilt/pitch pause help.
5. **Slopes make false obstacles.** A tilted beam hits the ground (handled by `_slope` in `_front()`); a tighter stop distance raises the false stop rate on the ramp and the cobblestones.
6. **Gearbox and stones.** The coast changes with the surface. The same ramp stopped between 0 and 0.22 m.
7. **People walk towards the rover.** A person at 1 m/s closes 0.25 m during one 250 ms scan. Stopping at 20 cm from a moving person is not guaranteed by any speed schedule at this scan rate.
8. **Hard rule:** stops stay ramps. We never trade the ramp for a shorter stop distance.

## What we could do (not implemented, your decision)
1. **Speed schedule:** cruise 0.22 → creep 0.12 when a confirmed obstacle (≥3 beams over 2 scans) is closer than 1.5 m in the path (±20°) → stop target 0.30 m (LiDAR) / 0.20 m with the ultrasonic. Expected: stop at 0.25–0.35 m for static obstacles, with the same ramps.
2. **Shorter debounce for big, confirmed obstacles** (≥5 beams): 0.5 s → 0.25 s, gives 0.05 m.
3. **Measure first, then decide:** a field test with a box at 2 m, driven at 0.22 and at 0.12, logs `front_m`/ultrasonic to a tape-measure stop distance. Without this the 20 cm number is a guess.
4. **Simulation first:** add the ramp and the 4 Hz scan to `tools/sim_open_sea.py` (people walking in, standing, a pole) and compare today's rule with the schedule: false stops, wait time and minimum distance.

Manikant's collision avoidance code is not in the shared folders; if it exists, send it and I will compare it with this table.
