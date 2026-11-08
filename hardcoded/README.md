# HARDCODED VALUES - kit 04 (box BOX_ID 4, WAVE ROVER)  — read this first

Give this file to any AI / teammate before they write driving code. Everything here was MEASURED.
Last update: 2026-11-07.

## 1. Wheel trim (drift)
```
left  = v * (1 - trim)          right = v * (1 + trim)
trim  = -0.046   at v = 0.25   (indoor floor tiles)
=> v 0.25:  left 0.2615, right 0.2385
```
- With EQUAL wheels the rover turns **+0.40 deg/s to the LEFT** (indoors, 10 runs, `Log_run/log3`).
- Drift vs trim: `drift = +0.396 + 8.58 * trim` deg/s. One run scatters +-0.07 deg/s.
  VERIFY runs at -0.065 / -0.048 / -0.035 gave -0.15 / -0.05 / -0.02 deg/s (PASS, limit 0.30).
- The rover reacts WEAKLY to trim: 8.6 deg/s per unit trim (= 34.4 deg/s per unit of half wheel difference).
- **Outdoors is NOT measured yet.** Old outdoor logs (009-013, equal wheels, no gyro hold) turned
  2-4 deg/s to the RIGHT. Measure once outdoors with `drift_trim_cal` (Step 3) before trusting -0.046 there.
- Values are in `RoverTrim.h`.

## 2. Straight driving = trim + Kethan's heading hold + failsafe  (`DriveStraight.h`)
- Kethan's gyro heading hold is kept UNCHANGED (field-tested in log_025, hold error < 2.3 deg):
  `c = 0.045*e - 0.0015*yawRate`, `|c| <= min(0.10, 0.4*v)`, c changes <= 0.30/s, `left -= c, right += c`.
- Added: **trim learning** - a constant push from the ground is slowly (15 s) moved from c into an extra
  trim (max +-0.20), so the correction range stays free. Only while straight and |error| < 6 deg.
- Added: **failsafe** (latched, the caller ramps to 0):
  IMU invalid 0.5 s | heading error > 12 deg for 2 s | correction at its limit for 5 s | 3 gyro jumps > 30 deg.
  WARN (keeps driving): error > 5 deg, or learned trim at its limit.
- Laptop test (`test_DriveStraight.cpp`, rover model from log3): indoor error 0.2 deg rms; ground push
  2-3 deg/s held within 1-2 deg; 8 deg knock recovers; 25 deg knock / -8 deg/s / 1 s IMU loss -> failsafe.

## 3. Motor rules (gearbox) - `GentleDrive.h`, last stage before rover.setDrive()
- Start: 0.05, then linear to the target in 2.5 s. The rover only starts rolling at ~0.12.
- Normal stop 0.30/s (0.25 -> 0 in 0.8 s). Emergency stop 0.80/s (0.3 s) - still a ramp, NEVER a jump.
- Turn on the spot ONLY at 0.50 (100 %), ramped up in 0.6 s. Below ~0.46 the rover stalls (logs 025-029).
- Speed on the route: 0.25 (max 0.30). Command range -0.5 .. 0.5.

## 4. Link / sensor facts (bugs found, keep the fixes)
- Send rover commands with **3 decimals** (`String(l, 3)`). With 2 decimals every trim < 0.02 was rounded away.
- Rover commands at a steady 10 Hz; no bursts on every change.
- **Front ultrasonic (horizontal) hears the floor**: reads ~60-70 cm in open space (62-74 % of the time).
  Use the LiDAR for "free space"; the front ultrasonic only below 25 cm for 0.6 s.
- **Front-angled ultrasonic (45 deg down, cliff / step sensor)**: 14-18 cm on flat ground, but NO echo
  29-51 % of the time on smooth paving -> "no echo" is NOT a cliff by itself. Needs a tested rule before use.
- Back ultrasonic: often a constant 16 cm (something in front of it) - do not trust alone.
- Ultrasonic pings >= 50 ms apart (5 ms gave false echoes).
- Hold timers: never `start = millis() | 1u` - when millis() is even the start is 1 ms in the future and
  `millis() - start` wraps around: the hold fires at once. Use a bool "running" + the start time.
  (This pattern is in the team's SafetyGuard.h too.)
- GPS: 0 satellites in logs 025-029 (hardware/antenna), not usable.

## 5. Flashing rules (do not damage the boards)
- Box (CP2102 adapter): Upload Speed 115200, Erase All Flash **Disabled**, partition Default.
- Camera (FTDI FT232R adapter, auto-reset works): esptool at 115200, flash freq **40 MHz** (clone flash chip 0x5e).
- The camera needs its power-down pin cycled before esp_camera_init (cam_strip_bench does it); the old R04
  firmware failed with "SCCB addr 0x30" / "psram=no" on the same board.
