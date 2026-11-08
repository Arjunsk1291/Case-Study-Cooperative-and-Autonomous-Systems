# Drift trim calibration + camera bench test (v2, 2026-11-06)

| Folder | Board | What |
|---|---|---|
| `cas_box_trimcal/` | box ESP32 ("ESP32 Dev Module", partition **Default**) | finds the left/right wheel trim that removes the drift, own phone page |
| `cam_strip_bench/` | ESP32-CAM ("AI Thinker ESP32-CAM") | tactile strip detection, **no Wi-Fi**, results over USB |
| `tools/safe_serial.py` | laptop | serial monitor that does **not** reset the boards |

Checked on the laptop (arduino-cli, esp32 core 3.3.12):
- `cas_box_trimcal`: 1 161 184 B program (88 %), 70 KB globals (21 %), 257 KB RAM left.
- `cam_strip_bench`: 374 873 B (11 %), 34 KB globals (10 %).
- Ramps (host test): 0 -> 0.25 in 2.5 s, soft stop 0.8 s, emergency 0.3 s (ramp), pivot 0 -> 0.50 in 0.6 s.
- Phone page tested in a browser against a fake box: no script errors, heartbeat only during a run.

## Changes 2026-11-07 (after LOG2: "not enough free space" / runs ending after a few cm)
1. Free space ahead = **LiDAR only**. The front ultrasonic hears the floor (reads ~60-70 cm in open
   space); it now counts only below 25 cm for 0.6 s. Replay on logs 013-029: false "wall < 0.70 m"
   38 % of the time -> 0.
2. Rover commands are sent with **3 decimals** (was 2): a trim smaller than +-0.02 used to be rounded
   away (0.2513 / 0.2487 -> 0.25 / 0.25), so the trims of LOG2 never reached the motors.
3. Step 2 uses suggestion, +0.02, -0.02 (clearly measurable). The rover curves a little in these
   runs - that is intended.
4. Phone heartbeat timeout 0.8 -> 1.2 s (short phone hiccups stopped runs).
5. Stop messages say which sensor stopped the run. Debug shows LiDAR and ultrasonic separately.

## Important
- This program does **not** use Kethan's heading hold. On purpose: the calibration must SEE the drift.
  The final navigation program = this trim (fixed) + a gentle heading hold on top.
- It is a separate test program: no mission / route mode. Afterwards flash the team's program again.
- Never use "Erase All Flash" (not needed; the team's drive logs stay untouched).
- During a run keep the phone **screen on and the page in front**. Leaving the page (other app,
  screen off, a download that opens a new tab) = the rover stops gently. That is a safety feature.

## Room / outdoor procedure
1. Box on first, then rover. Phone: Wi-Fi `CAS-Box-04`, open `http://192.168.14.1`.
   The top row must show **Rover OK, IMU OK, LiDAR OK** (green).
2. Speed: keep **0.25** (the route speed).
3. Put the rover on the floor at one end, pointing along the longest free line. The card shows
   "Free space ahead": green = full run, yellow = shorter run, red = too short. Do not touch the rover
   during the first 2 s of a run (gyro check).
4. **Step 1** 2-3 times. Carry the rover back between runs (or use "Move rover").
5. **Step 2** 3 times. The box picks the trims itself (suggestion, +0.01, -0.01).
6. **Step 3** once. **PASS** (green) = done. NOT YET = one more Step 2, then Step 3 again.
7. Press **Download results (CSV)** (when no run is active) and send the file to me.
   "Save trim in the box" is optional.
8. Do the same OUTDOORS on the paving later: indoors the rover turns slightly left, outdoors it turned
   2-4 deg/s right (old logs) - the ground matters, the outdoor value is the one we hard-code.
