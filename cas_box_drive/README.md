# cas_box_drive - strip following test (indoor), 2026-11-07

Box program: phone page `http://192.168.14.1` = **Strip follow**. Trim tool: `http://192.168.14.1/trim`.
Camera: `link_test/link_cam` (already on the camera). Link bench-tested: 0 lost at 10/s, recovers by itself.

## What it does
- **Align**: camera sees the strip angle -> rover turns on the spot (0.50, gyro) until |angle| < 6 deg (3 tries).
- **Follow**: drives at the chosen speed. Each camera message moves the TARGET HEADING a little towards the strip
  (max 8 deg/s, max 25 deg from the start heading). Kethan's heading hold + trim (DriveStraight) steers.
  The camera never drives the wheels directly.
- **Stops** (always ramped): wall ahead <= chosen distance (LiDAR) = goal | strip lost 3 s or 0.8 m | camera data
  stale = treated as lost | heading failsafe | page heartbeat lost 1.2 s | move button | STOP | distance / time limit.
- **Log**: 10 samples/s of the last run (offset, angle, yaw, target, wheel outputs, distance ahead) + runs table.

## Test procedure (rover on the floor, strip towards a wall, >= 1 m free behind the wall stop)
0. Box on first, then rover, camera on its power bank. Phone: `CAS-Box-04` -> `192.168.14.1`.
   Chips green: Rover, IMU, LiDAR, Camera paired, Data fresh.
1. **Direction check (no driving)**: put the strip ~10 cm to the RIGHT of the rover's centre -> page must say
   "Strip RIGHT". If it says LEFT, stop and tell me (sign flip in the camera mounting).
2. **Scale check (no driving)**: rover exactly on the strip -> press **Set centre**. Slide the rover 5 cm
   sideways -> note the offset in px. (Tells me px per cm; > 30 px per 5 cm = camera "zoomed in": start centred.)
3. **Width**: belt -> "Narrow", wide strap -> "Wide". Page must show "Strip FOUND" with 3-4/4 rows.
4. **Align only**: turn the rover ~20 deg off the strip -> button 1. It turns onto the strip and stops.
5. **Align + follow**: speed 0.15 first, stop at 0.50 m, max 2 m -> button 2. Then 0.20, max 4 m.
6. After each run: **Download last run log** + **Download all runs** -> put into `Log_run/follow_<n>/`.
Keep the phone screen on during a run. STOP is always at the bottom.
