# CAS Box B3 / R07 scan-and-wander navigation update

## What changed
- Dashboard includes **Download diagnostic JSON**, exporting the current named firmware values, all 90 LiDAR bins, ultrasonic raw/filtered/valid/age/timeout arrays, IMU/GPS data, loop timing, timestamps, autonomy state and operator notes.
- Replaced surface-dependent movement with a **scan-and-wander** controller:
  - Scores 8 candidate directions at 45-degree increments from the 360-degree LiDAR view.
  - Treats missing LiDAR returns as unknown, not clear.
  - Uses fresh front and angled-front ultrasonic readings as near-field stop checks, plus side/rear checks when evaluating a turn.
  - Moves in short bounded forward legs, then rescans rather than driving indefinitely.
  - Uses IMU yaw feedback to stop each 45-degree turn step; it does not assume a timed pivot equals 45 degrees.
  - Uses the maximum configured pivot command (0.50 normalized command) while the measured turn is in progress, with a timeout and feedback-mismatch stop.
  - Requires a working IMU and calibrated gyro before autonomous turning. If that is unavailable, it stops rather than doing a blind full-power pivot.
  - Preserves the original fault reason after a fault latches.


## Investigation mode (new dashboard workflow)
- Keep autonomy OFF. Start **Investigation mode — landmark survey** from the dashboard. It records full sensor snapshots approximately once per second in browser memory.
- At the initial position, set label `START` and press **Capture landmark scan**. Leave the rover stationary for 5–10 seconds so multiple `ROUTE_SAMPLE` records are collected.
- Manually drive to the next point, stop, enter a label (for example `CORRIDOR_TURN_1`), and press **Capture landmark scan**. Use **Mark transition** to annotate a route segment/event. Repeat for each landmark and press **Record END** at the destination.
- Press **Finish / stop logging**, then **Download investigation JSON**. Include operator notes describing room layout, rover orientation, what the rover did, and any sensor/motor anomalies.
- This logger stores browser-received snapshots, not hardware-synchronized data. It does not control autonomous exploration, calculate a metric map, or prove that two landmark scans are the same place. Keep the dashboard open; refreshing the page loses the in-memory investigation. The export can become large during long sessions.
- Prefer stationary landmark captures and short manually driven transitions. Do not ask the rover to wander and autonomously return to START until localization and return verification are implemented.

## Turning command is not a physical speed guarantee
The rover accepts normalized left/right commands in [-0.5, +0.5]. A pivot command of 0.50 is the maximum configured command, not a guarantee of a particular angular velocity. Actual rotation depends on motor controller limits, battery, floor friction, payload and wheel geometry. The controller therefore uses BNO055 yaw feedback and a timeout; validate yaw sign and motor polarity with the wheels lifted first. If the BNO055 yaw is magnetically disturbed, short relative turns may still be inaccurate, so supervise tests and record measured angle.

## Important limits
- This is a **prototype**, not a verified or safety-certified navigation stack. It has not been compiled against the user's exact Arduino environment or tested on the physical rover.
- It is local reactive scan-and-wander, not global indoor navigation from bed to washroom. It has no persistent map, reliable odometry, wheel encoders, or indoor GPS positioning. It chooses open sectors but does not identify the washroom door as a semantic landmark. A route with turns/doorways needs mapping/localization and waypoint logic (e.g., 2D LiDAR scan matching + IMU fusion).
- Validate that the 90-bin LiDAR orientation is correct. The controller assumes index 0 is forward and bins increase clockwise. Confirm this by placing obstacles at known directions and checking the dashboard.
- Validate motor signs and turning direction with the wheels lifted. If a left command turns right, stop and correct motor polarity/command mapping before floor tests.
- `300 cm` ultrasonic readings can mean no echo/out of range, not guaranteed free space. LiDAR zero bins are unknown, not free.
- Browser heartbeat stop remains enabled. Keep the dashboard foregrounded during supervised tests; do not remove heartbeat fail-safe to mask its fault.

## Recommended test sequence
1. Compile in Arduino IDE with ESP32 core 3.3.x and the exact libraries from README. Fix any build errors before flashing.
2. With rover wheels lifted, test manual forward/left/right commands and verify the pivot sign. Verify left pivot increases yaw and right pivot decreases yaw.
3. Keep autonomy disabled; point LiDAR at known obstacles at front, left, right, and rear, and verify the displayed bins/sector orientation.
4. In a taped-off clear area, use a spotter and physical stop access. Start at lowest practical speed. Test a single obstacle at >1 m first; do not test with people/pets nearby or near stairs.
5. Capture diagnostic JSON before and after each failure and include operator notes describing physical movement.
6. Only after obstacle tests pass should scan-and-wander be evaluated in a clear, taped-off room. Do not attempt unsupervised bed-to-washroom operation.

## Research basis and filtering decision
- Borenstein & Koren, **“The Vector Field Histogram—Fast Obstacle Avoidance for Mobile Robots”**, IEEE Transactions on Robotics and Automation (1991): https://doi.org/10.1109/70.88137. The updated controller borrows only the local free-sector idea; it is not a full VFH implementation.
- Fox, Burgard & Thrun, **“The Dynamic Window Approach to Collision Avoidance”** (1997): https://doi.org/10.1109/ROBOT.1997.614023. A full DWA controller evaluates dynamically reachable velocity commands; this firmware does not implement full DWA because the rover's dynamics/acceleration limits are not characterized and there are no encoders.
- ROS 2 Nav2 Regulated Pure Pursuit documentation (reference for later waypoint tracking, not currently implemented): https://docs.nav2.org/configuration/packages/configuring-regulated-pp.html

### Kalman filter?
Do **not** add a generic Kalman filter to all sensor readings as the next step. Filtering does not fix incorrect LiDAR angle mapping, stale samples, ultrasonic no-echo semantics, or missing odometry. First validate timestamps, sensor orientation, and the controller with replayable logs. For actual indoor localization, a later estimator could fuse IMU yaw/gyro with LiDAR scan-matching pose increments in an EKF; wheel-encoder odometry is unavailable, and the BNO055's fused heading may drift or be magnetically disturbed. A Kalman/EKF implementation needs explicit state, process/measurement models, covariance tuning, and validated scan-match quality checks.


## Investigation Mode v2 button-state fix
- Fixed the UI refresh dependency: investigation button availability is recalculated on each successful `/values` poll after `latestSnapshot` is assigned. Previously the initial `invUi()` ran before the first snapshot and the Start button stayed disabled indefinitely.
- Investigation button state is also refreshed after autonomy changes and after poll errors.
- This is a source-level fix; still compile with the target Arduino ESP32 core and test the dashboard on the actual box before relying on it.

## Compact-chassis doorway clearance adjustment (v4)
- Reduced forward avoid threshold from 85 cm to 42 cm and turn-sector clearance from 60 cm to 32 cm. These prior thresholds were large enough to reject many ordinary indoor doorway/room approaches for this compact chassis.
- Reduced side ultrasonic turn veto from 38 cm to 22 cm and rear turn veto from 25 cm to 20 cm. The 15 cm forward/angled hard-stop remains unchanged.
- This is a threshold correction, **not** a measured footprint model. Before autonomous movement, measure the widest chassis dimension and LiDAR position/offset; keep a margin appropriate to sensor error and stopping distance. If the doorway is narrow or the robot must rotate inside it, do not command a pivot until the full swept footprint fits.
- The controller still treats a low LiDAR return as the closest obstacle, assumes bin 0 is forward, and relies on IMU turn feedback. Verify angle mapping, yaw sign and braking distance under supervision. Ultrasonic sensors do not reliably detect floor drop-offs/steps; a downward-facing cliff sensor is needed for that hazard.


## Latest manual route survey analysis (2026-10-23)
- Added `ROUTE_SCAN_SIGNATURES.csv` and `ROUTE_ANALYSIS.md` from the latest investigation JSON. The CSV contains 90-bin per-checkpoint LiDAR median signatures and usable ultrasonic medians.
- These are offline references only. No route-following code is enabled by this package. The current controller remains scan-and-wander and does not recognize START/pillar/door/washroom landmarks.
- The latest survey was captured with `nav.mode=MANUAL`. It contains no proof of autonomous arrival, no metric map, and no valid wheel odometry. Do not replay travel durations as distances.
- The `pillar 1 turn` capture has a right ultrasonic median around 17 cm; the final entrance capture has right ultrasonic median around 33 cm. These are reasons to stop/re-scan, not turn-clearance guarantees.
- Before implementing waypoint navigation, validate LiDAR angle orientation and motor/IMU yaw sign, then add and replay-test a confidence-gated scan matcher.
