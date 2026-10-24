# CAS Box B3 / R07 diagnostic and navigation update

## What changed
- Dashboard now has **Download diagnostic JSON**. It exports the actual latest `/values` response, named firmware values, full sensor snapshot including all 90 LiDAR bins, ultrasonic raw/filtered/valid/age/timeout arrays, IMU/GPS data, loop timing, browser timestamp/state, autonomy state, and operator notes.
- Replaced the previous experimental navigation controller with a conservative reactive controller:
  - VFH-inspired left/right free-sector check from LiDAR sectors.
  - Obstacle avoidance takes priority over surface following.
  - Stops before pivot, uses a bounded low-speed pivot, settles, then rechecks.
  - Unknown LiDAR bins are not treated as free space; side turn requires several valid returns and clearance.
  - Uses fresh front ultrasonic as an additional near-field minimum and fresh side ultrasonics to veto a turn.
  - Differential proportional correction for ordinary wall-following instead of frequent stop/pivot pulses.
  - Latches critical LiDAR/front-clearance faults until autonomy is disabled and re-enabled.
  - Removed the old “no scene change means stalled” auto-recovery assumption because no encoders means that is not reliable evidence of a stall.

## Important limits
- This is a **prototype**, not a verified or safety-certified navigation stack. It has not been compiled against the user's exact Arduino environment or tested on the physical rover.
- It is local reactive surface-following, not global indoor navigation from bed to washroom. It has no map, reliable odometry, wheel encoders, or indoor GPS positioning. A route with turns/doorways needs mapping/localization and waypoint logic (e.g., 2D LiDAR scan matching + IMU fusion), not only wall following.
- Validate that the 90-bin LiDAR orientation is correct. Existing code assumes index 0 is forward and sectors 86–4 are front, 13–31 right, 59–77 left. Confirm this by rotating the unit and checking the dashboard.
- Validate motor signs and turning direction with the wheels lifted. If a left command turns right, stop and correct motor polarity/command mapping before floor tests.
- `300 cm` ultrasonic readings can mean no echo/out of range, not guaranteed free space. LiDAR zero bins are unknown, not free.
- Browser heartbeat stop remains enabled. Keep the dashboard foregrounded during supervised tests; do not remove heartbeat fail-safe to mask its fault.

## Recommended test sequence
1. Compile in Arduino IDE with ESP32 core 3.3.x and the exact libraries from README. Fix any build errors before flashing.
2. With rover wheels lifted, test manual forward/left/right commands and verify the pivot sign.
3. Keep autonomy disabled; point LiDAR at known obstacles at front, left, right, and verify the displayed bins/sector orientation.
4. In a taped-off clear area, use a spotter and physical stop access. Start at lowest practical speed. Test a single obstacle at >1 m first; do not test with people/pets nearby or near stairs.
5. Capture diagnostic JSON before and after each failure and include operator notes describing physical movement.
6. Only after obstacle tests pass should surface-following be evaluated along a straight, unobstructed wall. Do not attempt unsupervised bed-to-washroom operation.

## Research basis and filtering decision
- Borenstein & Koren, **“The Vector Field Histogram—Fast Obstacle Avoidance for Mobile Robots”**, IEEE Transactions on Robotics and Automation (1991): https://doi.org/10.1109/70.88137. The updated controller borrows only the local free-sector idea; it is not a full VFH implementation.
- Fox, Burgard & Thrun, **“The Dynamic Window Approach to Collision Avoidance”** (1997): https://doi.org/10.1109/ROBOT.1997.614023. A full DWA controller evaluates dynamically reachable velocity commands; this firmware does not implement full DWA because the rover's dynamics/acceleration limits are not characterized and there are no encoders.
- ROS 2 Nav2 Regulated Pure Pursuit documentation (reference for later waypoint tracking, not currently implemented): https://docs.nav2.org/configuration/packages/configuring-regulated-pp.html

### Kalman filter?
Do **not** add a generic Kalman filter to all sensor readings as the next step. Filtering does not fix incorrect LiDAR angle mapping, stale samples, ultrasonic no-echo semantics, or missing odometry. First validate timestamps, sensor orientation, and the controller with replayable logs. For actual indoor localization, a later estimator could fuse IMU yaw/gyro with LiDAR scan-matching pose increments in an EKF; wheel-encoder odometry is unavailable, and the BNO055's fused heading may drift or be magnetically disturbed. A Kalman/EKF implementation needs explicit state, process/measurement models, covariance tuning, and validated scan-match quality checks.
