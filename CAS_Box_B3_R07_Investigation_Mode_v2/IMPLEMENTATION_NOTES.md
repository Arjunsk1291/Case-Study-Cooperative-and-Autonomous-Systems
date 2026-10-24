# Investigation Mode UI update

- Replaced one-shot marker buttons with a guided sequence: Start/scan START, Next to begin travel, Next again to mark arrival and scan the waypoint, and Last to run the final scan and close the session.
- Scan duration is configurable at 10, 15, or 20 seconds. The countdown/progress bar remains visible while capturing and hides only when that capture completes.
- Stores each dashboard sensor frame during both waypoint scans and travel. Includes all LiDAR bins, ultrasonic raw/filtered/valid/age/timeouts, IMU/calibration, GPS, dashboard values, cycle timing, snapshot receive time/age, event timestamps, phase and labels.
- Final destination scan creates an `INVESTIGATION_STOP` event; the full JSON becomes downloadable after completion (and can also be downloaded mid-session).
- FreeRTOS tasks already exist for ultrasonic acquisition and RoverLink network I/O. The browser logger is kept in browser memory to avoid a growing capture buffer in ESP32 RAM.
- No autonomous rotation was added: timed spinning without verified yaw direction, obstacle interlocks and a physical stop is unsafe. The operator can manually rotate slowly with the drive pad during the timed scan if the area is clear.
- Explicit limitation: LiDAR may support scan matching, but this logger does not implement SLAM/localization or guarantee return-to-start. No wheel encoders are used.

Validation performed: embedded JavaScript syntax checked with Node after substituting the firmware's `%MAXSPD%` template token; archive integrity checked. Not compiled with Arduino CLI and not hardware-tested.
