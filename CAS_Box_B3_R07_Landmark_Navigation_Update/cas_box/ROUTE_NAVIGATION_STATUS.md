# R07 route navigation update — important status

This revision:
- Updates the dashboard copy to describe landmark-guided navigation, not scan-and-wander.
- Removes the manual-toggle requirement for the transient LiDAR unavailable/CRC fault; the controller commands zero and retries automatically.
- Uses WAITING FOR LOCALIZATION status when no validated target is available; it does not select arbitrary open sectors.
- Includes route waypoint scan signatures extracted from the supplied manual survey.

Not yet implemented/verified:
- A complete robust online scan matcher tied to physical pose and motion.
- A `WASHROOM_FINISH` scan captured inside the washroom. The provided dataset ends at `entrance`, so the firmware must not stop at that entrance signature and claim arrival.
- Arduino CLI compilation or physical rover validation.

This is intentionally fail-safe: it will not drive the route until the localization and finish reference are valid. The rover has no wheel encoders; IMU yaw alone does not provide position or distance.
