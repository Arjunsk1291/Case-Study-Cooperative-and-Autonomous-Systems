# CAS Box B3 / Rover R04 — Outdoor test

This branch is for outdoor data collection. It preserves the verified indoor firmware and does not enable unsupervised turning, pillar counting, or camera image processing.

## Before the run

- Flash the main ESP32 and ESP32-CAM from `build/box` and `build/camera`.
- Leave the rover firmware unchanged.
- Connect the phone to `CAS-Box-04` and open `http://192.168.14.1`.
- Confirm rover connection, fresh LiDAR, valid IMU calibration, camera readiness, battery voltage, and a clear safety latch.
- Enable the phone screen wake lock and keep the dashboard visible.
- Record lighting, surface type, weather, route description, and operator ground truth.

## Outdoor sequence

1. With wheels off the ground, verify STOP, deadman release, and safety-latch behavior.
2. Run a short straight movement test and record commanded speeds, yaw, pitch, roll, LiDAR validity, GPS fix quality, RSSI, and battery voltage.
3. Run a short wall-follow observation without an automatic turn.
4. Stop, inspect the saved data, then run the intended route manually.
5. Capture images at several lighting conditions, including glare or shadow, and verify that each saved JPEG opens.
6. End with STOP ROVER, then explicitly finalize the investigation and verify the ZIP before sharing it.

## Required observations

Record every connection loss, stale stream, invalid GPS fix, weak RSSI warning, low-voltage warning, overexposed/dark image, LiDAR validity drop, and phone visibility/heartbeat warning. Do not replace invalid values with zeros.

## Exit criteria

Do not call the outdoor test successful until three consecutive runs have physical movement, no unexplained rover disconnect, no unexplained safety stop, valid ZIPs, and an explicit report of dropped records and missing images.
