# CAS Box — R04 survey integration

This branch contains the **R04 manual survey implementation** alongside the existing repository. The existing R07 simulation assets are retained but are not the R04 flash target.

## R04 hardware targets

- Main ESP32 box controller: cas_box/cas_box.ino, with BOX_ID = 4.
- AI-Thinker ESP32-CAM / OV2640: camera_s3/CameraStation/CameraStation.ino.
- Phone Web HMI and IndexedDB recorder: served by the main ESP32.

## Quick links

- R04 separate-board upload guide: R04_FLASH_GUIDE.md
- Main ESP32 firmware and drivers: cas_box/
- AI-Thinker camera firmware: camera_s3/CameraStation/
- Laptop FastAPI service and HMI: laptop_survey/
- R04 build and test workflow: .github/workflows/r04-survey-checks.yml

## R04 network path

The main ESP32 starts CAS-Box-04 (password CASbox04) at 192.168.14.1. The R04 rover and camera
join that AP, and the phone joins it for the dashboard. The phone persistently records telemetry
and camera JPEGs during the investigation; the main ESP32 remains the rover controller and safety authority.

The Web HMI waits for three independent readiness states: box telemetry online, rover connected over RoverLink, and AI-Thinker camera heartbeat/camera-ready. It then enables Start Investigation. The rover is initially stopped, and manual driving requires explicit enable.

## Safety and limitations

The laptop Stop Rover command is a Wi-Fi software stop, not a physical E-stop. Keep a hardware stop/power disconnect accessible. No wheel encoder measurements are present. Wall-following motion remains disabled; LiDAR geometry, centreline suggestions and tactile-strip detection are non-actuating survey observations.
