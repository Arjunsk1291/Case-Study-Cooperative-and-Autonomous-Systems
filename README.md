# CAS Box — R04 survey integration

This branch contains the **R04 manual survey implementation** alongside the existing repository. The existing R07 simulation assets are retained but are not the R04 flash target.

## R04 hardware targets

- Main ESP32 box controller: cas_box/cas_box.ino, with BOX_ID = 4.
- AI-Thinker ESP32-CAM / OV2640: camera_s3/CameraStation/CameraStation.ino.
- Phone dashboard: served directly by the main ESP32 at 192.168.14.1.
- Laptop offline log packager: laptop_survey/app.py (not a live robot service).

## Quick links

- R04 separate-board upload guide: R04_FLASH_GUIDE.md
- Main ESP32 firmware and drivers: cas_box/
- AI-Thinker camera firmware and microSD archive: camera_s3/CameraStation/
- Offline log packager: laptop_survey/
- R04 build and test workflow: .github/workflows/r04-survey-checks.yml

## R04 live network path

The main ESP32 starts CAS-Box-04 (password CASbox04) at 192.168.14.1. The R04 rover, phone and AI-Thinker camera join this same access point.

The phone uses the main ESP32 HMI for sensor logging and manual control. Camera capture requests and health metadata pass through the main ESP32; original JPEG files are written directly to the camera board's onboard microSD card. The main ESP32 does not receive or buffer JPEG payloads. Use the camera archive link on the phone dashboard to download the JPEGs and manifest.csv.

The laptop is not part of startup, live sensor collection, camera transfer or robot control. After the run, bundle the phone's sensor log with the camera JPEGs for offline analysis.

## Compact offline route plan

The phone dashboard accepts a small CAS-R04-NAV-CMD-1 route-plan JSON package. The main ESP32 stores this document as a draft in RAM and reports execution_enabled=false. The current R04 firmware does not have validated indoor localization, wheel odometry or a waypoint execution layer, so importing a plan does not move the rover. Existing Navigation.h remains stopped until localization is validated.

## Safety and limitations

The main ESP32 STOP is a Wi-Fi software interlock, not a physical E-stop. Keep a physical power disconnect accessible. No wheel encoder measurements are present. Do not infer precise indoor x/y position from GPS or IMU yaw alone. LiDAR geometry and camera images are survey observations; the imported route plan remains non-actuating until the route executor is independently implemented and field-tested.
