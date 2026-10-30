# CAS Box B3 / Rover R04 — Updated Flash & Investigation Guide

## Correct system architecture

The main CAS Box ESP32 is the live hub and the only robot controller:

- Phone ──Wi-Fi──► main ESP32 (CAS-Box-04, 192.168.14.1)
- AI-Thinker ESP32-CAM ──Wi-Fi──► main ESP32 (same AP)
- Main ESP32 ──existing RoverLink──► R04 rover (motor controller firmware unchanged)

The camera does not send image bytes to the main ESP32. It receives small capture requests and reports status/metadata. It saves the original JPEG files directly to its own onboard microSD card. The main ESP32 has no configured persistent image storage, and the camera images must not be buffered in its RAM or flash.

The laptop is not part of startup, live sensing, camera transfer or robot control. Use it only after the survey to package files for offline analysis.

## Hardware requirement for camera logging

Insert a FAT32 microSD card into the AI-Thinker ESP32-CAM's built-in card slot. The camera sketch mounts it in SD_MMC 1-bit mode. It writes files to /captures/ and keeps a manifest.csv. If the card is not detected or has insufficient free space, image capture is disabled in the phone dashboard.

This is the storage path for original JPEGs. No external SD module or new wiring on the main ESP32 is required.

## 1. Flash the main CAS Box ESP32

Target: ESP32 Dev Module. Open the complete cas_box folder via cas_box/cas_box.ino and retain all adjacent .h/.cpp files.

Use Arduino IDE 2.x, Espressif Arduino-ESP32 core 3.3.1, and these libraries:

- ESP Async WebServer
- Async TCP
- Adafruit BNO055
- Adafruit Unified Sensor
- Adafruit SSD1306
- Adafruit GFX Library
- Adafruit BusIO
- TinyGPSPlus-ESP32

Set BOX_ID = 4 (already set in the R04 branch). Verify:
- SSID: CAS-Box-04
- Password: CASbox04
- Main HMI: http://192.168.14.1

Flash the main ESP32 and open Serial Monitor at 115200 baud. Power the main box first, then power/reboot the R04 rover so it joins the correct AP. Confirm RoverLink reports connected.

The main ESP32 remains the controller. Its motor deadman and software stop latch are retained. The camera routes do not call RoverLink or change motor setpoints.

## 2. Configure and flash the AI-Thinker ESP32-CAM

Target: AI Thinker ESP32-CAM (OV2640). It is a separate board and must be flashed separately.

### Replace the old camera config

The previous camera sketch required a laptop IP and shared token. The updated design removes both. In the camera sketch folder, overwrite the old local config file with the new example:

    cd camera_s3/CameraStation
    cp -f camera_user_config.example.h camera_user_config.h

The new config contains:
- CAS_BOX_HOST = 192.168.14.1
- CAS_BOX_SSID = CAS-Box-04
- CAS_BOX_PASSWORD = CASbox04

Do not retain the old CAS_CAMERA_LAPTOP_HOST or CAS_CAMERA_TOKEN definitions.

### Camera board upload

1. Insert a FAT32 microSD card in the AI-Thinker ESP32-CAM card slot.
2. Open camera_s3/CameraStation/CameraStation.ino in Arduino IDE.
3. Select Tools → Board → ESP32 Arduino → AI Thinker ESP32-CAM.
4. Select the camera serial port and upload.
5. Open Serial Monitor at 115200 baud.

The camera's pin map remains the confirmed AI-Thinker OV2640 mapping. Camera image settings remain JPEG QVGA 320x240, quality 12, XCLK 20 MHz and one internal-DRAM frame buffer. SD_MMC uses 1-bit mode.

Programming access limitation: the AI-Thinker ESP32-CAM has no native USB connector. Initial flashing requires an ESP32-CAM-MB programmer or USB-to-serial adapter and physical access to the board's programming pins. The current sketch does not implement OTA.

Expected serial messages:
- camera initialization success
- microSD mount and storage summary
- Wi-Fi connected to CAS-Box-04
- camera archive URL on port 81
- periodic heap / Wi-Fi / capture diagnostics

## 3. Start a survey using only the phone and robot

1. Power the main ESP32.
2. Power the R04 rover and verify RoverLink connection.
3. Power the camera with its microSD inserted.
4. Connect the phone to CAS-Box-04 using password CASbox04.
5. Open http://192.168.14.1.
6. Wait for the camera panel to show Camera ready and microSD available.
7. Use the normal investigation logger. Keep autonomy OFF and supervise all manual movement.
8. At a relevant location or event, tap Capture 3 JPEG frames. The main ESP32 creates an event ID; the camera polls the main board and saves the original frames on its microSD.
9. Continue recording route/sensor snapshots in the phone dashboard.
10. Download the sensor/route JSON or CSV before refreshing/closing the phone page.

The main ESP32 receives small camera health and capture metadata only. There is no continuous image preview, no laptop receiver and no JPEG relay through the main board.

## 4. Download camera files

The camera heartbeat reports its DHCP address to the main ESP32. When the camera is online, the phone dashboard shows an Open camera archive link.

Open that link on the phone while still connected to CAS-Box-04. The camera archive page lists individual JPEGs under /captures/ and provides manifest.csv. Download the manifest and the original JPEGs associated with the survey events.

Camera archive endpoint: http://<camera-dhcp-ip>:81/

If the archive link does not open, verify that the camera is online, has a mounted SD card, and is connected to the same CAS-Box-04 network. The phone and camera must be able to communicate as clients on that AP.

## 5. Offline log processing on the laptop

The laptop is not needed while the rover is operating.

After the run:
1. Put the dashboard JSON/CSV, camera manifest.csv and original JPEGs together.
2. Copy them to the laptop.
3. Optional: run the offline package builder:

       cd laptop_survey
       python3 -m venv .venv
       source .venv/bin/activate
       pip install -r requirements.txt
       uvicorn app:app --host 127.0.0.1 --port 8000

4. Open http://127.0.0.1:8000, select the log and image files, and build one checksummed ZIP.
5. Attach that ZIP to the analysis conversation for offline review.

The packager does not connect to the robot, run vision or invent coordinates. The uploaded source data and original images are analysed offline. Any generated route instructions must distinguish measured facts from inferred geometry.

## 6. Return only a compact command package to the main ESP32

The analysis result uses the bounded CAS-R04-NAV-CMD-1 JSON schema. It contains a route identifier, safety parameters and at most 12 waypoint records. The phone dashboard can submit that compact JSON via Send small plan to main ESP32.

The firmware stores the plan in RAM as a draft and reports execution_enabled=false. It does not run the route. This is intentional: the current R04 firmware does not implement validated indoor localization, wheel odometry or a waypoint-tracking executor. GPS indoors, IMU yaw and sparse sensor logs are not enough to claim reliable x/y localization. Existing Navigation.h holds motion at zero while localization is unavailable.

A future revision can connect the validated localization output to a motion executor after physical testing. Until then, use the route plan for review and manual supervised operation only. Do not bypass the safety latch or change Navigation.h to force motion.

## 7. Safety and verification

- Main ESP32 software STOP is a Wi-Fi software interlock, not a physical E-stop circuit.
- Keep the physical power disconnect/stop access available.
- The rover motor-controller firmware and RoverLink protocol remain unchanged.
- Camera failures cannot block the main ESP32 motor/safety loop.
- Camera images are stored locally on microSD; main ESP32 RAM/flash is not used as an image archive.
- The route-plan mailbox is draft-only and never changes motor commands.
- CI compilation does not prove physical Wi-Fi association, SD reliability, camera image capture or rover operation. Test all hardware functions at low speed in a clear, supervised area.
