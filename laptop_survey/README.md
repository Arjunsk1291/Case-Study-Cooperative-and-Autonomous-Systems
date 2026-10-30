# CAS Box B3 / R07 — Survey and Investigation System

This branch adds a laptop-owned survey logger and camera receiver alongside the existing CAS Box ESP32 firmware. The task title mentioned R04 once; the inspected repository, Wi-Fi identity and source code identify this hardware as B3 / Box 7 / Rover R07, so this implementation targets R07.

## Repository audit summary

The inspected repository contained the main ESP32 box firmware (cas_box/cas_box.ino) and its drivers/HMI, but no separate laptop backend, no independent frontend project, and no ESP32-S3 camera firmware or camera pin map.

Verified hardware/software interfaces in source:

| Subsystem | Existing implementation |
|---|---|
| Main controller | ESP32 Dev Module, Arduino ESP32 core documented as 3.3.1 |
| Box AP | SSID CAS-Box-07, password CASbox07, IP 192.168.17.1, channel 1 |
| Rover control | Main ESP32 HMI /drive -> RoverLink FreeRTOS task -> rover HTTP /js?json=...; 100 ms task, ~400 ms HTTP timeout, 500 ms deadman |
| Manual HMI | ESP Async WebServer endpoints /values, /drive and /autonomy; browser drivepad refreshes commands while held |
| LiDAR | LD20, Serial1, RX GPIO 27 / TX GPIO 33, 230400 baud; parser maintains 360 one-degree filtered bins; dashboard summary has 90 four-degree bins |
| IMU | BNO055 at I2C address 0x28, SDA 21 / SCL 22; existing update cadence ~100 ms |
| GPS | NEO-6M / GY-GPS6MV2, Serial2, RX 16 / TX 17, 9600 baud, TinyGPSPlus |
| Ultrasonic | Five HC-SR04 devices in front, left, right, back, front-angled order; GPIO pairs and timeout are in UsManager.h and session configuration |
| Encoders | No wheel-encoder driver or encoder measurements found in the repository |
| Camera | No camera board source, exact ESP32-S3 board model, camera module or GPIO pin map found |

The survey system preserves the existing RoverLink/manual command path. The main ESP32 samples sensors and controls the rover. The ESP32-S3 camera (once its pin map is supplied) sends JPEGs directly to the laptop; the main ESP32 does not relay image payloads.

## What is implemented

### Main ESP32 box firmware
- Publishes the box snapshot sequence and monotonic sample time.
- Publishes a separate response timestamp for each /values response.
- Publishes the 360-bin one-degree filtered LD20 distance map alongside the existing 90-bin dashboard map.
- Publishes BNO055 raw acceleration, gyro, magnetometer and temperature alongside orientation/calibration.
- Publishes decoded NEO-6M altitude, speed, course and UTC date/time with validity fields.
- Adds /clock to read the ESP32 monotonic millis() domain for laptop clock-offset measurements.
- Adds /safety/stop, /safety/clear, /safety/state and a latched drive inhibit.
- Rejects manual drive and autonomy-enable requests while the stop latch is active.
- Applies a final zero-command check in the main loop immediately before RoverLink.setDrive().
- Keeps the stop latch independent of camera availability.

The laptop Stop Rover control is a software stop over Wi-Fi. No physical mushroom E-stop input or wired safety circuit was present in the inspected repository. A real physical E-stop/power isolation must remain available during hardware tests.

### Laptop survey collector and HMI
- FastAPI backend owns session IDs, storage, data integrity, image indexing and ZIP finalization.
- Browser closure does not terminate the backend-owned session.
- Laptop polls the box /values endpoint at ~4.5 Hz, stores distinct device snapshot sequences, and detects sequence gaps and connection outages.
- Device sample time, device response time, laptop UTC receive time and laptop monotonic receive time are stored separately.
- Manual drive is proxied to the existing /drive endpoint and limited to ±0.25 command fraction. ESP32/RoverLink deadman remains active.
- Start Investigation latches the rover stopped. Manual drive requires an explicit Enable manual driving action.
- Stop Rover latches movement but keeps logging active.
- Investigation Capture writes a linked event and requests a three-frame camera burst; it does not pause or stop logging.
- Route segment start, rename, end, split and merge operations are event-logged.
- Operator annotations include environment tags and optional pillar/side/opposite-context fields.
- LiDAR wall candidates and corridor-midline candidates are observational suggestions. Operator decisions are recorded; suggestions never command motion.
- Open-area observations without sufficient boundary evidence explicitly report that no reliable centreline can be estimated.
- Wall-following approval is recorded, but wall-following motion remains disabled pending physical calibration and tests.
- A first-stage classical tactile-rib vision candidate detector is included using OpenCV when installed. It reports confidence/validity and cannot command motion.
- Finalization stops first, closes the active segment, writes a data-quality report, checksums and manifest, creates a ZIP, reopens it, checks ZIP CRC, and verifies every listed SHA-256 and byte count before marking it ready. On failure, raw files remain on disk and the session reports an error.

### ESP32-S3 camera transfer
A camera transmitter sketch and API contract are provided under camera_s3/. The data path is:

ESP32-S3 camera -> Wi-Fi station -> CAS-Box-07 AP -> laptop FastAPI /api/camera/frame

Camera images bypass the main ESP32. The sketch refuses to compile until camera_board_config.h contains the exact pin map for the actual ESP32-S3 camera board. This is intentional: guessing camera GPIO mappings would be unsafe.

## Run the laptop survey service (Ubuntu)

1. Join laptop Wi-Fi CAS-Box-07 using password CASbox07.
2. Confirm the box responds at http://192.168.17.1.
3. Find the laptop's address on the box Wi-Fi network using ip -4 addr.
4. Install and start the backend:

    cd laptop_survey
    python3 -m venv .venv
    source .venv/bin/activate
    pip install -r requirements.txt
    pip install -r requirements-vision.txt

    export CAS_BOX_URL="http://192.168.17.1"
    export CAS_SURVEY_DATA="$HOME/CAS_B3_R07_Survey_Data"
    export CAS_CAMERA_TOKEN="$(python -c 'import secrets; print(secrets.token_urlsafe(32))')"

    uvicorn app:app --host 0.0.0.0 --port 8000

5. Open http://127.0.0.1:8000 in the laptop browser. Keep laptop connected to the box AP throughout the survey.

For a camera board, use the laptop's AP-side IP as CAS_CAMERA_LAPTOP_HOST in the local, uncommitted camera board config, and use the same CAS_CAMERA_TOKEN. Never commit the real token or board-specific pin map.

## Survey operating sequence

1. Wait for the web page to show the main box online and current sensor freshness.
2. Press Start Investigation. The main ESP32 stop latch is engaged; the rover remains stationary.
3. Press Enable manual driving. This clears the latch but does not move the rover.
4. Drive manually using press-and-hold controls. Releasing sends zero. The laptop command range is capped at ±0.25.
5. Use Investigation Capture at landmarks, surface transitions, tactile-strip junctions, ramps, steps, drainage, pedestrians or uncertain areas.
6. Annotate environment tags and notes, and manage Part A / Part B route segments. These actions do not stop logging.
7. Stop Rover / Safety Latch can be used at any time; telemetry logging continues.
8. When the survey is finished, press Stop Investigation & Finalize Dataset. This is the only UI action that ends logging.
9. Wait for the UI to report ZIP CRC, manifest and SHA-256 checks passed, then use Download Survey ZIP. The completed archive remains under CAS_SURVEY_DATA/archives.

If camera is disconnected, sensor collection and finalization continue. The archive reports the camera gap/unavailability.

## Dataset layout

Each session is saved under sessions/<session_id>/ and the validated ZIP under archives/.

- manifest.json — schema, session ID and SHA-256/byte count for included files.
- session/metadata.json — start/finalize state, operator, box URL and record counts.
- session/configuration.json — sensor models, interfaces, units and source-verified hardware configuration.
- session/software_versions.json — backend and reported firmware identifiers; camera/rover versions are marked unavailable where not exposed.
- session/timing_and_sync.json — ESP32 millis clock domain, laptop UTC/monotonic clocks and clock-sync samples.
- telemetry/sensor_snapshots.jsonl — complete received /values record plus laptop receive times.
- telemetry/lidar_scans.jsonl — 90-bin dashboard scan and 360-bin rolling filtered distances where supplied.
- telemetry/imu.csv — orientation, calibration, raw vectors and temperature.
- telemetry/gps.csv — decoded GPS data, UTC and validity.
- telemetry/ultrasonic.csv — five sensors, raw/filtered distance, freshness and timeout counters.
- telemetry/motor_commands.csv — manual requested commands and reported box commands. No measured wheel speed is implied.
- telemetry/system_diagnostics.csv — heap, uptime, loop timings and rover link values reported by firmware.
- telemetry/navigation_events.jsonl and connection_events.jsonl — navigation state, sequence gaps and connectivity changes.
- camera/frames/ and camera/frame_index.csv — JPEG evidence and metadata if camera upload is configured.
- annotations/ — operator annotations, route segments and investigation events.
- derived/vision_tracking.jsonl and vision_tracking.csv — optional tactile-strip candidate analysis.
- integrity/data_quality_report.json and checksums.json — coverage, known unavailable fields and checksums.

## Known data limitations

- LD20 parser does not expose raw intensity or native per-point timestamps. The exported 360-bin map is rolling-filtered distance data, not a raw packet capture.
- BNO055 raw vectors are acquired at the current update cadence (~10 Hz); the latest sample is represented in ~4 Hz snapshots. Sensor buses are not hardware synchronized.
- TinyGPSPlus raw NMEA sentences are not retained; decoded values and validity are exported.
- No wheel encoder hardware/code exists in this repository. Do not infer distance travelled or physical wheel speed from drive command fractions.
- GPS fixes may be poor or unavailable on campus; HDOP/satellites/fix/age are retained for quality review.
- Camera board is not identified. The protocol/firmware template is present, but camera compilation/testing requires its model and verified GPIO pinout.
- Wall detection is a LiDAR line-fit candidate, not semantic wall recognition. Wall-following motion is deliberately disabled.
- LiDAR-derived open-area/corridor suggestion is survey-only; there is no autonomous route execution in this branch.
- Tactile-strip detector is a first-stage classical CV heuristic; parallel tile joints can resemble tactile ribs. Confidence must be evaluated against real images.

## Manual turn/deadband calibration requires the physical rover

Do not increase pivot commands blindly. With rover restrained and wheels lifted:
1. Verify left/right motor polarity and wheel response at lowest command increments.
2. Determine minimum sustained command for each wheel independently; record left/right deadband and battery voltage.
3. Test differential turn-in-place at low bounded command; observe whether both wheels overcome stiction together.
4. Repeat on a level, clear test floor at supervised low speed and measure yaw response with the IMU.
5. Document calibrated deadband/asymmetry before implementing a bounded turn controller.

This branch does not claim the low-speed turn issue is physically fixed.

## Build and verification

Main box firmware:
- Open cas_box/cas_box.ino in Arduino IDE.
- Select ESP32 Dev Module, ESP32 Arduino core 3.3.1 (version in repository documentation), and install the libraries listed in cas_box/README.md.
- Compile before flashing. CI attempts the same ESP32 Dev Module build.

Laptop tests:

    cd laptop_survey
    pip install -r requirements-test.txt
    PYTHONPATH=. pytest -q
    python -m py_compile app.py manager.py analysis.py

Camera:
- See camera_s3/README.md.
- Cannot be compiled until exact board model and pin map are supplied.

## Safety and deployment scope

This is a supervised manual survey collector. The main ESP32 control/safety loop does not depend on camera transfer or laptop file storage. Laptop stop is a Wi-Fi software stop, not a physical E-stop. If laptop or box connection drops, RoverLink deadman is expected to expire the last drive command, but physical safety must never rely on Wi-Fi alone. Do not survey around pedestrians, steps, traffic or public access areas without an operator spotter and route authorization.
