# CAS Box B3 / R04 — Survey Collector

This is the laptop service for the R04 survey firmware. The two embedded targets are:
- Main controller: cas_box/ — ESP32 Dev Module, BOX_ID 4.
- Camera: camera_s3/CameraStation/ — AI-Thinker ESP32-CAM / OV2640.

The camera joins the main box's AP and sends images directly to this laptop. It does not send JPEG bytes through the main ESP32.

## Network and readiness

BOX_ID 4 derives:
- SSID: CAS-Box-04
- Password: CASbox04
- Box address: 192.168.14.1

The laptop and AI-Thinker camera join that AP. The R04 rover must also be provisioned to join the same AP. The laptop HMI monitors box telemetry, the main ESP32's rover-connected flag and the camera heartbeat. Start Investigation is enabled only when all three links are ready and the camera reports camera_ready=true.

## Install

Connect laptop Wi-Fi to CAS-Box-04. Install Python 3.10+ and run:

    cd laptop_survey
    python3 -m venv .venv
    source .venv/bin/activate
    pip install -r requirements.txt
    pip install -r requirements-vision.txt

Find the laptop Wi-Fi IPv4 address using ip -4 addr. It must be an address on the 192.168.14.0/24 network, not the box address 192.168.14.1. Put that address into camera_s3/CameraStation/camera_user_config.h as CAS_CAMERA_LAPTOP_HOST.

Generate a long token:

    python3 -c 'import secrets; print(secrets.token_urlsafe(32))'

Use the same token in camera_user_config.h as CAS_CAMERA_TOKEN and in the laptop environment:

    export CAS_BOX_URL="http://192.168.14.1"
    export CAS_SURVEY_DATA="$HOME/CAS_B3_R04_Survey_Data"
    export CAS_CAMERA_TOKEN="same-random-token-as-camera-header"
    uvicorn app:app --host 0.0.0.0 --port 8000

Open http://127.0.0.1:8000. If Ubuntu firewall is enabled, allow inbound TCP port 8000 on the Wi-Fi interface.

## Camera protocol

The AI-Thinker ESP32-CAM sends an authenticated HTTP heartbeat once per second containing:
- board ID and firmware version
- camera initialization status, 320x240 QVGA dimensions, JPEG quality and 20 MHz XCLK
- PSRAM detection, free heap, minimum free heap and largest contiguous free block
- Wi-Fi RSSI and camera station IP
- frame sequence and last camera error

When no investigation is active, it uploads an idle preview approximately once per second to /api/camera/preview. The laptop retains only the latest preview image outside the dataset.

During an active session, it polls /api/camera/session and uploads approximately two JPEGs per second to /api/camera/frame. Each survey frame is indexed with session ID, camera board ID, sequence, camera monotonic timestamp, laptop receive time, dimensions, file size, SHA-256 and optional investigation capture event ID. A requested Investigation Capture requests a three-frame association burst.

The camera uses the confirmed AI-Thinker pin map. It is configured for QVGA 320x240 JPEG, quality 12, XCLK 20 MHz, one DRAM frame buffer. No onboard CV is performed; the supplied hardware report says PSRAM was not detected. Tactile-strip analysis is an optional laptop-side OpenCV heuristic and is observational only.

## Survey procedure

1. Power main Box R04 and verify its HMI at http://192.168.14.1.
2. Power R04 rover and wait for RoverLink connected.
3. Power AI-Thinker ESP32-CAM and wait for camera heartbeat plus preview.
4. Start this laptop service while laptop is connected to CAS-Box-04.
5. Wait for the HMI readiness badge to show BOX + ROVER + CAMERA READY.
6. Start Investigation. The box latches rover stop first; it does not move.
7. Enable manual driving explicitly. Manual commands are limited to ±0.25 in the laptop survey interface.
8. Use captures, route segments and annotations. Stop Rover does not end logging.
9. Use Stop Investigation & Finalize Dataset to stop rover motion and create the validated survey ZIP.

Camera loss does not block finalization. The dataset records missing frames and camera diagnostics. If the camera is not connected before session start, the HMI leaves Start Investigation disabled until it reconnects.

## Dataset and integrity

Data is stored on the laptop under CAS_SURVEY_DATA:
- sessions/<session_id>/telemetry/ — complete received sensor snapshots, LiDAR, IMU, GPS, ultrasonic, drive, system, camera diagnostics and connection events.
- sessions/<session_id>/camera/frames/ and frame_index.csv — survey JPEGs and metadata.
- sessions/<session_id>/annotations/ — operator annotations, segments and investigation events.
- sessions/<session_id>/derived/ — optional vision candidate results.
- sessions/<session_id>/integrity/ — data quality report and SHA-256 checksums.
- archives/ — validated CAS_B3_R04_Survey_<session_id>.zip files.

The finalizer stops the rover, closes the active segment, prevents new survey uploads, drains in-flight camera uploads for a bounded period, records unfinished capture bursts, writes checksums and manifest, then reopens the ZIP and verifies its CRC plus every included file's SHA-256 and byte count before reporting ready.

Timestamps keep separate clock domains: ESP32 millis since boot, camera millis since boot, laptop UTC and laptop monotonic receive time. These are not hardware-synchronized. The code does not infer wheel speed or travelled distance because the project has no wheel encoder interface.

## Test

    pip install -r requirements-test.txt
    PYTHONPATH=. pytest -q
    python -m py_compile app.py manager.py analysis.py

Main and camera ESP32 compile jobs are in .github/workflows/r04-survey-checks.yml. CI compilation does not prove physical board upload or camera/network operation.
