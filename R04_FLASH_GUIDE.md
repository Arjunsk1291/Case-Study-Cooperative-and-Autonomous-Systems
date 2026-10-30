# CAS Box B3 / Rover R04 — Firmware Flash & Survey Setup

## Exactly which two boards to flash

This package contains two independent Arduino sketches. Flash them separately:

| Target | Folder to open | Arduino board selection | Role |
|---|---|---|---|
| Main CAS Box ESP32 | cas_box/cas_box.ino | ESP32 Dev Module | Wi-Fi access point, sensors, RoverLink, motor commands, safety interlock and Web HMI endpoints |
| Camera board | camera_s3/CameraStation/CameraStation.ino | AI Thinker ESP32-CAM | OV2640 camera, Wi-Fi station, preview JPEGs, survey JPEG uploads and camera health heartbeat |

**Do not flash the camera sketch onto the main ESP32. Do not flash the main cas_box sketch onto the AI-Thinker camera board.**

The rover's motor-controller firmware is not one of the two flash targets here. The R04 rover must already be configured to join the R04 box network. The inspected project keeps the rover-side firmware/protocol unchanged.

## R04 network identity

The main firmware has BOX_ID = 4. It derives:

- Wi-Fi SSID: CAS-Box-04
- Wi-Fi password: CASbox04
- Main ESP32 box/HMI address: 192.168.14.1
- Laptop survey receiver: port 8000 on the laptop's address within 192.168.14.0/24
- Camera board: DHCP address assigned by the box AP

The camera and laptop are clients of the main ESP32 access point. The main ESP32 and camera do not use a wired UART or I²C link between boards.

## How all three connect at startup

1. Power the main CAS Box ESP32. It starts the CAS-Box-04 AP and the existing RoverLink search/task.
2. Power the R04 rover. It joins the matching AP; the main ESP32 reports rover.connected=true when RoverLink has discovered it.
3. Power the AI-Thinker ESP32-CAM. It joins CAS-Box-04 as a Wi-Fi station, then sends a heartbeat to the laptop every second and an idle preview JPEG approximately once per second.
4. Connect the laptop to CAS-Box-04, start the laptop survey service, and open the survey HMI.
5. The HMI separately shows box connectivity, rover connectivity and camera heartbeat/camera-ready status. **Start Investigation is enabled only when all three are ready.**

The main ESP32 does not wait synchronously for the camera or laptop. RoverLink and camera Wi-Fi association run independently. A camera or laptop failure therefore cannot block the main ESP32 motor/safety loop. If any link is missing, the HMI reports which one is not ready.

During an active investigation, the camera changes from idle preview to approximately 2 frames/second. Every JPEG is sent directly to the laptop; the main ESP32 never relays JPEG payloads.

---

# 1. Flash the main ESP32 (CAS Box)

### Software

Install Arduino IDE 2.x and Espressif Arduino-ESP32 core 3.3.1. The R04 branch CI also compiles this sketch with that core.

Install these libraries through Arduino IDE Library Manager:

- ESP Async WebServer
- Async TCP
- Adafruit BNO055
- Adafruit Unified Sensor
- Adafruit SSD1306
- Adafruit GFX Library
- Adafruit BusIO
- TinyGPSPlus-ESP32

### Upload

1. Open cas_box/cas_box.ino in Arduino IDE. Keep the full cas_box folder intact because the sketch includes the other .h and .cpp files in that folder.
2. Connect the main ESP32 development board to the laptop using its USB cable or its existing USB-to-UART adapter.
3. Select Tools → Board → ESP32 Arduino → ESP32 Dev Module.
4. Select the correct serial port.
5. Select a reliable upload speed (115200 is a conservative starting point).
6. Compile, then Upload.
7. Open Serial Monitor at 115200 baud.
8. Confirm that the box starts its AP and RoverLink begins searching for the rover.
9. Connect the laptop to CAS-Box-04 with password CASbox04.
10. Open http://192.168.14.1 to confirm the box HMI responds.

This does not flash the rover's motor-controller board. Do not change the rover firmware unless the R04 rover is not configured for this SSID/password; verify that separately before attempting a campus run.

---

# 2. Configure and flash the AI-Thinker ESP32-CAM

### Confirmed camera settings

This sketch uses the confirmed AI-Thinker pin mapping and preserves the already-tested configuration:

- Camera: AI-Thinker ESP32-CAM / OV2640
- Output: JPEG
- Resolution: QVGA 320 × 240
- JPEG quality: 12
- XCLK: 20 MHz
- Frame buffers: 1
- Frame-buffer location: internal DRAM
- No PSRAM is required

The camera pins are assigned by the sketch; they are not available for other peripherals:

| Camera signal | GPIO |
|---|---:|
| PWDN | 32 |
| RESET | -1 (not connected) |
| XCLK | 0 |
| SIOD / SCCB SDA | 26 |
| SIOC / SCCB SCL | 27 |
| Y9 / D7 | 35 |
| Y8 / D6 | 34 |
| Y7 / D5 | 39 |
| Y6 / D4 | 36 |
| Y5 / D3 | 21 |
| Y4 / D2 | 19 |
| Y3 / D1 | 18 |
| Y2 / D0 | 5 |
| VSYNC | 25 |
| HREF | 23 |
| PCLK | 22 |

### Configure laptop IP and shared token first

1. Connect the laptop to CAS-Box-04.
2. On Ubuntu, run:

    ip -4 addr

3. Find the IPv4 address on the Wi-Fi interface connected to CAS-Box-04. It should be in the 192.168.14.x subnet. **Do not use 192.168.14.1; that is the box.**
4. Generate one long random camera token:

    python3 -c 'import secrets; print(secrets.token_urlsafe(32))'

5. Open camera_s3/CameraStation/camera_user_config.h and set:
   - CAS_CAMERA_LAPTOP_HOST to the laptop IPv4 address you just found.
   - CAS_CAMERA_TOKEN to the generated token.
   - Keep the R04 SSID/password unchanged (CAS-Box-04 / CASbox04).

6. In the laptop terminal, export exactly the same token:

    export CAS_CAMERA_TOKEN='paste-the-same-token-here'

The token must match byte-for-byte in both places. Do not upload/commit a real token to GitHub.

### Upload the camera sketch

1. Open camera_s3/CameraStation/CameraStation.ino in Arduino IDE. Keep camera_user_config.h in the same folder.
2. Select Tools → Board → ESP32 Arduino → AI Thinker ESP32-CAM.
3. Select the camera's serial port.
4. Use Serial Monitor at 115200 baud.
5. Upload the sketch.

**Programming access limitation:** the standard AI-Thinker ESP32-CAM has no native USB connector. A USB-to-serial adapter or ESP32-CAM-MB programmer is required. If the programmer board handles download mode automatically, use its USB connection. With a bare USB-to-serial adapter, the usual connections are adapter TX → U0R/GPIO3, adapter RX → U0T/GPIO1, common GND, and a suitable 5 V supply to the camera 5V input. GPIO0 must be held low during reset to enter serial bootloader mode. These are physical programming connections; they cannot be performed remotely by changing firmware. **If you currently cannot access the camera board/programmer or its USB connection, the camera sketch can be prepared but cannot be flashed until that physical access is available.** The supplied firmware does not yet implement OTA bootloader updates.

6. After upload, open Serial Monitor at 115200 baud. Expected messages include:
   - camera initialized AI-Thinker OV2640
   - Wi-Fi connected to CAS-Box-04
   - camera IP and laptop URL
   - periodic memory diagnostics
   - heartbeat success (HTTP 200 is silent; lack of laptop response prints a retry warning)

If the camera fails to join Wi-Fi, check the R04 SSID/password and laptop/box network. If the camera joins but never becomes ready on the HMI, verify the laptop IPv4 address and shared token in both configurations.

---

# 3. Run the laptop receiver and Web HMI

Keep the laptop connected to CAS-Box-04. Open a terminal:

    cd laptop_survey
    python3 -m venv .venv
    source .venv/bin/activate
    pip install -r requirements.txt
    pip install -r requirements-vision.txt

Export the box URL, storage directory and the same camera token used in the camera sketch:

    export CAS_BOX_URL="http://192.168.14.1"
    export CAS_SURVEY_DATA="$HOME/CAS_B3_R04_Survey_Data"
    export CAS_CAMERA_TOKEN="paste-the-same-token-here"
    uvicorn app:app --host 0.0.0.0 --port 8000

Open http://127.0.0.1:8000 in the laptop browser. The camera sends preview frames to http://<laptop-ip>:8000 while idle. If Ubuntu's firewall is enabled, allow inbound TCP port 8000 on the Wi-Fi connection.

## Before starting a survey

Check the HMI connection badge and camera panel:

- Box: online (the HMI polls 192.168.14.1)
- Rover: connected (reported by RoverLink on the main ESP32)
- Camera: heartbeat recent and camera_ready=true; the preview should update
- Laptop storage: free space available

Start Investigation remains disabled until box + rover + camera are ready. The camera does not need to be physically wired to the main ESP32.

---

# 4. What the system does

| State | Camera action | Dataset effect |
|---|---|---|
| Idle / no investigation | Heartbeat every second; preview about 1 FPS | Only the latest preview is held on the laptop; no survey files are created |
| Investigation active | Heartbeat every second; JPEG at about 2 FPS | JPEG frames and index rows are saved under the active session |
| Investigation Capture | Server requests a three-frame capture association | Frames carry the capture event ID and are linked in the image index |
| Camera goes offline | Heartbeat/frame timeout is shown | Main ESP32 safety/control continues; finalization remains available |
| Stop Rover | Main ESP32 software stop latch is requested | Sensor and camera logging continue |
| Stop Investigation & Finalize Dataset | Rover stop first; camera uploads are drained for a bounded period | Laptop builds, CRC-checks and SHA-256-checks the survey ZIP |

The camera sends JPEG directly to the laptop, not through the main ESP32. The main ESP32 continues controlling the rover and sampling sensors independently.

---

# 5. Build and verification status

The R04 CI workflow compiles:
- Main box sketch: esp32:esp32:esp32 (ESP32 Dev Module)
- Camera sketch: esp32:esp32:esp32cam (AI Thinker ESP32-CAM)
- Python syntax and laptop backend tests

A successful CI compile proves source/toolchain compatibility only. It does not prove that firmware has been uploaded to your boards, that the R04 rover joins this AP, that the camera is physically connected, or that image transfer works on your network. Those still need your actual boards and laptop.

## Important safety boundary

The laptop's Stop Rover control is a Wi-Fi software stop. It is not a hardwired emergency-stop circuit. Keep the physical stop/power disconnect accessible. The camera has no authority to command rover movement, and camera failure must never be treated as a rover-safe-stop mechanism.
