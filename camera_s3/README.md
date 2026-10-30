# ESP32-S3 camera survey transmitter

## Selected data path

The ESP32-S3 camera joins the existing CAS-Box-04 Wi-Fi AP as a station and uploads JPEG frames directly to the laptop FastAPI service:

ESP32-S3 camera → Wi-Fi station → CAS-Box-04 AP → laptop POST /api/camera/frame.

JPEG payloads do not pass through the main ESP32. The main ESP32 continues sampling sensors and sending motor commands through its existing RoverLink task.

## Hardware configuration blocker

The inspected repository does not identify the ESP32-S3 camera board model, camera sensor/module or GPIO pin map. Several ESP32-S3 camera boards have incompatible pinouts. Therefore camera_station.ino intentionally refuses to compile until camera_board_config.h contains the exact, verified board mapping.

Do not copy pins from another camera board. camera_board_config.template.h is a stub, not a valid hardware configuration.

Required information:
- Exact ESP32-S3 board/module and camera sensor (for example OV2640, if confirmed).
- Manufacturer pin map for PWDN, RESET, XCLK, SIOD, SIOC, D0-D7, VSYNC, HREF and PCLK.
- Laptop IP address assigned by the CAS-Box-04 AP (use ip addr on Linux while connected to CAS-Box-04).
- A shared random upload token.

## Laptop API protocol

The camera transmitter polls:
- GET /api/camera/session
- GET /api/camera/commands
- Optional time probe GET /api/camera/time

It uploads raw JPEG bytes:
- POST /api/camera/frame
- Content-Type: image/jpeg
- X-Camera-Token
- X-Session-Id
- X-Frame-Id
- X-Frame-Sequence
- X-Camera-Board-Id
- X-Camera-Monotonic-Ms
- X-Capture-Event-Id when the laptop requests an investigation capture burst.

The current sketch targets 2 frames/s initially. Measure actual transfer rate and memory pressure before raising it. The laptop service limits images to 3 MiB, validates JPEG markers and dimensions, and stores them outside the main ESP32.

## Clock handling

The camera provides its monotonic millis() timestamp and the laptop records receive UTC and monotonic time. The camera timestamp is not UTC. Do not infer camera-to-box synchronization without a measured clock-offset exchange.

## Build

Install the Espressif ESP32 Arduino core and a version of esp32-camera compatible with the selected board/core. Add the verified camera_board_config.h, then select the exact ESP32-S3 board in Arduino IDE. This sketch cannot be reported as compiled until the board identity and pin mapping are supplied.

Camera loss does not stop the main box's sensor/motor loops. The laptop can finalize a survey even if the camera is disconnected.
