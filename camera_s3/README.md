# R04 camera firmware

The flash-ready camera sketch is in:

- CameraStation/CameraStation.ino
- CameraStation/camera_user_config.h

Target: AI-Thinker ESP32-CAM with OV2640 (classic ESP32, not ESP32-S3).

The sketch uses the confirmed AI-Thinker GPIO map directly and connects to CAS-Box-04. It sends:
- Camera heartbeat (camera ready, board ID, Wi-Fi RSSI/IP, free heap, minimum free heap, largest free block and PSRAM state) every second.
- Idle preview JPEG at approximately 1 FPS.
- Survey JPEG at approximately 2 FPS while an investigation is active.
- Capture event ID on the requested three-frame capture burst.

JPEGs go directly to the laptop FastAPI service. The main CAS Box ESP32 does not relay image bytes and does not depend on the camera for drive control or safety.

The selected camera memory configuration matches the supplied working setup: QVGA 320x240, JPEG quality 12, XCLK 20 MHz, one frame buffer in internal DRAM. The user-reported runtime diagnostics showed no PSRAM, so this sketch does not rely on PSRAM.

Set the laptop IPv4 address and shared camera token in camera_user_config.h before compiling. The laptop and camera must both join CAS-Box-04. The exact two-board flash sequence and physical programming notes are in the repository root file R04_FLASH_GUIDE.md.

The camera must initially be programmed via USB/serial or a compatible ESP32-CAM programmer board. OTA firmware update is not included in this baseline.
