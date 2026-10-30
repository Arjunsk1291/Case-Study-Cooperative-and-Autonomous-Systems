# AI-Thinker ESP32-CAM — R04 firmware

Open CameraStation.ino in Arduino IDE. The sketch uses the confirmed AI-Thinker ESP32-CAM / OV2640 GPIO map directly.

Before compiling, edit camera_user_config.h:
- CAS_CAMERA_LAPTOP_HOST: laptop's IPv4 address while joined to CAS-Box-04.
- CAS_CAMERA_TOKEN: exactly the same random token exported as CAS_CAMERA_TOKEN in the laptop terminal.
- SSID/password are already set for R04: CAS-Box-04 / CASbox04.

Camera settings preserve the verified working configuration: QVGA 320x240, JPEG quality 12, XCLK 20 MHz, one DRAM frame buffer. The user's diagnostics report no PSRAM, so the code does not allocate or queue multiple images. Every captured framebuffer is returned after the upload attempt.

On boot the camera connects to CAS-Box-04 and sends a heartbeat to the laptop every second. While no survey session is active it uploads a preview JPEG once per second. During an active investigation it uploads two JPEGs per second, carrying session and capture-event identifiers. Images go directly to the laptop; the main ESP32 never relays JPEG bytes.

The camera also reports free heap, minimum free heap, largest contiguous allocation, PSRAM detection and Wi-Fi RSSI. This is separate from the main rover stop/control path.
