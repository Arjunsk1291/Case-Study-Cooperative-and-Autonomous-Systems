# AI-Thinker ESP32-CAM — R04 firmware

The camera joins the main ESP32 AP and uploads authenticated heartbeat and JPEG data to the
laptop investigation service. The laptop writes original JPEGs to persistent session storage.
The camera does not process images or command the rover.

Open CameraStation.ino in Arduino IDE. The sketch uses the confirmed AI-Thinker ESP32-CAM / OV2640 GPIO map directly.

Before compiling, copy camera_user_config.example.h to camera_user_config.h and set the laptop
IPv4 address plus a shared random token. Keep the R04 SSID/password and box address unchanged.

Camera settings preserve the verified working configuration: QVGA 320x240, JPEG quality 12, XCLK 20 MHz, one DRAM frame buffer. The user's diagnostics report no PSRAM, so the code does not allocate or queue multiple images. Every captured framebuffer is returned after the upload attempt.

On boot the camera connects to CAS-Box-04 and sends a heartbeat to the laptop. During an active
investigation it uploads requested JPEG capture frames directly to the laptop service. The main
ESP32 is not used as an image relay, and camera failure cannot command rover motion.

The camera also reports free heap, minimum free heap, largest contiguous allocation, PSRAM detection and Wi-Fi RSSI. This is separate from the main rover stop/control path.
