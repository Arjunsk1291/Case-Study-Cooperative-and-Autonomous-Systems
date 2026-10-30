# AI-Thinker ESP32-CAM — R04 camera storage node

## Correct live architecture

- The main CAS Box ESP32 creates CAS-Box-04 / CASbox04 at 192.168.14.1.
- The AI-Thinker ESP32-CAM joins that access point as a Wi-Fi station.
- The phone also joins the same access point and uses the main ESP32 dashboard.
- The main ESP32 sends a small capture request to the camera and receives camera health/image metadata only.
- The camera writes original JPEG bytes directly to its own onboard microSD card.
- No JPEG payload is sent through the main ESP32, laptop or rover-control link.

The camera does not need a laptop IP, laptop server, shared token, preview receiver or permanent laptop connection.

## Required storage

Insert a FAT32 microSD card in the AI-Thinker ESP32-CAM card slot before powering the board. The firmware mounts the card in 1-bit SD mode and writes files under /captures/. If the card is missing, unmountable or low on space, the main dashboard disables image capture.

The camera board's card slot is used for images because the main ESP32 firmware has no configured persistent image storage. This avoids large JPEG buffers in the main ESP32's RAM/flash.

## Configure and flash

1. Copy camera_user_config.example.h to camera_user_config.h in this folder.
2. The example already contains the R04 AP address, SSID and password. No laptop address or token is needed.
3. Open CameraStation.ino in Arduino IDE.
4. Select AI Thinker ESP32-CAM and the camera's serial port.
5. Compile and upload. A USB-to-serial adapter or ESP32-CAM-MB programmer is required for the initial flash.
6. Open Serial Monitor at 115200 baud. Check for camera initialization, microSD mount, Wi-Fi connection and the archive URL.

## Operation

Open the main dashboard at http://192.168.14.1 from the phone. The R04 camera panel displays camera/SD readiness and provides Capture 3 JPEG frames. The camera polls the main ESP32 for requests and saves each frame as an original JPEG on microSD.

The dashboard also displays the camera's DHCP IP and an Open camera archive link. The archive page is served by the camera on port 81 and provides individual JPEG downloads plus manifest.csv.

The camera is event-driven. It does not stream preview images and does not continuously upload photos.

## Camera settings

- AI-Thinker ESP32-CAM with OV2640
- Confirmed pin mapping in CameraStation.ino
- QVGA 320x240 JPEG
- JPEG quality 12
- XCLK 20 MHz
- One framebuffer in internal DRAM; PSRAM is not assumed
- SD_MMC 1-bit mode

## Data flow after a survey

1. Download the sensor/route JSON or CSV from the phone dashboard.
2. Open the camera archive link and download manifest.csv and the requested JPEGs.
3. Keep these files together as the survey log. They can be packaged by laptop_survey for upload.
4. Provide the log and original images for offline analysis.
5. The analysis result is a small route-plan JSON. The phone dashboard can send that compact plan to the main ESP32's plan mailbox.
6. The current firmware stores the plan as a draft only. It does not execute route motion because indoor localization/odometry is not yet validated.

No laptop process participates in live robot control.
