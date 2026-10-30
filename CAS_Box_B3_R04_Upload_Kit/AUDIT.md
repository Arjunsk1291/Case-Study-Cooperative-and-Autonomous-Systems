# CAS Box B3 / Rover R04 Phase 0 Audit

## Status and evidence

- Workspace branch: `rebuild/cas-box-linear-history`.
- The original `cas_box/` project remains the earlier R07-oriented source.
- This directory is the R04 deliverable kit and is treated as the implementation baseline.
- Laptop software tests: **Software-tested — 8 passed**.
- Firmware compilation: **Compiled — not independently reproduced in this environment**.
- Board flashing, Wi-Fi association, camera relay, rover motion, and physical stop behavior: **Hardware-tested — not performed**.

## Runtime topology

```text
                         live control and telemetry
Phone/HMI ───────────────► Main ESP32 Box B3/R04 ── RoverLink HTTP ──► Rover R04
                              ▲       │
                              │       └── /camera/snapshot relay ──► Phone JPEG download
                              │
                         Camera ESP32-CAM ── snapshot ──► Main ESP32 ──► Phone

Phone: live investigation dashboard, persistent local recorder, metadata aligner, and ZIP exporter.
Laptop: optional later analysis destination only; it is not required during investigation.
```

The box AP is `CAS-Box-04`, password `CASbox04`, at `192.168.14.1`. These are project
configuration values; physical network operation is not verified here.

## Main ESP32

- Arduino sketch: `cas_box/cas_box.ino`; ESP32 Dev Module target.
- AP/HMI and sensors run in the box firmware; RoverLink network I/O runs in a FreeRTOS task.
- RoverLink sends `T:1` left/right commands every 100 ms, uses a 400 ms HTTP timeout,
  searches AP stations every 2 s, declares ten consecutive failures as lost, and applies a
  500 ms setpoint deadman. The rover firmware is unchanged.
- Manual HMI commands are bounded to the configured drive maximum and expire after the
  existing page timeout. Safety stop latches drive inhibition and zeros the command.
- The main ESP32 remains the motor/sensor/safety authority. The laptop uses its existing HTTP
  telemetry and stop interfaces; it does not bypass RoverLink or the rover deadman.
- The camera uploads validated JPEGs to the laptop service only during requested capture bursts.
- The laptop writes telemetry and image records continuously to persistent session directories.

## Sensors and pins

| Device | Model/interface | Verified project configuration |
|---|---|---|
| LiDAR | LD20, Serial1 | RX 27, TX 33, 230400 baud; 360 one-degree bins reduced to 90 four-degree values |
| Ultrasonic | Five HC-SR04 | Trigger/echo: (32,25), (4,18), (13,19), (5,14), (23,26); 17.5 ms timeout; 5-sample median |
| IMU | BNO055, I2C | Address 0x28; SDA 21, SCL 22; calibration and fused orientation exposed |
| GPS | GY-GPS6MV2 / NEO-6M, Serial2 | RX 16, TX 17, 9600 baud; TinyGPSPlus |
| OLED | SSD1306 | I2C address 0x3C; optional and non-blocking |

No wheel encoders are present in the inspected project. Motion distance and wheel speed
must therefore remain explicitly unavailable.

## Camera

- Board: AI-Thinker ESP32-CAM / OV2640.
- Pins are the supplied AI-Thinker map; they are not available for other I/O.
- QVGA 320x240 JPEG, quality 12, 20 MHz XCLK, one internal-D RAM frame buffer.
- Camera joins the box AP and serves `/snapshot` plus `/status`.
- It registers its DHCP address with the box; no laptop IP, camera-to-laptop upload, or
  continuous preview loop is required.
- The camera sends original JPEG bytes only when the phone requests a capture through the box.

## HMI and logging

The phone-hosted HMI provides Start Investigation, continuous local recording, manual drive,
Stop Rover, capture, annotations, route segments, wall/centreline decisions, and Stop & Finalize.
IndexedDB stores telemetry/events/images locally; browser ZIP generation creates the export.
Missing or failed captures are marked missing rather than substituted.

The laptop-side service is now offline processing support. It must not be required for live
camera readiness, live driving, or motor safety. Its output to the robot is limited to compact,
validated navigation instructions; images and raw logs do not return to the robot.

## Pin conflict and network findings

The camera uses GPIO 0, 5, 18, 19, 21, 22, 23, 25, 26, 27, 32, 34, 35, 36, and 39 for
OV2640/control. The main box has a separate board and its sensor mapping above; no wired
camera-to-box GPIO link is used. The box is the AP and HTTP server for HMI, RoverLink
discovery, camera registration, and camera relay.

## Gaps and risks

- Full Arduino compile and physical board tests remain outstanding.
- The relay currently bounds a complete QVGA JPEG in a temporary `String`; it does not persist
  it, but a future implementation should replace this with a true chunked response if larger
  frames are enabled.
- Phone archival is not durable unless the operator saves each downloaded image.
- The original repository still contains historical R07 source; this R04 kit is the scoped
  deliverable and must be used for flashing.

## R07 residue policy

R04 runtime identity is used for the flashable kit. Historical route/source labels are retained
only where needed to preserve provenance; user-facing runtime identifiers and live network
configuration must be R04. A final repository-wide residue check is required before release.

## Literature inventory

The supplied ESP32-WROOM, AI-Thinker ESP32-CAM, NEO-6, BNO055, LD20, HC-SR04, equipment
handover, course-guideline, presentation, and example-code archives were inventoried. Datasheet
facts above are marked VERIFIED when reflected in the source configuration; board-specific
network behavior and all physical wiring remain ASSUMED until tested on the actual kit.
