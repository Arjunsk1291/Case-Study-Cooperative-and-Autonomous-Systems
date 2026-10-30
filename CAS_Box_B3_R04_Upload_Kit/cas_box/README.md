# CAS Box – starter program

Keep in mind that this is just a testing program for you, it may contain mistakes or inefficient code parts. 
You can use this code, but you do not have to. 

Starter program for the sensor box ("brain") on the WAVE ROVER.
Case Study Cooperative and Autonomous Systems, version 1.0 (2026-10-15).

```
phone ──Wi-Fi──► box (this program): web page, sensors ──Wi-Fi──► rover (motors)
```

The box opens its own Wi-Fi, serves a web page for your phone (drive pad, LiDAR view,
ultrasonic, IMU, GPS) and drives the rover over Wi-Fi. Details: header of `cas_box.ino`.

## 1. Install (once per laptop)

**Arduino IDE 2.x** – https://www.arduino.cc/en/software

**Board package** (Tools → Board → Boards Manager): **esp32** by Espressif Systems,
version **3.3.1** (other 3.3.x should work; 2.x does not).

**USB driver**: the box ESP32 uses a **CH340** USB chip. Windows usually installs the driver
automatically; if no COM port appears, install the CH340 driver from the chip maker (WCH).

**Libraries** (Sketch → Include Library → Manage Libraries). Tested versions:

| Library (name in the Library Manager) | Version | Used for |
|---|---|---|
| ESP Async WebServer (by ESP32Async) | 3.9.2 | web server |
| Async TCP (by ESP32Async) | 3.4.9 | web server |
| Adafruit BNO055 | 1.6.4 | IMU |
| Adafruit Unified Sensor | 1.1.15 | IMU |
| Adafruit SSD1306 | 2.5.16 | OLED |
| Adafruit GFX Library | 1.12.4 | OLED |
| Adafruit BusIO | 1.17.4 | OLED / IMU (installed with the Adafruit libraries) |
| TinyGPSPlus-ESP32 (by Mikal Hart) | 0.0.2 | GPS |

WiFi, HTTPClient, Wire and SPI come with the esp32 board package.
If the Library Manager asks to install dependencies, answer "Install all".

## 2. Flash your box

1. Open `cas_box.ino` (all other files open as tabs).
2. Set `BOX_ID` to your **kit number** (sticker). Change nothing else of the naming scheme.
3. Tools → Board: **ESP32 Dev Module**, Partition Scheme: **default**, Port: the COM port of
   the box.
4. Upload. Serial monitor at **115200 baud** prints a status line every 10 s.

## 3. Use

1. Switch on the **box first, then the rover** (the rover looks for its box only while booting).
2. Phone: join Wi-Fi **`CAS-Box-NN`**, password **`CASboxNN`** (NN = kit number, two digits).
3. Open **http://192.168.(10+N).1** – kit 1: `192.168.11.1`, kit 7: `192.168.14.1`,
   kit 10: `192.168.20.1`.
4. Hold an arrow key to drive, release to stop. The slider sets the speed (max = full speed).

IMU angles use the vehicle frame of ISO 8855 (Euro NCAP): x forward, y left, z up;
roll + = right side down, pitch + = front down, yaw + = turning left (0 = magnetic north).

| Kit | Wi-Fi | Password | Web page | Channel |
|---|---|---|---|---|
| N | `CAS-Box-NN` | `CASboxNN` | `http://192.168.(10+N).1` | odd N: 1, even N: 11 |

The rover also shows its own Wi-Fi `CAS-Rover-NN` – that is the rover's service page, you do
not need it.

## 4. Rules

- **Do not change the rover firmware.** All your work happens on the box.
- Keep the naming scheme: every kit needs its own Wi-Fi name, otherwise teams disturb each other.
- Keep the stop logic (release key, page timeout, RoverLink deadman, rover heartbeat).
- Charge the rover batteries (3x 18650) in time; with low batteries the rover cannot turn.

## 5. Where to start with your own code

`loop()` in `cas_box.ino`, section "driving": replace the drive-pad values by your own
`l`/`r` (range −0.5 … 0.5 per side) computed from the sensors, e.g.
`sensors.usCm(0)` = front ultrasonic in cm. `rover.setDrive(l, r)` must be called every loop.

## Troubleshooting

| Problem | Check |
|---|---|
| State line says "Rover: searching" | Box on before rover? Restart the rover. Correct `BOX_ID`? |
| No Wi-Fi `CAS-Box-NN` | Box powered? Serial monitor: line `[WebHMI] AP ...` |
| No COM port | USB cable with data lines, CH340 driver |
| Sensor shows nothing | Serial monitor JSON line `[sys] {...}`; IMU/OLED start up to 5 s later |

## Ultrasonic diagnostics update (B3 / R04)

The updated box code preserves the existing HC-SR04 GPIO mapping and controller interface.
The web UI now shows the latest raw echo, a median-filtered distance from up to five valid
samples, whether the latest ping succeeded, age of the last successful echo, and cumulative
no-echo/out-of-range count. A failed latest ping still makes the legacy `usCm()` / `read*Cm()`
controller-facing value return `kUsMaxCm` (300 cm), preserving the prior no-echo convention.

The ultrasonic task scans sensors in the same order (front, left, right, back, front angled),
with a 25 ms quiet gap between sensors to reduce cross-talk. This is a starting configuration;
verify it on the actual kit before relying on readings for autonomous control. It does not replace
proper voltage dividers on the 5 V HC-SR04 ECHO outputs.

The diagnostic `Test_Code.ino` now shows raw and median readings in the same way. Its label order
remains 45-degree angled, front, back, left, right. Main CAS Box JSON order remains front, left,
right, back, front angled.

**Team configuration:** `BOX_ID = 4` for Team B3 / Rover R04. This yields SSID `CAS-Box-04`,
password `CASbox04`, web URL `http://192.168.14.1`, and Wi-Fi channel 1. Rover firmware is unchanged.


## Indoor sensor logging (B3 / R04)

The web dashboard includes a browser-side CSV logger. It records the current 90-bin LiDAR snapshot,
ultrasonic raw/median/valid/age/timeout fields, IMU yaw/pitch/roll/calibration, GPS status and
monotonic elapsed time at each dashboard poll (~4 Hz). Use Start recording, enter a waypoint label
(e.g. A, TURN_1, B) and press Mark waypoint while physically at that landmark. Stop and Download CSV
before refreshing/closing the page. Data is held in browser memory; it does not write to ESP32 flash
or require an SD card. Keep initial sessions short (e.g. 1–3 minutes) and download after each run.

Important: this logger captures sensor snapshots; it does not calculate indoor x/y coordinates or
perform SLAM. Indoor waypoint positions need a defined origin and a localization method (e.g. LiDAR
SLAM with odometry, or carefully measured manual waypoints). IMU yaw alone drifts and ultrasonic
ranges alone do not uniquely determine position. For first runs, keep the box stationary at A, mark
A, move to each known landmark, stop, mark it, and record the surroundings. Do not drive autonomously
from these logs yet. Rover stop/deadman logic is unchanged.


## Navigation research logger — B3 / R04

This revision expands browser-side CSV logs with:
- explicit event types and labels (waypoint, rotation, wall, obstacle, start, goal, note)
- operator-entered known facing reference and notes
- raw ultrasonic distance, median distance, validity, last-good age and timeout count
- IMU yaw/pitch/roll and per-axis calibration values
- all 90 available LiDAR distance bins, plus basic descriptive features (valid-bin count, min/median range, adjacent jumps)
- GPS status and timestamps

CSV export uses real CRLF line endings. Data is captured from the dashboard's `/values` endpoint at approximately 4 Hz, not at the LiDAR's native scan rate. Browser logging therefore is suitable for stationary landmark snapshots and slow manual route observations, not high-rate SLAM or reliable trajectory reconstruction. The left/front/right LiDAR sector features are provisional bin groups only; confirm angular bin orientation before using them as physical sectors.

Recommended collection protocol:
1. Keep the rover motors disabled during sensor calibration runs.
2. At each stationary location, wait 5–10 s, set a unique label (BED_A, CORNER_1, WASHROOM_B), select event type, select the independently known facing direction only if verified, enter notes, and click Mark waypoint.
3. For compass validation, mark rotation tests at known 45°/90° increments while stationary between turns. Record whether the heading is expected magnetic or local reference; do not assume yaw is geographic north until calibrated.
4. For wall identification, collect stationary scans facing parallel and perpendicular to a measured wall; note wall distance and orientation.
5. Download after each short run. Do not refresh/close the page before downloading.
6. This log is not a map and does not estimate x/y. With no wheel encoders, odometry will drift. Fully onboard autonomy needs a localization approach such as LiDAR scan matching/SLAM, enough RAM/CPU, a verified map frame, and safety-tested low-level control. Ultrasonic/IMU snapshots alone are not sufficient to infer reliable global waypoints.


## Experimental scan-and-wander controller (B3 / R04, 2026-10-23)

- `Navigation.h` evaluates eight candidate headings from the 90-bin LiDAR scan (bin 0 = forward; bins increase clockwise; 4 degrees/bin).
- If the forward sector is sufficiently clear, it drives a short bounded leg and rescans. If blocked, it selects a clear sector, checks the immediate 45-degree pivot sweep, executes one measured 45-degree turn step, stops/settles, and rescans.
- The pivot uses the maximum configured command (0.50 on the RoverLink -0.5..0.5 scale) until near the target angle, then tapers the command to limit overshoot. BNO055 yaw feedback—not a fixed-duration guess—determines when to stop. A timeout and feedback-mismatch stop remain active.
- Fresh front and angled-front ultrasonic readings participate in the near-field stop check. Side and rear readings can veto turns when valid and fresh. Invalid or missing LiDAR bins are treated as unknown, not free.
- The controller requires a working IMU with gyro calibration >= 2 before it will turn. If that condition is not met, it stops rather than performing a blind high-command pivot.
- This is **local reactive exploration**, not a destination planner. It does not recognize the washroom door semantically, maintain a persistent map, or estimate x/y. A separate mapping/localization and waypoint layer is needed for repeatable bed-to-washroom navigation.
- The dashboard heartbeat, RoverLink deadman and manual STOP remain active. Autonomy is OFF at boot.

### Mandatory test procedure

1. Compile in Arduino IDE with the target ESP32 core and installed libraries before flashing. The Navigation header was syntax-checked against a lightweight C++ stub, but the full firmware has not been compiled with the exact Arduino toolchain.
2. With the rover raised so wheels cannot touch the ground, verify manual forward/left/right motor polarity. Confirm that left pivot produces increasing IMU yaw and right pivot produces decreasing yaw.
3. With motors disabled, place an obstacle at known front/left/right/rear positions and confirm the 90-bin LiDAR direction mapping. Confirm the angled-front ultrasonic physical orientation too.
4. Test one 45-degree pivot with the wheels lifted and compare measured yaw change. If the sign is wrong, stop and correct motor polarity / yaw convention before floor tests.
5. In a taped-off, empty, level area, run one short supervised wander trial with an operator holding the STOP control. Keep away from people, pets, stairs, glass, fragile items and the luggage shown in the room photos.
6. Export diagnostics after each run. Do not attempt unsupervised bed-to-washroom operation until turning, stopping distance and obstacle avoidance are validated.

The 0.50 pivot command is the maximum configured command, not a guaranteed angular velocity. Battery voltage, floor friction, payload, motor controller limits and wheel geometry affect the actual turn. No wheel encoders or independent physical-motion feedback are available.


## Diagnostics and current navigation revision

The dashboard includes a troubleshooting panel and **Download diagnostic JSON**. The export includes the latest named firmware values, the complete 90-bin LiDAR array, ultrasonic raw/filtered/valid/age/timeout arrays, IMU/GPS values, browser timestamps, autonomy state, loop timing, and optional operator notes. Export immediately after a supervised test while the rover is in the state being investigated.

The current experimental controller is **scan-and-wander**, not surface-following. It uses short bounded motion legs, all-around LiDAR sector selection, IMU-feedback 45-degree pivot steps and a stop/settle/rescan cycle. The firmware is not safety-certified and has not been compiled against the exact target Arduino toolchain or tested on the physical rover.

Safety notes:
- Autonomy starts OFF. Keep it OFF while verifying compilation and manual drive.
- Do not run near stairs, people, pets, glass, traffic, fragile objects or outdoor drop-offs.
- The rover firmware is unchanged. This package changes only the sensor-box ESP32.
- If commanded speeds are nonzero but the wheels do not move, inspect rover power/battery, motor/driver, wheel binding, rover link/IP and the rover's own firmware. Do not keep increasing speed blindly.
- No wheel encoders means the box cannot verify physical movement or accurately measure distance travelled. A bounded timed leg is only a short motion command, not an odometry estimate.
- This revision has not been compiled with the exact Arduino ESP32 toolchain or tested on hardware. Verify in Arduino IDE before uploading.

## Guided investigation mode (B3 / R04)

The investigation panel uses a guided state sequence: start-location scan, travel logging,
arrival scan for each waypoint, then a final destination scan and JSON export. Scan durations are
selectable (10/15/20 seconds); the timer is visible only while that scan is active. During travel,
the browser stores each received dashboard sensor frame at the dashboard poll rate (nominally
4 Hz), including all 90 LiDAR bins, ultrasonic data and freshness, IMU, GPS, diagnostics values,
cycle timing and snapshot-age metadata. The JSON includes explicit scan/travel/arrival event types.

FreeRTOS remains in use for time-sensitive firmware work: the ultrasonic manager has its own task
and RoverLink runs its network I/O task on core 0. The investigation history is intentionally kept in
browser memory rather than adding a large, unbounded JSON buffer to ESP32 RAM. Keep autonomy OFF;
travel is manually supervised. The logger does not automatically spin the rover. If safe, an operator
may slowly rotate it during a scan with the drive pad, pausing first so stationary and changing-view
frames are both captured. A LiDAR alone does not guarantee return-to-start: that requires validated
scan matching/localization and a closed-loop controller, neither of which this data logger implements.
Download the JSON before refreshing/closing the dashboard.

## Survey firmware additions (branch survey/r04-investigation-2026-10-30)

- The /values response now includes a top-level box_response_device_ms timestamp and the existing sensor snapshot sequence/device time.
- The sensor JSON includes the existing 90-bin chassis-frame LD20 summary plus filtered_1deg_mm[360]. This 360-bin rolling-filtered map is the parser's current distance map; it is not a raw packet capture and has no intensity/native point timestamps. The 90-bin display applies the configured vehicle/LiDAR angular offset during snapshot conversion; the 360-bin array is the parser map before that additional snapshot remap.
- BNO055 raw acceleration (m/s²), gyro (rad/s), magnetic field (µT) and temperature (°C) are included with a validity flag. Raw vectors are sampled on the existing ~100 ms IMU update, not synchronized exactly to the 250 ms sensor snapshot.
- GPS decoded altitude, speed, course and UTC date/time are included only with separate validity flags. Raw NMEA sentences are not retained.
- Snapshot sequence increments on each 250 ms Sensors snapshot. The laptop compares sequence numbers to estimate lost snapshots.
- GET /clock returns the main ESP32 millis() clock value. This is a device uptime clock, not UTC.

### Latched software stop interlock

- POST /safety/stop latches the software stop, disables autonomy/heartbeat, zeros the requested drive setpoint and blocks /drive.
- POST /safety/clear clears the latch but leaves the setpoint at zero. A new explicit drive command is still required to move.
- GET /safety/state reports the latch.
- /drive returns HTTP 423 while stop is latched or autonomy is active.
- The main loop applies a final zero-command check immediately before RoverLink.setDrive().

This is a software stop over the CAS-Box-04 Wi-Fi link. It is not a physical E-stop input or a substitute for a hardwired emergency stop/power disconnect. The existing RoverLink 500 ms deadman is retained.

Survey manual commands are issued through the laptop collector and capped at ±0.25. The underlying RoverLink interface and rover firmware protocol are unchanged. No measured wheel speed/distance is available because the project has no encoder input.
