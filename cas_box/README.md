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
3. Open **http://192.168.(10+N).1** – kit 1: `192.168.11.1`, kit 7: `192.168.17.1`,
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

## Ultrasonic diagnostics update (B3 / R07)

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

**Team configuration:** `BOX_ID = 7` for Team B3 / Rover R07. This yields SSID `CAS-Box-07`,
password `CASbox07`, web URL `http://192.168.17.1`, and Wi-Fi channel 1. Rover firmware is unchanged.


## Indoor sensor logging (B3 / R07)

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
