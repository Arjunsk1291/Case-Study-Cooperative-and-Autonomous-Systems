# Open-sea mission - what to do (kit 04, 2026-11-07, version 2 after the campus run)

## What changed after the campus run (mis_001)
| seen on campus | cause (from the log) | fixed |
|---|---|---|
| drift right along B, drift left on the open sea | LiDAR mounted 7 deg rotated (same on log_027): "parallel" was 7 deg off, and the fake "crab" was compensated on the open sea | LiDAR offset 83 instead of 90; crab prior 0 |
| right turn failed | full pivot held for 10 s jammed after 7 deg | turns in 1.5 s bursts, rest, swing turn after 2 stalls |
| drove to ~5 cm from the end wall | open sea counted as "slope" -> end wall never accepted | end wall accepted on slopes; < 0.45 m = immediate stop |
| Resume after manual fought your steering | old heading restored | your current heading becomes the straight line |
| strip never confirmed | "2 of the last 3 frames" rule from the stream mode | each picture judged alone, wider strip allowed |
| no pictures to analyse | - | every snapshot picture is saved (img_NNN.bin) and shown on the phone |
| slow | speed 0.20 | 0.25 |
**Flash BOTH boards again** (camera first). Send me mis_NNN.bin AND img_NNN.bin after the run.

Two zip files:
- **1_BOX_firmware.zip** -> the box ESP32 (USB adapter "CP2102", the box's own port)
- **2_CAMERA_firmware.zip** -> the ESP32-CAM (USB adapter "FT232R" / FTDI)

Each zip has a ready **.bin** file (flash it - nothing to install) and the source code (only if you want the Arduino IDE).
Status: compiled and tested on the laptop (logic replayed on logs 026/027, dashboard tested in a browser, log
format checked byte by byte). **Not yet run on the rover** -> do the bench check (part B) first.

---------------------------------------------------------------------------------------------------
## A. Flash (friend's PC, Chrome or Edge, nothing to install) - camera first, then box
1. Open **https://espressif.github.io/esptool-js/**
2. **Baudrate: 115200** (not faster). Press **Connect**, choose the port:
   camera = "FT232R" / "USB Serial Port"; box = "CP2102" / "Silicon Labs".
   (Windows asks for a driver only if the port does not appear: box = Silicon Labs CP210x, camera = FTDI VCP.)
3. Flash Address **0x0**, choose the file:
   camera: `CAMERA_link_cam_0x0.bin` - box: `BOX_cas_box_mission_0x0.bin`
4. Press **Program**. Do **NOT** press "Erase Flash". Wait until it says done / "Hard resetting".
5. If it fails ("serial noise", "timed out"): unplug, plug in again, Connect again - always 115200.
The files end right after the program: the box's log storage is not touched.

(Arduino IDE instead: box = "ESP32 Dev Module", Partition **Default**, Flash Frequency **40 MHz**, Upload Speed
**115200**, Erase All Flash **Disabled**; libraries: ESP Async WebServer 3.9.2, Async TCP 3.4.9, Adafruit BNO055,
Adafruit Unified Sensor, Adafruit SSD1306, Adafruit GFX, TinyGPSPlus-ESP32. The .bin route is easier.)

---------------------------------------------------------------------------------------------------
## B. Bench check at home - 5 minutes, ROVER WHEELS IN THE AIR (on a box)
1. Camera on its own power bank. Box on, then rover. Phone: Wi-Fi `CAS-Box-04` (password `CASbox04`),
   open `http://192.168.14.1` -> page "Mission: B -> open sea".
2. Top chips green: Rover, IMU, LiDAR, **Camera paired** (within ~5 s). GPS may show 0 sat (known hardware issue).
3. Press **Start open sea HERE** -> it takes 3 pictures -> question -> press **Go straight**.
   The wheels turn gently (soft start). Press **STOP** -> "PAUSED" -> **Resume** -> wheels again.
   Hold a manual button -> "you took over". Press **Finish (save log)**.
   Card "Camera: last picture" must show a picture after the 3 snapshots (upright = the camera works).
4. In "Log": a file `mis_001.bin` appears -> **download** it. Send it to me if anything looked wrong.
5. Then **Delete all logs** (after downloading!).

---------------------------------------------------------------------------------------------------
## C. Outdoor run: B facade -> right turn -> across the open sea
Before: rover battery charged (>= 12.0 V on the page), camera power bank charged, phone charged.
Settings (page, "Settings"): speed 0.25, ask for the turn after 20 m, open sea 36 m, stop before end wall 1.5 m,
camera may steer ON, decide alone OFF. (These are the defaults.)

1. Put the rover at the **start of the B facade** (where the teaching drive started), **wall on the LEFT**
   about 1.3 m away, **parallel** to the wall. Phone screen ON, page in front, stay **behind** the rover
   (anything in front closer than 0.6 m = it stops and waits until clear - that is the people rule).
2. Press **Start mission (leg 1)**. It follows the wall, counts pillars, measures its sideways slide (crab).
3. At ~20 m it **stops and asks**: "Is this the right turn?"
   - at the end of the pillar row, plaza opening on the right -> **Turn RIGHT here**
   - not there yet -> **Drive on 1 m** (as often as needed)
   - already past it -> **Manual**: drive back, then **Start open sea HERE** (after turning yourself)
4. After the turn: 3 pictures. "Strip in N/3 pictures" -> **Align on the strip & go**; no strip -> **Go straight**.
5. It drives the open sea (~36 m, ~3 min), camera check every 2.6 m (if it fails it just continues).
   If it heads somewhere wrong (an edge, the steps): hold a **manual** button (takes over) or **STOP**.
   Press **Mark** whenever something notable happens, and remember what.
6. At the end wall it stops: "OPEN SEA CROSSED" -> **Finish (save log)**.
7. **Download** the newest `mis_NNN.bin` **and** `img_NNN.bin` (same number). If it failed somewhere, finish the crossing in **Manual** - still logged,
   still useful.
8. Put the log file(s) + a few words (what happened, the Mark moments) into a folder, zip it, send it to me.

## D. What I do with it
Decode it (`cas_box_mission/tools/decode_mission_log.py`): where the real turn was (pillars, right side open),
the measured crab, every camera snapshot, heading errors, stops, ramps -> then the final working code.
