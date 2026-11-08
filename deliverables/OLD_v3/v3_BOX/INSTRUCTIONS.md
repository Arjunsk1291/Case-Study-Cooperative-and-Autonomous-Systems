# Mission v3: autonomous, no camera (kit 04, 2026-11-07 evening, after campus_2)

## What changed after campus_2 (mis_002)
| seen on campus_2 | cause (from the log) | v3 |
|---|---|---|
| right turn did not happen (also not with your manual pivot) - first try is now an exact copy of Kethan's Right 90 (0.7 s ramp to 0.50, held while it turns) | at the turn spot (~20 m along B) the pivot turns 4-7 deg and then **locks**: yaw frozen while both wheels got full power (same in campus run 1, 10 s). The same pivot turned 42 deg/s on the open sea -> **the ground at that spot holds the wheels**, motors are fine | turn tries several ways in a loop, each kept while the gyro sees progress: **full pivot -> forward swing -> roll 0.3 m forward (wheels onto other stones) -> pivot -> rolling arc -> roll 0.3 m -> ...** up to 2 min |
| swing turn came out weak (0.42 / -0.07 instead of 0.50 / 0) | the soft-ramp stage mixed both wheels | turns now ramp **each wheel on its own**: full power in 0.25 s, gentle 1.0/s down |
| stopped "at the end wall" ~10 m too early (your photo) | crossing the kerb: rover tipped +9 deg, LiDAR saw the ground at 1.3 m | end wall only counted when the rover is **level for 1 s** (checked: campus 1 real wall still found at 1.3 m) |
| drift | campus_2 with our code (= Kethan's heading hold + measured trim): leg 1 wall distance 1.56-1.62 m over 20 m, open-sea heading error median 0.33 deg | unchanged (same gains as Kethan: 0.045 / 0.0015 / limit 0.10) |
| a bit fast / sometimes not moving | - | speed **0.22**; if the LiDAR shows it is **not moving** while driving -> +0.03 every 2 s (max 0.32), back down when it moves |
| 3 pauses on the open sea | page heartbeat 1.5 s | 3 s |
| camera / lights gone | - | **camera removed completely**. Turn happens **by itself** at 20 m (setting) |
| Kethan's latest code turned twice | full power AT ONCE (no ramp), stop at ~83 deg (rover coasts ~7), landmark at the end of B | **pivot = Kethan's** (0 -> 0.50 at once, released ~7 deg early, kept while >= 1 deg per 2 s). **Turn point = Kethan's landmark** (the wide pillar on the right at the end of B; replayed on campus_2: found exactly where you turned). It turns when the pillar's near face is **0.60 m behind**. Then **POSITIONING**: it rolls slowly (0.18), steering max 10 deg, until the pillar is **beside it, 60 cm on its RIGHT**, **stops 1 s**, and only then drives straight into the open sea. No landmark -> turns at 22.5 m and goes straight |
| end wall vs. people / posts | - | end wall must also look like a flat wall (Kethan's check, tuned: real wall found from 1.54 m); waiting for something in front > 20 s -> pause |
| (add-on) | - | **Leg 3**: at the open-sea end wall it turns RIGHT (same turn tactics), follows that wall on its LEFT (same wall follower as leg 1) and **stops when the wall ends** (no wall for 1.5 m of travel - door gaps are shorter). Also stops at a wall ahead or after 60 m. Settings: on/off, "wall has ended after" |

## Flash (only the BOX - the camera is not needed any more)
Folder `v3_BOX`:
- Arduino IDE: open `v3_BOX/cas_box_mission/cas_box_mission.ino`. Board **ESP32 Dev Module**, Upload Speed **115200**,
  CPU **240 MHz**, Flash Frequency **40 MHz**, Flash Mode **DIO**, Flash Size **4MB**, Partition **Default 4MB with spiffs**,
  PSRAM **Disabled**, Erase All Flash **Disabled**. Close the Serial Monitor before uploading.
- or browser (https://espressif.github.io/esptool-js/, 115200): `BOX_v3_cas_box_mission_0x0.bin` at address **0x0**,
  do **not** press Erase.
Logs already on the box stay.

## Run
1. Battery >= 12.0 V. Phone: Wi-Fi `CAS-Box-04` / `CASbox04` -> http://192.168.14.1 -> title "Mission v3".
2. Rover at the start of B, wall on the LEFT, parallel. Stay behind it. **Start mission (leg 1)**.
3. Leg 1: buttons **Turn RIGHT now** (if you see it is at the turn) / **Turn later (+1 m)**. Otherwise it turns by itself at 20 m.
4. Turn: watch "Steps 2" - it shows which way it is trying (pivot / swing / shift / arc). It may roll forward a bit
   between tries - that is intended. Do not help unless it pauses after 2 min (then turn by hand, **Resume**).
5. Open sea: straight on the gyro to 1.5 m before the end wall, then turns right by itself (step 5 on the page).
6. Leg 3: follows the wall on its left; stops ~1.5-2 m after the wall ends ("LEFT WALL ENDED - route done") ->
   **Finish (save log)**.
7. Download the newest `mis_NNN.bin` and send it (+ a photo of the end position like last time - that was very useful).

Hard rules kept: nothing written to the rover, people in front -> stop and wait, STOP button / manual take-over always,
gentle ramps for driving and stopping (turns: quick full power as the rover needs, gentle stop).
- Page: holding a button no longer selects text.
