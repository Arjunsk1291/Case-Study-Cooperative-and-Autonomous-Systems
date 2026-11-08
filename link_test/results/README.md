# Camera -> box link test, 2026-11-07 (bench, rover off)

Setup: box on laptop USB (`link_box`, real access point CAS-Box-04 on channel 11, 15 dBm), camera on its own
power bank (`link_cam`, ESP-NOW only, 2 dBm, strip detector on the rotated image), ~10 cm apart.

| | 10 messages/s | 50/s requested |
|---|---|---|
| duration / messages | 50 s / 500 | 20 s / 250 |
| received per second | 10.0 | 12.5 (= camera frame limit) |
| lost | **0** | 0 |
| longest gap | 190 ms | 136 ms |
| round trip box -> camera -> box | min 3.8, typical 10-36, max 61 ms | 34-140 ms |
| signal | -45 .. -48 dBm | -46 .. -48 dBm |
| camera re-inits / stale data | 0 / 0 ms | 0 / 0 ms |
| box free memory | 222 kB steady | 222 kB |

- Pairing: automatic, ~2 s after both run (camera broadcasts HELLO, box answers ACK).
- Round trip minimum 3.8 ms = the radio itself. Larger values = the camera was waiting for its next picture
  before it could answer the ping. Age of a strip result at the box = picture age (<= 100 ms) + 0.3 ms
  detection + ~2 ms radio.
- The camera delivers at most ~12.5 pictures/s (160x120 grayscale). Asking for more only raises the delay.
  **Use 10/s.**
- No brownout on either board (box on laptop USB with AP running, camera on power bank).
- run1 log is garbled from ~30 s on: the Arduino IDE Serial Monitor was reading the same port at the same
  time (two readers split the bytes). Not a board or radio problem. Close the Serial Monitor before tests.

## Fail-safe tests (run3 / run4)
| test | result |
|---|---|
| box restart (port opened) | camera re-paired by itself in ~2-4 s, 10/s, 0 lost |
| camera power off | box flagged "stale" at once (within the 2 s report window, threshold 0.3 s) |
| camera power on again (run3, OLD code) | BUG: camera answered pings, but 0 data accepted - the box kept the old sequence number and dropped the restarted camera's messages as "duplicates". Safe direction (= no camera), but no recovery. |
| camera power on again (run4, FIXED) | HELLO from our camera resets the numbering (+ big backward jump = restart): data back at 10/s in one report, 0 lost, 0 duplicates |

Still to test: the link while the box also runs the rover link + phone page (real radio load), rover on, wheels up.
