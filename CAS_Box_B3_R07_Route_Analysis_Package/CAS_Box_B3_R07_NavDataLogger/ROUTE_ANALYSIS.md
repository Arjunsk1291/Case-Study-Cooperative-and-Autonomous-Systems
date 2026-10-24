# CAS Box B3 / WAVE R07 — analysis of investigation capture

Dataset: `CAS-B3-R07-investigation-2.0`; captured 2026-10-23T14:08:09.173Z to 2026-10-23T14:12:45.968Z.

## What this capture establishes

- 1123 events; 1123 samples; 8 logged locations.
 - The capture records a **manual** survey (`nav.mode=MANUAL`), not an autonomous route trial.
 - It provides repeated 90-bin LiDAR snapshots at labeled stops; it does not provide a metric map or reliable distance traveled.
 - Repeated `near entrance` scan labels are separate stops. The final `entrance` capture is another distinct observation, not proof of reaching the washroom.
 - IMU yaw changes between capture points are not safe absolute pose estimates; BNO055 calibration reported incomplete magnetometer/system calibration.
 - GPS fix/coordinates in this file are not suitable for indoor localization; use neither as room-level position nor as destination proof.

## Checkpoint scan summaries

| Checkpoint | elapsed (s) | samples | median yaw (°) | Front US | Left US | Right US | Rear US | Angled front US |
|---|---:|---:|---:|---:|---:|---:|---:|---:|
| START | 0.0 | 60 | -52.0 | 193 | 57 | 44 | 32 | n/a |
| pillar 1 | 33.4 | 60 | -56.4 | 98 | 35 | 43 | 132 | 111 |
| pillar 1 turn | 70.7 | 60 | -148.2 | 113 | 127 | 17 | n/a | n/a |
| after pillar to door | 107.0 | 59 | -136.8 | 207 | 78 | 46 | 100 | n/a |
| near entrance | 139.1 | 60 | -145.6 | 44 | 102 | 78 | n/a | 47 |
| near entrance | 162.2 | 60 | 125.3 | 74 | 47 | 164 | 105 | 152 |
| entrance | 201.3 | 60 | 121.0 | 89 | 51 | 24 | 182 | 106 |
| entrance | 257.8 | 59 | 127.1 | 70 | 98 | 33 | 92 | 74 |

Ultrasonic columns are medians of samples marked valid and no older than 600 ms; `n/a` means no usable sample met those conditions. LiDAR signature CSV contains per-bin medians over each 15-second capture, with zero for bins without enough in-range returns. These are reference signatures for offline comparison, not a map.

## Important evidence from the route capture

- The START scan median ultrasonic values were approximately front 193 cm, left 57 cm, right 44 cm, rear 32 cm; angled-front had no fresh valid median. This is not an all-clear signal: the rear/right area is close and the angled-front sensor was stale/invalid.
- At `pillar 1`, left US median was about 35 cm and right about 43 cm. At `pillar 1 turn`, the right sensor median was about 17 cm, so a pivot at that recorded configuration should be vetoed unless fresh scans prove the swept footprint is clear.
- At the two `near entrance` captures, the heading values differ sharply (about -145.6° vs +125.3°). Since they are separate captures and heading wraps at ±180°, this may reflect orientation change, wrap, or magnetic effects; it must not be converted into a metric turn/pose without validating the IMU behavior.
- At the final `entrance` capture, right US median was about 33 cm. A conservative doorway approach should slow/stop and re-scan rather than pivot or continue open-loop based on a route timer.
- The source prototype already recorded `Expected left turn; measured delta...` diagnostic in snapshots. The capture does not prove the pivot direction/sign issue is resolved.

## Firmware decision

This package deliberately does **not** enable autonomous waypoint following. The present firmware is reactive scan-and-wander; it has no scan matcher, pose graph, persistent map, or validated checkpoint recognizer. Simply playing back the manual route timing would be unsafe because wheel encoders are absent and route sample times do not equal distance.

Recommended next implementation sequence:
1. Verify LiDAR bin 0 is physically forward and bins increase clockwise using known obstacles.
2. With wheels lifted, verify command polarity against BNO055 relative yaw; do not assume positive yaw is left.
3. Build a scan-matching module that estimates relative pose from the 90-bin scans and outputs match score/quality; reject low-confidence matches.
4. Add checkpoint matching against `ROUTE_SCAN_SIGNATURES.csv` only after replay testing; require multiple consecutive matches and fresh ultrasonic vetoes.
5. Use short, slow motion segments with stop-and-rescan at pillar/door checkpoints. Require explicit endpoint confirmation before declaring arrival.
6. Test in a taped, clear area with a spotter and immediate physical stop access. Do not test near people, stairs, or an occupied doorway.

## Included files

- `cas_box/`: source archive as supplied, unchanged except documentation additions below.
- `ROUTE_SCAN_SIGNATURES.csv`: offline median scan signatures for each labeled stop.
- `ROUTE_ANALYSIS.md`: this analysis.
- `IMPLEMENTATION_NOTES.md`: original source notes plus this route-data status.

No ESP32 compilation or physical hardware test was performed as part of this analysis.
