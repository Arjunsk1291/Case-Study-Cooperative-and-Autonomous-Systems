# CAS Box B3 / R07 — Waypoint & Finish Fix Assessment

## Decision
The current Navigation.h is a scan-and-wander prototype. It chooses open sectors and has no destination state, route progress, or finish latch. That behavior can lead into unmapped areas.

## Crucial endpoint issue in the supplied dataset
The latest survey has START, `pillar 1`, `pillar 1 turn`, `after pillar to door`, two `near entrance` captures, and `entrance`. The `END_SCAN` label is also `entrance`. There is no labelled scan explicitly captured *inside the washroom*. Therefore the current dataset cannot support a reliable washroom-arrival signature. If the firmware treats the final recorded signature as finish, it would likely stop at the entrance—the exact failure the user wants to avoid.

## What this package adds
- `route_waypoint_signatures.json`: per-bin LiDAR median references computed from the 15-second scan window at each marker, with fresh-valid ultrasonic medians and median IMU yaw.
- `firmware_source/`: original v4 source preserved for patching/review.
- This report identifies the controller mismatch and defines the safe implementation contract.

## Required firmware behavior
1. Replace scan-and-wander with ordered waypoint progression; never select arbitrary open sectors as a route goal.
2. Match a landmark using robust range-profile comparison, allow cyclic LiDAR bin shift only if scan orientation handling is validated, and require stable match across multiple fresh scans.
3. Advance only in order: START -> pillar 1 -> pillar 1 turn -> after pillar/door approach -> near entrance -> entrance -> washroom interior.
4. At the doorway, estimate both jamb boundaries and verify usable width against the actual robot footprint + turning/position uncertainty margin. Slow down and stop if width/confidence is insufficient.
5. Finish only after a separately captured and validated *inside-washroom* signature matches repeatedly. On finish: command both motors zero, set a latched FINISHED state, ignore further autonomous motion requests until explicit reset/disable-enable.
6. If scan match is lost, heading mismatch occurs, sensors are stale, or rover link is unhealthy: stop; do not wander.
7. Keep ultrasonic safety vetoes; they are not landmark identity or finish evidence. GPS is not used for indoor waypoint localization.

## Why no claim of a fixed autonomous firmware is made
The route dataset is a manual survey; there are no wheel encoders, no validated scan-matching localization, and no final washroom-interior capture. A safe waypoint/finish patch cannot be verified against the target location without that missing final reference. Guessing an endpoint would encode the wrong stop location.

## Next capture needed (small)
Drive manually through the doorway and place the robot fully inside the washroom at the desired stopping point. Keep it stationary and capture a new 15-second scan labelled `WASHROOM_FINISH` (not `entrance`). Capture a few metres before the doorway too if possible. Also measure actual chassis width and length, and confirm the LiDAR's physical forward direction. With those, the finish matcher can be anchored to the intended endpoint rather than the doorway.
