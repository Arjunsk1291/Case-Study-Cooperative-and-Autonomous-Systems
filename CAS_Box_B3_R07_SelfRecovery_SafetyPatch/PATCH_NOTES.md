# CAS Box B3 / R07 — self-recovery and anti-wander safety patch

## What the latest diagnostic actually says
At export time (2026-10-23 14:38:48Z):
- LiDAR status OK, 334 packets/s, CRC 0%.
- The forward sector nevertheless had fewer than the required four usable range returns.
- Front ultrasonic was 216 cm and fresh; angled-front was invalid/stale at 27,089 ms.
- Rover link was connected, RTT ~108 ms, battery voltage ~11.49 V.
- Dashboard was in AUTONOMOUS / WANDER and fault-latched.

This means packet transport was running, but the narrow forward sector did not contain enough usable ranges. The old code latched this transient/geometry condition and required manual disable/re-enable.

## What this patch changes
1. Sparse forward LiDAR returns no longer trigger a manual-reset fault. Motors stay at zero and the controller automatically retries with fresh scans.
2. The scan-and-wander behavior is disabled in this build. A clear-looking sector is not a goal, and the controller will not drive toward arbitrary openings or unexplored areas.
3. If there is no validated waypoint planner, it remains stopped and reports that route target is unavailable.
4. Existing source and hardware interfaces are retained.

## Important limitation
This is a fail-safe self-recovery patch, not a completed washroom navigator. It deliberately will not drive autonomously until waypoint localization and the final washroom signature are implemented. The current route survey ends at a landmark labelled `entrance`, not a confirmed interior washroom finish point. Continuing to move autonomously without that information would recreate the wandering failure.

## Before testing
- Keep the physical emergency stop accessible and test with wheels lifted / robot restrained first.
- Do not rely on ultrasonic 300 cm values as proof of clear space.
- Confirm LiDAR bin 0 physical forward direction.
- The source has not been compiled with the Arduino toolchain or tested on hardware in this environment.
