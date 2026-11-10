#pragma once
// RoverTrim - MEASURED wheel trim of kit 04 (copy this file into the navigation program).
//
//   left = v * (1 - trim)      right = v * (1 + trim)        (trim < 0: left side a bit faster)
//
// Measured 2026-11-07 with cas_box_trimcal, corridor (floor tiles), speed 0.25, battery 12.2 V,
// 10 valid runs (Log_run/log3):
//   drift = +0.40 deg/s (LEFT) with equal wheels   ->   drift = +0.396 + 8.58 * trim  (deg/s)
//   best trim -0.046 (90 % range -0.056 .. -0.039), one run scatters +-0.07 deg/s
//   VERIFY at -0.065 / -0.048 / -0.035: -0.15 / -0.05 / -0.02 deg/s  -> PASS (limit 0.30)
// So with this trim: left 0.2615, right 0.2385 at v = 0.25.
//
// NOT measured yet: outdoors on the paving. Old outdoor logs (009-013, equal wheels, no gyro hold)
// turned 2-4 deg/s RIGHT - 10x more, other direction, and more than any trim can fix with a gain of
// only 8.6 deg/s per unit. => verify outdoors (one Step 3 run), and the navigation program keeps a
// gentle gyro heading hold on top of this trim.
static const float kTrimSpeed = 0.25f;
// ROVER 7 (campus_10.3, 10 Nov): the learning needed +0.30 on top of -0.046, so the total is about +0.25. It is the start value now (rover 1-6: -0.046).
static const float kTrimIndoor025 = +0.25f;
static const float kTrimGainDegSPerUnit = 8.6f; // measured at 0.25: drift change per unit trim
