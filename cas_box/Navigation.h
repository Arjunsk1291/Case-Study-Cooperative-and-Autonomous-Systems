#pragma once
// Conservative experimental surface-following controller for CAS Box B3 / Rover R07.
// LiDAR bins: 90 values, bin 0 = front, increasing clockwise, 4 degrees/bin.
// Autonomous mode is intentionally OFF at boot and is enabled only from the dashboard.
#include <Arduino.h>
#include "Sensors.h"

class Navigation {
public:
  enum Side : uint8_t { FOLLOW_RIGHT = 0, FOLLOW_LEFT = 1 };

  struct Sector {
    uint16_t minCm = 0;
    uint16_t medianCm = 0;
    uint8_t count = 0;
  };

  void reset() {
    _side = FOLLOW_RIGHT;
    _lostSinceMs = 0;
    _lastSwitchMs = 0;
    _state = "STOP";
  }

  Side side() const { return _side; }
  const char* sideName() const { return _side == FOLLOW_RIGHT ? "RIGHT" : "LEFT"; }

  // Returns normalized wheel commands in the existing RoverLink -0.5..0.5 range.
  // This prototype deliberately caps autonomous commands at 0.12 (24% of 0.5 full scale).
  void update(const Sensors& sensors, float& left, float& right, String& status) {
    left = right = 0;
    const uint32_t now = millis();

    if (!sensors.lidarOk() || sensors.lidarCrcErrorPct() > 5.0f) {
      _state = "FAULT: LiDAR unavailable / CRC high - STOP";
      status = _state;
      return;
    }

    const Sector front = sector(sensors, 86, 4); // wraps through bin 0
    const Sector rightSide = sector(sensors, 13, 31);
    const Sector leftSide  = sector(sensors, 59, 77);
    const Sector rear = sector(sensors, 39, 51);
    (void)rear;

    // A sparse front scan is not evidence of free space. Stop instead of guessing.
    if (front.count < 2) {
      _state = "FAULT: front LiDAR coverage uncertain - STOP";
      status = _state;
      return;
    }

    uint16_t frontMin = front.minCm;
    // Fresh front ultrasonic may only make the stop decision more conservative.
    if (sensors.usValid(0) && sensors.usAgeMs(0) <= 600) {
      const int u = sensors.usCm(0);
      if (u > 0 && u < frontMin) frontMin = (uint16_t)u;
    }

    // Emergency stop: no automatic blind pivot around an obstacle.
    if (frontMin > 0 && frontMin < kEmergencyStopCm) {
      _state = "STOP: obstacle too close - manual recovery required";
      status = _state;
      return;
    }

    const Sector selected = (_side == FOLLOW_RIGHT) ? rightSide : leftSide;
    const Sector opposite = (_side == FOLLOW_RIGHT) ? leftSide : rightSide;
    const bool selectedWall = isSurface(selected);
    const bool oppositeWall = isSurface(opposite);
    const uint16_t oppositeClearance = opposite.count ? opposite.minCm : 0;

    // Switch sides only after the chosen surface has been absent long enough and the
    // opposite surface and forward path are both positively observed as safe.
    if (!selectedWall) {
      if (_lostSinceMs == 0) _lostSinceMs = now;
      const bool safeToSwitch = oppositeWall && frontMin >= kSwitchFrontClearCm &&
                                oppositeClearance >= kSwitchSideClearCm &&
                                (now - _lastSwitchMs >= kSwitchCooldownMs);
      if (safeToSwitch && now - _lostSinceMs >= kLostSurfaceSwitchDelayMs) {
        _side = (_side == FOLLOW_RIGHT) ? FOLLOW_LEFT : FOLLOW_RIGHT;
        _lastSwitchMs = now;
        _lostSinceMs = 0;
      } else {
        _state = "STOP: surface lost; waiting for safe reacquisition";
        status = _state;
        return;
      }
    } else {
      _lostSinceMs = 0;
    }

    const Sector follow = (_side == FOLLOW_RIGHT) ? rightSide : leftSide;
    if (!isSurface(follow)) {
      _state = "STOP: no stable surface after side selection";
      status = _state;
      return;
    }

    // Conservative crawl when an obstacle is approaching; stop at emergency threshold above.
    const float base = (frontMin < kSlowFrontCm) ? kSlowSpeed : kCruiseSpeed;
    const float distanceCm = (float)follow.medianCm;
    const float error = kTargetClearanceCm - distanceCm; // positive means surface too close
    const float turn = constrain(error * kSteerGain, -kMaxSteer, kMaxSteer);
    // Differential drive convention verified against this project's manual pad:
    // left wheel slower / right faster turns left; reverse the mapping for left-side following.
    if (_side == FOLLOW_RIGHT) {
      left = constrain(base - turn, 0.0f, kMaxAutoSpeed);
      right = constrain(base + turn, 0.0f, kMaxAutoSpeed);
    } else {
      left = constrain(base + turn, 0.0f, kMaxAutoSpeed);
      right = constrain(base - turn, 0.0f, kMaxAutoSpeed);
    }

    if (frontMin < kCautionFrontCm) {
      // Reduce speed and bias away from the nearer side only if that side's sector is known.
      left = min(left, kSlowSpeed);
      right = min(right, kSlowSpeed);
    }

    _state = (follow.medianCm < kPillarLikeMaxCm && follow.count < 7)
      ? String("FOLLOW_PILLAR ") + sideName()
      : String("FOLLOW_SURFACE ") + sideName();
    status = _state + " d=" + String(follow.medianCm) + "cm front=" + String(frontMin) + "cm";
  }

private:
  static constexpr uint16_t kEmergencyStopCm = 32;
  static constexpr uint16_t kCautionFrontCm = 65;
  static constexpr uint16_t kSlowFrontCm = 90;
  static constexpr uint16_t kSwitchFrontClearCm = 85;
  static constexpr uint16_t kSwitchSideClearCm = 55;
  static constexpr uint16_t kTargetClearanceCm = 55;
  static constexpr uint16_t kPillarLikeMaxCm = 85;
  static constexpr uint32_t kLostSurfaceSwitchDelayMs = 1200;
  static constexpr uint32_t kSwitchCooldownMs = 3000;
  static constexpr float kCruiseSpeed = 0.10f;
  static constexpr float kSlowSpeed = 0.055f;
  static constexpr float kMaxAutoSpeed = 0.12f;
  static constexpr float kSteerGain = 0.0016f;
  static constexpr float kMaxSteer = 0.035f;

  Side _side = FOLLOW_RIGHT;
  uint32_t _lostSinceMs = 0;
  uint32_t _lastSwitchMs = 0;
  String _state = "STOP";

  static bool isSurface(const Sector& s) {
    return s.count >= 4 && s.medianCm >= 20 && s.medianCm <= 140;
  }

  static Sector sector(const Sensors& sensors, int start, int end) {
    Sector out;
    uint16_t vals[20];
    uint8_t n = 0;
    // end < start means wrap through bin 0 (front sector).
    int i = start;
    for (int guard = 0; guard < 90; ++guard) {
      const uint16_t d = sensors.lidarCm((uint8_t)(i % 90));
      if (d >= 10 && d <= 300 && n < 20) {
        vals[n++] = d;
        if (out.minCm == 0 || d < out.minCm) out.minCm = d;
      }
      if (i == end) break;
      i = (i + 1) % 90;
    }
    out.count = n;
    if (!n) return out;
    for (uint8_t a = 0; a < n; ++a)
      for (uint8_t b = a + 1; b < n; ++b)
        if (vals[b] < vals[a]) { uint16_t t = vals[a]; vals[a] = vals[b]; vals[b] = t; }
    out.medianCm = vals[n / 2];
    return out;
  }
};
