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
    _motionState = MOTION_STOPPED;
    _turnStartedMs = 0;
    _turnCount = 0;
    _turnDirection = 0;
    _state = "STOP";
  }

  Side side() const { return _side; }
  const char* sideName() const { return _side == FOLLOW_RIGHT ? "RIGHT" : "LEFT"; }

  // Returns normalized wheel commands in the existing RoverLink -0.5..0.5 range.
  // Movement is deliberately split into low-speed straight motion and full-scale pivot pulses:
  // never ask the rover to make a tiny differential-speed turn while creeping forward.
  void update(const Sensors& sensors, float& left, float& right, String& status) {
    left = right = 0;
    const uint32_t now = millis();

    if (!sensors.lidarOk() || sensors.lidarCrcErrorPct() > 5.0f) {
      _motionState = MOTION_STOPPED;
      _state = "FAULT: LiDAR unavailable / CRC high - STOP";
      status = _state;
      return;
    }

    const Sector front = sector(sensors, 86, 4); // wraps through bin 0
    const Sector rightSide = sector(sensors, 13, 31);
    const Sector leftSide  = sector(sensors, 59, 77);

    if (front.count < 2) {
      _motionState = MOTION_STOPPED;
      _state = "FAULT: front LiDAR coverage uncertain - STOP";
      status = _state;
      return;
    }

    uint16_t frontMin = front.minCm;
    if (sensors.usValid(0) && sensors.usAgeMs(0) <= 600) {
      const int u = sensors.usCm(0);
      if (u > 0 && u < frontMin) frontMin = (uint16_t)u;
    }

    // Any emergency/uncertain condition interrupts pivoting immediately.
    if (frontMin > 0 && frontMin < kEmergencyStopCm) {
      _motionState = MOTION_STOPPED;
      _turnCount = 0;
      _state = "STOP: obstacle too close - manual recovery required";
      status = _state;
      return;
    }

    const Sector selected = (_side == FOLLOW_RIGHT) ? rightSide : leftSide;
    const Sector opposite = (_side == FOLLOW_RIGHT) ? leftSide : rightSide;
    const bool selectedWall = isSurface(selected);
    const bool oppositeWall = isSurface(opposite);
    const uint16_t oppositeClearance = opposite.count ? opposite.minCm : 0;

    if (!selectedWall) {
      if (_lostSinceMs == 0) _lostSinceMs = now;
      const bool safeToSwitch = oppositeWall && frontMin >= kSwitchFrontClearCm &&
                                oppositeClearance >= kSwitchSideClearCm &&
                                (now - _lastSwitchMs >= kSwitchCooldownMs);
      if (safeToSwitch && now - _lostSinceMs >= kLostSurfaceSwitchDelayMs) {
        _side = (_side == FOLLOW_RIGHT) ? FOLLOW_LEFT : FOLLOW_RIGHT;
        _lastSwitchMs = now;
        _lostSinceMs = 0;
        _motionState = MOTION_STOPPED; // stop before re-evaluating on the other side
      } else {
        _motionState = MOTION_STOPPED;
        _state = "STOP: surface lost; waiting for safe reacquisition";
        status = _state;
        return;
      }
    } else {
      _lostSinceMs = 0;
    }

    const Sector follow = (_side == FOLLOW_RIGHT) ? rightSide : leftSide;
    if (!isSurface(follow)) {
      _motionState = MOTION_STOPPED;
      _state = "STOP: no stable surface after side selection";
      status = _state;
      return;
    }

    // If a turn pulse is in progress, hold the pivot only for a short bounded interval,
    // then stop and re-read sensors before deciding whether another pulse is needed.
    if (_motionState == MOTION_PIVOTING) {
      if (now - _turnStartedMs < kPivotPulseMs) {
        left = (_turnDirection < 0) ? -kPivotSpeed : kPivotSpeed;
        right = -left;
        _state = (_turnDirection < 0) ? "PIVOT_LEFT full-speed pulse" : "PIVOT_RIGHT full-speed pulse";
        status = _state;
        return;
      }
      _motionState = MOTION_SETTLING;
      _settleStartedMs = now;
      left = right = 0;
      _state = "STOP after pivot; rechecking sensors";
      status = _state;
      return;
    }
    if (_motionState == MOTION_SETTLING) {
      left = right = 0;
      if (now - _settleStartedMs < kSettleMs) {
        _state = "STOP after pivot; settling";
        status = _state;
        return;
      }
      _motionState = MOTION_STOPPED;
      // Continue below and re-evaluate geometry; no blind chained rotation.
    }

    const float distanceCm = (float)follow.medianCm;
    const float error = distanceCm - kTargetClearanceCm; // positive: surface too far away
    int desiredTurn = 0;
    if (error < -kTurnDeadbandCm) {
      // Surface too close: pivot away from it.
      desiredTurn = (_side == FOLLOW_RIGHT) ? -1 : 1; // -1 = left pivot
    } else if (error > kTurnDeadbandCm) {
      // Surface too far: pivot toward it, but only with a confirmed clear front.
      if (frontMin >= kTurnFrontClearCm) desiredTurn = (_side == FOLLOW_RIGHT) ? 1 : -1;
    }

    if (desiredTurn != 0) {
      if (_turnCount >= kMaxConsecutivePivots) {
        _motionState = MOTION_STOPPED;
        _state = "STOP: repeated pivot limit reached; operator check required";
        status = _state;
        return;
      }
      _motionState = MOTION_PIVOTING;
      _turnStartedMs = now;
      _turnDirection = desiredTurn;
      ++_turnCount;
      left = (desiredTurn < 0) ? -kPivotSpeed : kPivotSpeed;
      right = -left;
      _state = (desiredTurn < 0) ? "PIVOT_LEFT full-speed pulse" : "PIVOT_RIGHT full-speed pulse";
      status = _state + " #" + String(_turnCount);
      return;
    }

    // Straight/creep movement only. Do not use a differential steering correction here.
    _turnCount = 0;
    _motionState = MOTION_STOPPED;
    float base = (frontMin < kSlowFrontCm) ? kSlowSpeed : kCruiseSpeed;
    if (frontMin < kCautionFrontCm) base = min(base, kSlowSpeed);
    left = right = base;
    _state = String("MOVE_FORWARD FOLLOW_") + sideName();
    status = _state + " surface=" + String(follow.medianCm) + "cm front=" + String(frontMin) + "cm";
  }

private:
  static constexpr uint16_t kEmergencyStopCm = 32;
  static constexpr uint16_t kCautionFrontCm = 65;
  static constexpr uint16_t kSlowFrontCm = 90;
  static constexpr uint16_t kSwitchFrontClearCm = 85;
  static constexpr uint16_t kSwitchSideClearCm = 55;
  static constexpr uint16_t kTargetClearanceCm = 55;
  static constexpr uint16_t kTurnDeadbandCm = 15;
  static constexpr uint16_t kTurnFrontClearCm = 85;
  static constexpr uint16_t kPillarLikeMaxCm = 85;
  static constexpr uint32_t kLostSurfaceSwitchDelayMs = 1200;
  static constexpr uint32_t kSwitchCooldownMs = 3000;
  static constexpr uint32_t kPivotPulseMs = 180;       // tune on a clear floor at low risk
  static constexpr uint32_t kSettleMs = 250;           // stop and re-evaluate after each pulse
  static constexpr uint8_t kMaxConsecutivePivots = 4;
  static constexpr float kCruiseSpeed = 0.10f;         // slow straight motion
  static constexpr float kSlowSpeed = 0.055f;
  static constexpr float kPivotSpeed = 0.50f;           // full-scale rotation required by this rover

  enum MotionState : uint8_t { MOTION_STOPPED, MOTION_PIVOTING, MOTION_SETTLING };
  Side _side = FOLLOW_RIGHT;
  MotionState _motionState = MOTION_STOPPED;
  uint32_t _lostSinceMs = 0;
  uint32_t _lastSwitchMs = 0;
  uint32_t _turnStartedMs = 0;
  uint32_t _settleStartedMs = 0;
  uint8_t _turnCount = 0;
  int8_t _turnDirection = 0; // -1 left, +1 right
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
