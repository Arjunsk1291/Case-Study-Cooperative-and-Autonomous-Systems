#pragma once
// Conservative reactive navigation prototype for CAS Box B3 / Rover R07.
// Based on a VFH-style free-sector choice and differential surface-following.
// NOT certified. Validate LiDAR orientation and motor turn direction at low speed first.
#include <Arduino.h>
#include "Sensors.h"

class Navigation {
public:
  enum Side : uint8_t { FOLLOW_RIGHT = 0, FOLLOW_LEFT = 1 };

  void reset() {
    _side = FOLLOW_RIGHT; _phase = PH_DRIVE; _phaseStartedMs = millis();
    _pendingTurn = 0; _turnCount = 0; _faultLatched = false; _lastCommandL = _lastCommandR = 0;
    _state = "AUTO WAIT"; _diagnostic = "Waiting for valid sensor data";
  }
  Side side() const { return _side; }
  const char* sideName() const { return _side == FOLLOW_RIGHT ? "RIGHT" : "LEFT"; }
  float commandLeft() const { return _lastCommandL; }
  float commandRight() const { return _lastCommandR; }
  float adaptiveSpeed() const { return kCruiseSpeed; }
  uint8_t recoveryCount() const { return _turnCount; }
  const String& diagnostic() const { return _diagnostic; }

  void update(const Sensors& s, float& left, float& right, String& status) {
    left = right = 0; _lastCommandL = _lastCommandR = 0;
    const uint32_t now = millis();
    if (_faultLatched) { left = right = 0; _state = "FAULT LATCHED: disable/re-enable autonomy"; _diagnostic = "Fault remains latched until autonomy is disabled and enabled again."; finish(left, right, status); return; }
    if (!s.lidarOk() || s.lidarCrcErrorPct() > kMaxCrcPct) {
      fault("FAULT STOP: LiDAR unavailable / CRC high", "Disable autonomy; check LiDAR power, UART, checksum and packet rate.", left, right, status); return;
    }

    const Sector front = sector(s, 86, 4);
    const Sector rightSide = sector(s, 13, 31);
    const Sector leftSide = sector(s, 59, 77);
    if (front.count < kMinFrontReturns) {
      fault("FAULT STOP: uncertain front scan", "Too few valid front returns. Verify LiDAR orientation and scan coverage.", left, right, status); return;
    }

    uint16_t frontMin = front.minCm;
    // Ultrasonic is used only when its sample is fresh and valid. 300 cm is treated as no echo.
    if (s.usValid(0) && s.usAgeMs(0) <= kUsMaxAgeMs) {
      const int u = s.usCm(0);
      if (u > 0 && u < Sensors::kUsMaxCm && u < frontMin) frontMin = (uint16_t)u;
    }

    // A close obstacle is a latched stop: do not rotate in place when the footprint clearance
    // is already compromised. Operator must clear the path and re-enable autonomy.
    if (frontMin < kEmergencyStopCm) {
      fault("STOP: obstacle inside emergency clearance", "Front clearance below 32 cm. Disable autonomy, clear the path, then re-enable.", left, right, status); return;
    }

    if (_phase == PH_STOP_BEFORE_TURN) {
      if (now - _phaseStartedMs < kStopBeforeTurnMs) {
        _state = "AVOID: stopped before pivot"; _diagnostic = "Brake/settle interval before obstacle-avoidance pivot.";
        finish(left, right, status); return;
      }
      const Sector& chosen = _pendingTurn < 0 ? leftSide : rightSide;
      if (!turnSideClear(chosen, s, _pendingTurn)) {
        _phase = PH_DRIVE; _pendingTurn = 0;
        _state = "STOP: turn clearance not confirmed";
        _diagnostic = "No verified side clearance for a pivot. Reposition manually; unknown space is not free space.";
        finish(left, right, status); return;
      }
      _phase = PH_PIVOT; _phaseStartedMs = now;
    }
    if (_phase == PH_PIVOT) {
      if (now - _phaseStartedMs < kPivotPulseMs) {
        // For conventional differential drive: left turn = left reverse/right forward.
        left = _pendingTurn < 0 ? -kPivotSpeed : kPivotSpeed;
        right = -left;
        _state = _pendingTurn < 0 ? "AVOID: pivot left" : "AVOID: pivot right";
        _diagnostic = "Short bounded pivot; angle is time-based and not guaranteed without encoders.";
        finish(left, right, status); return;
      }
      _turnCount++;
      _phase = PH_SETTLE; _phaseStartedMs = now;
    }
    if (_phase == PH_SETTLE) {
      if (now - _phaseStartedMs < kSettleMs) {
        _state = "AVOID: settle and rescan"; _diagnostic = "Stopped after pivot; waiting for the next sensor snapshot.";
        finish(left, right, status); return;
      }
      _phase = PH_DRIVE; _pendingTurn = 0;
    }

    // Obstacle avoidance takes priority over wall/surface following.
    if (frontMin < kAvoidFrontCm) {
      const int8_t turn = chooseTurn(leftSide, rightSide, s);
      if (!turn) {
        _state = "STOP: no safe avoidance sector";
        _diagnostic = "Obstacle ahead and neither side has confirmed clearance; manual intervention required.";
        finish(left, right, status); return;
      }
      _pendingTurn = turn; _phase = PH_STOP_BEFORE_TURN; _phaseStartedMs = now;
      _state = "AVOID: obstacle detected; stop";
      _diagnostic = String("Front clearance ") + String(frontMin) + " cm; selected " + (turn < 0 ? "left" : "right") + " only after side-sector check.";
      finish(left, right, status); return;
    }

    const Sector& follow = _side == FOLLOW_RIGHT ? rightSide : leftSide;
    const Sector& opposite = _side == FOLLOW_RIGHT ? leftSide : rightSide;
    if (!isSurface(follow)) {
      // Only switch if the opposite surface is detected and the front plus opposite side are clear.
      if (isSurface(opposite) && frontMin >= kSwitchFrontClearCm && opposite.minCm >= kSwitchSideClearCm) {
        _side = _side == FOLLOW_RIGHT ? FOLLOW_LEFT : FOLLOW_RIGHT;
        _state = "SURFACE: switched following side";
        _diagnostic = String("Original surface lost; opposite surface detected. Following ") + sideName() + ".";
      } else {
        _state = "STOP: surface not confidently detected";
        _diagnostic = "No reliable continuous surface. This controller is surface-following, not waypoint navigation; move to a wall/furniture edge or use manual control.";
      }
      finish(left, right, status); return;
    }

    // Differential proportional steering instead of repeated stop/pivot pulses for small wall errors.
    const float distance = follow.medianCm;
    const float error = distance - kTargetClearanceCm;
    float correction = 0.0f;
    if (fabsf(error) > kWallDeadbandCm) {
      correction = constrain(error * kWallGain, -kMaxWallCorrection, kMaxWallCorrection);
      // right-side surface too close => steer left (right wheel faster); signs reverse for left side.
      if (_side == FOLLOW_RIGHT) correction = -correction;
    }
    float speed = kCruiseSpeed;
    if (frontMin < kSlowFrontCm) speed = kSlowSpeed;
    if (frontMin < kCautionFrontCm) speed = kCautionSpeed;
    left = constrain(speed - correction, 0.0f, kCruiseSpeed);
    right = constrain(speed + correction, 0.0f, kCruiseSpeed);
    _state = String("MOVE: surface follow ") + sideName();
    _diagnostic = String("Front=") + String(frontMin) + " cm; surface=" + String((int)distance) +
      " cm; target=" + String(kTargetClearanceCm) + " cm; differential correction=" + String(correction, 3) +
      ". Verify steering sign on blocks before floor test.";
    finish(left, right, status);
  }

private:
  static constexpr uint16_t kEmergencyStopCm = 32;
  static constexpr uint16_t kAvoidFrontCm = 90;
  static constexpr uint16_t kSlowFrontCm = 125;
  static constexpr uint16_t kCautionFrontCm = 75;
  static constexpr uint16_t kSwitchFrontClearCm = 110;
  static constexpr uint16_t kSwitchSideClearCm = 60;
  static constexpr uint16_t kTargetClearanceCm = 55;
  static constexpr uint16_t kWallDeadbandCm = 12;
  static constexpr uint16_t kTurnSideClearCm = 65;
  static constexpr uint16_t kMinFrontReturns = 3;
  static constexpr float kMaxCrcPct = 5.0f;
  static constexpr uint32_t kUsMaxAgeMs = 600;
  static constexpr uint32_t kStopBeforeTurnMs = 250;
  static constexpr uint32_t kPivotPulseMs = 180;
  static constexpr uint32_t kSettleMs = 350;
  static constexpr float kCruiseSpeed = 0.16f;
  static constexpr float kSlowSpeed = 0.12f;
  static constexpr float kCautionSpeed = 0.08f;
  static constexpr float kPivotSpeed = 0.28f;
  static constexpr float kWallGain = 0.0020f;
  static constexpr float kMaxWallCorrection = 0.045f;

  struct Sector { uint16_t minCm = 0, medianCm = 0; uint8_t count = 0; };
  enum Phase : uint8_t { PH_DRIVE, PH_STOP_BEFORE_TURN, PH_PIVOT, PH_SETTLE };
  Side _side = FOLLOW_RIGHT;
  Phase _phase = PH_DRIVE;
  uint32_t _phaseStartedMs = 0;
  int8_t _pendingTurn = 0;
  uint8_t _turnCount = 0;
  bool _faultLatched = false;
  float _lastCommandL = 0, _lastCommandR = 0;
  String _state = "AUTO WAIT", _diagnostic = "Initialising";

  void finish(float& l, float& r, String& status) {
    _lastCommandL = l; _lastCommandR = r; status = _state;
  }
  void fault(const char* state, const char* diag, float& l, float& r, String& status) {
    _phase = PH_DRIVE; _pendingTurn = 0; _faultLatched = true; l = r = 0; _state = state; _diagnostic = diag; finish(l, r, status);
  }
  static bool isSurface(const Sector& s) { return s.count >= 4 && s.medianCm >= 20 && s.medianCm <= 160; }
  static Sector sector(const Sensors& s, int start, int end) {
    Sector out; uint16_t vals[24]; uint8_t n = 0; int i = start;
    for (int guard = 0; guard < 90; ++guard) {
      uint16_t d = s.lidarCm((uint8_t)(i % 90));
      if (d >= 10 && d <= 300 && n < 24) { vals[n++] = d; if (!out.minCm || d < out.minCm) out.minCm = d; }
      if (i == end) break; i = (i + 1) % 90;
    }
    out.count = n; if (!n) return out;
    for (uint8_t a=0; a<n; ++a) for (uint8_t b=a+1; b<n; ++b) if (vals[b]<vals[a]) { uint16_t t=vals[a]; vals[a]=vals[b]; vals[b]=t; }
    out.medianCm = vals[n/2]; return out;
  }
  static bool turnSideClear(const Sector& side, const Sensors& s, int8_t turn) {
    // Require several real LiDAR returns and conservative minimum clearance; 0/no-return is unknown.
    if (side.count < 5 || side.minCm < kTurnSideClearCm) return false;
    const uint8_t usIndex = turn < 0 ? 1 : 2; // left / right ultrasonic
    if (s.usValid(usIndex) && s.usAgeMs(usIndex) <= kUsMaxAgeMs) {
      const int d = s.usCm(usIndex);
      if (d > 0 && d < Sensors::kUsMaxCm && d < kTurnSideClearCm) return false;
    }
    return true;
  }
  static int8_t chooseTurn(const Sector& left, const Sector& right, const Sensors& s) {
    const bool lc = turnSideClear(left, s, -1), rc = turnSideClear(right, s, 1);
    if (!lc && !rc) return 0;
    if (lc && !rc) return -1; if (rc && !lc) return 1;
    // Prefer the side with larger minimum clearance, then median. Ties deterministically turn left.
    if (right.minCm > left.minCm) return 1;
    if (left.minCm > right.minCm) return -1;
    return right.medianCm > left.medianCm ? 1 : -1;
  }
};
