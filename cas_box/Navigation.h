#pragma once
// Experimental local surface-following controller. NOT a certified safety controller.
// 90 LiDAR bins, index 0 = front, clockwise, 4 degrees/bin.
#include <Arduino.h>
#include "Sensors.h"

class Navigation {
public:
  enum Side : uint8_t { FOLLOW_RIGHT = 0, FOLLOW_LEFT = 1 };

  void reset() {
    _side = FOLLOW_RIGHT; _lostSinceMs = 0; _lastSwitchMs = 0;
    _phase = PH_NORMAL; _phaseStartedMs = 0; _lastSceneMs = millis();
    _lastSignatureAtMs = 0; _signatureReady = false; _stallSinceMs = 0;
    _recoveryCount = 0; _adaptiveSpeed = kBaseSpeed; _lastCommandL = _lastCommandR = 0;
    _state = "STOP"; _diagnostic = "Waiting for valid sensor snapshot";
  }

  Side side() const { return _side; }
  const char* sideName() const { return _side == FOLLOW_RIGHT ? "RIGHT" : "LEFT"; }
  float commandLeft() const { return _lastCommandL; }
  float commandRight() const { return _lastCommandR; }
  float adaptiveSpeed() const { return _adaptiveSpeed; }
  uint8_t recoveryCount() const { return _recoveryCount; }
  const String& diagnostic() const { return _diagnostic; }

  void update(const Sensors& sensors, float& left, float& right, String& status) {
    left = right = 0;
    _lastCommandL = _lastCommandR = 0;
    const uint32_t now = millis();

    if (!sensors.lidarOk() || sensors.lidarCrcErrorPct() > 5.0f) {
      _phase = PH_FAULT_STOP; _state = "FAULT: LiDAR unavailable / CRC high - STOP";
      _diagnostic = "Check LiDAR power, UART wiring and packet/checksum status";
      finish(left, right, status); return;
    }

    const Sector front = sector(sensors, 86, 4);
    const Sector rightSide = sector(sensors, 13, 31);
    const Sector leftSide = sector(sensors, 59, 77);
    if (front.count < 2) {
      _phase = PH_FAULT_STOP; _state = "FAULT: front LiDAR coverage uncertain - STOP";
      _diagnostic = "Front scan has too few valid returns; inspect LiDAR orientation/coverage";
      finish(left, right, status); return;
    }

    uint16_t frontMin = front.minCm;
    if (sensors.usValid(0) && sensors.usAgeMs(0) <= 600) {
      int u = sensors.usCm(0); if (u > 0 && u < frontMin) frontMin = (uint16_t)u;
    }

    // Never auto-recover by turning into a close obstacle.
    if (frontMin > 0 && frontMin < kEmergencyStopCm) {
      _phase = PH_FAULT_STOP; _state = "STOP: obstacle inside emergency clearance";
      _diagnostic = "Front clearance too small. Remove obstacle and re-enable autonomy.";
      finish(left, right, status); return;
    }

    // Bounded stop-turn-settle self-correction. It is triggered only by suspected lack of
    // scene motion while a forward command was being issued; no encoder means this is an inference.
    if (_phase == PH_RECOVERY_STOP) {
      if (now - _phaseStartedMs < kRecoveryStopMs) {
        _state = "RECOVERY: stopped before corrective pivot";
        _diagnostic = "Suspected no progress; pause before a short full-speed pivot";
        finish(left, right, status); return;
      }
      const bool clearanceUnsafe = _recoveryIsCorrection
          ? (frontMin < kRecoveryFrontClearCm || rightSide.minCm < kRecoverySideClearCm ||
             leftSide.minCm < kRecoverySideClearCm)
          : (frontMin < kTurnFrontClearCm);
      if (clearanceUnsafe) {
        _phase = PH_FAULT_STOP; _state = "FAULT: turn lacks verified clearance";
        _diagnostic = "No safe pivot clearance confirmed. Manual inspection required.";
        finish(left, right, status); return;
      }
      _phase = PH_RECOVERY_PIVOT; _phaseStartedMs = now;
    }
    if (_phase == PH_RECOVERY_PIVOT) {
      if (now - _phaseStartedMs < kRecoveryPivotMs) {
        left = (_pendingTurn < 0) ? -kPivotSpeed : kPivotSpeed; right = -left;
        _state = (_pendingTurn < 0) ? "TURN: short left pivot" : "TURN: short right pivot";
        _diagnostic = "Bounded self-correction pulse; turn angle is not guaranteed";
        finish(left, right, status); return;
      }
      _phase = PH_RECOVERY_SETTLE; _phaseStartedMs = now; _pendingTurn = 0;
    }
    if (_phase == PH_RECOVERY_SETTLE) {
      if (now - _phaseStartedMs < kRecoverySettleMs) {
        _state = "RECOVERY: stopped; rechecking scan";
        _diagnostic = "Waiting for fresh LiDAR scene before resuming";
        finish(left, right, status); return;
      }
      _phase = PH_NORMAL; _stallSinceMs = 0;
      _lastSceneMs = now; _lastSignatureAtMs = 0; _signatureReady = false;
      if (_recoveryIsCorrection) {
        _adaptiveSpeed = min(kMaxAdaptiveSpeed, _adaptiveSpeed + kRecoverySpeedBoost);
        _recoveryIsCorrection = false;
      }
    }
    if (_phase == PH_FAULT_STOP) {
      _state = "FAULT STOP — disable autonomy and inspect";
      finish(left, right, status); return;
    }

    updateSceneMotion(sensors, now);

    const Sector selected = (_side == FOLLOW_RIGHT) ? rightSide : leftSide;
    const Sector opposite = (_side == FOLLOW_RIGHT) ? leftSide : rightSide;
    const bool selectedWall = isSurface(selected);
    const bool oppositeWall = isSurface(opposite);

    if (!selectedWall) {
      if (_lostSinceMs == 0) _lostSinceMs = now;
      const bool safeToSwitch = oppositeWall && frontMin >= kSwitchFrontClearCm &&
          opposite.minCm >= kSwitchSideClearCm && now - _lastSwitchMs >= kSwitchCooldownMs;
      if (safeToSwitch && now - _lostSinceMs >= kLostSurfaceSwitchDelayMs) {
        _side = (_side == FOLLOW_RIGHT) ? FOLLOW_LEFT : FOLLOW_RIGHT;
        _lastSwitchMs = now; _lostSinceMs = 0;
        _state = "STOP: switched side; reacquiring surface";
        _diagnostic = String("Switched to ") + sideName() + " only after checking clearance";
        finish(left, right, status); return;
      }
      _state = "STOP: surface not confidently detected";
      _diagnostic = "Move rover near a continuous wall/furniture surface or verify LiDAR angle mapping";
      finish(left, right, status); return;
    }
    _lostSinceMs = 0;

    const Sector follow = (_side == FOLLOW_RIGHT) ? rightSide : leftSide;
    const float distanceCm = (float)follow.medianCm;
    const float error = distanceCm - kTargetClearanceCm;
    int desiredTurn = 0;
    if (error < -kTurnDeadbandCm) desiredTurn = (_side == FOLLOW_RIGHT) ? -1 : 1;
    else if (error > kTurnDeadbandCm && frontMin >= kTurnFrontClearCm)
      desiredTurn = (_side == FOLLOW_RIGHT) ? 1 : -1;

    // Turn only after an explicit stop, at full pivot command. Then stop and re-evaluate.
    if (desiredTurn != 0) {
      left = right = 0;
      _state = "STOP before pivot";
      _diagnostic = String("Surface error ") + String(error, 0) + " cm; stop-turn-settle control";
      // A short pivot starts on the next controller pass after the stop phase.
      _pendingTurn = desiredTurn;
      _phase = PH_RECOVERY_STOP;
      _phaseStartedMs = now;
      _recoveryIsCorrection = false;
      finish(left, right, status); return;
    }

    // Detect a likely stall only during straight commanded movement. No encoders are present,
    // so unchanged LiDAR geometry is a suspicion, not proof that wheels did not move.
    if (_stallSinceMs == 0) _stallSinceMs = now;
    if (now - _lastSceneMs >= kNoSceneChangeMs && now - _stallSinceMs >= kMinDriveBeforeStallMs) {
      if (_recoveryCount >= kMaxRecoveries) {
        _phase = PH_FAULT_STOP; _state = "FAULT: no scene progress after recovery attempts";
        _diagnostic = "Stop and inspect wheels, motor power, rover Wi-Fi link and sensor geometry";
        finish(left, right, status); return;
      }
      ++_recoveryCount;
      _phase = PH_RECOVERY_STOP; _phaseStartedMs = now; _recoveryIsCorrection = true; _pendingTurn = 1;
      _state = "RECOVERY: suspected no movement";
      _diagnostic = "LiDAR scene unchanged while drive commanded; checking clearance before one bounded pivot";
      finish(left, right, status); return;
    }

    float base = _adaptiveSpeed;
    if (frontMin < kCautionFrontCm) base = min(base, kCautionSpeed);
    else if (frontMin < kSlowFrontCm) base = min(base, kSlowSpeed);
    left = right = base;
    _state = String("MOVE FORWARD · FOLLOW ") + sideName();
    _diagnostic = String("Front ") + String(frontMin) + " cm; surface " + String(follow.medianCm) +
                  " cm; commanded speed " + String(base, 2);
    finish(left, right, status);
  }

private:
  static constexpr uint16_t kEmergencyStopCm = 32;
  static constexpr uint16_t kCautionFrontCm = 65;
  static constexpr uint16_t kSlowFrontCm = 100;
  static constexpr uint16_t kSwitchFrontClearCm = 100;
  static constexpr uint16_t kSwitchSideClearCm = 55;
  static constexpr uint16_t kTargetClearanceCm = 55;
  static constexpr uint16_t kTurnDeadbandCm = 15;
  static constexpr uint16_t kTurnFrontClearCm = 95;
  static constexpr uint16_t kRecoveryFrontClearCm = 110;
  static constexpr uint16_t kRecoverySideClearCm = 45;
  static constexpr uint32_t kLostSurfaceSwitchDelayMs = 1200;
  static constexpr uint32_t kSwitchCooldownMs = 3000;
  static constexpr uint32_t kRecoveryStopMs = 300;
  static constexpr uint32_t kRecoveryPivotMs = 160;
  static constexpr uint32_t kRecoverySettleMs = 350;
  static constexpr uint32_t kNoSceneChangeMs = 3000;
  static constexpr uint32_t kMinDriveBeforeStallMs = 2800;
  static constexpr uint8_t kMaxRecoveries = 2;
  static constexpr float kBaseSpeed = 0.22f;          // raised from 0.10: verify on clear floor
  static constexpr float kSlowSpeed = 0.18f;
  static constexpr float kCautionSpeed = 0.15f;
  static constexpr float kMaxAdaptiveSpeed = 0.30f;   // bounded; never automatically jumps to full speed
  static constexpr float kRecoverySpeedBoost = 0.04f;
  static constexpr float kPivotSpeed = 0.50f;

  struct Sector { uint16_t minCm = 0, medianCm = 0; uint8_t count = 0; };
  enum InternalPhase : uint8_t { PH_NORMAL = 0, PH_RECOVERY_STOP = 1, PH_RECOVERY_PIVOT = 2,
                                 PH_RECOVERY_SETTLE = 3, PH_FAULT_STOP = 4 };
  Side _side = FOLLOW_RIGHT;
  InternalPhase _phase = PH_NORMAL;
  uint32_t _lostSinceMs = 0, _lastSwitchMs = 0, _phaseStartedMs = 0;
  uint32_t _lastSceneMs = 0, _lastSignatureAtMs = 0, _stallSinceMs = 0;
  bool _signatureReady = false, _recoveryIsCorrection = false;
  uint16_t _signature[18] = {};
  uint8_t _recoveryCount = 0;
  float _adaptiveSpeed = kBaseSpeed, _lastCommandL = 0, _lastCommandR = 0;
  int8_t _pendingTurn = 0;
  String _state = "STOP", _diagnostic = "Initialising";

  void finish(float& left, float& right, String& status) {
    _lastCommandL = left; _lastCommandR = right;
    status = _state;
    if (_pendingTurn && _phase == PH_RECOVERY_STOP && !_recoveryIsCorrection) {
      // pending wall-follow correction pivots after the stop interval; handled on subsequent pass
    }
  }

  void updateSceneMotion(const Sensors& sensors, uint32_t now) {
    if (now - _lastSignatureAtMs < 220) return;
    _lastSignatureAtMs = now;
    uint16_t current[18] = {};
    for (uint8_t i = 0; i < 18; ++i) {
      uint16_t d = sensors.lidarCm((uint8_t)(i * 5)); // 20-degree samples around full scan
      current[i] = (d >= 10 && d <= 400) ? d : 0;
    }
    if (!_signatureReady) {
      memcpy(_signature, current, sizeof(current));
      _signatureReady = true; _lastSceneMs = now; return;
    }
    uint8_t changed = 0, compared = 0;
    for (uint8_t i = 0; i < 18; ++i) {
      if (_signature[i] && current[i]) {
        ++compared;
        if (abs((int)_signature[i] - (int)current[i]) >= 18) ++changed;
      } else if (_signature[i] != current[i]) ++changed;
    }
    if (changed >= 3 || (compared >= 8 && changed >= 2)) _lastSceneMs = now;
    memcpy(_signature, current, sizeof(current));
  }

  static bool isSurface(const Sector& s) {
    return s.count >= 4 && s.medianCm >= 20 && s.medianCm <= 160;
  }

  static Sector sector(const Sensors& sensors, int start, int end) {
    Sector out; uint16_t vals[24]; uint8_t n = 0;
    int i = start;
    for (int guard = 0; guard < 90; ++guard) {
      uint16_t d = sensors.lidarCm((uint8_t)(i % 90));
      if (d >= 10 && d <= 300 && n < 24) {
        vals[n++] = d; if (out.minCm == 0 || d < out.minCm) out.minCm = d;
      }
      if (i == end) break;
      i = (i + 1) % 90;
    }
    out.count = n; if (!n) return out;
    for (uint8_t a = 0; a < n; ++a)
      for (uint8_t b = a + 1; b < n; ++b)
        if (vals[b] < vals[a]) { uint16_t t = vals[a]; vals[a] = vals[b]; vals[b] = t; }
    out.medianCm = vals[n / 2]; return out;
  }
};
