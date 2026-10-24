#pragma once
// CAS Box B3 / R07: scan-and-wander reactive navigation prototype.
// Uses all-around 90-bin LiDAR sector scoring, fresh ultrasonic safety vetoes,
// short bounded drive legs, and IMU-feedback 45-degree pivot steps.
// NOT safety-certified. Validate LiDAR orientation and motor polarity before floor tests.
#include <Arduino.h>
#include <math.h>
#include "Sensors.h"

class Navigation {
public:
  enum Side : uint8_t { FOLLOW_RIGHT = 0, FOLLOW_LEFT = 1 };

  void reset() {
    _phase = PH_SCAN; _phaseStartedMs = millis(); _driveStartedMs = 0;
    _turnStartYaw = 0; _turnDir = 0; _lastTurnDir = 0; _turnCount = 0;
    _faultLatched = false; _lastCommandL = _lastCommandR = 0;
    _state = "WANDER: scanning room"; _diagnostic = "Collecting a fresh 360-degree LiDAR sector view.";
  }
  Side side() const { return _side; }
  const char* sideName() const { return "WANDER"; }
  float commandLeft() const { return _lastCommandL; }
  float commandRight() const { return _lastCommandR; }
  float adaptiveSpeed() const { return kForwardSpeed; }
  uint8_t recoveryCount() const { return _turnCount; }
  const String& diagnostic() const { return _diagnostic; }

  void update(const Sensors& s, float& left, float& right, String& status) {
    left = right = 0; _lastCommandL = _lastCommandR = 0;
    const uint32_t now = millis();
    if (_faultLatched) {
      _state = "FAULT LATCHED: disable/re-enable autonomy";
      _diagnostic = _faultReason;
      finish(left, right, status); return;
    }
    if (!s.lidarOk() || s.lidarCrcErrorPct() > kMaxCrcPct) {
      latchFault("FAULT: LiDAR unavailable / CRC high", "No reliable all-around scan; check LiDAR power, UART, packet rate and checksum.", left, right, status); return;
    }

    const Sector front = sector(s, 0, 3);
    if (front.count < kMinSectorReturns) {
      latchFault("FAULT: front scan unknown", "Too few valid LiDAR returns in the forward sector. Verify scan orientation/coverage.", left, right, status); return;
    }

    // Very close valid readings are a hard stop. At normal obstacle distances the controller
    // stops and chooses another sector instead of permanently latching a recoverable obstacle.
    int nearFront = front.minCm;
    if (freshUs(s, 0)) nearFront = min(nearFront, usableUs(s, 0));
    if (freshUs(s, 4)) nearFront = min(nearFront, usableUs(s, 4));
    if (nearFront < kHardStopCm) {
      latchFault("FAULT: object inside hard-stop distance", String("Closest forward/angled reading ") + String(nearFront) + " cm. Disable autonomy, clear the object, then re-enable.", left, right, status); return;
    }

    if (_phase == PH_TURN) {
      if (!s.imuReady() || s.gyroCalibration() < 2) {
        _phase = PH_SCAN;
        _state = "STOP: turn feedback unavailable";
        _diagnostic = "A measured 45-degree turn requires a working IMU with calibrated gyro; refusing a blind full-speed pivot.";
        finish(left, right, status); return;
      }
      const float delta = wrap180(s.yawDeg() - _turnStartYaw);
      const float expected = _turnDir > 0 ? 1.0f : -1.0f;
      const float progress = delta * expected;
      if (progress >= kTurnTargetDeg - kTurnToleranceDeg) {
        _phase = PH_SETTLE; _phaseStartedMs = now; _turnCount++; _lastTurnDir = _turnDir;
        _state = "TURN: 45 degrees reached";
        _diagnostic = String("IMU turn delta=") + String(delta, 1) + " deg; stopped for settle and rescan.";
        finish(left, right, status); return;
      }
      if (progress < -8.0f || now - _phaseStartedMs > kTurnTimeoutMs) {
        latchFault("FAULT: turn feedback mismatch",
          String("Expected ") + (expected > 0 ? "left" : "right") + " turn; measured delta=" + String(delta, 1) +
          " deg. Check motor polarity, IMU yaw sign, wheel traction and battery before re-enabling.", left, right, status);
        return;
      }
      // Maximum configured pivot command, bounded by IMU angle feedback and timeout.
      const float turnCmd = progress >= 27.0f ? kTurnFinishCommand : kPivotCommand;
      left = _turnDir > 0 ? -turnCmd : turnCmd;
      right = -left;
      _state = _turnDir > 0 ? "TURN: left 45-degree step" : "TURN: right 45-degree step";
      _diagnostic = String("Measured yaw progress=") + String(progress, 1) + "/45 deg; pivot command=" + String(kPivotCommand, 2) + ".";
      finish(left, right, status); return;
    }

    if (_phase == PH_SETTLE) {
      if (now - _phaseStartedMs < kSettleMs) {
        _state = "SCAN: stopped, settling";
        _diagnostic = "Motors zero; allow chassis motion and LiDAR view to settle before selecting the next sector.";
        finish(left, right, status); return;
      }
      _phase = PH_SCAN;
    }

    if (_phase == PH_DRIVE_LEG) {
      // Re-check front and angled-front every controller cycle. Never continue a leg on a close return.
      if (nearFront < kAvoidDistanceCm || !frontHasRoom(front)) {
        _phase = PH_SCAN;
        _state = "WANDER: obstacle detected; rescanning";
        _diagnostic = String("Forward path no longer clear; closest reading=") + String(nearFront) + " cm.";
        finish(left, right, status); return;
      }
      if (now - _driveStartedMs >= kDriveLegMs) {
        _phase = PH_SCAN;
        _state = "WANDER: leg complete; rescan";
        _diagnostic = "Short open-loop leg complete. No encoders are fitted, so this is time-bounded motion, not a measured distance.";
        finish(left, right, status); return;
      }
      left = right = kForwardSpeed;
      _state = "WANDER: moving through open sector";
      _diagnostic = String("Forward sector minimum=") + String(front.minCm) + " cm; next full scan in bounded leg.";
      finish(left, right, status); return;
    }

    // PH_SCAN: evaluate 8 candidate headings at 45-degree intervals over the 360-degree scan.
    // Bin 0 is assumed forward; bins increase clockwise. Zero LiDAR bins are unknown, not free.
    const int centers[8] = {0, 11, 22, 34, 45, 56, 68, 79};
    int best = -1; float bestScore = -1e9f; Sector bestSector;
    const bool forwardClear = frontHasRoom(front) && nearFront >= kAvoidDistanceCm;
    if (forwardClear) {
      _phase = PH_DRIVE_LEG; _driveStartedMs = now;
      _state = "WANDER: forward sector clear";
      _diagnostic = String("Forward clearance=") + String(nearFront) + " cm; moving only for a short bounded leg before rescanning.";
      left = right = kForwardSpeed; finish(left, right, status); return;
    }

    for (int c = 0; c < 8; ++c) {
      Sector cand = sector(s, centers[c], 3);
      if (cand.count < kMinSectorReturns || cand.minCm < kTurnClearanceCm) continue;
      // A pivot always starts with one 45-degree step, so validate the immediate swept direction,
      // not just the eventual candidate heading several steps away.
      const int signedBins = centers[c] == 45 ? (_lastTurnDir >= 0 ? -45 : 45) : (centers[c] < 45 ? -centers[c] : 90 - centers[c]);
      if (!turnSweepClear(s, signedBins < 0 ? 11 : 79)) continue;
      // Reward robust minimum clearance; modestly penalize large heading changes and repeated turns.
      const float angle = signedBins * 4.0f;
      const float absAngle = fabsf(angle);
      float score = (float)cand.minCm * 1.0f + (float)cand.medianCm * 0.20f - absAngle * 0.25f;
      if ((_lastTurnDir > 0 && angle < 0) || (_lastTurnDir < 0 && angle > 0)) score -= 4.0f;
      if (c == 4) score -= 8.0f; // do not prefer a full reversal unless it is materially safer
      if (score > bestScore) { bestScore = score; best = c; bestSector = cand; }
    }

    if (best < 0) {
      _state = "STOP: no verified free sector";
      _diagnostic = "All candidate headings have insufficient valid LiDAR returns or clearance. Unknown space is not treated as free; reposition manually.";
      finish(left, right, status); return;
    }

    // Step only 45 degrees per decision, then stop and rescan. This avoids a blind 90/180-degree spin.
    const int targetBin = centers[best];
    const int signedBins = targetBin == 45 ? (_lastTurnDir >= 0 ? -45 : 45) : (targetBin < 45 ? -targetBin : 90 - targetBin);
    if (abs(signedBins) < 5) {
      // Best sector is essentially forward; don't spin due to bin rounding.
      if (nearFront >= kAvoidDistanceCm && frontHasRoom(front)) {
        _phase = PH_DRIVE_LEG; _driveStartedMs = now; left = right = kForwardSpeed;
        _state = "WANDER: open forward sector"; _diagnostic = "Forward sector selected; bounded move then rescan.";
      } else {
        _state = "STOP: forward sector still blocked";
        _diagnostic = "Selected heading is too close to forward while forward clearance is insufficient; no unsafe micro-turn attempted.";
      }
      finish(left, right, status); return;
    }

    if (!s.imuReady() || s.gyroCalibration() < 2) {
      _state = "STOP: IMU gyro not calibrated";
      _diagnostic = "A measured 45-degree pivot requires a working calibrated gyro. Calibrate the BNO055 before autonomous turning.";
      finish(left, right, status); return;
    }
    _turnDir = signedBins < 0 ? -1 : 1; // positive yaw = left; negative yaw = right
    _turnStartYaw = s.yawDeg(); _phaseStartedMs = now; _phase = PH_TURN;
    _state = _turnDir > 0 ? "TURN: selected open sector left" : "TURN: selected open sector right";
    _diagnostic = String("Chosen sector min=") + String(bestSector.minCm) + " cm, median=" + String(bestSector.medianCm) +
      " cm; turning one measured 45-degree step at pivot command " + String(kPivotCommand, 2) + ", then rescanning.";
    finish(left, right, status);
  }

private:
  static constexpr uint16_t kHardStopCm = 15;
  static constexpr uint16_t kAvoidDistanceCm = 42; // short-horizon clearance for the compact R07 chassis
  static constexpr uint16_t kTurnClearanceCm = 32; // local swept-sector clearance; still require physical validation
  static constexpr uint16_t kMinSectorReturns = 4;
  static constexpr float kMaxCrcPct = 5.0f;
  static constexpr uint32_t kUsMaxAgeMs = 600;
  static constexpr uint32_t kDriveLegMs = 650;
  static constexpr uint32_t kSettleMs = 300;
  static constexpr uint32_t kTurnTimeoutMs = 1500;
  static constexpr float kTurnTargetDeg = 45.0f;
  static constexpr float kTurnToleranceDeg = 7.0f;
  static constexpr float kForwardSpeed = 0.16f;
  static constexpr float kPivotCommand = 0.50f; // maximum configured differential-drive pivot command
  static constexpr float kTurnFinishCommand = 0.24f; // taper near target to limit overshoot

  struct Sector { uint16_t minCm = 0, medianCm = 0; uint8_t count = 0; };
  enum Phase : uint8_t { PH_SCAN, PH_TURN, PH_SETTLE, PH_DRIVE_LEG };
  Side _side = FOLLOW_RIGHT;
  Phase _phase = PH_SCAN;
  uint32_t _phaseStartedMs = 0, _driveStartedMs = 0;
  float _turnStartYaw = 0;
  int8_t _turnDir = 0, _lastTurnDir = 0;
  uint16_t _turnCount = 0;
  bool _faultLatched = false;
  float _lastCommandL = 0, _lastCommandR = 0;
  String _state = "WANDER: scanning room", _diagnostic = "Initialising";
  String _faultReason = "Fault latched; disable autonomy, correct the cause, and re-enable.";

  void finish(float& l, float& r, String& status) {
    _lastCommandL = l; _lastCommandR = r; status = _state;
  }
  void latchFault(const char* state, const String& diag, float& l, float& r, String& status) {
    _phase = PH_SCAN; _faultLatched = true; _faultReason = diag;
    l = r = 0; _state = state; _diagnostic = diag; finish(l, r, status);
  }
  static float wrap180(float a) {
    while (a > 180.0f) a -= 360.0f;
    while (a < -180.0f) a += 360.0f;
    return a;
  }
  static bool freshUs(const Sensors& s, uint8_t i) {
    return s.usValid(i) && s.usAgeMs(i) <= kUsMaxAgeMs;
  }
  static int usableUs(const Sensors& s, uint8_t i) {
    const int d = s.usCm(i);
    return (d > 0 && d < Sensors::kUsMaxCm) ? d : 1000;
  }
  static Sector sector(const Sensors& s, int center, int radius) {
    Sector out; uint16_t vals[9]; uint8_t n = 0;
    for (int off = -radius; off <= radius; ++off) {
      int idx = (center + off + 90) % 90;
      uint16_t d = s.lidarCm((uint8_t)idx);
      if (d >= 10 && d <= 350 && n < 9) {
        vals[n++] = d;
        if (!out.minCm || d < out.minCm) out.minCm = d;
      }
    }
    out.count = n; if (!n) return out;
    for (uint8_t a = 0; a < n; ++a) for (uint8_t b = a + 1; b < n; ++b)
      if (vals[b] < vals[a]) { uint16_t t = vals[a]; vals[a] = vals[b]; vals[b] = t; }
    out.medianCm = vals[n / 2]; return out;
  }
  static bool frontHasRoom(const Sector& front) {
    return front.count >= kMinSectorReturns && front.minCm >= kTurnClearanceCm;
  }
  static bool turnSweepClear(const Sensors& s, int targetBin) {
    // Reject candidate if its near-side sector or a fresh side ultrasonic shows a close object.
    const int sideIndex = targetBin < 45 ? 2 : 1; // clockwise bins are right; counter-clockwise bins left
    const Sector side = sector(s, targetBin, 4);
    if (side.count < kMinSectorReturns || side.minCm < kTurnClearanceCm) return false;
    if (freshUs(s, sideIndex)) {
      const int d = usableUs(s, sideIndex);
      if (d < 22) return false; // close side obstacle veto; footprint margin is handled by LiDAR sector clearance
    }
    // During rotation, rear-facing proximity is also relevant to the chassis sweep.
    if (freshUs(s, 3) && usableUs(s, 3) < 20) return false;
    return true;
  }
};
