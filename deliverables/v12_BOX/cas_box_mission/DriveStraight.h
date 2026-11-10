#pragma once
// DriveStraight - straight driving for kit 04 = fixed wheel trim + Kethan's gyro heading hold + failsafe.
// (2026-11-07, see hardcoded/README.md). Header-only, no Arduino dependency except millis() via the caller.
//
//   1 FIXED TRIM (measured, RoverTrim.h):   left = v*(1 - trim)   right = v*(1 + trim)
//   2 KETHAN'S HEADING HOLD (unchanged, field-tested in log_025, hold error < 2.3 deg):
//        c = 0.045 * e - 0.0015 * yawRate,  |c| <= min(0.10, 0.4 * v),  c changes <= 0.30 per second
//        left -= c, right += c          (e = target heading - yaw, + = turn left)
//   3 IMPROVEMENT - TRIM LEARNING: when the ground keeps pushing the rover to one side (outdoors: 2-4 deg/s
//        right in the old logs), the heading hold needs a constant correction c all the time. That
//        constant part is slowly (time constant learnTauS) moved into an extra trim, so c goes back
//        to ~0 and the full correction range is free again for real disturbances. Only while driving
//        straight with a small error (no learning from bumps or turns). Limited to +-learnMax.
//   4 FAILSAFE (latched until reset(); the caller ramps the speed to 0 with GentleDrive):
//        - IMU invalid for imuLostMs                                -> FAIL "IMU lost"
//        - heading error > failErrDeg for failErrMs                 -> FAIL "heading off"
//        - correction at its limit for failSatMs (drift too strong) -> FAIL "correction saturated"
//        - 3 gyro jumps > jumpDeg in a row (single jumps ignored)   -> FAIL "gyro jumps"
//        WARN (keeps driving) above warnErrDeg or when the learned trim reaches its limit.
//
//   DriveStraight ds;
//   ds.start(yawDeg, nowMs);                 // hold the current heading (or ds.setTarget(deg))
//   every loop: auto st = ds.update(imuOk, yawDeg, v, nowMs, l, r);   // v = wanted forward speed (>0)
//               if (st == DriveStraight::FAIL) -> stop (GentleDrive ramps), show ds.reason()
#include <math.h>
#include <stdint.h>
#include "RoverTrim.h"

class DriveStraight {
public:
  enum Status : uint8_t { OK = 0, WARN = 1, FAIL = 2 };

  // 1 fixed trim
  float trim = kTrimIndoor025;
  // 2 Kethan (unchanged values)
  float kP = 0.045f, kD = 0.0015f, corrMax = 0.10f, corrFrac = 0.40f, corrRate = 0.30f, deadDeg = 0.5f;
  // 3 trim learning
  bool learn = true;
  // v10.3 (new rover): the first 40 s of driving learn fast (6 s -> 15 s), limit 0.40 (about +-3.4 deg/s of natural drift; 0.20 failed at +-4)
  float learnTauS = 15.0f, learnTauFastS = 6.0f, learnMax = 0.60f, learnBelowDeg = 6.0f;
  // 4 failsafe
  float failErrDeg = 12.0f, warnErrDeg = 5.0f, jumpDeg = 30.0f;
  uint32_t failErrMs = 2000, failSatMs = 5000, imuLostMs = 500;

  void start(float yawDeg, uint32_t nowMs) { reset(); _target = yawDeg; _prevYaw = yawDeg; _lastMs = nowMs; _started = true; }
  void setTarget(float yawDeg) { _target = yawDeg; }
  void reset() { _status = OK; _reason = ""; _c = 0; _rate = 0; _errOn = _satOn = _imuBad = false; _jumps = 0; _started = false; }
  void clearLearned() { _learned = 0; }

  Status update(bool imuOk, float yawDeg, float v, uint32_t nowMs, float& l, float& r) {
    l = r = 0;
    if (_status == FAIL || !_started) return _status;
    const float dt = (nowMs - _lastMs) / 1000.0f;
    _lastMs = nowMs;
    if (!imuOk) {
      if (!_imuBad) { _imuBad = true; _imuBadMs = nowMs; }
      if (nowMs - _imuBadMs >= imuLostMs) return _fail("IMU lost");
      _out(v, l, r);                                   // short gap: keep the last correction
      return _status;
    }
    _imuBad = false;
    const float d = _wrap(yawDeg - _prevYaw);
    if (fabsf(d) > jumpDeg) {                          // one bad sample: ignore it
      if (++_jumps >= 3) return _fail("gyro jumps");
      _out(v, l, r);
      return _status;
    }
    _jumps = 0;
    _prevYaw = yawDeg;
    if (dt > 0 && dt < 0.1f) {
      const float keep = powf(0.7f, dt / 0.02f);       // Kethan's ~0.06 s yaw-rate filter
      _rate = keep * _rate + (1.0f - keep) * d / dt;
    } else {
      _rate = 0;
    }
    _err = _wrap(_target - yawDeg);
    const float ae = fabsf(_err);

    // 2 Kethan's heading hold
    const float limit = fminf(corrMax, corrFrac * fabsf(v));
    const float want = _clamp((ae > deadDeg ? kP * _err : 0.0f) - kD * _rate, -limit, limit);
    if (dt <= 0 || dt >= 0.1f) _c = 0;
    else _c += _clamp(want - _c, -corrRate * dt, corrRate * dt);
    _c = _clamp(_c, -limit, limit);

    // 3 learning: move the steady part of c into the trim (c = v * trimEquivalent)
    if (learn && v > 0.1f && ae < learnBelowDeg && dt > 0 && dt < 0.1f) {
      const float tau = learnTauFastS + (learnTauS - learnTauFastS) * fminf(1.0f, _driveS / 40.0f);
      const float move = (_c / v) * dt / tau;
      _learned = _clamp(_learned + move, -learnMax, learnMax);
      _c -= move * v;                                  // the same amount leaves the correction: output unchanged now
    }

    if (v > 0.1f && dt > 0 && dt < 0.1f) _driveS += dt;      // total driving time (not reset by start(): the learned trim stays too)
    // 4 failsafe
    _status = OK; _reason = "";
    // timers: a separate "running" flag + the start time (2026-11-07: "start = now | 1" could lie 1 ms in the
    // future -> the unsigned difference wrapped around -> every hold time fired at once)
    if (ae > failErrDeg) { if (!_errOn) { _errOn = true; _errMs = nowMs; } if (nowMs - _errMs >= failErrMs) return _fail("heading off > 12 deg for 2 s"); }
    else _errOn = false;
    if (limit > 0 && fabsf(_c) >= 0.98f * limit) { if (!_satOn) { _satOn = true; _satMs = nowMs; } if (nowMs - _satMs >= failSatMs) return _fail("correction saturated 5 s (drift too strong)"); }
    else _satOn = false;
    if (ae > warnErrDeg) { _status = WARN; _reason = "heading error > 5 deg"; }
    else if (fabsf(_learned) >= 0.98f * learnMax) { _status = WARN; _reason = "learned trim at its limit"; }

    _out(v, l, r);
    return _status;
  }

  Status status() const { return _status; }
  const char* reason() const { return _reason; }
  float errorDeg() const { return _err; }          // + = target is to the left
  float correction() const { return _c; }
  float driveSeconds() const { return _driveS; }
  float yawRateDegS() const { return _rate; }
  float learned() const { return _learned; }       // extra trim learned on this ground
  float totalTrim() const { return trim + _learned; }

private:
  void _out(float v, float& l, float& r) const {
    const float t = trim + _learned;
    l = _clamp(v * (1.0f - t) - _c, -0.5f, 0.5f);
    r = _clamp(v * (1.0f + t) + _c, -0.5f, 0.5f);
  }
  Status _fail(const char* why) { _status = FAIL; _reason = why; _c = 0; return FAIL; }
  static float _wrap(float a) { a = fmodf(a + 180.0f, 360.0f); if (a < 0) a += 360.0f; return a - 180.0f; }
  static float _clamp(float x, float lo, float hi) { return x < lo ? lo : (x > hi ? hi : x); }

  Status _status = OK;
  const char* _reason = "";
  bool _started = false;
  float _target = 0, _prevYaw = 0, _rate = 0, _c = 0, _err = 0, _learned = 0, _driveS = 0;
  uint32_t _lastMs = 0, _errMs = 0, _satMs = 0, _imuBadMs = 0;
  bool _errOn = false, _satOn = false, _imuBad = false;
  uint8_t _jumps = 0;
};
