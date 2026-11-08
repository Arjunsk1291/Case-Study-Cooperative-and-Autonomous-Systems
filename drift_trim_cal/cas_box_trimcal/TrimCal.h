#pragma once
// TrimCal - wheel-trim calibration for the drift of the WAVE ROVER (v2, 2026-11-06).
//
// IDEA: while "driving straight" the rover turns slowly by itself (outdoors, logs 009-013, equal wheel
// commands, no gyro correction: 2.3-4.5 deg/s to the right). A fixed TRIM gives one side a little
// more and the other a little less:
//      left = v * (1 - trim)     right = v * (1 + trim)
//      trim > 0 -> right side faster -> turns LEFT;   trim < 0 -> turns RIGHT
// TrimCal finds the trim that makes the yaw rate zero. It is OPEN LOOP on purpose: no heading hold
// (no Kethan PD) while measuring, otherwise the controller would hide the drift we want to measure.
// The final navigation program uses the trim (feed-forward) PLUS a gentle heading hold (feedback).
//
// ONE RUN:
//   BIAS     2 s standing still: gyro's own drift (subtracted)
//   RAMP     0.05 -> v in 2.5 s (gentle start; the rover only starts rolling at ~0.12)
//   SETTLE   0.6 s at v, not measured
//   MEASURE  up to measureS s: yaw rate = slope of a straight line fitted to the yaw
//            AUTO: the trim moves slowly against the drift (max 0.01 / s);  FIXED / VERIFY: trim fixed
//   STOPPING gentle stop. Also: < 0.70 m free ahead (data so far counts), heartbeat lost, STOP,
//            drive button, IMU invalid. < 0.40 m or tilt: faster ramp (still a ramp, never a jump).
//
// GUIDED STEPS per speed (the page shows them):  1 AUTO (rough, 2-3 runs)
//   2 MEASURE: FIXED runs at suggestion, +0.01, -0.01 (cycled by the box)   3 VERIFY at the suggestion:
//   PASS if |drift| < passDegS. Suggestion = straight line through (trim, drift) of all runs, at zero.
//
// MEMORY: RAM only (runs 32 x ~72 B, last-run samples 300 x 16 B at 10 Hz). Flash write ONLY on
// "Save" (2 floats, NVS). Nothing is ever written to the rover (its ratio T:139 can only be READ).
#include <Arduino.h>
#include <math.h>
#include <Preferences.h>
#include <ESPAsyncWebServer.h>
#include "Sensors.h"
#include "RoverLink.h"

class TrimCal {
public:
  enum Phase : uint8_t { IDLE = 0, BIAS, RAMP, SETTLE, MEASURE, STOPPING };
  enum Kind : uint8_t { AUTO = 0, FIXED = 1, VERIFY = 2, CUSTOM = 3 };

  // ---- tuning ----
  float frontSoftM = 0.70f;   // closer: gentle stop (run ends, data so far counts)
  float frontHardM = 0.40f;   // closer: emergency ramp
  float tiltDeg = 12.0f;
  float minMeasureS = 1.5f;   // shorter MEASURE: shown, but not used
  float measureS = 4.0f;      // MEASURE length (s) - ~1 m at 0.25
  float trimMax = 0.15f;
  float trimRate = 0.02f;     // AUTO: max trim change per second
  float autoTauS = 2.0f;      // with the measured gain (log3) AUTO now really moves the trim
  float passDegS = 0.30f;     // VERIFY: |drift| below this = PASS (= 3 deg in 10 s)
  float fixedStep = 0.04f;    // MEASURE step: suggestion, +step, -step. log3: 8.6 deg/s per unit trim ->
                              // +-0.04 changes the drift by ~0.35 deg/s, 5x the run-to-run scatter (0.07)
  float mPerUnit = 1.03f;     // m/s per unit forward command (log_027), distance estimate only
  int usNearCm = 25;          // front ultrasonic: obstacle only below this ...
  uint32_t usHoldMs = 600;    // ... continuously this long (it hears the floor at ~60-70 cm)
  uint32_t heartbeatMs = 1200;   // page heartbeat lost this long -> gentle stop (0.8 s stopped runs on short phone hiccups)

  bool begin(AsyncWebServer* server);
  void setResetReason(const String& r) { _reset = r; }
  // every loop pass, BEFORE GentleDrive. padActive = a move button is held.
  void update(const Sensors& s, RoverLink& rover, bool padActive, float& l, float& r, bool& emergency);
  // every loop pass, AFTER GentleDrive: what really goes to the rover (log + distance)
  void noteOutput(float l, float r) { _outV = 0.5f * (l + r); }
  bool active() const { return _phase != IDLE; }
  float trim() const { return _trim; }
  float yawRateDegS() const { return _rateF - _bias; }
  float frontM() const { return _frontM; }
  String phaseText() const;

private:
  struct Run {
    uint8_t n, kind; float spd, trim0, trim1, trimMean, omega, resid, bias, measS, distM, batt;
    char why[40];
  };
  struct Sample { uint16_t tDs; uint8_t phase; int16_t vX1000, trimX10000, rateX100; float yaw; uint16_t frontCm; };
  static const uint8_t kRuns = 32;
  static const uint16_t kSamples = 300;

  static int _si(float spd) { return spd < 0.225f ? 0 : 1; }   // speed index: 0 = 0.20, 1 = 0.25
  float _plannedTrim(uint8_t kind, int si, bool& ok, char* no, size_t n) const;
  void _start(const Sensors& s, RoverLink& rover);
  void _finish(const char* why);
  void _stopping(const char* why, bool hard);
  float _front(const Sensors& s);
  String _statusJson(const Sensors& s, RoverLink& rover);
  String _resultsCsv();
  String _lastCsv();
  bool _suggest(float spd, float& t, int& nRuns, bool& fitted) const;

  // requests from the web task
  volatile bool _reqStart = false, _reqStop = false, _reqRate = false, _reqSave = false, _reqClear = false;
  volatile float _reqSpd = 0.25f, _reqTrim = 0;
  volatile uint8_t _reqKind = AUTO;
  volatile uint32_t _aliveMs = 0;

  // run state (loop only)
  volatile uint8_t _phase = IDLE;
  uint8_t _kind = AUTO;
  float _spd = 0.25f, _trim = 0, _trim0 = 0, _outV = 0;
  uint32_t _phaseMs = 0, _runMs = 0, _lastMs = 0, _lastSampleMs = 0, _closeUsMs = 0, _imuBadMs = 0;
  bool _hard = false;
  float _yawPrev = 0, _yawU = 0, _rateF = 0, _frontM = 9.9f, _dist = 0, _batt = 0, _y0 = 0;
  float _lidarFrontM = 9.9f;   // LiDAR alone (debug)
  int _usFrontCm = 0;          // front ultrasonic raw (debug)
  bool _closeUs = false;
  double _bN = 0, _bT = 0, _bY = 0, _bTT = 0, _bTY = 0;
  double _mN = 0, _mT = 0, _mY = 0, _mTT = 0, _mTY = 0, _mYY = 0, _trimSum = 0;
  float _bias = 0;
  char _why[40] = "";
  char _msg[64] = "";          // last refusal / info for the page

  // guided steps, per speed index
  float _autoTrim[2] = {0, 0};
  uint8_t _autoN[2] = {0, 0}, _fixedK[2] = {0, 0}, _verifyN[2] = {0, 0};
  float _verifyRate[2] = {NAN, NAN}, _verifyTrim[2] = {NAN, NAN};

  // shared with the web task (guarded by _mtx)
  SemaphoreHandle_t _mtx = nullptr;
  Run _runs[kRuns];
  uint8_t _nRuns = 0, _runNo = 0;
  Sample _smp[kSamples];
  uint16_t _nSmp = 0;
  String _status = "{}";
  uint32_t _statusMs = 0;
  String _rateReply, _reset;
  float _saved[2] = {NAN, NAN};
};
