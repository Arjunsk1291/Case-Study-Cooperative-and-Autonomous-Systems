#pragma once
// StripFollow - align onto a dark strip on the floor and follow it to a wall (2026-11-07, indoor test of the
// tactile-strip idea). Uses: camera result (CamLink), gyro heading hold + trim + failsafe (DriveStraight),
// free space ahead (LiDAR, TrimCal::frontM), GentleDrive ramps (in cas_box_drive.ino).
//
// HOW IT STEERS: the camera never drives the wheels directly. Each camera message (10/s) moves the TARGET
// HEADING a little towards the strip (strip angle + offset), at most 8 deg/s and +-25 deg from the start
// heading. DriveStraight (Kethan's heading hold, field-tested) holds that heading. Without camera data the
// target simply stays -> the rover keeps its gyro heading.
//
// RUN (button on the page):
//   ALIGN   (optional) camera sees the strip angle -> turn on the spot (0.50, gyro) until it points along
//           the strip (|angle| < 6 deg), up to 3 tries. No strip in view -> stop "no strip in view".
//   FOLLOW  drive at `speed` (ramped), steer on the strip as above.
//   STOP    LiDAR ahead <= stopWallM            -> gentle stop "reached the wall"   (the goal)
//           ahead < 0.30 m / tilt               -> emergency ramp
//           strip not seen 1 s                  -> slow (0.15); 3 s or 0.8 m -> stop "strip lost"
//           camera data stale                   -> as "not seen" (gyro only)
//           DriveStraight failsafe, IMU lost, page heartbeat lost 1.2 s, move button, STOP,
//           maxDistM, maxTimeS                  -> gentle stop with the reason
// LOG: 10 samples/s of the last run in RAM (60 s), runs table (20). Download from the page. No flash writes.
#include <Arduino.h>
#include <ESPAsyncWebServer.h>
#include "Sensors.h"
#include "RoverLink.h"
#include "CamLink.h"
#include "DriveStraight.h"

class StripFollow {
public:
  enum Phase : uint8_t { IDLE = 0, ALIGN_TURN, ALIGN_SETTLE, FOLLOW, STOPPING };

  // ---- settings (page can change speed / stop distance / max distance)
  float speed = 0.20f;
  float stopWallM = 0.50f, hardM = 0.30f, tiltDeg = 12.0f;
  float maxDistM = 6.0f, maxTimeS = 90.0f;
  float kAng = 0.6f;          // deg of heading correction per deg of strip angle
  float kOff = 0.25f;         // deg of heading correction per px of strip offset (upright image is 120 px wide)
  float tgtRateDegS = 8.0f;   // target heading moves at most this fast
  float tgtMaxDeg = 25.0f;    // ... and at most this far from the heading at the start of FOLLOW
  float alignDoneDeg = 6.0f;
  uint32_t heartbeatMs = 1200;
  float mPerUnit = 1.03f;

  bool begin(AsyncWebServer* server);
  void update(const Sensors& s, RoverLink& rover, const CamLink& cam, float frontM, bool padActive,
              float& l, float& r, bool& emergency);
  void noteOutput(float l, float r) { _outV = 0.5f * (l + r); _outL = l; _outR = r; }
  bool active() const { return _phase != IDLE; }
  bool blocked = false;       // set every loop pass: another run (trim tool) is active -> a start is REFUSED, never delayed
  float offZero() const { return _offZero; }

private:
  struct Smp { uint16_t tDs; uint8_t ph, flags, rows; int16_t vX1000, lX1000, rX1000, yawX10, tgtX10, offX10, angX10; uint16_t frontCm; };
  struct Run { uint8_t n, what; float spd, distM, timeS, seenPct, offAbsMax, offEnd, frontEnd; uint8_t pivots; char why[40]; };
  static const uint16_t kSmp = 600;
  static const uint8_t kRuns = 20;

  void _start(const Sensors& s, RoverLink& rover, const CamLink& cam);
  void _stop(const char* why, bool hard);
  void _finish();
  String _statusJson(const Sensors& s, const CamLink& cam, float frontM);
  String _logCsv();
  String _runsCsv();
  float _offCorr(const CamLink& cam) const { return cam.data().offX10 / 10.0f - _offZero; }

  // requests (web task)
  volatile bool _reqStart = false, _reqStop = false, _reqZero = false;
  volatile uint8_t _reqWhat = 2;      // 0 align only, 1 follow only, 2 align + follow
  volatile float _reqSpd = 0.20f, _reqStopM = 0.50f, _reqMaxM = 6.0f;
  volatile uint32_t _aliveMs = 0;

  // run state (loop)
  volatile uint8_t _phase = IDLE;
  uint8_t _what = 2, _tries = 0, _pivots = 0;
  DriveStraight _ds;
  float _offZero = 0;
  float _tgt = 0, _yaw0 = 0, _alignTgt = 0, _rate = 0, _prevYaw = 0;
  float _outV = 0, _outL = 0, _outR = 0, _dist = 0, _distSinceStrip = 0;
  uint32_t _runMs = 0, _phaseMs = 0, _lastMs = 0, _lastStripMs = 0, _lastCamSeq = 0, _lastSmpMs = 0;
  uint32_t _seenN = 0, _framesN = 0;
  float _offAbsMax = 0;
  bool _hard = false;
  char _why[40] = "", _msg[64] = "";

  // shared with the web task
  SemaphoreHandle_t _mtx = nullptr;
  Smp _smp[kSmp]; uint16_t _nSmp = 0;
  Run _runs[kRuns]; uint8_t _nRuns = 0, _runNo = 0;
  String _status = "{}";
  uint32_t _statusMs = 0;
};
