#pragma once
// RouteMission - supervised mission B facade -> first right turn -> across the open area ("open sea") to its
// end wall (2026-11-07). Built from what the logs and the simulation showed:
//   - straight driving = gyro heading hold + measured trim + failsafe (DriveStraight, Kethan's PD)
//   - leg 1 follows the LEFT wall at the start distance; pillars are counted and logged, but the turn is
//     CONFIRMED by the operator (pillar counting differed between log_026 and log_027)
//   - the CRAB (sliding sideways while pointing straight, invisible to the gyro) is MEASURED on leg 1 from the
//     wall distance and compensated on leg 2 (prior -4 deg if leg 1 gave too few samples)
//   - the camera is a HELPER on demand: 3 snapshots after the turn, then 2 every 2.6 m. A failed / missing
//     snapshot never stops the rover: it continues straight on the gyro.
//   - front stop / end wall use the LiDAR with a GROUND FILTER on slopes (the tilted beam hits the ramp:
//     48 % of front readings < 1.5 m were ramp floor in logs 013-027; LiDAR height ~0.25 m)
//   - people in front: stop, wait until clear for 2 s, continue on the same heading (no swerving)
// Everything is logged (MissionLog). Every operator decision becomes a log event.
#include <Arduino.h>
#include <ESPAsyncWebServer.h>
#include "Sensors.h"
#include "RoverLink.h"
#include "CamLink.h"
#include "DriveStraight.h"
#include "WallFit.h"
#include "MissionLog.h"

class RouteMission {
public:
  enum St : uint8_t { IDLE = 0, L1_FOLLOW, T1_ASK, T1_TURN, T1_SETTLE, L2_LOOK, L2_ASK, L2_ALIGN, L2_GO, HOLD, END_ASK,
                      MANUAL, PAUSED, DONE };

  // ---- settings (page: Settings)
  float speed = 0.20f;
  float t1AskM = 20.0f;       // leg 1: stop and ask "turn here?" after this distance (log_027: turn at 21.2 m)
  float leg2ExpM = 36.0f;     // open sea length (log_027: 36 m). End wall accepted after 70 % of it
  float leg2MaxM = 50.0f;     // never drive more on leg 2
  float endWallM = 1.5f;      // stop this far from the end wall (logs 026/027 replay: found exactly at the true end)
  float holdM = 0.60f;        // something closer -> stop and wait (people)
  float pxPerM = 300.0f;      // camera scale (unknown until measured; offset is used only gently)
  float crabPriorDeg = -4.0f; // used when leg 1 measured fewer than 6 crab samples (logs: -5.6 .. -7.6)
  float crabGain = 0.7f;      // share of the crab estimate that is compensated
  bool camAssist = true;      // false: snapshots are taken and LOGGED, but never steer
  bool autoL2 = false;        // true: after the look, decide alone (align if 2 of 3 snapshots agree)
  uint8_t stripMode = 0;      // 0 wide (tactile), 1 narrow (belt)
  float lidarH = 0.25f;       // LiDAR height above the ground (from the ramp logs)

  bool begin(AsyncWebServer* server, MissionLog* log);
  // every loop pass, BEFORE GentleDrive. pad = drive pad command (manual / takeover). emergency = fast ramp.
  void update(const Sensors& s, RoverLink& rover, CamLink& cam, float padL, float padR, bool padActive,
              float& l, float& r, bool& emergency);
  void noteOutput(float l, float r, float dt);
  void fillLog(MissionLog::Sample& d, const Sensors& s, const RoverLink& rover, const CamLink& cam) const;
  bool active() const { return _st != IDLE && _st != DONE; }
  bool manual() const { return _st == MANUAL; }

private:
  void _go(uint8_t st);
  void _pause(const char* why);
  void _ask(uint8_t st, const char* q);
  float _front(const Sensors& s);
  bool _near(float lim, uint32_t now);
  void _leg1(const Sensors& s, float& l, float& r);
  void _pillars(const Sensors& s);
  bool _pivot(const Sensors& s, float tgt, float& l, float& r);   // true = done
  void _snapStep(CamLink& cam);
  void _startLeg2(const Sensors& s);
  String _statusJson(const Sensors& s, const RoverLink& rover, const CamLink& cam);

  MissionLog* _log = nullptr;
  DriveStraight _ds;
  volatile uint8_t _st = IDLE;
  uint8_t _prev = IDLE, _leg = 0;   // state before PAUSE / MANUAL / HOLD
  char _msg[96] = "", _q[96] = "";
  // requests from the web task
  volatile uint8_t _cmd = 0;        // see kCmd in .cpp
  volatile uint32_t _aliveMs = 0;
  // motion / sensing
  float _outV = 0, _outL = 0, _outR = 0, _dist = 0, _dist2 = 0, _tgt = 0, _rate = 0, _prevYaw = 0, _pitch0 = 0;
  float _frontM = 9.9f, _lidarFront = 9.9f;
  bool _slope = false, _usNearOn = false; uint32_t _usNearMs = 0;
  uint32_t _stMs = 0, _lastMs = 0, _wallMs = 0, _clearMs = 0, _endMs = 0;
  bool _clearOn = false, _endOn = false, _nearOn = false; uint32_t _nearMs = 0;
  // leg 1: wall + pillars + crab
  WallFit _wall = {false, 0, 0, 0, 0};
  bool _wallOk = false;
  float _dSet = 1.2f, _wallDir = 0, _askAt = 20.0f;
  float _wyBuf[5]; uint8_t _wyN = 0, _wyI = 0;
  uint8_t _pillarsN = 0; bool _pillarNear = false; float _lastPillarM = -9;
  float _rightOpenFrom = -1; bool _rightOpenLogged = false;
  float _crabEst = 0; uint8_t _crabN = 0;
  float _cs[10], _cd[10], _cb[10]; uint8_t _cN = 0, _cI = 0; float _nextCrabM = 0;
  // turn
  float _turnTgt = 0; uint8_t _turnTries = 0;
  // leg 2 + camera
  float _base = 0, _mergeLeft = 0, _offM = 0, _nextCheck = 0;
  uint8_t _snapWant = 0, _snapGot = 0, _snapOk = 0, _snapPurpose = 0;   // purpose 1 look, 2 check
  float _sOff[3], _sAng[3]; uint32_t _snapGapMs = 0;
  float _lookOff = 0, _lookAng = 0; uint8_t _lookN = 0;
  float _lastCorr = 0;
  // status for the page
  SemaphoreHandle_t _mtx = nullptr;
  String _status = "{}"; uint32_t _statusMs = 0;
};
