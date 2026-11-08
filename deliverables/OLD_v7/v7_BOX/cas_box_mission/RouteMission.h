#pragma once
// RouteMission - AUTONOMOUS mission B facade -> right turn -> across the open area ("open sea") to its end wall
// (version 3, 2026-11-07 evening, after campus_2). NO CAMERA any more.
//   - straight driving = gyro heading hold + measured trim + failsafe (DriveStraight, Kethan's PD)
//   - leg 1 follows the LEFT wall at the start distance (LiDAR, offset 83 - verified on campus_2: wall held
//     1.57-1.59 m for 20 m); pillars are counted and logged
//   - the right turn starts BY ITSELF after turnAtM metres (campus runs 1 + 2: the operator chose 20 m both times);
//     "Turn now" on the page starts it earlier, "Turn later +1 m" delays it
//   - TURN (campus_2: the full pivot locks after 4-7 deg at the turn spot, the same pivot turned 42 deg/s on the open
//     sea): tactics in a cycle - full pivot, forward swing, shift 0.3 m, pivot, rolling arc, shift ... - each kept
//     as long as the gyro shows progress, changed when it stalls. Per-wheel ramps (GentleDrive turn mode).
//   - forward speed 0.22; if the LiDAR shows the rover is NOT moving while driving -> speed raised step by step
//     (max +0.10), lowered again once it moves
//   - end wall only while the rover is level (campus_2: a +9 deg bump at a kerb made the LiDAR see the ground at
//     1.3 m -> fake end wall 10 m before the real one)
//   - LEG 3 (add-on): at the open-sea end wall -> turn right (same turn tactics) -> follow the wall on the LEFT
//     (same wall follower as leg 1) -> stop when that wall ENDS (no wall seen for wallEndM of travel), or at a wall
//     ahead, or after leg3MaxM
//   - people in front: stop, wait until clear for 2 s, continue on the same heading (no swerving)
// Everything is logged (MissionLog). Camera fields of the log now carry: turn tactic, turn cycles, speed boost,
// measured motion.
#include <Arduino.h>
#include <ESPAsyncWebServer.h>
#include "Sensors.h"
#include "RoverLink.h"
#include "DriveStraight.h"
#include "WallFit.h"
#include "MissionLog.h"
#include "LineRef.h"

class RouteMission {
public:
  enum St : uint8_t { IDLE = 0, L1_FOLLOW, T1_ASK, T1_TURN, T1_SETTLE, L2_LOOK, L2_ASK, L2_ALIGN, L2_GO, HOLD, END_ASK,
                      MANUAL, PAUSED, DONE, L3_FOLLOW, L4_FOLLOW };   // L2_LOOK / L2_ASK / L2_ALIGN: not used any more (camera)
  enum Tac : uint8_t { TK_PIVOT = 0, TK_REST, TK_SWING, TK_ARC, TK_SHIFT };

  // ---- settings (page: Settings)
  float speed = 0.22f;        // campus_2: 0.25 a bit fast; boosted automatically when the rover does not move
  float turnAtM = 20.0f;      // leg 1: turn right after this distance (campus 1 + 2: the operator turned at 20 m)
  bool autoTurn = true;       // false: stop and ask at turnAtM (old behaviour)
  float leg2ExpM = 36.0f;     // open sea length (log_027: 36 m). End wall accepted after 70 % of it
  float leg2MaxM = 50.0f;     // never drive more on leg 2
  float endWallM = 1.5f;      // stop this far from the end wall
  float holdM = 0.60f;        // something closer -> stop and wait (people)
  float crabPriorDeg = 0.0f;  // LiDAR offset fixed -> no crab
  float crabGain = 0.7f;
  float lidarH = 0.25f;       // LiDAR height above the ground (from the ramp logs)
  float boostMax = 0.10f;     // max extra speed when the rover does not move
  bool leg3 = true;           // after the open sea: turn right + follow the left wall until it ends
  float wallEndM = 1.5f;      // leg 3: no left wall for this distance = the wall has ended (doors / gaps are shorter)
  float leg3MaxM = 60.0f;     // never drive more on leg 3
  // TURN LANDMARK (Kethan's turnLandmark, widened): the wide pillar/wall stub on the right at the end of B
  // (campus_2: near face 0.97 m right, ~0.6 m wide; Kethan's window 1.64-1.83 m behind = exactly where you turned).
  // Turn when its near face is pillarPastM BEHIND -> after the 90 deg right turn it stands ahead, ~pillarPastM to the
  // right. POSITIONING (state L2_ALIGN): roll slowly, steer to keep it pillarSideM on the right, STOP beside it,
  // then the open sea straight.
  bool usePillar = true;
  // OPEN-SEA ENTRANCE BOARD (campus_3: signboard on the right ~1 m, along the open sea): after the T1 turn the rover
  // rolls slowly until it sees it (right-side line fit), STOPS, moves sideways to boardSideM (pivot - roll - pivot)
  // and turns PARALLEL to it; that heading (+ boardOffsetDeg) is locked for the crossing. Measured live every run.
  float boardSideM = 0.60f;
  float boardOffsetDeg = 0.0f;
  bool leg4 = true;
  // v5 (campus4): the board lock was 9 deg off (campus_3: 5 deg the other way) -> NOT the heading reference any more.
  // Open-sea heading priority: wall behind / wall ahead (perpendicular) > board (parallel, +-5 deg gate) > gyro +
  // measured drift. Board line-up stop and end-board following are off by default.
  bool boardLineUp = false;
  // v6 (campus5 + run.mp4): LANDMARK CORRIDOR. The tactile strip of the open sea runs parallel to a straight row of
  // permanent landmarks on its LEFT (trees in grates + stakes + sign / lamp posts, ~9-24 m) at ~6.5 m (campus_2 6.2-6.6,
  // campus5 on the strip 6.8), and to the far building front at ~3.0 m (~28-37 m). Both are followed like distant
  // walls: heading parallel + side distance. No gyro drift correction in the open sea (campus5: it caused the drift).
  bool useRows = true;
  float rowOffsetM = 6.65f;   // strip -> left landmark row (v7: median of the campus_6 teaching run, 6.55-6.87)
  float bldOffsetM = 3.0f;    // strip -> left building front near the end
  // v7 cross-track PID on the row distance (simulated V6 vs V7, tools/sim_open_sea.py): deg per m of side error,
  // deg per (m*s) integral (a constant sideways pull: slanted cobbles), deg per (m/s) damping. Integral max 4 deg.
  float seaKp = 10.0f, seaKi = 1.5f, seaKd = 20.0f;
  // v7 last leg (after T3): only the wall ahead-left within 15 deg of the expected direction counts (campus_6: the
  // old fit caught the corner and chased -45 deg instead of -80); close in to leg4WallM while driving ahead
  float leg4WallM = 1.0f;
  float seaTrimDeg = 0.0f;    // optional fixed heading trim (logs: no constant pull, mean -0.9 +- 2.5 deg) - leave 0
  bool useEndBoard = false;           // after leg 3: T3 = turn LEFT at the building corner, follow the next left wall until it ends
  float pillarPastM = 0.40f;   // v5c: turn this far past the pillar (was 0.60): crossing ~0.2-0.3 m closer to the pillar line
  float pillarSideM = 0.60f;
  float turnWinM = 5.0f;      // look for the turn pillar from (turnAtM - turnWinM); without one: turn at turnAtM + 2.5

  bool begin(AsyncWebServer* server, MissionLog* log);
  // every loop pass, BEFORE GentleDrive. pad = drive pad command (manual / takeover). emergency = fast ramp.
  // wheels = true: turn mode (GentleDrive ramps each wheel on its own)
  void update(const Sensors& s, RoverLink& rover, float padL, float padR, bool padActive,
              float& l, float& r, bool& emergency, bool& wheels);
  void noteOutput(float l, float r, float dt);
  void fillLog(MissionLog::Sample& d, const Sensors& s, const RoverLink& rover) const;
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
  void _startTurn(const Sensors& s, float tgt);
  bool _turn(const Sensors& s, float tgt, float& l, float& r);   // true = done (tactics, see .cpp)
  void _tac(uint8_t t, const Sensors& s);
  void _motion(const Sensors& s, uint32_t now);
  float _cruise() const { return speed + _boost; }
  void _startLeg2(const Sensors& s);
  void _startLeg3(const Sensors& s);
  void _endWall(const Sensors& s);              // open-sea end wall reached: leg 3 turn, or stop
  void _finishRoute(const char* why);
  void _afterTurn(const Sensors& s);            // what comes after a finished turn (by _turnNo)
  void _startLeg4(const Sensors& s);
  void _realign(const Sensors& s, float tgt, const char* why);   // stop + pivot to tgt, then back to the same leg
  bool _frontIsWallLine(const Sensors& s) const;                 // front blocker = the followed left wall (not a person)
  bool _boardFit(const Sensors& s, float dMin, float dMax, float& dir, float& dist) const;
  void _leg4Wall(const Sensors& s, uint32_t now);   // v7: wall detection for the last leg
  bool _guard(const Sensors& s, uint32_t now, bool& emergency);  // front: wall -> realign, else people rule (HOLD)
  bool _landmark(const Sensors& s, float& nearBehind, float& nearRight) const;
  bool _frontWall(const Sensors& s, float maxM) const;   // flat wall ahead (Kethan's frontWallWithin)
  String _statusJson(const Sensors& s, const RoverLink& rover);

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
  float _pitchLp = 0;
  uint32_t _levelMs = 0;            // last time the rover was NOT level (pitch transient) - end wall waits 1 s after it
  uint32_t _stMs = 0, _lastMs = 0, _wallMs = 0, _clearMs = 0, _endMs = 0;
  bool _clearOn = false, _endOn = false, _nearOn = false; uint32_t _nearMs = 0;
  // stuck detection (LiDAR ranges 1 s apart) + speed boost
  uint16_t _mvRef[Sensors::kLidarPoints]; uint32_t _mvMs = 0; bool _mvHave = false;
  float _mvMinV = 1, _motionM = -1, _boost = 0; uint8_t _stuckN = 0;
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
  uint8_t _tk = TK_PIVOT, _tkNext = 0, _tCycles = 0, _shifts = 0; uint32_t _tMs = 0, _tWinMs = 0; float _tYaw = 0, _tShiftD = 0;
  float _turnD = -1;
  uint8_t _turnNo = 1;              // 1 = turn after leg 1, 2 = turn at the open-sea end wall
  bool _l3Seen = false; float _noWallFrom = -1;
  // turn pillar + aim after the turn
  uint32_t _tpMs = 0, _aimMs = 0, _aimSeenMs = 0; uint8_t _tpClear = 0;
  bool _tpTurn = false, _aimOn = false; float _tpY = 0, _tpX = 0, _aimDeg = 0;
  uint32_t _posStopMs = 0; float _posLat = 0;
  float _tpEdges[7]; uint8_t _tpEN = 0;          // pillar front-edge position along leg 1 (odometry), per sighting
  float _shiftEndM = 0, _shiftOff = 0;                          // open-sea start: steer 10 deg right until this distance (sideways shift)
  // wall follower v4: separate scan / last-good timers, new-wall acquisition, consistent-jump (bend) acceptance
  uint32_t _wallScanMs = 0, _wallGoodMs = 0; uint8_t _acqN = 0, _jumpN = 0; float _acqDir = 0, _jumpDir = 0;
  // realign (stop + pivot) and where to go back to
  uint8_t _rlBack = 0; uint32_t _rlMs = 0; bool _dSetPending = false;
  // entrance board: 0 search, 1 sideways pivot, 2 sideways roll, 3 parallel pivot
  uint8_t _bp = 0, _bdN = 0; float _bdDir[7], _bdDist[7], _bdLockDir = 0, _bdRollM = 0, _bdRollStart = 0; uint32_t _bdMs = 0;
  // end board
  uint8_t _ebN = 0; float _ebKeep = 0, _ebDir = 0; bool _ebOn = false;
  // v5 open-sea references + gyro drift
  DriftEst _drift1;                             // gyro drift measured on the leg-1 wall stretches
  float _mvT = 0, _lastMvT = 0;                 // driving time (s) - the BNO055 drifts only while driving
  uint32_t _refMs = 0; uint8_t _refType = 0, _refN = 0, _refI = 0; float _refBuf[5]; bool _rearSeen = false, _frontSeen = false;
  float _frontBase0 = 0; uint8_t _refKind = 0;
  float _latOff = 0, _refOff = 0, _seaEi = 0, _expDir = 0; bool _rowSeen = false, _bldSeen = false;   // v6 corridor: side term + last offset   // last NON-zero reference (gap scans do not reset the drift segment)                        // heading when the end wall was first used (it may move it max +-5 deg)                // +1 left / -1 right, fixed at the start (no hunting across the target)
  // leg 2
  float _base = 0;
  // status for the page
  SemaphoreHandle_t _mtx = nullptr;
  String _status = "{}"; uint32_t _statusMs = 0;
};
