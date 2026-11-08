#include "RouteMission.h"

static float wrap180(float a) { a = fmodf(a + 180.0f, 360.0f); if (a < 0) a += 360.0f; return a - 180.0f; }
static float clampf(float x, float lo, float hi) { return x < lo ? lo : (x > hi ? hi : x); }
static const char* kSt[] = {"ready", "LEG 1: following the left wall", "TURN? waiting for you", "turning right 90 deg",
                            "checking the turn", "-", "-", "OPEN SEA START: lining up on the board", "OPEN SEA: driving straight", "WAITING: something in front",
                            "STOPPED AT THE GOAL", "MANUAL driving (logging)", "PAUSED", "finished",
                            "LEG 3: following the left wall", "LEG 4: following the left wall (after T3)"};
static const char* kTac[] = {"full pivot", "rest", "forward swing", "rolling arc", "shift 0.3 m"};
enum Cmd : uint8_t { C_NONE = 0, C_START, C_TURN, C_ON1M, C_MANUAL, C_RESUME, C_L2HERE, C_FINISH, C_STOP };
static const uint32_t kAliveMs = 3000;      // page heartbeat (was 1.5 s: 3 needless pauses on campus_2)
static const uint32_t kTurnMaxMs = 120000;  // give up the turn after 2 min -> pause (turn by hand, Resume)

static void jsonEsc(String& o, const char* s) {
  for (; *s; s++) { if (*s == '"' || *s == '\\') { o += '\\'; o += *s; } else if ((uint8_t)*s >= 0x20) o += *s; }
}

bool RouteMission::begin(AsyncWebServer* server, MissionLog* log) {
  _log = log;
  _mtx = xSemaphoreCreateMutex();
  if (!server) return false;
  server->on("/mis/status", HTTP_GET, [this](AsyncWebServerRequest* q) {
    String j; if (xSemaphoreTake(_mtx, pdMS_TO_TICKS(50)) == pdTRUE) { j = _status; xSemaphoreGive(_mtx); } else j = "{}";
    q->send(200, "application/json", j);
  });
  server->on("/mis/alive", HTTP_GET, [this](AsyncWebServerRequest* q) { _aliveMs = millis(); q->send(200, "application/json", "{\"ok\":true}"); });
  server->on("/mis/cmd", HTTP_GET, [this](AsyncWebServerRequest* q) {
    const String c = q->hasParam("c") ? q->getParam("c")->value() : String("");
    const char* names[] = {"", "start", "turn", "on1m", "manual", "resume", "l2here", "finish", "stop"};
    for (uint8_t i = 1; i <= C_STOP; i++) if (c == names[i]) _cmd = i;
    _aliveMs = millis();
    q->send(200, "application/json", "{\"ok\":true}");
  });
  server->on("/mis/set", HTTP_GET, [this](AsyncWebServerRequest* q) {
    auto f = [&](const char* k, float& v, float lo, float hi) { if (q->hasParam(k)) v = clampf(q->getParam(k)->value().toFloat(), lo, hi); };
    f("speed", speed, 0.12f, 0.30f); f("t1", turnAtM, 2.0f, 60.0f); f("l2", leg2ExpM, 5.0f, 80.0f);
    f("end", endWallM, 0.6f, 2.0f); f("crab", crabPriorDeg, -12.0f, 12.0f);
    if (q->hasParam("auto")) autoTurn = q->getParam("auto")->value() == "1";
    if (q->hasParam("l3")) leg3 = q->getParam("l3")->value() == "1";
    f("wend", wallEndM, 0.5f, 5.0f);
    f("bside", boardSideM, 0.3f, 1.5f); f("boff", boardOffsetDeg, -15.0f, 15.0f);
    if (q->hasParam("l4")) leg4 = q->getParam("l4")->value() == "1";
    leg2MaxM = fmaxf(50.0f, leg2ExpM * 1.4f);
    q->send(200, "application/json", "{\"ok\":true}");
  });
  return true;
}

void RouteMission::_go(uint8_t st) { _st = st; _stMs = millis(); _clearOn = _endOn = false; }

void RouteMission::_pause(const char* why) {
  if (_st != PAUSED && _st != MANUAL) _prev = _st;
  strlcpy(_msg, why, sizeof(_msg));
  _go(PAUSED);
  if (_log) _log->event(MissionLog::EV_PAUSE);
  Serial.printf("[mission] PAUSED: %s\n", why);
}

void RouteMission::_ask(uint8_t st, const char* q) { strlcpy(_q, q, sizeof(_q)); _go(st); }

// Free space ahead (m): LiDAR +-16 deg with a GROUND FILTER on slopes, plus the front ultrasonic only when it
// reads < 25 cm for 0.6 s (it hears the floor at 60-70 cm otherwise).
// something closer than lim for >= 0.5 s (single LiDAR blips of 1-2 samples do not stop the rover: log_027)
bool RouteMission::_near(float lim, uint32_t now) {
  if (_frontM >= lim) { _nearOn = false; return false; }
  if (!_nearOn) { _nearOn = true; _nearMs = now; }
  return now - _nearMs >= 500;
}

float RouteMission::_front(const Sensors& s) {
  const float dp = s.pitchDeg() - _pitch0;
  _slope = fabsf(dp) > 3.0f;
  const float rg = _slope ? lidarH / tanf(fabsf(dp) * DEG_TO_RAD) : 99.0f;   // where the tilted beam meets the ground
  float best = 9.9f;
  if (s.lidarOk()) {
    for (int k = -4; k < 4; k++) {
      const uint16_t cm = s.lidarCm((uint8_t)((k + Sensors::kLidarPoints) % Sensors::kLidarPoints));
      const float m = cm / 100.0f;
      if (cm && m < best && m < rg - 0.30f) best = m;
    }
  }
  _lidarFront = best;
  const int us = s.usCm(0);
  const bool near = us > 2 && us < 25;
  const uint32_t now = millis();
  if (!near) _usNearOn = false; else if (!_usNearOn) { _usNearOn = true; _usNearMs = now; }
  if (near && now - _usNearMs >= 600) best = fminf(best, us / 100.0f);
  return best;
}

// ---------------------------------------------------------------------------------------------------------------
// TURN (v3, after campus_2). What the logs show:
//   - campus 1 + 2, turn spot at ~20 m along B: full pivot 0.50 -> 4-7 deg, then LOCKED (yaw frozen for 10 s while
//     the command stayed at full power; the operator's own pivot presses did the same). Forward driving there was fine.
//   - campus 1, open sea: the same pivot turned 33 deg at 42 deg/s. Teaching logs: 400 pivots, 36 % worked when full
//     power came at once, 13 % with a slow ramp; working pivots often start only 0.5-1.2 s after full power.
//   => the place locks the wheels (paving / kerb edge), not the motors. So: never insist on one way of turning.
// Tactics in a cycle; each is KEPT while the gyro shows progress (>= 3 deg per window) and CHANGED when it stalls:
//   PIVOT  Kethan's turn: both sides 0 -> 0.50 opposite in 0.7 s, held while >= 1 deg per 2 s
//   SWING  outer side full forward, inner side stands (only if > 0.8 m free ahead)
//   SHIFT  roll 0.3 m straight ahead (wheels onto other stones), only if > 1.0 m free ahead
//   ARC    outer side full forward, inner side 0.15 forward, max 6 s (only if > 1.0 m free ahead)
// cycle: PIVOT, SWING, SHIFT, PIVOT, ARC, SHIFT, PIVOT ... with a 0.5 s REST between two tactics.
// Wheels ramped per wheel (GentleDrive turn mode: 0 -> 0.5 in 0.25 s up, 1.0 /s down) - full power quickly, as the
// rover needs, gentle when stopping. Done at the target minus the coast (|rate| * 0.3 s, min 3 deg).
static const uint8_t kSeq[] = {RouteMission::TK_PIVOT, RouteMission::TK_SWING, RouteMission::TK_SHIFT,
                               RouteMission::TK_PIVOT, RouteMission::TK_ARC, RouteMission::TK_SHIFT};

void RouteMission::_tac(uint8_t t, const Sensors& s) {
  _tk = t; _tMs = _tWinMs = millis(); _tYaw = s.yawNowDeg(); _tShiftD = 0;
  if (t != TK_REST) Serial.printf("[mission] turn: %s\n", kTac[t]);
}

void RouteMission::_startTurn(const Sensors& s, float tgt) {
  _turnTgt = tgt;
  _turnD = wrap180(tgt - s.yawNowDeg()) >= 0 ? 1.0f : -1.0f;
  _tkNext = 0; _tCycles = 0; _shifts = 0;
  _tac(kSeq[_tkNext++], s);
  _go(T1_TURN);
}

bool RouteMission::_turn(const Sensors& s, float tgt, float& l, float& r) {
  const uint32_t now = millis();
  const float yaw = s.yawNowDeg(), d = _turnD;
  const float left = wrap180(tgt - yaw) * d;                   // degrees still to go (negative = past it)
  l = r = 0;
  // release before the target: the rover coasts ~7 deg (Kethan: stop at 83 of 90 deg outdoors)
  if (left <= fminf(7.0f, fmaxf(3.0f, fabsf(_rate) * 0.15f))) return true;
  const float moved = fabsf(wrap180(yaw - _tYaw));
  auto window = [&](uint32_t first, uint32_t next) -> bool {   // true = stalled
    const uint32_t w = (_tWinMs == _tMs) ? first : next;
    if (now - _tWinMs < w) return false;
    if (moved >= 3.0f) { _tWinMs = now; _tYaw = yaw; return false; }
    return true;
  };
  bool stalled = false;
  switch (_tk) {
    case TK_PIVOT: {
      // = Kethan's "Right 90" (it turned at the B corner in the team's drives): 0 -> 0.50 linear in 0.7 s, then
      // constant 0.50; given up only when the gyro shows less than 1 deg in 2 s
      // Kethan's current code (worked twice on campus): full power AT ONCE (no ramp), progress >= 1 deg per 2 s
      l = -0.5f * d; r = 0.5f * d;
      const uint32_t w = 2000;
      if (now - _tWinMs >= w) { if (moved >= 1.0f) { _tWinMs = now; _tYaw = yaw; } else stalled = true; }
      break;
    }
    case TK_SWING:
      if (_frontM <= 0.8f) { stalled = true; break; }
      if (d < 0) { l = 0.5f; r = 0; } else { l = 0; r = 0.5f; }
      stalled = window(2500, 1500);
      break;
    case TK_ARC:
      if (_frontM <= 1.0f) { stalled = true; break; }
      if (d < 0) { l = 0.5f; r = 0.15f; } else { l = 0.15f; r = 0.5f; }
      stalled = window(3000, 2000) || now - _tMs > 6000;
      break;
    case TK_SHIFT:
      if (_frontM <= 1.0f || _tShiftD >= 0.30f || now - _tMs > 3500) { if (_tShiftD > 0.05f) _shifts++; stalled = true; break; }
      l = r = 0.25f;
      break;
    default:                                                  // REST
      if (now - _tMs >= 500 && _outL == 0 && _outR == 0) {
        if (_tkNext >= sizeof(kSeq)) { _tkNext = 0; _tCycles++; }
        _tac(kSeq[_tkNext++], s);
      }
      return false;
  }
  if (stalled) {
    if (_tk != TK_SHIFT && _log) _log->event(MissionLog::EV_TURN_STALL);
    snprintf(_msg, sizeof(_msg), "turning: %s stalled, %.0f deg to go - trying the next way", kTac[_tk], left);
    _tac(TK_REST, s);
    l = r = 0;
  }
  return false;
}

// STUCK detection: the LiDAR ranges ahead and behind change when the rover moves (0.2 m/s -> ~0.2 m per second).
// Every 1 s: median |change| of the ranges (0.3-10 m, front +-60 deg and back +-60 deg). Driving forward but the
// median < 3 cm for 2 s -> speed +0.03 (max +boostMax). Moving again (> 10 cm/s) -> the boost drops 0.01 per second.
// No usable ranges (open sea, nothing within 10 m) -> no decision, the boost stays.
void RouteMission::_motion(const Sensors& s, uint32_t now) {
  _mvMinV = fminf(_mvMinV, _outV);
  if (now - _mvMs < 1000) return;
  const float dtS = _mvHave ? (now - _mvMs) / 1000.0f : 1.0f;
  _mvMs = now;
  if (!s.lidarOk()) { _mvHave = false; _motionM = -1; return; }
  float dv[Sensors::kLidarPoints]; int n = 0;
  for (int i = 0; i < Sensors::kLidarPoints; i++) {
    const uint16_t c = s.lidarCm(i);
    const int a = i * 4;
    const bool sector = a <= 60 || a >= 300 || (a >= 120 && a <= 240);
    if (_mvHave && sector && c >= 30 && c <= 1000 && _mvRef[i] >= 30 && _mvRef[i] <= 1000)
      dv[n++] = fabsf((float)c - (float)_mvRef[i]);
    _mvRef[i] = c;
  }
  const bool had = _mvHave; _mvHave = true;
  const bool driving = _mvMinV >= 0.12f && (_st == L1_FOLLOW || _st == L2_GO);
  _mvMinV = 1;
  if (!had || n < 6) { _motionM = -1; return; }
  for (int i = 1; i < n; i++) { float v = dv[i]; int j = i - 1; while (j >= 0 && dv[j] > v) { dv[j + 1] = dv[j]; j--; } dv[j + 1] = v; }
  _motionM = dv[n / 2] / 100.0f / dtS;                         // m per s
  if (!driving) { _stuckN = 0; return; }
  if (_motionM < 0.03f) {
    if (++_stuckN >= 2) {
      _stuckN = 0;
      if (_boost < boostMax) {
        _boost = fminf(boostMax, _boost + 0.03f);
        snprintf(_msg, sizeof(_msg), "not moving -> speed %.2f", _cruise());
        Serial.printf("[mission] %s\n", _msg);
      }
    }
  } else if (_motionM > 0.10f) { _stuckN = 0; _boost = fmaxf(0.0f, _boost - 0.01f); }
}

void RouteMission::_startLeg2(const Sensors& s) {
  const float crabUse = _crabN >= 6 ? _crabEst : crabPriorDeg;
  _base = wrap180(_base - crabGain * crabUse);             // _base set by the caller (turn target or current heading)
  _tgt = _base;
  _leg = 2; _dist2 = 0; _ebOn = false; _ebN = 0; _aimMs = 0;
  _ds.start(s.yawNowDeg(), millis()); _ds.setTarget(_tgt);
  if (_log) _log->event(MissionLog::EV_GO_L2);
  snprintf(_msg, sizeof(_msg), "open sea: heading %.1f deg (crab %.1f %s)", _tgt, crabUse, _crabN >= 6 ? "measured" : "prior");
  Serial.printf("[mission] leg 2: %s\n", _msg);
  _go(L2_GO);
}

// Kethan's turnLandmark on our frame (offset 83), widened to see the object from abeam to 2.1 m behind:
// right-rear bins 88-162 deg, points 0.3-2.1 m behind and 0.9-2.9 m right, one missing bin allowed, >= 4 points,
// >= 0.45 m wide to the right (a thin pillar is not it). Out: near face (smallest behind / smallest right).
// Replayed on campus_2: seen from 18.96 m on, nothing else between 15 m and the turn.
bool RouteMission::_landmark(const Sensors& s, float& nearBehind, float& nearRight) const {
  if (!s.lidarOk()) return false;
  int n = 0, best = 0; bool miss = false;
  float bMin = 0, rMin = 0, rMax = 0, lb = 0, lr = 0;
  auto close = [&]() {
    if (n >= 4 && rMax - rMin >= 0.45f && n > best) { best = n; nearBehind = bMin; nearRight = rMin; }
    n = 0;
  };
  for (int i = 22; i <= 40; i++) {
    const uint16_t cm = s.lidarCm(i);
    const float a = (i * 4 + 2) * DEG_TO_RAD, d = cm / 100.0f;
    const float bh = -d * cosf(a), rt = d * sinf(a);
    const bool use = cm && bh >= 0.3f && bh <= 2.1f && rt >= 0.9f && rt <= 2.9f;
    if (!use && n && !miss) { miss = true; continue; }
    if (n && (!use || hypotf(bh - lb, rt - lr) > 0.45f)) close();
    if (!use) continue;
    miss = false;
    if (!n) { bMin = bh; rMin = rMax = rt; }
    bMin = fminf(bMin, bh); rMin = fminf(rMin, rt); rMax = fmaxf(rMax, rt); lb = bh; lr = rt; n++;
  }
  close();
  return best > 0;
}

// flat wall ahead within maxM (after Kethan's frontWallWithin, tuned on campus 1 + 2): beams in +-36 deg within
// 0.6 m to each side, >= 4 hits, >= 0.25 m wide, depth spread <= 0.30 m. Real end wall: found from 1.54 m;
// a person / post (campus 1, t=360) is not a wall.
bool RouteMission::_frontWall(const Sensors& s, float maxM) const {
  if (!s.lidarOk()) return false;
  int hits = 0; float nearF = 99, farF = 0, le = 0, re = 0;
  for (int k = -9; k < 9; k++) {
    const int i = (k + Sensors::kLidarPoints) % Sensors::kLidarPoints;
    const uint16_t cm = s.lidarCm(i);
    if (!cm) continue;
    const float a = (k * 4 + 2) * DEG_TO_RAD, d = cm / 100.0f, f = d * cosf(a), side = d * sinf(a);
    if (f < 0.2f || f > maxM || fabsf(side) > 0.6f) continue;
    nearF = fminf(nearF, f); farF = fmaxf(farF, f); le = fminf(le, side); re = fmaxf(re, side); hits++;
  }
  return hits >= 4 && re - le >= 0.25f && farF - nearF <= 0.30f;
}

// open-sea end wall: with leg 3 -> turn right 90 deg from the open-sea heading; without -> stop and ask
void RouteMission::_endWall(const Sensors& s) {
  if (_log) _log->event(MissionLog::EV_END_WALL);
  if (!leg3) { _ask(END_ASK, "End wall reached: open sea crossed."); return; }
  strlcpy(_msg, "open sea crossed - turning right for leg 3", sizeof(_msg));
  _turnNo = 2; _turnTries = 0;
  _startTurn(s, wrap180(_tgt - 90.0f));
}

// LEG 3: the end wall of the open sea is now on the LEFT -> same wall follower as leg 1, distance = what it sees now
void RouteMission::_startLeg3(const Sensors& s) {
  const WallFit w = WallFit::measure(s, true, nullptr);
  _leg = 3; _dist = 0; _wyN = _wyI = 0; _cN = _cI = 0; _nextCrabM = 0; _pillarNear = false; _lastPillarM = -9;
  if (w.ok && fabsf(w.angleDeg) < 20) {
    _dSet = clampf(w.distM, 0.7f, 2.2f); _wallDir = wrap180(s.yawDeg() + w.angleDeg); _wall = w; _wallOk = true;
  } else { _dSet = clampf(endWallM, 0.7f, 2.2f); _wallDir = _turnTgt; _wallOk = false; }
  _dSetPending = !_wallOk; _acqN = _jumpN = 0; _wallGoodMs = millis();
  _tgt = _wallDir; _l3Seen = _wallOk; _noWallFrom = -1;
  _ds.start(s.yawNowDeg(), millis()); _ds.setTarget(_tgt);
  if (_log) _log->event(MissionLog::EV_L3_START);
  snprintf(_msg, sizeof(_msg), "leg 3: left wall %s, keeping %.2f m", _wallOk ? "seen" : "NOT seen yet", _dSet);
  Serial.printf("[mission] %s\n", _msg);
  _go(L3_FOLLOW);
}

// stop + pivot to tgt (Kethan pivot), then back to the leg it came from (campus_3: hand-placed / converging rover)
void RouteMission::_realign(const Sensors& s, float tgt, const char* why) {
  _rlBack = _st; _rlMs = millis();
  if (_log) _log->event(MissionLog::EV_REALIGN);
  snprintf(_msg, sizeof(_msg), "%s (%.0f deg)", why, wrap180(tgt - s.yawNowDeg()));
  Serial.printf("[mission] realign: %s\n", _msg);
  _turnNo = 9; _turnTries = 0;
  _startTurn(s, tgt);
}

// The thing in front is the followed LEFT wall (its line continued ahead), not a person: >= 2 front points
// (+-16 deg, closer than holdM + 0.3 m) and all of them within 0.15 m of the fitted wall line.
bool RouteMission::_frontIsWallLine(const Sensors& s) const {
  if (!_wallOk || !s.lidarOk()) return false;
  const float th = _wall.angleDeg * DEG_TO_RAD, nx = -sinf(th), ny = cosf(th);
  int on = 0, off = 0;
  for (int k = -4; k < 4; k++) {
    const uint16_t cm = s.lidarCm((uint8_t)((k + Sensors::kLidarPoints) % Sensors::kLidarPoints));
    if (!cm || cm > (holdM + 0.3f) * 100) continue;
    const float a = (k * 4 + 2) * DEG_TO_RAD, d = cm / 100.0f;
    const float x = d * cosf(a), yl = -d * sinf(a);              // side frame: y towards the LEFT wall
    if (fabsf(nx * x + ny * yl - _wall.distM) < 0.15f) on++; else off++;
  }
  return on >= 2 && off == 0;
}

// front guard for the wall-following legs: wall ahead -> realign parallel; anything else -> people rule (HOLD)
bool RouteMission::_guard(const Sensors& s, uint32_t now, bool& emergency) {
  const bool veryClose = _frontM < 0.45f;
  if (!veryClose && !_near(holdM, now)) return false;
  if (_frontIsWallLine(s) && now - _rlMs > 2000) {
    if (_log) _log->event(MissionLog::EV_FRONT_IS_WALL);
    const float yaw = s.yawNowDeg();
    const float tgt = fabsf(wrap180(_tgt - yaw)) >= 10.0f ? _tgt : wrap180(_tgt - 15.0f);   // turn away from the wall
    _realign(s, tgt, "wall ahead, not a person - turning parallel");
    return true;
  }
  if (veryClose) emergency = true;
  _prev = _st; if (_log) _log->event(MissionLog::EV_HOLD); _go(HOLD);
  return true;
}

// right-side board / wall line: fresh fit, rms < 0.05, |angle| < 30, distance dMin..dMax. dir = its direction (gyro)
bool RouteMission::_boardFit(const Sensors& s, float dMin, float dMax, float& dir, float& dist) const {
  const WallFit w = WallFit::measure(s, false, nullptr);
  if (!w.ok || w.rmsM > 0.05f || fabsf(w.angleDeg) > 30.0f || w.distM < dMin || w.distM > dMax) return false;
  dir = wrap180(s.yawDeg() - w.angleDeg); dist = w.distM;
  return true;
}

void RouteMission::_afterTurn(const Sensors& s) {
  const uint32_t now = millis();
  switch (_turnNo) {
    case 2: _startLeg3(s); break;
    case 3: _startLeg4(s); break;
    case 8: _go(L2_ALIGN); _bdMs = 0; break;                       // board positioning continues
    case 9: _ds.start(s.yawNowDeg(), now); _ds.setTarget(_tgt); _rlMs = now; _wallScanMs = 0; _go(_rlBack); break;
    default:                                                       // T1 done: line up on the entrance board
      _leg = 2; _dist2 = 0; _base = _turnTgt; _tgt = _base;
      _bp = 0; _bdN = 0; _bdMs = 0;
      _ds.start(s.yawNowDeg(), now); _ds.setTarget(_tgt);
      strlcpy(_msg, "open sea start: looking for the board on the right", sizeof(_msg));
      _go(L2_ALIGN);
  }
}

void RouteMission::_startLeg4(const Sensors& s) {
  const WallFit w = WallFit::measure(s, true, nullptr);
  _leg = 4; _dist = 0; _wyN = _wyI = 0; _cN = _cI = 0; _nextCrabM = 0; _acqN = _jumpN = 0;
  if (w.ok && fabsf(w.angleDeg) < 20) {
    _dSet = clampf(w.distM, 0.7f, 2.2f); _wallDir = wrap180(s.yawDeg() + w.angleDeg); _wall = w; _wallOk = true; _dSetPending = false;
  } else { _wallDir = _turnTgt; _wallOk = false; _dSetPending = true; }
  _wallGoodMs = millis();
  _tgt = _wallDir; _l3Seen = _wallOk; _noWallFrom = -1;
  _ds.start(s.yawNowDeg(), millis()); _ds.setTarget(_tgt);
  if (_log) _log->event(MissionLog::EV_L4_START);
  snprintf(_msg, sizeof(_msg), "leg 4 (after T3): left wall %s", _wallOk ? "seen" : "not seen yet");
  _go(L4_FOLLOW);
}

void RouteMission::_finishRoute(const char* why) {
  if (_log) _log->event(MissionLog::EV_WALL_END);
  strlcpy(_msg, why, sizeof(_msg));
  Serial.printf("[mission] END: %s\n", why);
  _ask(END_ASK, why);
}

void RouteMission::noteOutput(float l, float r, float dt) {
  _outL = l; _outR = r; _outV = 0.5f * (l + r);
  if (_outV > 0 && dt > 0 && dt < 0.2f) {
    const float d = _outV * 1.03f * dt;
    if (_leg == 2) _dist2 += d; else _dist += d;
    if (_st == T1_TURN && _tk == TK_SHIFT) _tShiftD += d;
  }
}

void RouteMission::update(const Sensors& s, RoverLink& rover, float padL, float padR, bool padActive,
                          float& l, float& r, bool& emergency, bool& wheels) {
  const uint32_t now = millis();
  const float dt = _lastMs ? (now - _lastMs) / 1000.0f : 0; _lastMs = now;
  const float yaw = s.yawNowDeg();
  wheels = false;
  if (s.imuOk() && dt > 0 && dt < 0.2f) { const float d = wrap180(yaw - _prevYaw); if (fabsf(d) < 30) _rate += (d / dt - _rate) * (dt / (0.15f + dt)); }
  _prevYaw = yaw;
  _frontM = _front(s);
  // LEVEL check for the end wall: pitch against its own 3 s average (a constant slope is fine, a bump is not)
  if (s.imuOk() && dt > 0 && dt < 0.2f) {
    _pitchLp += (s.pitchDeg() - _pitchLp) * (dt / (3.0f + dt));
    if (fabsf(s.pitchDeg() - _pitchLp) > 2.0f) _levelMs = now;
  }
  const bool level = now - _levelMs >= 1000;
  _motion(s, now);

  // ---- commands from the page
  const uint8_t c = _cmd; _cmd = C_NONE;
  if (c == C_START && (_st == IDLE || _st == DONE)) {
    const WallFit w = WallFit::measure(s, true, nullptr);
    if (!rover.connected()) strlcpy(_msg, "Not started: rover not connected", sizeof(_msg));
    else if (!s.imuOk() || !s.lidarOk()) strlcpy(_msg, "Not started: IMU / LiDAR not ready", sizeof(_msg));
    else if (!w.ok || fabsf(w.angleDeg) > 20) strlcpy(_msg, "Not started: no wall on the LEFT (place the rover parallel to the B wall)", sizeof(_msg));
    else {
      _dSet = clampf(w.distM, 0.7f, 2.2f); _wallDir = wrap180(s.yawDeg() + w.angleDeg); _wall = w; _wallOk = true; _wyN = _wyI = 0;
      _tgt = _wallDir; _dist = _dist2 = 0; _leg = 1; _pillarsN = 0; _pillarNear = false; _lastPillarM = -9; _rightOpenFrom = -1;
      _crabEst = 0; _crabN = 0; _cN = _cI = 0; _nextCrabM = 0; _askAt = turnAtM; _pitch0 = s.pitchDeg(); _pitchLp = s.pitchDeg(); _boost = 0; _tpTurn = _aimOn = false; _tpClear = 0;
      _wallGoodMs = now; _acqN = _jumpN = 0; _dSetPending = false; _rlMs = 0;
      _ds.start(yaw, now); _ds.setTarget(_tgt);
      if (_log) { _log->start(); _log->event(MissionLog::EV_START); }
      snprintf(_msg, sizeof(_msg), "started: left wall at %.2f m, turn right after %.1f m", _dSet, _askAt);
      _go(L1_FOLLOW);
    }
  } else if (c == C_STOP && active() && _st != PAUSED) {
    _pause("STOP pressed");
  } else if (c == C_MANUAL) {
    if (_st != MANUAL) { _prev = (_st == PAUSED) ? _prev : _st; if (_st == IDLE || _st == DONE) { _prev = IDLE; if (_log) _log->start(); } }
    if (_log) _log->event(MissionLog::EV_MANUAL_ON);
    strlcpy(_msg, "manual driving - everything is logged", sizeof(_msg));
    _go(MANUAL);
  } else if (c == C_RESUME && (_st == PAUSED || _st == MANUAL)) {
    if (_log) _log->event(_st == MANUAL ? MissionLog::EV_MANUAL_OFF : MissionLog::EV_RESUME);
    uint8_t back = (_prev == HOLD || _prev == PAUSED || _prev == MANUAL || _prev == IDLE)
                   ? (uint8_t)(_leg == 4 ? L4_FOLLOW : (_leg == 3 ? L3_FOLLOW : (_leg == 2 ? L2_GO : L1_FOLLOW))) : _prev;
    if (_st == MANUAL && back == L2_GO) { _base = yaw; _tgt = yaw; }       // your heading = the new straight line
    if (_st == MANUAL && (back == L1_FOLLOW || back == L3_FOLLOW || back == L4_FOLLOW)) { _tgt = yaw; _wallScanMs = 0; }
    _msg[0] = 0;
    if (back == L2_ALIGN) { _base = _turnTgt; _startLeg2(s); }   // after a pause / take-over: straight on
    else if (back == L1_FOLLOW || back == L2_GO || back == L3_FOLLOW || back == L4_FOLLOW) { _ds.start(yaw, now); _ds.setTarget(_tgt); _rlMs = now; _go(back); }
    else if (back == T1_TURN || back == T1_SETTLE) {
      // the operator may have turned it (partly) by hand: finish the remaining part, or go on if it is done
      if (fabsf(wrap180(_turnTgt - yaw)) <= 6.0f) _afterTurn(s);
      else { _turnTries = 0; _startTurn(s, _turnTgt); }
    } else _go(back);
  } else if (c == C_TURN && (_st == T1_ASK || _st == L1_FOLLOW)) {
    if (_log) _log->event(MissionLog::EV_OP_TURN);
    _tpTurn = false; _turnNo = 1; _turnTries = 0; _startTurn(s, wrap180((_wallOk ? _wallDir : yaw) - 90.0f));
  } else if (c == C_ON1M && (_st == T1_ASK || _st == L1_FOLLOW)) {
    if (_log) _log->event(MissionLog::EV_OP_ON1M);
    _askAt = fmaxf(_askAt, _dist) + 1.0f;
    snprintf(_msg, sizeof(_msg), "turn moved to %.1f m", _askAt);
    if (_st == T1_ASK) { _ds.start(yaw, now); _ds.setTarget(_tgt); _go(L1_FOLLOW); }
  } else if (c == C_L2HERE && (_st == MANUAL || _st == PAUSED || _st == T1_ASK || _st == IDLE || _st == DONE)) {
    if ((_st == IDLE || _st == DONE) && _log) _log->start();
    if (_dist < 2.0f) _pitch0 = s.pitchDeg();
    _base = yaw; _startLeg2(s);
  } else if (c == C_FINISH && _st != IDLE) {
    if (_log) { _log->event(MissionLog::EV_DONE); _log->stop(); }
    strlcpy(_msg, "finished - log saved", sizeof(_msg));
    _go(DONE);
  }

  // ---- safety in every moving state
  const bool moving = _st == L1_FOLLOW || _st == T1_TURN || _st == T1_SETTLE || _st == L2_ALIGN || _st == L2_GO || _st == L3_FOLLOW ||
                      _st == L4_FOLLOW;
  if (moving) {
    if (padActive) { _prev = _st; if (_log) _log->event(MissionLog::EV_MANUAL_ON); strlcpy(_msg, "you took over - manual (logging)", sizeof(_msg)); _go(MANUAL); }
    else if (now - _aliveMs > kAliveMs) _pause("phone page lost (screen off / other app?)");
    else if (!rover.connected()) _pause("rover lost");
    else if (s.imuOk() && (fabsf(s.rollDeg()) > 15 || fabsf(s.pitchDeg() - _pitch0) > 18)) { emergency = true; _pause("tilt"); }
  }

  // pad touched while standing (ready / asking / paused / done): manual driving, logged
  if (padActive && (_st == IDLE || _st == DONE || _st == PAUSED || _st == T1_ASK || _st == END_ASK)) {
    if (_st == IDLE || _st == DONE) { _prev = IDLE; if (_log) _log->start(); } else if (_st != PAUSED) _prev = _st;
    if (_log) _log->event(MissionLog::EV_MANUAL_ON);
    strlcpy(_msg, "manual driving - everything is logged", sizeof(_msg));
    _go(MANUAL);
  }

  l = r = 0;
  switch (_st) {
    case L1_FOLLOW:
      if (_guard(s, now, emergency)) break;
      // TURN LANDMARK: near face pillarPastM behind on 2 scans in a row (from askAt - turnWinM on)
      if (autoTurn && usePillar && _dist >= _askAt - turnWinM && now - _tpMs >= 250) {
        _tpMs = now;
        float nb, nr;
        if (_landmark(s, nb, nr) && nb >= pillarPastM) {
          if (++_tpClear >= 2) {
            _tpClear = 0; _tpTurn = true; _tpX = nr; _tpY = nb;
            if (_log) { _log->event(MissionLog::EV_TURN_PILLAR); _log->event(MissionLog::EV_OP_TURN); }
            snprintf(_msg, sizeof(_msg), "turn landmark %.2f m behind, %.2f m right (%.1f m): turning right", nb, nr, _dist);
            Serial.printf("[mission] %s\n", _msg);
            _turnNo = 1; _turnTries = 0; _startTurn(s, wrap180((_wallOk ? _wallDir : yaw) - 90.0f));
            wheels = true; _turn(s, _turnTgt, l, r);
            break;
          }
        } else _tpClear = 0;
      }
      if (_dist >= _askAt + ((autoTurn && usePillar) ? 2.5f : 0.0f)) {
        if (_log) _log->event(MissionLog::EV_T1_ASK);
        _tpTurn = false;
        if (autoTurn) {
          snprintf(_msg, sizeof(_msg), "%.1f m, %u pillars: turning right", _dist, _pillarsN);
          if (_log) _log->event(MissionLog::EV_OP_TURN);
          _turnNo = 1; _turnTries = 0; _startTurn(s, wrap180((_wallOk ? _wallDir : yaw) - 90.0f));
          wheels = true; _turn(s, _turnTgt, l, r);
        } else {
          char q[96]; snprintf(q, sizeof(q), "%.1f m driven, %u pillars counted. Is this the RIGHT TURN to the open area?", _dist, _pillarsN);
          _ask(T1_ASK, q);
        }
        break;
      }
      _leg1(s, l, r);
      // far off the wanted heading (hand-placed, wall bend, after a stop): stop and pivot parallel instead of
      // creeping round with the small rolling correction (and instead of tripping the 12 deg failsafe)
      if (_st == L1_FOLLOW && fabsf(wrap180(_tgt - yaw)) > 10.0f && now - _rlMs > 3000) { l = r = 0; _realign(s, _tgt, "off the wall line - turning parallel"); }
      break;
    case T1_TURN:
      wheels = _tk != TK_SHIFT;
      if (_turn(s, _turnTgt, l, r)) { l = r = 0; _go(T1_SETTLE); }
      else if (now - _stMs > kTurnMaxMs) _pause("turn failed (2 min) - turn it by hand, then Resume");
      break;
    case T1_SETTLE:
      wheels = true;
      if (now - _stMs >= 1000) {
        // up to 10 deg left over is taken out by the heading hold while rolling (small pivots overshoot);
        // positioning / realign pivots: 6 deg, one retry
        const bool small = _turnNo >= 8;
        if (fabsf(wrap180(_turnTgt - yaw)) > (small ? 6.0f : 10.0f) && _turnTries < (small ? 1 : 3)) { _turnTries++; _startTurn(s, _turnTgt); }
        else {
          if (_log) _log->event(MissionLog::EV_TURN_DONE);
          _afterTurn(s);
        }
      }
      break;
    case L2_ALIGN: {
      // OPEN-SEA START (v4, campus_3): the signboard on the right is the reference. Roll slowly (0.18) on the
      // B-perpendicular heading until 5 consistent right-side line fits (0.35-1.8 m); then STOP and
      //   distance off by > 0.15 m -> pivot 12-30 deg towards / away, roll |e|/sin(a), pivot back = sideways move
      //   then pivot PARALLEL to the board (if > 3 deg off) -> heading locked -> open sea.
      // No board within 4 m / 30 s -> open sea on the B-perpendicular heading (as v3).
      if (_frontM < 0.45f) { emergency = true; _prev = L2_ALIGN; if (_log) _log->event(MissionLog::EV_HOLD); _go(HOLD); break; }
      if (_bp != 2 && _near(holdM, now)) { _prev = L2_ALIGN; if (_log) _log->event(MissionLog::EV_HOLD); _go(HOLD); break; }
      if (_bp == 0) {
        if (now - _bdMs >= 250) {
          _bdMs = now;
          float dir, dist;
          if (_boardFit(s, 0.35f, 1.8f, dir, dist)) {
            if (_bdN && fabsf(wrap180(dir - _bdDir[_bdN - 1])) > 5.0f) _bdN = 0;        // not the same line: restart
            if (_bdN < 7) { _bdDir[_bdN] = dir; _bdDist[_bdN] = dist; _bdN++; }
            if (_bdN >= 5) {                                                          // first fits only see its front edge
              float rd[7], dd[7];                                                    // medians (relative directions)
              for (int i = 0; i < _bdN; i++) { rd[i] = wrap180(_bdDir[i] - _bdDir[0]); dd[i] = _bdDist[i]; }
              for (int i = 1; i < _bdN; i++) { float v = rd[i]; int j = i - 1; while (j >= 0 && rd[j] > v) { rd[j + 1] = rd[j]; j--; } rd[j + 1] = v; }
              for (int i = 1; i < _bdN; i++) { float v = dd[i]; int j = i - 1; while (j >= 0 && dd[j] > v) { dd[j + 1] = dd[j]; j--; } dd[j + 1] = v; }
              const float md = wrap180(_bdDir[0] + rd[_bdN / 2]), mdist = dd[_bdN / 2];
              _bdLockDir = wrap180(md + boardOffsetDeg);
              if (_log) _log->event(MissionLog::EV_BOARD_SEEN);
              const float e = mdist - boardSideM;                                      // + = too far -> move right
              snprintf(_msg, sizeof(_msg), "board %.2f m right, direction %.1f deg - lining up at %.2f m", mdist, md, boardSideM);
              Serial.printf("[mission] %s\n", _msg);
              _turnNo = 8; _turnTries = 0;
              if (fabsf(e) > 0.15f) {
                const float a = clampf(atan2f(fabsf(e), 1.0f) * RAD_TO_DEG, 12.0f, 30.0f);
                _bdRollM = fminf(1.5f, fabsf(e) / sinf(a * DEG_TO_RAD));
                _bp = 1;
                _startTurn(s, wrap180(md - (e > 0 ? a : -a)));                         // right = yaw minus
              } else {
                _bp = 3;
                if (fabsf(wrap180(_bdLockDir - yaw)) > 3.0f) _startTurn(s, _bdLockDir);
              }
              break;
            }
          }
        }
        if (_dist2 > 4.0f || now - _stMs > 30000) {
          _base = _turnTgt; _startLeg2(s);
          strlcpy(_msg, "no board seen - open sea on the B-perpendicular heading", sizeof(_msg));
          break;
        }
        _ds.setTarget(_base);
        if (_ds.update(s.imuOk(), yaw, 0.18f, now, l, r) == DriveStraight::FAIL) { if (_log) _log->event(MissionLog::EV_FAILSAFE); _pause(_ds.reason()); }
      } else if (_bp == 1) {                          // sideways pivot done -> roll
        _bp = 2; _bdRollStart = _dist2; _tgt = yaw; _ds.start(yaw, now); _ds.setTarget(_tgt);
      } else if (_bp == 2) {                          // sideways roll
        if (_dist2 - _bdRollStart >= _bdRollM || now - _stMs > 15000 || _frontM < 1.0f) {
          _bp = 3; _turnNo = 8; _turnTries = 0; _startTurn(s, _bdLockDir);
          break;
        }
        _ds.setTarget(_tgt);
        if (_ds.update(s.imuOk(), yaw, 0.18f, now, l, r) == DriveStraight::FAIL) { if (_log) _log->event(MissionLog::EV_FAILSAFE); _pause(_ds.reason()); }
      } else {                                        // parallel to the board: lock and go
        if (_log) _log->event(MissionLog::EV_BOARD_LOCK);
        _base = _bdLockDir; _startLeg2(s);
        snprintf(_msg, sizeof(_msg), "locked on the board: open sea heading %.1f deg", _bdLockDir);
      }
      break;
    }
    case L2_GO: {
      const float gate = 0.7f * leg2ExpM;
      if (_frontM < 0.45f) { emergency = true; if (_log) _log->event(MissionLog::EV_HOLD); _prev = L2_GO; _go(HOLD); break; }
      // end wall: after 70 % of the open sea, LEVEL for 1 s (campus_2: kerb bump +9 deg -> ground at 1.3 m = fake wall)
      if (_dist2 >= gate && _frontM <= endWallM && level && _frontWall(s, endWallM + 0.3f)) {
        if (!_endOn) { _endOn = true; _endMs = now; }
        if (now - _endMs >= 800) { _endWall(s); break; }
      } else _endOn = false;
      if (_near(_dist2 >= gate ? holdM * 0.6f : holdM, now)) { _prev = L2_GO; if (_log) _log->event(MissionLog::EV_HOLD); _go(HOLD); break; }
      if (_dist2 > leg2MaxM) { if (_log) _log->event(MissionLog::EV_L2_MAX); _ask(END_ASK, "Maximum open-sea distance driven, no end wall seen."); break; }
      // END BOARD (campus_3: signboard on the right ~0.65 m before the end wall): after 70 %, 2 consistent right-side
      // line fits -> stay parallel to it at the distance first seen (no forced distance); when it ends, keep its direction
      if (_dist2 >= gate && now - _aimMs >= 250) {
        _aimMs = now;
        float dir, dist;
        if (_boardFit(s, 0.3f, 1.5f, dir, dist)) {
          if (!_ebOn) {
            if (_ebN && fabsf(wrap180(dir - _ebDir)) < 5.0f) _ebN++; else _ebN = 1;
            _ebDir = dir;
            if (_ebN >= 2) { _ebOn = true; _ebKeep = clampf(dist, 0.5f, 1.2f); if (_log) _log->event(MissionLog::EV_END_BOARD);
              snprintf(_msg, sizeof(_msg), "end board %.2f m right - following it", dist); }
          } else {
            _ebDir = wrap180(_ebDir + 0.3f * wrap180(dir - _ebDir));
            _aimSeenMs = now;
            _tgt = wrap180(_ebDir - clampf(10.0f * (dist - _ebKeep), -6.0f, 6.0f));   // right side: too far -> turn right
          }
        } else if (_ebOn && now - _aimSeenMs > 1000) _tgt = _ebDir;
      }
      _ds.setTarget(_tgt);
      if (_ds.update(s.imuOk(), yaw, _cruise(), now, l, r) == DriveStraight::FAIL) { if (_log) _log->event(MissionLog::EV_FAILSAFE); _pause(_ds.reason()); }
      break;
    }
    case L3_FOLLOW:
    case L4_FOLLOW: {
      const bool l4 = _st == L4_FOLLOW;
      if (_frontM <= endWallM && level && _frontWall(s, endWallM + 0.3f)) {   // a wall right ahead: the corridor ends here
        if (!_endOn) { _endOn = true; _endMs = now; }
        if (now - _endMs >= 800) { _finishRoute(l4 ? "leg 4: wall ahead - stopped" : "leg 3: wall ahead - stopped"); break; }
      } else _endOn = false;
      if (_guard(s, now, emergency)) break;
      if (_dist > leg3MaxM) { _finishRoute("maximum leg distance driven"); break; }
      _leg1(s, l, r);                               // left wall follower (same as leg 1)
      if (_st != L3_FOLLOW && _st != L4_FOLLOW) break;   // failsafe pause inside
      if (_wallOk) { _l3Seen = true; _noWallFrom = -1; }
      else {
        if (_noWallFrom < 0) _noWallFrom = _dist;
        if (_l3Seen && _dist - _noWallFrom >= wallEndM) {
          l = r = 0;
          if (!l4 && leg4) {                        // T3: building corner -> turn LEFT around it
            if (_log) _log->event(MissionLog::EV_T3_TURN);
            strlcpy(_msg, "arcade wall ended - T3: turning LEFT", sizeof(_msg));
            _turnNo = 3; _turnTries = 0; _startTurn(s, wrap180(_wallDir + 90.0f));
          } else _finishRoute(l4 ? "LEG 4: left wall ended - stop (open space 2)" : "LEFT WALL ENDED - route done");
          break;
        }
        if (!_l3Seen && _dist >= 4.0f) { l = r = 0; _finishRoute("no left wall found in 4 m - stopped"); break; }
      }
      if (fabsf(wrap180(_tgt - yaw)) > 10.0f && now - _rlMs > 3000) { l = r = 0; _realign(s, _tgt, "off the wall line - turning parallel"); }
      break;
    }
    case HOLD:
      if (_leg == 2 && _dist2 >= 0.7f * leg2ExpM && _frontM <= endWallM && level) { _endWall(s); break; }
      if (_leg >= 3 && _frontM <= endWallM && level && _frontWall(s, endWallM + 0.3f)) { _finishRoute("wall ahead - stopped"); break; }
      if (now - _stMs > 20000) { _pause("blocked for 20 s (not a person?) - steer around by hand, then Resume"); break; }
      if (_frontM > 1.0f) {
        if (!_clearOn) { _clearOn = true; _clearMs = now; }
        if (now - _clearMs >= 2000) {
          if (_log) _log->event(MissionLog::EV_HOLD_END);
          _ds.start(yaw, now); _ds.setTarget(_tgt); _go(_prev);
          // turned while waiting (someone moved it): pivot back first instead of tripping the 12 deg failsafe
          if ((_prev == L1_FOLLOW || _prev == L3_FOLLOW || _prev == L4_FOLLOW || _prev == L2_GO) && fabsf(wrap180(_tgt - yaw)) > 10.0f)
            _realign(s, _tgt, "after the wait - turning back");
        }
      } else _clearOn = false;
      break;
    case MANUAL: {
      l = padL; r = padR;
      const bool pivot = (l > 0 && r < 0) || (l < 0 && r > 0);
      if (pivot) { const float d = r > 0 ? 1.0f : -1.0f; l = -0.5f * d; r = 0.5f * d; wheels = true; }
      if (0.5f * (l + r) > 0 && _frontM < 0.35f) { l = r = 0; emergency = true; }
      break;
    }
    default: break;                                   // IDLE, ASK states, PAUSED, DONE: standing
  }

  if (now - _statusMs >= 250) {
    _statusMs = now;
    String j = _statusJson(s, rover);
    if (xSemaphoreTake(_mtx, pdMS_TO_TICKS(20)) == pdTRUE) { _status = j; xSemaphoreGive(_mtx); }
  }
}

// Pillars on the RIGHT (B facade): a short object 0.5-2.5 m to the right, abeam, not a long wall.
void RouteMission::_pillars(const Sensors& s) {
  float xs[Sensors::kLidarPoints]; int nb = 0; bool cand = false;
  float rightSec[16]; int nr = 0;
  for (int i = 0; i < Sensors::kLidarPoints; i++) {
    const uint16_t cm = s.lidarCm(i);
    const float a = (i * 4 + 2) * DEG_TO_RAD;
    const float adeg = i * 4 + 2;
    if (adeg >= 60 && adeg <= 120 && nr < 16) rightSec[nr++] = cm ? cm / 100.0f : 12.0f;
    if (!cm) continue;
    const float d = cm / 100.0f, x = d * cosf(a), y = d * sinf(a);   // x forward, y right
    if (y > 0.5f && y < 2.5f && fabsf(x) < 1.2f) { xs[nb++] = x; if (fabsf(x) < 0.3f) cand = true; }
  }
  float lo = 9, hi = -9; for (int i = 0; i < nb; i++) { lo = fminf(lo, xs[i]); hi = fmaxf(hi, xs[i]); }
  const bool pillar = cand && (nb < 2 || hi - lo < 0.9f);
  if (pillar && !_pillarNear && _dist - _lastPillarM > 2.0f) {
    _pillarsN++; _lastPillarM = _dist;
    if (_log) _log->event(MissionLog::EV_PILLAR);
    Serial.printf("[mission] pillar %u at %.1f m\n", _pillarsN, _dist);
  }
  _pillarNear = pillar;
  // right side "open" (nothing within 4 m) for >= 1 m of travel: logged (candidate turn cue for the final code)
  for (int i = 1; i < nr; i++) { float v = rightSec[i]; int j = i - 1; while (j >= 0 && rightSec[j] > v) { rightSec[j + 1] = rightSec[j]; j--; } rightSec[j + 1] = v; }
  const bool open = nr == 0 || rightSec[nr / 2] > 4.0f;
  if (!open) { _rightOpenFrom = -1; _rightOpenLogged = false; }
  else if (_rightOpenFrom < 0) _rightOpenFrom = _dist;
  else if (!_rightOpenLogged && _dist - _rightOpenFrom >= 1.0f) { _rightOpenLogged = true; if (_log) _log->event(MissionLog::EV_RIGHT_OPEN); }
}

void RouteMission::_leg1(const Sensors& s, float& l, float& r) {
  const uint32_t now = millis();
  // flat-ground pitch reference (the IMU reads ~+2.7 deg on flat ground: mounting). Leg 1 is flat (log_027).
  if (s.imuOk()) _pitch0 += 0.002f * (s.pitchDeg() - _pitch0);
  // v4 (campus_3): before, _wallMs was reset on EVERY scan, so "wall lost" never fired: after the gap at 6.3 m the
  // fit stayed glued to the old line (1.20 m) and the next wall (0.90 m, converging 5-7 deg) was never accepted.
  // Now: scan timer and last-good timer are separate; lost after 1 s without an accepted fit; a NEW wall is taken
  // after 3 consistent fresh fits; a jump > 8 deg is noise unless 3 fits in a row agree (the wall really bends).
  if (now - _wallScanMs >= 250) {                   // one wall fit per LiDAR snapshot
    _wallScanMs = now;
    const WallFit w = WallFit::measure(s, true, _wallOk ? &_wall : nullptr);
    bool good = w.ok && fabsf(w.angleDeg) < 35.0f && w.rmsM < (_wallOk ? 0.08f : 0.06f);
    const float wy = wrap180(s.yawDeg() + w.angleDeg);       // wall direction in the gyro frame
    if (good && !_wallOk) {                                   // new wall: 3 consistent fits first
      if (_acqN && fabsf(wrap180(wy - _acqDir)) < 4.0f) _acqN++; else _acqN = 1;
      _acqDir = wy;
      if (_acqN < 3) good = false;
      else {
        _wallDir = wy; _wyN = _wyI = 0; _jumpN = 0;
        if (_dSetPending) { _dSet = clampf(w.distM, 0.7f, 2.2f); _dSetPending = false; }
      }
    } else if (good) {
      if (_wyN >= 3 && fabsf(wrap180(wy - _wallDir)) > 8.0f) {
        if (_jumpN && fabsf(wrap180(wy - _jumpDir)) < 4.0f) _jumpN++; else _jumpN = 1;
        _jumpDir = wy;
        if (_jumpN >= 3) { _wallDir = wy; _wyN = _wyI = 0; _jumpN = 0; } else good = false;
      } else _jumpN = 0;
    }
    if (good) {                                               // median of the last 5 directions
      _wyBuf[_wyI] = wrap180(wy - _wallDir); _wyI = (_wyI + 1) % 5; if (_wyN < 5) _wyN++;
      float t[5]; for (int i = 0; i < _wyN; i++) t[i] = _wyBuf[i];
      for (int i = 1; i < _wyN; i++) { float v = t[i]; int j = i - 1; while (j >= 0 && t[j] > v) { t[j + 1] = t[j]; j--; } t[j + 1] = v; }
      const float med = t[_wyN / 2];
      _wallDir = wrap180(_wallDir + med);
      for (int i = 0; i < _wyN; i++) _wyBuf[i] -= med;
    }
    if (good) {
      if (!_wallOk && _log) _log->event(MissionLog::EV_WALL_FOUND);
      _wallOk = true; _wallGoodMs = now; _wall = w;
      // stay PARALLEL (wall direction) + keep the distance: too far -> turn left, too close -> turn right
      const float approach = clampf(10.0f * (w.distM - _dSet), -8.0f, 8.0f);
      _tgt = wrap180(_wallDir + approach);
      // CRAB: lateral drift towards / away from the wall vs the body angle, every 0.5 m over a 2 m window
      if (_dist >= _nextCrabM) {
        _nextCrabM = _dist + 0.5f;
        _cs[_cI] = _dist; _cd[_cI] = w.distM; _cb[_cI] = -w.angleDeg; _cI = (_cI + 1) % 10; if (_cN < 10) _cN++;
        if (_cN >= 5) {
          const int iNow = (_cI + 9) % 10, iOld = (_cI + 5) % 10;
          const float ds = _cs[iNow] - _cs[iOld], dd = _cd[iNow] - _cd[iOld];
          if (ds > 1.5f) {
            float body = 0; for (int k = 0; k < 5; k++) body += _cb[(_cI + 5 + k) % 10]; body /= 5;
            const float track = atan2f(-dd, ds) * RAD_TO_DEG;               // + = moving left (towards the wall)
            const float c = track - body;
            if (fabsf(c) < 15.0f) { _crabEst = _crabN == 0 ? c : _crabEst + 0.15f * (c - _crabEst); _crabN++; }
          }
        }
      }
    } else if (_wallOk && now - _wallGoodMs > 1000) {
      _wallOk = false; _cN = 0; _acqN = 0; _jumpN = 0; _wyN = _wyI = 0;
      if (_log) _log->event(MissionLog::EV_WALL_LOST);     // gap / door: keep the last target heading (gyro)
    }
    _pillars(s);
  }
  _ds.setTarget(_tgt);
  if (_ds.update(s.imuOk(), s.yawNowDeg(), _cruise(), now, l, r) == DriveStraight::FAIL) {
    if (_log) _log->event(MissionLog::EV_FAILSAFE);
    _pause(_ds.reason());
  }
}

void RouteMission::fillLog(MissionLog::Sample& d, const Sensors& s, const RoverLink& rover) const {
  d.batteryV = rover.voltage(); d.rttMs = rover.rttMs();
  d.outL = _outL; d.outR = _outR;
  d.state = _st; d.leg = _leg; d.pillars = _pillarsN; d.dsStatus = _ds.status();
  d.tgtDeg = _st == T1_TURN || _st == T1_SETTLE ? _turnTgt : _tgt; d.errDeg = _ds.errorDeg(); d.crabDeg = _crabEst;
  d.wallD = _wallOk ? _wall.distM : 0; d.wallAngDeg = _wallOk ? _wall.angleDeg : 0;
  d.distM = _leg == 2 ? _dist2 : _dist; d.frontM = _frontM;
  d.manual = _st == MANUAL; d.hold = _st == HOLD; d.slope = _slope;
  // former camera fields (decoder v3 names): turn tactic, turn cycles, speed boost (cm/s), measured motion (cm/s)
  d.camPaired = false; d.snapOk = false;
  d.camRows = _tk; d.camConf = _tCycles; d.camOff = _boost * 100.0f; d.camAng = _motionM < 0 ? -1.0f : _motionM * 100.0f;
  d.snapsSent = _shifts; d.snapsOk = 0;
}

String RouteMission::_statusJson(const Sensors& s, const RoverLink& rover) {
  String j; j.reserve(1100);
  j += "{\"st\":"; j += _st;
  j += ",\"state\":\""; j += kSt[_st < 16 ? _st : 0];
  j += "\",\"leg\":"; j += _leg;
  j += ",\"msg\":\""; jsonEsc(j, _msg);
  j += "\",\"q\":\""; jsonEsc(j, (_st == T1_ASK || _st == END_ASK) ? _q : "");
  j += "\",\"prev\":\""; j += kSt[_prev < 16 ? _prev : 0];
  j += "\",\"dist\":"; j += String(_dist, 1);
  j += ",\"dist2\":"; j += String(_dist2, 1);
  j += ",\"askAt\":"; j += String(_askAt, 1);
  j += ",\"pillars\":"; j += _pillarsN;
  j += ",\"wallOk\":"; j += _wallOk ? 1 : 0;
  j += ",\"wallD\":"; j += String(_wallOk ? _wall.distM : 0, 2);
  j += ",\"dSet\":"; j += String(_dSet, 2);
  j += ",\"crab\":"; j += String(_crabEst, 1);
  j += ",\"yaw\":"; j += String(s.yawNowDeg(), 1);
  j += ",\"tgt\":"; j += String((_st == T1_TURN || _st == T1_SETTLE) ? _turnTgt : _tgt, 1);
  j += ",\"err\":"; j += String(_ds.errorDeg(), 1);
  j += ",\"learned\":"; j += String(_ds.learned(), 3);
  j += ",\"front\":"; j += String(_frontM, 2);
  j += ",\"lidarF\":"; j += String(_lidarFront, 2);
  j += ",\"slope\":"; j += _slope ? 1 : 0;
  j += ",\"pitch\":"; j += String(s.pitchDeg() - _pitch0, 1);
  j += ",\"turnNo\":"; j += _turnNo;
  j += ",\"bp\":"; j += _bp; j += ",\"board\":"; j += String(_bdLockDir, 1); j += ",\"eb\":"; j += _ebOn ? 1 : 0;
  j += ",\"aim\":"; j += _aimOn ? 1 : 0; j += ",\"aimDeg\":"; j += String(_aimDeg, 1);
  j += ",\"tac\":\""; j += kTac[_tk < 5 ? _tk : 0];
  j += "\",\"tCycles\":"; j += _tCycles;
  j += ",\"shifts\":"; j += _shifts;
  j += ",\"boost\":"; j += String(_boost, 2);
  j += ",\"cruise\":"; j += String(_cruise(), 2);
  j += ",\"motion\":"; j += String(_motionM, 2);
  j += ",\"rover\":"; j += rover.connected() ? 1 : 0;
  j += ",\"batt\":"; j += String(rover.voltage(), 2);
  j += ",\"imu\":"; j += s.imuOk() ? 1 : 0;
  j += ",\"lidar\":"; j += s.lidarOk() ? 1 : 0;
  j += ",\"gps\":"; j += s.gpsFix() ? 1 : 0; j += ",\"sats\":"; j += s.sats();
  j += ",\"heap\":"; j += (unsigned)(ESP.getFreeHeap() / 1024);
  j += ",\"set\":{\"speed\":"; j += String(speed, 2); j += ",\"t1\":"; j += String(turnAtM, 1); j += ",\"l2\":"; j += String(leg2ExpM, 1);
  j += ",\"end\":"; j += String(endWallM, 2); j += ",\"crab\":"; j += String(crabPriorDeg, 1);
  j += ",\"auto\":"; j += autoTurn ? 1 : 0;
  j += ",\"l3\":"; j += leg3 ? 1 : 0; j += ",\"l4\":"; j += leg4 ? 1 : 0;
  j += ",\"bside\":"; j += String(boardSideM, 2); j += ",\"boff\":"; j += String(boardOffsetDeg, 1); j += ",\"wend\":"; j += String(wallEndM, 1);
  j += "}}";
  return j;
}
