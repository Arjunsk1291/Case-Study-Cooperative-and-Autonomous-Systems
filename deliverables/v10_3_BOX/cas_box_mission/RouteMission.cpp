#include "RouteMission.h"

static float wrap180(float a) { a = fmodf(a + 180.0f, 360.0f); if (a < 0) a += 360.0f; return a - 180.0f; }
static float clampf(float x, float lo, float hi) { return x < lo ? lo : (x > hi ? hi : x); }
static const char* kSt[] = {"ready", "LEG 1: following the left wall", "TURN? waiting for you", "turning right 90 deg",
                            "checking the turn", "-", "-", "OPEN SEA START: lining up on the board", "OPEN SEA: driving straight", "WAITING: something in front",
                            "STOPPED AT THE GOAL", "MANUAL driving (logging)", "PAUSED", "finished",
                            "LEG 3: following the left wall", "LEG 4: following the left wall (after T3)"};
static const char* kTac[] = {"full pivot", "rest", "forward swing", "rolling arc", "shift 0.3 m"};
enum Cmd : uint8_t { C_NONE = 0, C_START, C_TURN, C_ON1M, C_MANUAL, C_RESUME, C_L2HERE, C_FINISH, C_STOP, C_RECORD };
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
  server->on("/mis/climb", HTTP_GET, [this](AsyncWebServerRequest* q) { climbRequest(); _aliveMs = millis(); q->send(200, "application/json", "{\"ok\":true}"); });
  server->on("/mis/spd", HTTP_GET, [this](AsyncWebServerRequest* q) {      // v9: speed override - slower / faster / reset
    speedStep(q->hasParam("d") ? q->getParam("d")->value().toInt() : 0); q->send(200, "application/json", "{\"ok\":true}");
  });
  server->on("/mis/alive", HTTP_GET, [this](AsyncWebServerRequest* q) { _aliveMs = millis(); q->send(200, "application/json", "{\"ok\":true}"); });
  server->on("/mis/cmd", HTTP_GET, [this](AsyncWebServerRequest* q) {
    const String c = q->hasParam("c") ? q->getParam("c")->value() : String("");
    const char* names[] = {"", "start", "turn", "on1m", "manual", "resume", "l2here", "finish", "stop", "record"};
    for (uint8_t i = 1; i <= C_RECORD; i++) if (c == names[i]) _cmd = i;
    _aliveMs = millis();
    q->send(200, "application/json", "{\"ok\":true}");
  });
  server->on("/mis/set", HTTP_GET, [this](AsyncWebServerRequest* q) {
    auto f = [&](const char* k, float& v, float lo, float hi) { if (q->hasParam(k)) v = clampf(q->getParam(k)->value().toFloat(), lo, hi); };
    f("speed", speed, 0.12f, 0.40f); f("spast", structPastM, 0.2f, 1.5f); f("l4speed", l4Speed, 0.15f, 0.40f); f("climbp", climbPow, 0.25f, 0.50f); f("t1", turnAtM, 2.0f, 60.0f); f("l2", leg2ExpM, 5.0f, 80.0f);
    f("end", endWallM, 0.6f, 2.0f); f("crab", crabPriorDeg, -12.0f, 12.0f);
    if (q->hasParam("auto")) autoTurn = q->getParam("auto")->value() == "1";
    if (q->hasParam("l3")) leg3 = q->getParam("l3")->value() == "1";
    f("wend", wallEndM, 0.5f, 5.0f);
    f("ppast", pillarPastM, 0.3f, 1.5f); f("rowoff", rowOffsetM, 3.0f, 9.0f); f("bldoff", bldOffsetM, 1.5f, 4.5f);
    f("trim", seaTrimDeg, -8.0f, 8.0f); f("l4wall", leg4WallM, 0.5f, 2.5f); f("l3wall", leg3WallM, 0.4f, 1.5f);
    f("ninten", nearMinInten, 0.0f, 255.0f);
    f("kp", seaKp, 0.0f, 30.0f); f("ki", seaKi, 0.0f, 5.0f); f("kd", seaKd, 0.0f, 60.0f);
    if (q->hasParam("rows")) useRows = q->getParam("rows")->value() == "1"; f("bside", boardSideM, 0.3f, 1.5f); f("boff", boardOffsetDeg, -15.0f, 15.0f);
    if (q->hasParam("l4")) leg4 = q->getParam("l4")->value() == "1";
    leg2MaxM = fmaxf(50.0f, leg2ExpM * 1.4f);
    q->send(200, "application/json", "{\"ok\":true}");
  });
  return true;
}

void RouteMission::_go(uint8_t st) {
  _st = st; _stMs = millis(); _clearOn = _endOn = false; if (st != HOLD) _endCnt = 0;
  if (st == HOLD) { _hLastNear = 9.9f; _hReceded = false; }   // v10: track how the obstacle leaves
}

int RouteMission::_usMed() const {
  int a = _usHist[0], b = _usHist[1], c = _usHist[2], t;
  if (a > b) { t = a; a = b; b = t; } if (b > c) { t = b; b = c; c = t; } if (a > b) { t = a; a = b; b = t; }
  return b;
}

void RouteMission::_pause(const char* why) {
  if (_st != PAUSED && _st != MANUAL) _prev = _st;
  strlcpy(_msg, why, sizeof(_msg));
  _go(PAUSED);
  if (_log) {
    _log->event(MissionLog::EV_PAUSE);
    // v7: WHY it paused (campus_6 had 6 pauses and the log could not tell STOP from phone from failsafe)
    uint8_t cause = MissionLog::EV_P_DRIVE;                       // default: DriveStraight failsafe / IMU
    if (!strncmp(why, "STOP", 4)) cause = MissionLog::EV_P_STOP;
    else if (!strncmp(why, "phone", 5)) cause = MissionLog::EV_P_PHONE;
    else if (!strncmp(why, "rover", 5)) cause = MissionLog::EV_P_ROVER;
    else if (!strncmp(why, "tilt", 4)) cause = MissionLog::EV_P_TILT;
    else if (!strncmp(why, "turn failed", 11)) cause = MissionLog::EV_P_TURN;
    else if (!strncmp(why, "blocked", 7)) cause = MissionLog::EV_P_BLOCKED;
    _log->event(cause);
  }
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
  const int us = s.usCm(0);
  if (millis() - _usHistMs >= 250) { _usHistMs = millis(); _usHist[_usHistI] = us; _usHistI = (_usHistI + 1) % 3; }
  if (s.lidarOk()) {
    auto bin = [&](int k) { return s.lidarCm((uint8_t)((k + 2 * Sensors::kLidarPoints) % Sensors::kLidarPoints)); };
    int kBest = 0;
    for (int k = -4; k < 4; k++) {
      const uint16_t cm = bin(k);
      const float m = cm / 100.0f;
      if (cm && m < best && m < rg - 0.30f) { best = m; kBest = k; }
    }
    // v8.1: a NEAR LiDAR obstacle (< 0.6 m) is VERIFIED before the rover stops for it:
    //  - the FRONT ultrasonic (horizontal one; the 45-deg-down one only sees the ground) agrees: median of its last
    //    3 readings below 50 cm (its floor echo is 50-90 cm: 40 % of all readings with nothing in front) and not
    //    further than the LiDAR distance + 15 cm, OR
    //  - it is WIDE: >= 4 neighbouring 4-deg beams within 15 cm (a person, a wall at an angle). Ghosts: 1-2 beams.
    // Replayed on all logs: daytime false holds 7 -> 0 and 21 -> 0; the person, the tree stake and the converging
    // wall of campus_3 still stop the rover.
    if (best < 0.6f) {
      const uint16_t c0 = (uint16_t)(best * 100);
      int run = 1;
      for (int d = -1; d <= 1; d += 2)
        for (int k = kBest + d; k >= -7 && k <= 6; k += d) { const uint16_t c = bin(k); if (c && c <= c0 + 15) run++; else break; }
      int a = _usHist[0], b = _usHist[1], c = _usHist[2], t;
      if (a > b) { t = a; a = b; b = t; } if (b > c) { t = b; b = c; c = t; } if (a > b) { t = a; a = b; b = t; }
      const int usMed = b;
      const bool usOk = usMed > 2 && usMed < 50 && usMed <= c0 + 15;
      if (run < 4 && !usOk) {
        if (_log && millis() - _ghostLogMs > 2000) { _ghostLogMs = millis(); _log->event(MissionLog::EV_GHOST); }
        float b2 = 9.9f;                                         // ignore the near ghost: next real distance
        for (int k = -4; k < 4; k++) { const uint16_t c = bin(k); const float m = c / 100.0f; if (c && m >= 0.6f && m < b2 && m < rg - 0.30f) b2 = m; }
        best = b2;
      }
    }
  }
  _lidarFront = best;
  const bool near = us > 2 && us < 35;   // v10: was 25 (campus_9: bollard at 28-35 cm was not stopped for)
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
  if (t == TK_SHIFT) _tBack = _frontM <= 1.0f;                  // v10.3: front blocked -> this shift goes backwards
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
    case TK_SHIFT: {
      // v10.3 (campus_10.2 runs 3+4: 35-42 s stalled at the open-sea end wall, front 0.7-0.9 m): with the front
      // blocked SWING / ARC / SHIFT were all skipped and only the PIVOT was left, which the stones there lock.
      // Front blocked -> shift BACKWARDS 0.3 m at 0.20 (nothing within 0.8 m behind, LiDAR 148-210 deg) = room again.
      bool rearOk = true;
      for (int i = 37; i <= 52; i++) { const uint16_t cm = s.lidarCm(i); if (cm && cm < 80) { rearOk = false; break; } }
      if ((_tBack ? !rearOk : _frontM <= 1.0f) || _tShiftD >= 0.30f || now - _tMs > 3500) { if (_tShiftD > 0.05f) _shifts++; stalled = true; break; }
      l = r = _tBack ? -0.20f : 0.25f;
      break;
    }
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
  (void)driving; _boost = 0;                                   // v9: no automatic speed-up (you: stays slow); shown only
}

void RouteMission::_startLeg2(const Sensors& s) {
  const float crabUse = _crabN >= 6 ? _crabEst : crabPriorDeg;
  _base = wrap180(_base - crabGain * crabUse);             // _base set by the caller (turn target or current heading)
  _tgt = _base;
  _leg = 2; _dist2 = 0; _ebOn = false; _ebN = 0; _aimMs = 0;
  _refType = 0; _refKind = 0; _refN = _refI = 0; _refMs = 0; _rearSeen = _frontSeen = false; _lastMvT = _mvT;
  _latOff = 0; _refOff = 0; _seaEi = 0; _rowSeen = _bldSeen = false;
  // the turn came later than pillarPastM past the pillar (it was only seen late): move right WHILE DRIVING -
  // 10 deg right for (extra / sin 10 deg), then straight. No stop, no extra pivots. Max 1.0 m sideways.
  {
    const float extra = _tpTurn ? clampf(_tpY - pillarPastM, 0.0f, 1.0f) : 0.0f;
    _shiftEndM = extra > 0.15f ? extra / sinf(10.0f * DEG_TO_RAD) : 0.0f;
    _shiftOff = 0;
  }
  _ds.start(s.yawNowDeg(), millis()); _ds.setTarget(_tgt);
  if (_log) _log->event(MissionLog::EV_GO_L2);
  snprintf(_msg, sizeof(_msg), "open sea: heading %.1f, shift right %.2f m, landmark corridor %s", _tgt,
           _shiftEndM * sinf(10.0f * DEG_TO_RAD), useRows ? "on" : "off");
  Serial.printf("[mission] leg 2: %s\n", _msg);
  _go(L2_GO);
}

// Kethan's turnLandmark on our frame (offset 83), widened to see the object from abeam to 2.1 m behind:
// right-rear bins 88-162 deg, points 0.3-2.1 m behind and 0.9-2.9 m right, one missing bin allowed, >= 4 points,
// >= 0.45 m wide to the right (a thin pillar is not it). Out: near face (smallest behind / smallest right).
// Replayed on campus_2: seen from 18.96 m on, nothing else between 15 m and the turn.
bool RouteMission::_landmark(const Sensors& s, float& nearBehind, float& nearRight, int maxDeg) const {
  if (!s.lidarOk()) return false;
  int n = 0, best = 0; bool miss = false;
  float bMin = 0, rMin = 0, rMax = 0, lb = 0, lr = 0;
  auto close = [&]() {
    if (n >= 4 && rMax - rMin >= 0.45f && n > best) { best = n; nearBehind = bMin; nearRight = rMin; }
    n = 0;
  };
  const int iMax = (maxDeg - 2) / 4;            // v9: 138 deg without a lock (log8: a person following behind-right)
  for (int i = 22; i <= iMax; i++) {
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

// ---------------------------------------------------------------------------------------------------------------
// v9 TURN STRUCTURE (log8 + all successful runs). log8 turned one pillar too early: a person walking 1.2-2.2 m
// behind-right merged with an ordinary pillar into a "wide" landmark. Beside the real turn pillar there is a face
// running from the pillar line to >= 2.3 m right (the structure you line the rover up on). Ordinary pillars have
// nothing behind them. Evidence grid in the leg-1 frame (odometry along x lateral right), fed only from beams 2-134 deg
// (ahead to abeam - a follower is never in it). Structure = in a 0.4 m band a contiguous run (one empty cell allowed)
// of >= 5 lateral 0.25 m cells with >= 3 hits each, starting <= 2.3 m and reaching >= 2.3 m right.
// Window: the left wall comes back after the B gap 5.2-5.5 m before the structure in every run (log8 15.7 -> 21.2,
// test2 14.7 -> 20.0, campus6 12.9 -> 18.1, campus5 13.7 -> 19.2, test1 15.2 -> 20.5) -> window gap end + 5.35 +-1.2;
// without that anchor turnAt -5 .. +5. A candidate LOCKS after 3 sightings within +-0.3 m seen from rover positions
// >= 0.8 m apart (people walking along never collect them); the nearest lock without another candidate 0.8-7 m
// further ahead is the turn (people standing in a row near an earlier pillar). Replayed: tools/replay_turn.py.
void RouteMission::_structReset() {
  for (int i = 0; i < kSA; i++) _sgK[i] = INT32_MIN;
  _candN = 0; _sLock = -1; _e2N = 0; _gapAnchor = -1; _lostAt = -1; _sgMs = 0;
}

void RouteMission::_structAdd(const Sensors& s) {
  if (!s.lidarOk()) return;
  const float rel = wrap180(s.yawDeg() - _wallDir);           // rover heading, deg LEFT of the B wall direction
  const int32_t kMin = (int32_t)floorf(_dist / 0.1f) - 30;
  for (int i = 0; i < 34; i++) {
    const uint16_t cm = s.lidarCm(i);
    if (!cm || cm > 600) continue;
    const float a = (i * 4 + 2 - rel) * DEG_TO_RAD, d = cm / 100.0f, f = d * cosf(a), sd = d * sinf(a);
    if (f < -0.3f || f > 5.0f || sd < 0.8f) continue;
    const int j = (int)((sd - 0.8f) / 0.25f);
    if (j >= kSL) continue;
    const int32_t k = (int32_t)floorf((_dist + f) / 0.1f);
    if (k < kMin) continue;
    const int idx = (int)(((k % kSA) + kSA) % kSA);
    if (_sgK[idx] != k) { _sgK[idx] = k; memset(_sg[idx], 0, kSL); }
    if (_sg[idx][j] < 255) _sg[idx][j]++;
  }
}

int RouteMission::_structDetect(float ahead, float* pos, int maxN) const {
  const int32_t k0 = (int32_t)floorf((_dist - 1.0f) / 0.1f), k1 = (int32_t)floorf((_dist + ahead) / 0.1f);
  int n = 0, lastCells = 0;
  for (int32_t k = k0; k <= k1 - 4; k++) {
    uint16_t col[kSL]; memset(col, 0, sizeof(col));
    for (int32_t kk = k; kk < k + 4; kk++) {
      const int idx = (int)(((kk % kSA) + kSA) % kSA);
      if (_sgK[idx] == kk) for (int j = 0; j < kSL; j++) col[j] += _sg[idx][j];
    }
    int best = 0;
    for (int j = 0; j < kSL;) {
      if (col[j] < 3) { j++; continue; }
      int a = j, e = j, gap = 0;
      while (e + 1 < kSL && (col[e + 1] >= 3 || (gap == 0 && e + 2 < kSL && col[e + 2] >= 3))) { if (col[e + 1] < 3) gap++; e++; }
      const float nearL = 0.8f + a * 0.25f, farL = 0.8f + (e + 1) * 0.25f;
      const int cells = e - a + 1;
      if (cells >= 4 && nearL <= 2.3f && farL >= 2.3f && cells > best) best = cells;   // v10.3: 4 (c102_4: sign face 1.3-2.3 m = 4 cells)
      j = e + 1;
    }
    if (!best) continue;
    const float p = (k + 2) * 0.1f;
    if (n && p - pos[n - 1] <= 0.3001f) { if (best > lastCells) { pos[n - 1] = p; lastCells = best; } }
    else if (n < maxN) { pos[n++] = p; lastCells = best; }
  }
  return n;
}

void RouteMission::_structWin(float& lo, float& hi) const {
  // v10 (campus_9 started 7.7 m earlier: the sign was at 27.9 m, outside turnAt +-5): no start-dependent window any more
  if (_gapAnchor >= 0) { lo = _gapAnchor + 5.35f - 2.0f; hi = _gapAnchor + 5.35f + 2.0f; }   // v10.3: +-2.0 (c102_4: sign 6.4-6.9 m after the gap)
  else { lo = _askAt - turnWinM; hi = _askAt + 15.0f; }   // no B gap seen: 15 m on (a gap makes the anchor window)
}

void RouteMission::_structUpdate() {
  float lo, hi; _structWin(lo, hi);
  float pos[8]; const int n = _structDetect(6.0f, pos, 8);
  for (int q = 0; q < n; q++) {
    const float p = pos[q];
    if (p < lo || p > hi) continue;
    int ci = -1; float bd = 99;
    for (int c = 0; c < _candN; c++) { const float d = fabsf(_cand[c].a - p); if (d < bd) { bd = d; ci = c; } }
    if (ci < 0 || bd > 0.4f) {
      if (_candN >= 6) continue;
      ci = _candN++; Cand& c = _cand[ci]; c.a = p; c.nS = 0; c.n = 0; c.d0 = c.d1 = _dist; c.last = -9;
    }
    Cand& c = _cand[ci];
    if (_dist - c.last >= 0.2f) {
      if (c.nS < 9) c.S[c.nS++] = p; else { for (int i = 0; i < 8; i++) c.S[i] = c.S[i + 1]; c.S[8] = p; }
      c.last = _dist; c.d1 = _dist;
      float t[9]; for (int i = 0; i < c.nS; i++) t[i] = c.S[i];
      for (int i = 1; i < c.nS; i++) { float v = t[i]; int j = i - 1; while (j >= 0 && t[j] > v) { t[j + 1] = t[j]; j--; } t[j + 1] = v; }
      c.a = t[c.nS / 2];
      c.n = 0; for (int i = 0; i < c.nS; i++) if (fabsf(c.S[i] - c.a) <= 0.3f) c.n++;
      if (c.n == 1) c.d0 = _dist;
    }
  }
}

float RouteMission::_structChoice(uint8_t nMin, float spread) const {
  float best = -1;
  for (int i = 0; i < _candN; i++) {
    const Cand& c = _cand[i];
    if (c.n < nMin || c.d1 - c.d0 < spread) continue;
    bool further = false;
    for (int o = 0; o < _candN; o++) if (o != i && _cand[o].n >= 2 && _cand[o].a - c.a >= 0.8f && _cand[o].a - c.a <= 7.0f) further = true;
    if (!further && (best < 0 || c.a < best)) best = c.a;
  }
  return best;
}

// v9: the open-sea END WALL must also be WIDE (tools/sim_open_sea.py --crowd: one person or a group of 3 standing in
// front passed the flat-wall check -> fake end wall -> right turn in the middle of the open sea). Real end walls in
// every log: the same forward distance (+-0.3 m) over +-48 deg = more than +-1.3 m. Here: on EACH side >= 2 beams
// (22-62 deg) within 0.5 m of the front's forward distance and >= 1.1 m to the side.
// v10.2 (campus_10: turned with the pillar still 0.3-0.6 m AHEAD at 1.0 m right -> faced it at 0.84 m after the turn).
// v6 timing = the pillar's near face pillarPastM (0.40 m) BEHIND. Every good turn (campus2/3/5/6, test1/2) had nothing on
// the right 0.3-2.0 m out between -0.40 m and +1.5 m; every bad one (campus_9, campus_10, log8) had a pillar there.
bool RouteMission::_rightAhead(const Sensors& s) const {
  for (int i = 0; i < Sensors::kLidarPoints; i++) {
    const uint16_t cm = s.lidarCm(i);
    if (cm < 35) continue;
    const float a = (i * 4 + 2) * DEG_TO_RAD, d = cm / 100.0f, x = d * sinf(a), y = d * cosf(a);
    if (x >= 0.3f && x <= 2.0f && y > -pillarPastM && y < 1.5f) return true;
  }
  return false;
}

bool RouteMission::_frontWallWide(const Sensors& s, float maxM) const {
  if (!_frontWall(s, maxM)) return false;
  float fc[8]; int nf = 0;
  for (int k = -4; k < 4; k++) {
    const uint16_t cm = s.lidarCm((uint8_t)((k + Sensors::kLidarPoints) % Sensors::kLidarPoints));
    if (cm) fc[nf++] = cm / 100.0f * cosf((k * 4 + 2) * DEG_TO_RAD);
  }
  if (!nf) return false;
  for (int i = 1; i < nf; i++) { float v = fc[i]; int j = i - 1; while (j >= 0 && fc[j] > v) { fc[j + 1] = fc[j]; j--; } fc[j + 1] = v; }
  const float f0 = fc[0];                       // v10: the NEAREST surface (a median can land on the far wall behind a person)
  auto side = [&](int k0, int k1) {
    int n = 0;
    for (int k = k0; k < k1; k++) {
      const uint16_t cm = s.lidarCm((uint8_t)((k + Sensors::kLidarPoints) % Sensors::kLidarPoints));
      if (!cm) continue;
      const float a = (k * 4 + 2) * DEG_TO_RAD, d = cm / 100.0f;
      if (fabsf(d * cosf(a) - f0) <= 0.5f && fabsf(d * sinf(a)) >= 1.1f) n++;
    }
    return n;
  };
  return side(5, 16) >= 2 && side(-16, -5) >= 2;
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
    _wallDir = wrap180(s.yawDeg() + w.angleDeg); _wall = w; _wallOk = true;
  } else { _wallDir = _turnTgt; _wallOk = false; }
  _dSet = leg3WallM;                                   // v8: close in to leg3WallM (0.6 m), whatever it sees first
  _dSetPending = false; _acqN = _jumpN = 0; _wallGoodMs = millis();
  _tgt = _wallDir; _l3Seen = _wallOk; _noWallFrom = -1;
  _ds.start(s.yawNowDeg(), millis()); _ds.setTarget(_tgt);
  if (_log) _log->event(MissionLog::EV_L3_START);
  snprintf(_msg, sizeof(_msg), "leg 3: left wall %s, keeping %.2f m", _wallOk ? "seen" : "NOT seen yet", _dSet);
  Serial.printf("[mission] %s\n", _msg);
  _go(L3_FOLLOW);
}

// stop + pivot to tgt (Kethan pivot), then back to the leg it came from (campus_3: hand-placed / converging rover)
void RouteMission::_realign(const Sensors& s, float tgt, const char* why) {
  _rlBack = _st; _rlMs = millis(); _drift1.newSegment();   // a pivot may shift the gyro a little: new stretch
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

// v10.3 START ALIGNMENT. The sign pillar / board stands 0.9-1.6 m right of the strip in every good run (campus_3/5/6,
// test2, log8, campus_9, c102_4); the late turns read 2.0 m (campus_10) and 2.9 m (c102_3). Nearest object 0.3-4.5 m right,
// abeam (y -1.0..+1.0 m), >= 2 points within 0.8 m of it; after a roll it is followed by odometry (0.6 m around where it must be).
// v10.3 START POSE: the B wall (cement structure) behind the start = the dominant straight line within 0.6-4 m (>= 8 points on it,
// >= 2 m long, rms <= 6 cm; RANSAC over pairs >= 0.5 m apart, 10 cm band). Out: bearing of its foot point (deg clockwise from the
// front: 180 = exactly behind) and its distance. The sign pillar / board (1.1 m) and the structure ahead-left are too short.
bool RouteMission::_wallLine(const Sensors& s, float& bearing, float& dist) const {
  float px[Sensors::kLidarPoints], py[Sensors::kLidarPoints]; int m = 0;
  for (int i = 0; i < Sensors::kLidarPoints; i++) {
    const uint16_t cm = s.lidarCm(i);
    if (!cm || cm < 60 || cm > 400) continue;
    const float a = (i * 4 + 2) * DEG_TO_RAD, d = cm / 100.0f;
    px[m] = d * cosf(a); py[m] = d * sinf(a); m++;                    // x forward, y right
  }
  if (m < 8) return false;
  int bestK = 0; float bnx = 0, bny = 0, bc = 0;
  for (int i = 0; i < m; i++) for (int j = i + 1; j < m; j++) {
    const float dx = px[j] - px[i], dy = py[j] - py[i], n = sqrtf(dx * dx + dy * dy);
    if (n < 0.5f) continue;
    const float nx = -dy / n, ny = dx / n, c = nx * px[i] + ny * py[i];
    int k = 0; for (int q = 0; q < m; q++) if (fabsf(nx * px[q] + ny * py[q] - c) < 0.10f) k++;
    if (k > bestK) { bestK = k; bnx = nx; bny = ny; bc = c; }
  }
  if (bestK < 8) return false;
  float mx = 0, my = 0; int k = 0;
  for (int q = 0; q < m; q++) if (fabsf(bnx * px[q] + bny * py[q] - bc) < 0.10f) { mx += px[q]; my += py[q]; k++; }
  mx /= k; my /= k;
  float sxx = 0, syy = 0, sxy = 0;
  for (int q = 0; q < m; q++) if (fabsf(bnx * px[q] + bny * py[q] - bc) < 0.10f) { const float a = px[q] - mx, b = py[q] - my; sxx += a * a; syy += b * b; sxy += a * b; }
  const float th = 0.5f * atan2f(2 * sxy, sxx - syy), ux = cosf(th), uy = sinf(th);
  float lo = 1e9f, hi = -1e9f, sq = 0;
  for (int q = 0; q < m; q++) if (fabsf(bnx * px[q] + bny * py[q] - bc) < 0.10f) {
    const float al = (px[q] - mx) * ux + (py[q] - my) * uy, r = -uy * (px[q] - mx) + ux * (py[q] - my);
    lo = fminf(lo, al); hi = fmaxf(hi, al); sq += r * r;
  }
  if (hi - lo < 2.0f || sqrtf(sq / k) > 0.06f) return false;
  const float d = -uy * mx + ux * my, fx = -uy * d, fy = ux * d;     // foot point of the perpendicular
  bearing = atan2f(fy, fx) * RAD_TO_DEG; dist = fabsf(d);
  return true;
}

// Start of the pose routine: stand still, find the wall, face away from it, then the sign pillar (phase 0...). placed = the "Start
// open sea HERE" button (any position / heading in the start area); otherwise right after the autonomous turn (only small corrections).
void RouteMission::_beginStartPose(const Sensors& s, bool placed) {
  const uint32_t now = millis();
  _leg = 2; _dist2 = 0; _tgt = _base; _apOn = true; _apPh = 10; _apRound = 0; _apN = 0; _apHead = 0; _apLastE = 99; _apWn = 0; _apOrient = 0;
  _apTrack = _apFallback = false; _apPlaced = false; _apPlaced0 = placed; _apYMin = -1.0f;
  _apT0 = _apMs = _apMeasT0 = now;
  _ds.start(s.yawNowDeg(), now); _ds.setTarget(_tgt);
  strlcpy(_msg, placed ? "start here: looking for the B wall behind" : "turn done: checking the wall behind, then the sign pillar", sizeof(_msg));
  _go(L2_ALIGN);
}

// The rover STANDS STILL while it measures, so a real object is at the same place in every scan; snow flakes, ghosts and a person
// walking by are not (simulated: random near returns made the plain "nearest point" fail, the vote did not). Per scan the points
// that may be the sign (<= 24); the last 8 scans; the nearest point that has a point within 0.15 m in >= 4 of them is the sign.
void RouteMission::_apAddScan(const Sensors& s) {
  uint8_t n = 0; float* px = _apPX[_apHead]; float* py = _apPY[_apHead];
  for (int i = 0; i < Sensors::kLidarPoints && n < 24; i++) {
    const uint16_t cm = s.lidarCm(i);
    if (!cm || cm < 35) continue;
    const float a = (i * 4 + 2) * DEG_TO_RAD, d = cm / 100.0f, x = d * sinf(a), y = d * cosf(a);
    const float yl = _apPlaced ? 2.6f : 1.0f;                       // placed anywhere: the pillar may be 2.6 m ahead / behind; the wall itself is excluded (_apYMin)
    const bool in = _apTrack ? (x > 0.3f && hypotf(x - _apTX, y - _apTY) <= 0.6f) : (x >= 0.3f && x <= 4.5f && y > fmaxf(-yl, _apYMin) && y < yl);
    if (in) { px[n] = x; py[n] = y; n++; }
  }
  _apNs[_apHead] = n; _apHead = (_apHead + 1) & 7; if (_apN < 8) _apN++;
}

bool RouteMission::_apVote(float& bx, float& by) const {
  if (_apN < 8) return false;
  bool have = false; float bestX = 99, bestY = 0;
  for (int a = 0; a < 8; a++) for (int i = 0; i < _apNs[a]; i++) {
    const float x = _apPX[a][i], y = _apPY[a][i]; float sx = 0, sy = 0; int cnt = 0;
    for (int b = 0; b < 8; b++) {
      float bd = 0.15f + 0.05f * x, qx = 0, qy = 0; bool f = false;           // beam spacing grows with range
      for (int j = 0; j < _apNs[b]; j++) { const float d = hypotf(_apPX[b][j] - x, _apPY[b][j] - y); if (d <= bd) { bd = d; qx = _apPX[b][j]; qy = _apPY[b][j]; f = true; } }
      if (f) { sx += qx; sy += qy; cnt++; }
    }
    if (cnt >= 4) { const float mx = sx / cnt; if (!have || mx < bestX - 0.05f) { have = true; bestX = mx; bestY = sy / cnt; } }
  }
  if (have) { bx = bestX; by = bestY; }
  return have;
}

void RouteMission::_apFinish(const Sensors& s, const char* msg) {
  _apOn = false; _tpTurn = false; _base = _apBase;
  if (_log) _log->event(MissionLog::EV_ALIGN_START);
  strlcpy(_msg, msg, sizeof(_msg)); Serial.printf("[mission] align: %s\n", _msg);
  _startLeg2(s);
  strlcpy(_msg, msg, sizeof(_msg));
}

void RouteMission::_alignStart(const Sensors& s, uint32_t now, float& l, float& r) {
  const float yaw = s.yawNowDeg();
  l = r = 0;
  if (now - _apT0 > (_apPlaced ? 100000u : 60000u)) { _apFinish(s, "alignment: time used - going on"); return; }
  if (_apPh == 11) { _apPh = 10; _apWn = 0; _apMs = now; return; }   // orient pivot done: look at the wall again
  if (_apPh == 10) {                                                // stand still, find the B wall (8 scans), face away from it
    if (now - _apMs < 250) return;
    _apMs = now;
    float b, d; const bool f = _wallLine(s, b, d);
    _apBear[_apWn] = b; _apWd[_apWn] = d; _apWok[_apWn] = f; _apWn++;
    if (_apWn < 8) return;
    _apWn = 0;
    float v[8]; int n = 0; for (int i = 0; i < 8; i++) if (_apWok[i]) v[n++] = _apBear[i];
    float med = 0; int agree = 0;
    if (n >= 5) {
      for (int i = 1; i < n; i++) { float t = v[i]; int j = i - 1; while (j >= 0 && v[j] > t) { v[j + 1] = v[j]; j--; } v[j + 1] = t; }
      med = v[n / 2]; for (int i = 0; i < 8; i++) if (_apWok[i] && fabsf(wrap180(_apBear[i] - med)) <= 8.0f) agree++;
    }
    if (agree < 5) { _apYMin = -1.0f; _apPlaced = false; _apPh = 0; _apN = 0; _apHead = 0; _apMeasT0 = now;
      strlcpy(_msg, "no B wall line seen - using the heading as it is", sizeof(_msg)); return; }
    float ds[8]; int k = 0; for (int i = 0; i < 8; i++) if (_apWok[i] && fabsf(wrap180(_apBear[i] - med)) <= 8.0f) ds[k++] = _apWd[i];
    for (int i = 1; i < k; i++) { float t = ds[i]; int j = i - 1; while (j >= 0 && ds[j] > t) { ds[j + 1] = ds[j]; j--; } ds[j + 1] = t; }
    const float wd = ds[k / 2], turn = wrap180(med + 180.0f);          // deg to the RIGHT that makes the wall exactly behind
    const bool autoMode = !_apPlaced0;
    if (fabsf(turn) > TURN_OK_DEG && _apOrient < 2 && (!autoMode || fabsf(turn) <= 25.0f)) {
      _apOrient++; _apBase = wrap180(yaw - turn);
      snprintf(_msg, sizeof(_msg), "B wall %.1f m behind-ish: turning %.0f deg to face away from it", wd, turn);
      Serial.printf("[mission] start pose: %s\n", _msg); if (_log) _log->event(MissionLog::EV_ALIGN_START);
      _apPh = 11; _turnNo = 8; _turnTries = 0; _startTurn(s, _apBase); return;
    }
    if (_apPlaced0) _apBase = wrap180(yaw - turn);                      // placed: the wall-normal heading is the open-sea heading
    _apYMin = _apPlaced0 ? -wd + 0.4f : -1.0f; _apPlaced = _apPlaced0;
    snprintf(_msg, sizeof(_msg), "B wall %.1f m behind, heading %s - looking for the sign pillar", wd, fabsf(turn) <= TURN_OK_DEG ? "ok" : "left as it is");
    _apPh = 0; _apN = 0; _apHead = 0; _apMeasT0 = now; return;
  }
  if (_apPh == 1) {                                                 // pivot done -> roll sideways (slowly, straight)
    _apPh = 2; _apRollStart = _dist2; _tgt = yaw; _ds.start(yaw, now); _ds.setTarget(_tgt); _apMs = now; return;
  }
  if (_apPh == 2) {
    if (_dist2 - _apRollStart >= _apRoll || now - _apMs > 14000 || _frontM < 1.0f) {
      _apPh = 3; _turnNo = 8; _turnTries = 0; _startTurn(s, _apBase); return;   // pivot back to the B-perpendicular heading
    }
    _ds.setTarget(_tgt);
    if (_ds.update(s.imuOk(), yaw, 0.18f, now, l, r) == DriveStraight::FAIL) { if (_log) _log->event(MissionLog::EV_FAILSAFE); l = r = 0; _realign(s, _tgt, "heading failsafe - turning back"); }
    return;
  }
  if (_apPh == 3) {                                                 // pivot back done: the sign moved in our frame
    const float rolled = _dist2 - _apRollStart, lat = (_apE > 0 ? 0.5f : -0.5f) * rolled, fwd = 0.866f * rolled;
    if (_apFallback) { _apFinish(s, "late turn: moved by the measured lateness - going on"); return; }
    _apTX -= lat; _apTY -= fwd; _apTrack = true; _apPh = 0; _apN = 0; _apHead = 0; _apMs = now; _apMeasT0 = now; return;
  }
  // phase 0: stand still and measure (one LiDAR snapshot per 250 ms; 8 scans = 2 s, up to 8 s)
  if (now - _apMs < 250) return;
  _apMs = now;
  _apAddScan(s);
  float e = 0; bool go = false, giveUp = false;
  float sx, sy;
  if (_apVote(sx, sy)) {
    const float d = sx;
    if (d >= alignLo && d <= alignHi) {
      char m[64]; snprintf(m, sizeof(m), "sign %.2f m right: aligned - going on", d); _apFinish(s, m); return;
    }
    if (_apRound >= (_apPlaced ? 4 : 3)) { char m[64]; snprintf(m, sizeof(m), "sign %.2f m right after %u rounds - going on", d, _apRound); _apFinish(s, m); return; }
    e = d - alignTarget;
    if (_apRound >= 1 && fabsf(e) > _apLastE + 0.15f) giveUp = true;      // the last move did not improve: stop moving
    _apTX = d; _apTY = sy; go = true;
  } else if (now - _apMeasT0 > 8000) {                              // no sign seen
    if (_apRound == 0 && _apLate > 0.15f) { e = _apLate; _apFallback = true; go = true; }
    else { _apFinish(s, "no sign seen - going on"); return; }
  }
  if (giveUp) { _apFinish(s, "alignment: the last move did not improve - going on"); return; }
  if (!go) return;
  if (_frontM < 1.5f) { _apFinish(s, "alignment: front not free - going on"); return; }
  _apE = e; _apLastE = fabsf(e); _apRoll = fminf(fabsf(e) / 0.5f, alignMaxRoll); _apRound++; _apN = 0; _apHead = 0;
  snprintf(_msg, sizeof(_msg), "sign %.2f m off the target: moving %.2f m %s (round %u)", fabsf(e), fabsf(e), e > 0 ? "right" : "left", _apRound);
  Serial.printf("[mission] align: %s\n", _msg);
  if (_log) _log->event(MissionLog::EV_ALIGN_START);
  _apPh = 1; _turnNo = 8; _turnTries = 0; _startTurn(s, wrap180(_apBase + (e > 0 ? -30.0f : 30.0f)));   // right = yaw minus
}

void RouteMission::_afterTurn(const Sensors& s) {
  const uint32_t now = millis();
  switch (_turnNo) {
    case 2: _startLeg3(s); break;
    case 3: _startLeg4(s); break;
    case 8: _go(L2_ALIGN); _bdMs = 0; break;                       // board positioning continues
    case 9: _ds.start(s.yawNowDeg(), now); _ds.setTarget(_tgt); _rlMs = now; _wallScanMs = 0; _go(_rlBack); break;
    default:                                                       // T1 done
      if (!boardLineUp) {                                          // v5: straight on, walls steer
        _leg = 2; _base = _turnTgt;
        if (startShift) {                                          // v10.3: POSITION FIRST - wall check, then the sign pillar
          _apBase = _turnTgt; _apLate = _tpTurn ? clampf(_lateM, 0.0f, 1.2f) : 0.0f; _tpTurn = false;
          _beginStartPose(s, false); _apBase = _turnTgt;
          break;
        }
        _tpTurn = false; _startLeg2(s); break;
      }
      _leg = 2; _dist2 = 0; _base = _turnTgt; _tgt = _base;
      _bp = 0; _bdN = 0; _bdMs = 0;
      _ds.start(s.yawNowDeg(), now); _ds.setTarget(_tgt);
      strlcpy(_msg, "open sea start: looking for the board on the right", sizeof(_msg));
      _go(L2_ALIGN);
  }
}

// v7 last-leg wall: robust line in the forward-left sector (300 deg +-45, 0.5-5 m, 0.12 m band), accepted only within
// 15 deg of the expected direction. Target = wall direction + approach (towards leg4WallM, 20 deg/m, max 15 deg).
// No wall -> keep the expected direction (gyro). Lost after 1 s without a fit.
void RouteMission::_leg4Wall(const Sensors& s, uint32_t now) {
  if (now - _wallScanMs < 250) return;
  _wallScanMs = now;
  const float yaw = s.yawDeg();
  float ang, off; int n;
  bool good = false;
  float want = _tgt;
  if (LineRef::rowFit(s, 300, 45, 0.5f, 5.0f, 0.12f, 5, 1.0f, 0.5f, ang, off, n) && off < 0) {
    float d = wrap180(yaw - ang);
    if (fabsf(wrap180(d - _expDir)) > 90.0f) d = wrap180(d + 180.0f);
    if (fabsf(wrap180(d - _expDir)) <= 15.0f) {
      good = true;
      if (!_wallOk && _log) _log->event(MissionLog::EV_L4_WALL);
      _wallOk = true; _wallGoodMs = now;
      _wallDir = wrap180(_wallDir + clampf(wrap180(d - _wallDir), -1.0f, 1.0f));   // smooth, max 1 deg per scan
      _wall.ok = true; _wall.distM = -off; _wall.angleDeg = wrap180(_wallDir - yaw); _wall.rmsM = 0; _wall.points = (uint8_t)n;
      const float approach = clampf(20.0f * (-off - _dSet), -15.0f, 15.0f);   // too far -> turn left (+), 20 deg/m
      want = wrap180(_wallDir + approach);
    }
  }
  if (!good && _wallOk && now - _wallGoodMs > 1000) {
    _wallOk = false;
    if (_log) _log->event(MissionLog::EV_WALL_LOST);
  }
  if (!good && !_wallOk) want = _expDir;
  // never more than 10 deg ahead of the current heading, and max 2 deg per scan: the heading hold sees <= 10 deg,
  // so neither the 12 deg failsafe nor a stop-and-pivot can fire (simulated: 2.2 m -> 1.3 m in 5 m, 1.0 m at 12 m,
  // no overshoot, start angle -10..+10 deg)
  want = wrap180(yaw + clampf(wrap180(want - yaw), -10.0f, 10.0f));
  _tgt = wrap180(_tgt + clampf(wrap180(want - _tgt), -2.0f, 2.0f));
}

void RouteMission::_startLeg4(const Sensors& s) {
  const WallFit w = WallFit::measure(s, true, nullptr);
  _leg = 4; _dist = 0; _wyN = _wyI = 0; _cN = _cI = 0; _nextCrabM = 0; _acqN = _jumpN = 0;
  (void)w;
  // v7 (campus_6): the wall after T3 runs at the arcade direction + 90 deg (measured -79.5 vs -78..-85 seen); the old
  // whole-left-side fit caught the corner behind. Expected direction = the T3 turn target; close in to leg4WallM.
  _expDir = _turnTgt; _wallDir = _turnTgt; _wallOk = false; _dSetPending = false; _dSet = leg4WallM;
  _wallGoodMs = millis(); _wallScanMs = 0;
  _tgt = _wallDir; _l3Seen = _wallOk; _noWallFrom = -1;
  _ds.start(s.yawNowDeg(), millis()); _ds.setTarget(_tgt);
  _l4Ph = 0; _rampSeen = false; _railN = 0; _railOk = false; _rampMs = _levelOkMs = 0; _l4ScanMs = 0; _entN = 0; _entFrom = _corrFrom = -1;
  _gN = _gAge = _gLineN = 0; _gHave = _gCommit = _gWaitOn = false; _gWaitMs = 0;
  _eaPh = 0; _eaN = 0; _eaHighMs = 0; _pMin = _pMinNext = s.pitchDeg(); _pMinMs = millis();
  if (_log) _log->event(MissionLog::EV_L4_START);
  snprintf(_msg, sizeof(_msg), "leg 4 (after T3): left wall %s", _wallOk ? "seen" : "not seen yet");
  _go(L4_FOLLOW);
}

// v9 LEG 4 after its wall (log8 manual drive + photos 1-3): the walkway along the interlock (grass / strip / bench
// on the left at ~1.6-2.1 m, nothing close on the right), then the RAILING CORRIDOR before Block I (log8: right railing
// 0.55-0.60 m, left 2.2-2.3 m), then the RAMP (pitch up), stop at the top in front of the door.
//   phase 1 walkway: heading of the last wall; a left edge line (1-3 m, parallel +-12 deg) keeps its first distance
//   phase 2 railing: right line 0.35-0.85 m + something left at 1.6-2.9 m for 1 s -> follow the right railing at that
//            distance (0.5-0.7 m); ramp = pitch > 3 deg above the walkway for 1 s (speed >= 0.35 from there)
//   phase 3 top: level again for 1.5 s -> stop at the door / wall (flat wall <= 1.5 m) or after 6 m
// Target law as leg 3/4: max 10 deg ahead of the heading, max 2 deg per scan.
void RouteMission::_l4Walk(const Sensors& s, uint32_t now) {
  if (now - _l4ScanMs < 250) return;
  _l4ScanMs = now;
  const float yaw = s.yawDeg(), pitch = s.pitchDeg();
  if (_l4Ph == 1) _l4Flat += 0.025f * (pitch - _l4Flat);          // ~10 s average pitch of the walkway
  float ang, off; int n; float want = _l4Dir;
  auto dirOf = [&](float a, float ref) { float d = wrap180(yaw - a); if (fabsf(wrap180(d - ref)) > 90.0f) d = wrap180(d + 180.0f); return d; };
  const float ref = _l4Ph >= 2 ? _railDir : _l4Dir;
  bool rail = false; float rd = 0, roff = 0;
  if (LineRef::rowFit(s, 90, 40, 0.3f, 1.8f, 0.08f, 4, 0.8f, 0.4f, ang, off, n) && off > 0) {   // log8: from 641 s, none before
    const float d = dirOf(ang, ref);
    if (fabsf(wrap180(d - ref)) <= (_l4Ph >= 2 ? 20.0f : 15.0f)) { rail = true; rd = d; roff = off; }
  }
  if (_l4Ph == 1) {
    if (LineRef::rowFit(s, 270, 40, 1.0f, 3.0f, 0.10f, 5, 0.8f, 0.4f, ang, off, n) && off < 0) {
      const float d = dirOf(ang, _l4Dir);
      if (fabsf(wrap180(d - _l4Dir)) <= 12.0f) {
        if (!_l4KeepSet) { _l4KeepSet = true; _l4Keep = clampf(-off, 1.3f, 2.4f); }
        _l4Dir = wrap180(_l4Dir + clampf(wrap180(d - _l4Dir), -0.5f, 0.5f));
        want = wrap180(_l4Dir + clampf(10.0f * (-off - _l4Keep), -6.0f, 6.0f));   // too far from the left edge -> left
      }
    }
    bool leftWide = false;
    float leftD = 0;
    for (int i = 62; i <= 72; i++) {                                // 250-290 deg: abeam left
      const uint16_t cm = s.lidarCm(i);
      if (!cm) continue;
      const float a = (i * 4 + 2) * DEG_TO_RAD, d = cm / 100.0f, sd = -d * sinf(a), f = d * cosf(a);
      if (fabsf(f) <= 0.6f && sd >= 1.4f && sd <= 2.6f) { leftWide = true; leftD = sd; }
    }
    if (leftWide && rail && (leftD + roff < 2.4f || leftD + roff > 3.3f)) leftWide = false;   // corridor width 2.8 m (log8, campus_9)
    if (rail && roff >= 0.35f && roff <= 1.3f && leftWide) _railN++; else _railN = 0;   // v10: campus_9 right railing 0.90-0.99 m
    if (_railN >= 4) {
      _l4Ph = 2; _l4PhStart = _dist; _railDir = rd; _railD = clampf(roff, 0.5f, 0.7f); _railOk = true; _railGoodMs = now;
      if (_log) _log->event(MissionLog::EV_RAILING);
      snprintf(_msg, sizeof(_msg), "railing corridor: keeping %.2f m to the right railing", _railD);
      Serial.printf("[mission] %s\n", _msg);
    }
  } else {
    if (rail && roff <= 1.2f) {
      _railOk = true; _railGoodMs = now;
      if (_corrFrom < 0) _corrFrom = _dist;
      _railDir = wrap180(_railDir + clampf(wrap180(rd - _railDir), -1.0f, 1.0f));
      want = wrap180(_railDir - clampf(10.0f * (roff - _railD), -8.0f, 8.0f));     // too far from the right railing -> right
    } else {
      if (_railOk && now - _railGoodMs > 1500) _railOk = false;
      want = _railDir;
    }
    // v10: KARL ENTRANCE (campus_9 47-53 m): the left facade steps back 0.2-0.5 m and the glass drops beams out of it
    if (_l4Ph == 2 && _corrFrom >= 0 && _dist - _corrFrom > 3.0f) {
      int hit = 0, miss = 0; float dMin = 9;
      for (int i = 60; i <= 74; i++) {
        const uint16_t cm = s.lidarCm(i);
        if (!cm) { miss++; continue; }
        const float a = (i * 4 + 2) * DEG_TO_RAD, sd = -cm / 100.0f * sinf(a);
        if (sd > 0.5f) { hit++; dMin = fminf(dMin, sd); }
      }
      if (_entFrom < 0 && miss >= 3 && hit >= 4) { if (++_entN >= 3) { _entFrom = _dist; if (_log) _log->event(MissionLog::EV_L4_OPEN); strlcpy(_msg, "KARL entrance (glass) beside the rover", sizeof(_msg)); } }
      else if (_entFrom < 0) _entN = 0;
    }
    const float up = pitch - _l4Flat;
    if (_l4Ph == 2 && !_rampSeen) {
      if (up > 3.0f) {
        if (!_rampMs) _rampMs = now;
        if (now - _rampMs >= 1000) {
          _rampSeen = true; if (_log) _log->event(MissionLog::EV_RAMP);
          snprintf(_msg, sizeof(_msg), "RAMP (%.1f deg up): climbing", up);
        }
      } else _rampMs = 0;
    } else if (_l4Ph == 2 && _rampSeen) {
      if (fabsf(up) < 1.5f) {
        if (!_levelOkMs) _levelOkMs = now;
        if (now - _levelOkMs >= 1500) { _l4Ph = 3; _l4PhStart = _dist; strlcpy(_msg, "top of the ramp - to the door", sizeof(_msg)); }
      } else _levelOkMs = 0;
    }
  }
  if (_l4Ph == 1) {                                                // v10.3: the ramp opening overrides the walkway rules
    float gw; const int8_t gr = _l4Gap(s, yaw, gw);
    _gWaitOn = gr == 2;
    if (gr == 2) return;
    if (gr == 1) {                                                  // > 18 deg: jump (realign = ramped stop + full-power pivot)
      const float d = wrap180(gw - _tgt);
      _tgt = fabsf(d) > 18.0f ? gw : wrap180(_tgt + clampf(d, -2.0f, 2.0f));
      return;
    }
  } else _gWaitOn = false;
  want = wrap180(yaw + clampf(wrap180(want - yaw), -10.0f, 10.0f));
  _tgt = wrap180(_tgt + clampf(wrap180(want - _tgt), -2.0f, 2.0f));
}

// v10.3 RAMP OPENING (campus_10.2 run 4: the rover reached the step line 1.9 m right of the ramp and drove onto the
// stairs - they are below the LiDAR). The ramp is a 2.7-3.1 m opening in a CROSS LINE (step edge / lights / low wall)
// with the KARL facade running on from its left end and the railing from its right end. Walkway frame (the scan turned
// by the heading rel. to _l4Dir, + = right): the nearest y bin (0.25 m) with >= 4 points over >= 2 m of x is the line;
// an opening = x gap 2.2-4.0 m with nothing in front of it, and >= 3 facade points from its left end (or railing points
// from its right end) beyond the line; the LEFTMOST one within +-4 m (the stairs right of the railing look open too).
// Replayed on c102_2/3/4 + campus_9 (0 false openings) and simulated (tools/sim_l4_gap.py: 0 hits in 150 starts).
bool RouteMission::_l4FindGap(const Sensors& s, float rot, float ymin, Gap& g) const {
  float px[Sensors::kLidarPoints], py[Sensors::kLidarPoints]; int n = 0;
  for (int i = 0; i < Sensors::kLidarPoints; i++) {
    const uint16_t cm = s.lidarCm(i);
    if (!cm || cm < 35) continue;
    const float a = (i * 4 + 2 + rot) * DEG_TO_RAD, d = cm / 100.0f, x = d * sinf(a);
    if (fabsf(x) > 7.0f) continue;
    px[n] = x; py[n] = d * cosf(a); n++;
  }
  for (int k = (int)(ymin / 0.25f); k < 40; k++) {
    const float y0 = k * 0.25f;
    float xs[Sensors::kLidarPoints]; int m = 0;
    for (int i = 0; i < n; i++) if (py[i] >= y0 - 0.3f && py[i] <= y0 + 0.3f) {
      const float v = px[i]; int j = m - 1; while (j >= 0 && xs[j] > v) { xs[j + 1] = xs[j]; j--; } xs[j + 1] = v; m++;
    }
    if (m < 4 || xs[m - 1] - xs[0] < 2.0f) continue;
    bool found = false;
    for (int q = 0; q + 1 < m && !found; q++) {                    // left to right: the first valid one is the leftmost
      const float a = xs[q], b = xs[q + 1], w = b - a, cx = 0.5f * (a + b);
      if (w < 2.2f || w > 4.0f || fabsf(cx) > 4.0f) continue;
      bool nearPt = false; int fac = 0, rail = 0;
      for (int i = 0; i < n; i++) {
        if (px[i] > a + 0.2f && px[i] < b - 0.2f && py[i] > 0.3f && py[i] < y0 - 0.6f) { nearPt = true; break; }
        if (fabsf(px[i] - a) <= 0.5f && py[i] >= y0 + 0.4f && py[i] <= y0 + 8.0f) fac++;
        if (fabsf(px[i] - b) <= 0.5f && py[i] >= y0 + 0.4f && py[i] <= y0 + 5.0f) rail++;
      }
      if (nearPt || (fac < 3 && rail < 3)) continue;
      g.y0 = y0; g.c = cx; g.a = a; g.b = b; found = true;
    }
    return found;                                                  // the nearest line only
  }
  return false;
}

float RouteMission::_l4LineAhead(const Sensors& s, float rot) const {   // nearest cross line 1-10 m (-1: none)
  float px[Sensors::kLidarPoints], py[Sensors::kLidarPoints]; int n = 0;
  for (int i = 0; i < Sensors::kLidarPoints; i++) {
    const uint16_t cm = s.lidarCm(i);
    if (!cm || cm < 35) continue;
    const float a = (i * 4 + 2 + rot) * DEG_TO_RAD, d = cm / 100.0f, x = d * sinf(a);
    if (fabsf(x) > 5.0f) continue;
    px[n] = x; py[n] = d * cosf(a); n++;
  }
  for (int k = 4; k < 40; k++) {
    const float y0 = k * 0.25f; float lo = 99, hi = -99; int m = 0;
    for (int i = 0; i < n; i++) if (py[i] >= y0 - 0.3f && py[i] <= y0 + 0.3f) { lo = fminf(lo, px[i]); hi = fmaxf(hi, px[i]); m++; }
    if (m >= 6 && hi - lo >= 2.0f) return y0;
  }
  return -1;
}

// one 250 ms scan in the walkway phase. 0 = nothing (normal walkway rules), 1 = want set (heading to drive),
// 2 = WAIT (cross line close and the rover not inside the opening, or a line without an opening: someone standing
// in front of it) - the caller stops with the ramps; after 20 s it asks. Opening confirmed = 3 of the last 5 scans
// agree within 0.5 m. Off by > 0.3 m: aim at a point 1 m before its centre (>= 18 deg away = the realign rule stops
// and pivots at full power); <= 0.3 m: the walkway direction; inside it at <= 1.6 m: committed, straight through.
int8_t RouteMission::_l4Gap(const Sensors& s, float yaw, float& want) {
  if (_gCommit) { want = _l4Dir; return 1; }
  const float rot = wrap180(_l4Dir - yaw);
  Gap g; const bool got = _l4FindGap(s, rot, _gHave ? 0.8f : 2.5f, g);
  const float yl = _l4LineAhead(s, rot);
  _gLineN = (yl > 0 && yl <= 6.0f) ? (uint8_t)min(_gLineN + 1, 50) : 0;
  if (got) { if (_gN == 5) { for (int i = 0; i < 4; i++) _gBuf[i] = _gBuf[i + 1]; _gN = 4; } _gBuf[_gN++] = g; _gAge = 0; }
  else if (_gAge < 255) _gAge++;
  if (_gAge > 8) { _gN = 0; _gHave = false; }
  if (_gN >= 3) {
    float cs[5]; for (int i = 0; i < _gN; i++) cs[i] = _gBuf[i].c;
    for (int i = 1; i < _gN; i++) { float v = cs[i]; int j = i - 1; while (j >= 0 && cs[j] > v) { cs[j + 1] = cs[j]; j--; } cs[j + 1] = v; }
    const float md = cs[_gN / 2]; int good = 0, last = -1;
    for (int i = 0; i < _gN; i++) if (fabsf(_gBuf[i].c - md) <= 0.5f) { good++; last = i; }
    if (good >= 3) {
      if (!_gHave) { if (_log) _log->event(MissionLog::EV_RAMP_GAP);
        snprintf(_msg, sizeof(_msg), "ramp opening %.1f m ahead, centre %.2f m %s", _gBuf[last].y0, fabsf(_gBuf[last].c), _gBuf[last].c < 0 ? "left" : "right"); }
      _gHave = true; _gConf = _gBuf[last];
    }
  }
  if (!_gHave) return (_gLineN >= 3 && yl <= 2.0f) ? 2 : 0;
  const float half = 0.13f;
  if (_gConf.y0 <= 1.6f) {
    if (!(_gConf.a + 0.35f <= -half && half <= _gConf.b - 0.35f)) return 2;
    _gCommit = true; strlcpy(_msg, "inside the ramp opening - straight on", sizeof(_msg));
    want = _l4Dir; return 1;
  }
  if (fabsf(_gConf.c) <= 0.3f) { want = _l4Dir; return 1; }
  const float left = clampf(atan2f(-_gConf.c, fmaxf(_gConf.y0 - 1.0f, 0.8f)) * RAD_TO_DEG, -50.0f, 50.0f);
  want = wrap180(_l4Dir + left);                                    // left = yaw plus
  return 1;
}

// v9 EDGE ASSIST on leg 4 (log8: at 0.22 the rover stood 60 s at the cobble -> interlock edge, pitch +3 -> +6 deg,
// LiDAR distance ahead unchanged; only CLIMB moved it). Front wheels on an edge = pitch >= 3 deg above its minimum of
// the last 10-20 s for 3 s while driving. Then: push climbPow (ramped 0.3 s) for 1.5 s, settle 0.7 s; after 3 pushes:
// back up 0.25 m at 0.20 (only if the LiDAR sees nothing behind within 0.6 m; never on the ramp) and push once more
// (climbPow + 0.05, 2 s); still stuck -> wait 30 s ("please help") and start again. Every stop: wheel ramps.
bool RouteMission::_edgeAssist(const Sensors& s, uint32_t now, float& l, float& r, bool& wheels) {
  const float p = s.pitchDeg();
  if (now - _pMinMs > 10000) { _pMin = _pMinNext; _pMinNext = p; _pMinMs = now; }
  _pMin = fminf(_pMin, p); _pMinNext = fminf(_pMinNext, p);
  const uint32_t t = now - _eaMs;
  auto rearClear = [&]() {
    for (int i = 37; i <= 52; i++) { const uint16_t cm = s.lidarCm(i); if (cm && cm < 60) return false; }
    return true;
  };
  auto again = [&]() { _eaPh = 0; _eaHighMs = 0; _ds.start(s.yawNowDeg(), now); _ds.setTarget(_tgt); };
  wheels = true;
  switch (_eaPh) {
    case 0: {
      wheels = false;
      const bool driving = _outV >= 0.15f;
      // v10: pitch spikes (campus_9: -3.9 deg at 613 s) are not an edge; never push while anything was close ahead
      // in the last 10 s (held / < 1.2 m) or when the front is not free
      const bool quiet = now - _obsNearMs > 10000 && _frontM >= 1.2f && _l4Ph < 2;
      if (driving && quiet && p - _pMin >= 3.0f && p - _pMin < 12.0f) { if (!_eaHighMs) _eaHighMs = now; } else _eaHighMs = 0;
      if (driving && p - _pMin < 1.5f) _eaN = 0;                  // rolling normally again
      if (!_eaHighMs || now - _eaHighMs < 3000 || _frontM < 0.8f) return false;
      _eaMs = now; _eaPitch = p; _pushFront = _frontM;
      if (_eaN < 3) _eaPh = 1;
      else if (_eaN < 10 && !_rampSeen && rearClear()) { _eaPh = 3; if (_log) _log->event(MissionLog::EV_BACKUP); }
      else { _eaPh = 9; strlcpy(_msg, "stuck at an edge - please help (CLIMB / manual); tries again in 30 s", sizeof(_msg)); }
      if (_eaPh == 1) { if (_log) _log->event(MissionLog::EV_EDGE_PUSH); snprintf(_msg, sizeof(_msg), "edge (pitch +%.1f): climb push %u", p - _pMin, _eaN + 1); }
      if (_eaPh == 3) strlcpy(_msg, "edge: backing up 0.25 m, then climbing again", sizeof(_msg));
      l = r = 0; return true;
    }
    case 1: {                                                      // push
      const float c = climbPow * fminf(1.0f, t / 300.0f);
      l = r = (_frontM >= 1.0f) ? c : 0.0f;                          // v10: nothing within 1 m ahead
      if (t >= 1500 || _frontM < 1.0f || (t > 800 && _frontM < _pushFront - 0.05f)) { _eaPh = 2; _eaMs = now; _eaN++; }   // closing in = not an edge
      return true;
    }
    case 2: l = r = 0; if (t >= 700) again(); return true;          // settle, then normal driving / next check
    case 3: {                                                      // back up 0.25 m
      bool clear = rearClear();
      l = r = (t < 1300 && clear) ? -0.20f * fminf(1.0f, t / 300.0f) : 0.0f;
      if (t >= 1300 || !clear) { _eaPh = 4; _eaMs = now; }
      return true;
    }
    case 4: l = r = 0; if (t >= 700) { _eaPh = 5; _eaMs = now; if (_log) _log->event(MissionLog::EV_EDGE_PUSH); } return true;
    case 5: {
      const float c = fminf(0.5f, climbPow + 0.05f) * fminf(1.0f, t / 300.0f);
      l = r = (_frontM >= 1.0f) ? c : 0.0f;
      if (t >= 2000 || _frontM < 1.0f) { _eaPh = 6; _eaMs = now; _eaN = 10; }
      return true;
    }
    case 6: l = r = 0; if (t >= 700) again(); return true;
    default:                                                       // 9: wait for help
      l = r = 0;
      if (t >= 30000) { _eaN = 0; again(); }
      return true;
  }
}

void RouteMission::_finishRoute(const char* why) {
  if (_log) _log->event(MissionLog::EV_WALL_END);
  strlcpy(_msg, why, sizeof(_msg));
  Serial.printf("[mission] END: %s\n", why);
  _ask(END_ASK, why);
}

void RouteMission::noteOutput(float l, float r, float dt) {
  _outL = l; _outR = r; _outV = 0.5f * (l + r);
  if ((fabsf(l) > 0.02f || fabsf(r) > 0.02f) && dt > 0 && dt < 0.2f) _mvT += dt;   // driving time (gyro drift clock)
  if (_outV > 0 && dt > 0 && dt < 0.2f) {
    const float d = _outV * 1.03f * dt;
    if (_leg == 2) _dist2 += d; else _dist += d;
    if (_st == T1_TURN && _tk == TK_SHIFT) _tShiftD += d;
  }
  if (_outV < 0 && dt > 0 && dt < 0.2f && _st == T1_TURN && _tk == TK_SHIFT) _tShiftD += -_outV * 1.03f * dt;   // v10.3 backwards shift
}

void RouteMission::update(const Sensors& s, RoverLink& rover, float padL, float padR, bool padActive,
                          float& l, float& r, bool& emergency, bool& wheels) {
  const uint32_t now = millis();
  const float dt = _lastMs ? (now - _lastMs) / 1000.0f : 0; _lastMs = now;
  const float yaw = s.yawNowDeg();
  wheels = false;
  const bool climb = now - _climbReqMs < 300;           // v8: climb button held (page repeats it every 150 ms)
  if (climb) padActive = true;
  if (s.imuOk() && dt > 0 && dt < 0.2f) { const float d = wrap180(yaw - _prevYaw); if (fabsf(d) < 30) _rate += (d / dt - _rate) * (dt / (0.15f + dt)); }
  _prevYaw = yaw;
  _frontM = _front(s);
  // v10 (campus_9 collision): raw nearest LiDAR return ahead (+-16 deg) and how a held obstacle leaves. Below 0.35 m
  // the LiDAR returns nothing (minMm) - an obstacle that VANISHES from < 0.45 m has not left, it is too close to see.
  {
    float raw = 9.9f; _frontFar = 0;
    for (int k = -4; k < 4; k++) {
      const uint16_t cm = s.lidarCm((uint8_t)((k + Sensors::kLidarPoints) % Sensors::kLidarPoints));
      if (!cm) continue;
      raw = fminf(raw, cm / 100.0f);
      if (cm > 100) _frontFar++;
    }
    if (_frontM < 1.2f) _obsNearMs = now;
    if (_st == HOLD) {
      if (raw < 0.6f) { _hLastNear = raw; _hReceded = false; }
      else if (_hLastNear < 0.6f && raw < 1.6f && raw > _hLastNear + 0.3f) _hReceded = true;   // seen walking away
    }
  }
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
      _crabEst = 0; _crabN = 0; _cN = _cI = 0; _nextCrabM = 0; _askAt = turnAtM; _pitch0 = s.pitchDeg(); _pitchLp = s.pitchDeg(); _boost = 0; _tpTurn = _aimOn = false; _tpClear = 0; _tpEN = 0; _structReset();
      _wallGoodMs = now; _acqN = _jumpN = 0; _dSetPending = false; _rlMs = 0;
      _drift1.reset();
      _ds.start(yaw, now); _ds.setTarget(_tgt);
      if (_log) { _log->start(); _log->event(MissionLog::EV_START); }
      snprintf(_msg, sizeof(_msg), "started: left wall at %.2f m, turn right after %.1f m", _dSet, _askAt);
      _go(L1_FOLLOW);
    }
  } else if (c == C_STOP && active() && _st != PAUSED) {
    _pause("STOP pressed");
  } else if (c == C_RECORD) {
    // v8: "record a manual drive" (leg 4 along the interlock): log on + event, manual mode - you drive, it records
    if (_st != MANUAL) { _prev = (_st == PAUSED) ? _prev : _st; if (_st == IDLE || _st == DONE) _prev = IDLE; }
    if (_log) { if (!_log->active()) _log->start(); _log->event(MissionLog::EV_RECORD); }
    strlcpy(_msg, "RECORDING your manual drive - drive along the interlock, then Finish", sizeof(_msg));
    _go(MANUAL);
  } else if (c == C_MANUAL) {
    if (_st != MANUAL) { _prev = (_st == PAUSED) ? _prev : _st; if (_st == IDLE || _st == DONE) { _prev = IDLE; if (_log) _log->start(); } }
    if (_log) _log->event(MissionLog::EV_MANUAL_ON);
    strlcpy(_msg, "manual driving - everything is logged", sizeof(_msg));
    _go(MANUAL);
  } else if (c == C_RESUME && _st == MANUAL && _leg == 0) {
    // campus4: "Resume" after manual driving BEFORE Start ran leg 1 without its start-up (wall distance, B
    // direction, drift reset). Not started yet -> back to ready.
    if (_log) _log->event(MissionLog::EV_MANUAL_OFF);
    strlcpy(_msg, "not started yet - place the rover and press Start mission", sizeof(_msg));
    _go(IDLE);
  } else if (c == C_RESUME && (_st == PAUSED || _st == MANUAL)) {
    if (_log) _log->event(_st == MANUAL ? MissionLog::EV_MANUAL_OFF : MissionLog::EV_RESUME);
    uint8_t back = (_prev == HOLD || _prev == PAUSED || _prev == MANUAL || _prev == IDLE)
                   ? (uint8_t)(_leg == 4 ? L4_FOLLOW : (_leg == 3 ? L3_FOLLOW : (_leg == 2 ? L2_GO : L1_FOLLOW))) : _prev;
    if (_st == MANUAL && back == L2_GO) { _base = yaw; _tgt = yaw; _refKind = 0; _refN = _refI = 0; _frontSeen = false; _latOff = 0; _seaEi = 0; _shiftOff = 0; _shiftEndM = 0; }
    if (_st == MANUAL && (back == L1_FOLLOW || back == L3_FOLLOW || back == L4_FOLLOW)) { _tgt = yaw; _wallScanMs = 0; }
    _msg[0] = 0;
    if (back == L2_ALIGN) { _apOn = false; _base = _turnTgt; _startLeg2(s); }   // after a pause / take-over: straight on
    else if (back == L1_FOLLOW || back == L2_GO || back == L3_FOLLOW || back == L4_FOLLOW) { _ds.start(yaw, now); _ds.setTarget(_tgt); _rlMs = now; _go(back); }
    else if (back == T1_TURN || back == T1_SETTLE) {
      // the operator may have turned it (partly) by hand: finish the remaining part, or go on if it is done
      if (fabsf(wrap180(_turnTgt - yaw)) <= 6.0f) _afterTurn(s);
      else { _turnTries = 0; _startTurn(s, _turnTgt); }
    } else _go(back);
  } else if (c == C_TURN && (_st == T1_ASK || _st == L1_FOLLOW)) {
    if (_log) _log->event(MissionLog::EV_OP_TURN);
    _tpTurn = false; _turnNo = 1; _turnTries = 0; _startTurn(s, wrap180(_wallDir - 90.0f));   // v5 target: B wall - 90 (last good wall direction)
  } else if (c == C_ON1M && (_st == T1_ASK || _st == L1_FOLLOW)) {
    if (_log) _log->event(MissionLog::EV_OP_ON1M);
    _askAt = fmaxf(_askAt, _dist) + 1.0f;
    snprintf(_msg, sizeof(_msg), "turn moved to %.1f m", _askAt);
    if (_st == T1_ASK) { _ds.start(yaw, now); _ds.setTarget(_tgt); _go(L1_FOLLOW); }
  } else if (c == C_L2HERE && (_st == MANUAL || _st == PAUSED || _st == T1_ASK || _st == IDLE || _st == DONE)) {
    if ((_st == IDLE || _st == DONE) && _log) _log->start();
    if (_dist < 2.0f) _pitch0 = s.pitchDeg();
    _base = yaw;
    if (startShift) { _apBase = yaw; _apLate = 0; _tpTurn = false; _beginStartPose(s, true); _apBase = yaw; }   // v10.3: any position, any heading
    else _startLeg2(s);
  } else if (c == C_FINISH && _st != IDLE) {
    if (_log) { _log->event(MissionLog::EV_DONE); _log->stop(); }
    strlcpy(_msg, "finished - log saved", sizeof(_msg));
    _go(DONE);
  }

  if (_spdEv) {                                       // v9: speed override pressed on the page
    _spdEv = false;
    if (_log) _log->event(MissionLog::EV_SPEED);
    snprintf(_msg, sizeof(_msg), "speed override %+.2f -> forward speed %.2f", speedAdj, _cruise());
  }

  // ---- safety in every moving state
  const bool moving = _st == L1_FOLLOW || _st == T1_TURN || _st == T1_SETTLE || _st == L2_ALIGN || _st == L2_GO || _st == L3_FOLLOW ||
                      _st == L4_FOLLOW;
  if (moving) {
    if (padActive) { _prev = _st; if (_log) _log->event(MissionLog::EV_MANUAL_ON); strlcpy(_msg, "you took over - manual (logging)", sizeof(_msg)); _go(MANUAL); }
    // v8: no phone heartbeat any more (you only need the phone for STOP / manual). Rover link lost: stand still and
    // go on by itself when it is back. Only STOP and a real tilt need Resume.
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
  // v8: rover link lost while driving -> stand still (soft stop by GentleDrive), no pause, continue when it is back
  if (moving && !rover.connected()) {
    if (!_linkWait) { _linkWait = true; if (_log) _log->event(MissionLog::EV_LINK_WAIT); strlcpy(_msg, "rover link lost - waiting, goes on by itself", sizeof(_msg)); }
    _ds.start(yaw, now);
    goto status;
  }
  if (_linkWait) { _linkWait = false; _msg[0] = 0; }
  switch (_st) {
    case L1_FOLLOW:
      if (_guard(s, now, emergency)) break;
      // TURN (v10): the STRUCTURE (black THD sign beside the turn pillar) decides; the v5 pillar edge (+ pillarPastM =
      // 0.40 m, exactly v5) sets the moment next to it. Window from the B gap (gap end + 5.35 m), independent of where
      // the rover was started. No structure: the pillar inside that window; otherwise it STOPS AND ASKS (no blind turn).
      if (autoTurn && usePillar) {
        float lo, hi; _structWin(lo, hi);
        if (now - _tpMs >= 250) {
          _tpMs = now;
          _structAdd(s);
          if (_dist >= lo - 6.0f && (_wallOk || _lostAt < 0)) _structUpdate();   // v10.2: no sign sightings inside the B gap (left wall lost)
          if (_dist >= lo - 1.0f) {
            float nb, nr;
            if (_landmark(s, nb, nr, 162)) {           // v8 edge (exact v8 timing) - used only next to the structure
              if (_tpEN < 7) _tpEdges[_tpEN++] = _dist - nb; else { for (int i = 0; i < 6; i++) _tpEdges[i] = _tpEdges[i + 1]; _tpEdges[6] = _dist - nb; }
              _tpX = nr;
            }
            if (_landmark(s, nb, nr, 138)) {           // no follower behind-right in it
              if (_e2N < 7) _edges2[_e2N++] = _dist - nb; else { for (int i = 0; i < 6; i++) _edges2[i] = _edges2[i + 1]; _edges2[6] = _dist - nb; }
            }
          }
          const float lk = _structChoice(3, 0.8f);
          if (lk >= 0 && fabsf(lk - _sLock) > 0.3f) {
            if (_log) _log->event(MissionLog::EV_STRUCT);
            snprintf(_msg, sizeof(_msg), "turn structure locked at %.1f m (turn at ~%.1f m)", lk, lk + structPastM);
            Serial.printf("[mission] %s\n", _msg);
          }
          _sLock = lk;
        }
        auto med = [](const float* v, int n, float lo2, float hi2) -> float {
          float t[7]; int m = 0;
          for (int i = 0; i < n; i++) if (v[i] >= lo2 && v[i] <= hi2) t[m++] = v[i];
          if (!m) return -99.0f;
          for (int i = 1; i < m; i++) { float x = t[i]; int j = i - 1; while (j >= 0 && t[j] > x) { t[j + 1] = t[j]; j--; } t[j + 1] = x; }
          return t[m / 2];
        };
        float tgt = -1, lockPos = _sLock; const char* why = "";
        if (_sLock >= 0) {
          const float e = med(_tpEdges, _tpEN, _sLock - 0.8f, _sLock + 2.0f);   // v10.2: the pillar may be up to 2 m after the lock (campus_10)
          tgt = e > -90 ? e + pillarPastM : _sLock + structPastM; why = "turn structure";
        } else {
          const float soft = _structChoice(2, 0.4f);   // partly hidden (people in front of it): 0.3 m later
          if (soft >= 0) { tgt = soft + structPastM + 0.3f; lockPos = soft; why = "turn structure (partly hidden)"; }
          else if (_gapAnchor >= 0 && _dist >= lo) {   // v10: the pillar inside the B-gap window only
            const float e = med(_edges2, _e2N, lo - 0.5f, hi + 0.5f);
            if (e > -90) { tgt = e + pillarPastM; why = "pillar in the B-gap window (structure not seen)"; }
          }
        }
        // v10.3 hard failsafe: never earlier than 4.5 m after the B gap (the open space on the left) ended - the sign is
        // 5.2-6.9 m after it in all 12 logs, so no pillar before it can ever start the turn
        if (tgt >= 0 && _dist >= tgt && (_gapAnchor < 0 || _dist >= _gapAnchor + 4.5f) && !_rightAhead(s)) {   // v10.2: never with the pillar still beside / ahead (v6 timing)
          _tpTurn = true; _tpY = pillarPastM + (_dist - tgt);
          // v10.3: lateness vs the STRUCTURE (good turns: lock + 0.41..0.66 m; test1 +0.99 failed in the open sea, c102_3
          // +0.92 ran 1 m off the row, c102_2 +1.89). After the right turn this is the sideways offset to the LEFT.
          _lateM = lockPos >= 0 ? (_dist - lockPos) - 0.55f : (_dist - tgt);
          if (_log) { _log->event(MissionLog::EV_TURN_PILLAR); _log->event(MissionLog::EV_OP_TURN); }
          snprintf(_msg, sizeof(_msg), "%s at %.1f m: turning right", why, _dist);
          Serial.printf("[mission] %s\n", _msg);
          _turnNo = 1; _turnTries = 0; _startTurn(s, wrap180(_wallDir - 90.0f));   // v5 target: B wall - 90 (last good wall direction)
          wheels = true; _turn(s, _turnTgt, l, r);
          break;
        }
      }
      if (_dist >= ((autoTurn && usePillar) ? (_gapAnchor >= 0 ? _gapAnchor + 5.35f + 2.0f + 1.5f : _askAt + 15.0f) : _askAt) ||
          (autoTurn && usePillar && _sLock >= 0 && _dist >= _sLock + 3.5f)) {   // v10.2: locked but the pillar never passed -> ask
        if (_log) _log->event(MissionLog::EV_T1_ASK);
        _tpTurn = false;
        if (autoTurn && !usePillar) {
          snprintf(_msg, sizeof(_msg), "%.1f m, %u pillars: turning right", _dist, _pillarsN);
          if (_log) _log->event(MissionLog::EV_OP_TURN);
          _turnNo = 1; _turnTries = 0; _startTurn(s, wrap180(_wallDir - 90.0f));   // v5 target: B wall - 90 (last good wall direction)
          wheels = true; _turn(s, _turnTgt, l, r);
        } else {
          char q[96]; snprintf(q, sizeof(q), "%.1f m: turn sign NOT found. Turn RIGHT here (at the THD sign pillar)?", _dist);
          _ask(T1_ASK, q);
        }
        break;
      }
      // far off the wanted heading (hand-placed, wall bend, after a stop): stop and pivot parallel BEFORE the heading
      // hold runs (v7: campus_6 leg 4 - the 12 deg failsafe fired 5 times because realign waited 3 s after Resume)
      if (fabsf(wrap180(_tgt - yaw)) > 10.0f && now - _rlMs > 1000) { _realign(s, _tgt, "off the wall line - turning parallel"); break; }
      _leg1(s, l, r);
      break;
    case T1_TURN:
      wheels = _tk != TK_SHIFT;
      if (_turn(s, _turnTgt, l, r)) { l = r = 0; _go(T1_SETTLE); }
      else if (now - _stMs > kTurnMaxMs) {        // v8: no pause - start the turn tactics again (you can still help by hand)
        if (_log) _log->event(MissionLog::EV_RETRY);
        strlcpy(_msg, "turn still not done after 2 min - trying again (STOP / manual if you want to help)", sizeof(_msg));
        _startTurn(s, _turnTgt);
      }
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
      const bool orienting = _apOn && _apPh >= 10;                     // standing still / pivoting in place: no hold (placed next to a wall)
      if (!orienting && _frontM < 0.45f) { emergency = true; _prev = L2_ALIGN; if (_log) _log->event(MissionLog::EV_HOLD); _go(HOLD); break; }
      if (!orienting && _bp != 2 && _near(holdM, now)) { _prev = L2_ALIGN; if (_log) _log->event(MissionLog::EV_HOLD); _go(HOLD); break; }
      if (_apOn) { _alignStart(s, now, l, r); break; }                // v10.3 start alignment
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
        if (_ds.update(s.imuOk(), yaw, 0.18f, now, l, r) == DriveStraight::FAIL) { if (_log) _log->event(MissionLog::EV_FAILSAFE); l = r = 0; _realign(s, _tgt, "heading failsafe - turning back"); }
      } else if (_bp == 1) {                          // sideways pivot done -> roll
        _bp = 2; _bdRollStart = _dist2; _tgt = yaw; _ds.start(yaw, now); _ds.setTarget(_tgt);
      } else if (_bp == 2) {                          // sideways roll
        if (_dist2 - _bdRollStart >= _bdRollM || now - _stMs > 15000 || _frontM < 1.0f) {
          _bp = 3; _turnNo = 8; _turnTries = 0; _startTurn(s, _bdLockDir);
          break;
        }
        _ds.setTarget(_tgt);
        if (_ds.update(s.imuOk(), yaw, 0.18f, now, l, r) == DriveStraight::FAIL) { if (_log) _log->event(MissionLog::EV_FAILSAFE); l = r = 0; _realign(s, _tgt, "heading failsafe - turning back"); }
      } else {                                        // parallel to the board: lock and go
        if (_log) _log->event(MissionLog::EV_BOARD_LOCK);
        _base = _bdLockDir; _startLeg2(s);
        if (_posMove) { _posMove = false; snprintf(_msg, sizeof(_msg), "lined up after the late turn: open sea heading %.1f deg", _bdLockDir); }
        else snprintf(_msg, sizeof(_msg), "locked on the board: open sea heading %.1f deg", _bdLockDir);
      }
      break;
    }
    case L2_GO: {
      const float gate = 0.85f * leg2ExpM;
      if (_frontM < 0.45f) { emergency = true; if (_log) _log->event(MissionLog::EV_HOLD); _prev = L2_GO; _go(HOLD); break; }
      // end wall: after 70 % of the open sea, LEVEL for 1 s (campus_2: kerb bump +9 deg -> ground at 1.3 m = fake wall)
      // v10: leaky counter (+2 per good 250 ms tick, -1 per bad one, trigger at 6): rain / snow dropouts no longer reset it
      if (now - _endMs >= 250) {
        _endMs = now;
        if (_dist2 >= gate && _frontM <= endWallM && level && _frontWallWide(s, endWallM + 0.3f)) { if (_endCnt < 8) _endCnt += 2; }
        else if (_endCnt > 0) _endCnt--;
      }
      if (_endCnt >= 6) { _endCnt = 0; _endWall(s); break; }
      if (_near(_dist2 >= gate ? holdM * 0.6f : holdM, now)) { _prev = L2_GO; if (_log) _log->event(MissionLog::EV_HOLD); _go(HOLD); break; }
      if (_dist2 > leg2MaxM) { if (_log) _log->event(MissionLog::EV_L2_MAX); _ask(END_ASK, "Maximum open-sea distance driven, no end wall seen."); break; }
      // REFERENCES (v6 corridor). First match wins:
      //   4 LEFT ROW (trees/stakes/posts, 10-26 m, 4.5-8.5 m away): parallel + keep rowOffsetM -> heading AND side
      //   5 LEFT BUILDING (26-40 m): parallel + keep bldOffsetM                -> heading AND side position
      //   1 B wall BEHIND (< 14 m): perpendicular                                -> heading
      //   2 END WALL ahead (> 50 %): perpendicular, max 8 deg from where it was first used -> heading
      //   3 board (< 6 m, +-5 deg): parallel                                     -> heading (weak)
      //   none: hold the heading (NO drift correction: campus5 showed it walks the rover off the strip)
      // _base = path direction (median of 5, max 0.5 deg per scan). Side term _latOff only while a row is seen
      // (max 8 deg, 0.5 deg per scan), fades out after -> a lost row never leaves a sideways bias.
      if (now - _refMs >= 250) {
        _refMs = now;
        float ang, dist, h = 0, lat = 0; uint8_t type = 0; int npt = 0;
        const float remain = leg2ExpM - _dist2;
        auto rowDir = [&](float a) { float d = wrap180(yaw - a); if (fabsf(wrap180(d - _base)) > 90.0f) d = wrap180(d + 180.0f); return d; };
        // v7: row window 10-24.5 m (real locks end ~24.6 m; at 26 m the building front was mistaken for the row in the
        // simulation), distance 4.0-9.5 m and fit range 11 m (a rover 1.3 m off saw the row on only 1 scan in 10)
        if (useRows && _dist2 >= 10.0f && _dist2 <= 24.5f &&
            LineRef::rowFit(s, 270, 70, 1.5f, 11.0f, 0.25f, 5, 4.0f, 1.0f, ang, dist, npt) && dist < 0) {
          const float d = rowDir(ang), off = -dist;
          if (fabsf(wrap180(d - _base)) <= 8.0f && off >= 4.0f && off <= 9.5f) {
            // v7: the row gives the SIDE position only (its direction wobbled +-6 deg in campus_6 and a wrong row
            // direction kept after the row was the worst simulated failure). Cross-track PID on the distance:
            const float e = off - rowOffsetM;                                 // + = too far from the row -> turn left
            const float iMax = 4.0f / fmaxf(seaKi, 0.01f);
            _seaEi = clampf(_seaEi + e * 0.25f, -iMax, iMax);
            const float eDot = -0.22f * sinf(wrap180(yaw - d) * DEG_TO_RAD);  // pointing left of the row -> closing in
            type = 4; _refOff = off;
            lat = clampf(seaKp * e + seaKi * _seaEi + seaKd * eDot, -8.0f, 8.0f);
          }
        }
        if (!type && useRows && _dist2 >= 26.0f && _dist2 <= 40.0f &&
            LineRef::rowFit(s, 270, 70, 1.5f, 7.0f, 0.25f, 5, 4.0f, 1.0f, ang, dist, npt) && dist < 0) {
          const float d = rowDir(ang), off = -dist;
          if (fabsf(wrap180(d - _base)) <= 8.0f && off >= 2.0f && off <= 6.0f) {   // v7: up to 6 m (a rover 2 m off)
            type = 5; _refOff = off;
            lat = clampf(10.0f * (off - bldOffsetM), -6.0f, 6.0f);
          }
        }
        // v7: not in the first 2 m and not closer than 2.5 m (campus_6: at 0-2 m it fitted the arcade corner -> -95 deg)
        if (!type && _dist2 >= 2.0f && _dist2 < 14.0f && LineRef::sectorFit(s, 180, 50, 1.0f, 12.0f, 1.5f, ang, dist) &&
            dist >= 2.5f && dist <= _dist2 + 6.0f) {
          const float hh = LineRef::perpHeading(yaw, ang, _base);
          if (fabsf(wrap180(hh - _base)) <= 12.0f) { h = hh; type = 1; }
        }
        if (!type && _dist2 > 0.5f * leg2ExpM && LineRef::sectorFit(s, 0, 40, 2.5f, 12.0f, 1.5f, ang, dist) &&
            dist >= remain - 6.0f && dist <= remain + 8.0f) {
          const float hh = LineRef::perpHeading(yaw, ang, _base);
          if (fabsf(wrap180(hh - _base)) <= 10.0f) { h = hh; type = 2; }
        }
        if (!type && _dist2 < 6.0f) {
          float dir, bd;
          if (_boardFit(s, 0.35f, 1.8f, dir, bd) && fabsf(wrap180(dir - _base)) <= 5.0f) { h = dir; type = 3; }
        }
        if (type == 2) _seaEi = 0;              // v7: the end wall fixes the direction - the kept integral would do it twice
        if (type) {
          if (type != _refKind) {               // a different reference (not just after a scan without one)
            _refN = _refI = 0; _refKind = type;
            if (type == 1 && !_rearSeen) { _rearSeen = true; if (_log) _log->event(MissionLog::EV_REF_REAR); }
            if (type == 2 && !_frontSeen) { _frontSeen = true; _frontBase0 = _base; if (_log) _log->event(MissionLog::EV_REF_FRONT); }
            if (type == 3 && _log) _log->event(MissionLog::EV_REF_BOARD);
            if (type == 4 && !_rowSeen) { _rowSeen = true; if (_log) _log->event(MissionLog::EV_REF_ROW); }
            if (type == 5 && !_bldSeen) { _bldSeen = true; if (_log) _log->event(MissionLog::EV_REF_BLD); }
          }
          if (type == 4 || type == 5) _frontSeen = false;     // the end-wall cap re-anchors after a row correction
          _refType = type;
          if (type != 4 && type != 5) {                         // direction references move the path direction
            _refBuf[_refI] = wrap180(h - _base); _refI = (_refI + 1) % 5; if (_refN < 5) _refN++;
            float t5[5]; for (int i = 0; i < _refN; i++) t5[i] = _refBuf[i];
            for (int i = 1; i < _refN; i++) { float v = t5[i]; int j = i - 1; while (j >= 0 && t5[j] > v) { t5[j + 1] = t5[j]; j--; } t5[j + 1] = v; }
            const float lim = type == 3 ? 0.25f : 0.5f;
            const float step = clampf(t5[_refN / 2], -lim, lim);
            _base = wrap180(_base + step);
            if (type == 2) _base = wrap180(_frontBase0 + clampf(wrap180(_base - _frontBase0), -8.0f, 8.0f));
            for (int i = 0; i < _refN; i++) _refBuf[i] -= step;
          }
        } else _refType = 0;
        // side term: PID while a row is seen; afterwards only its integral part stays (a constant sideways pull)
        _latOff += clampf(((type == 4 || type == 5) ? lat : seaKi * _seaEi) - _latOff, -0.5f, 0.5f);
        _lastMvT = _mvT;
        // sideways shift right at the start: offset ramps in / out at 1 deg per scan (the 12 deg failsafe never sees a jump)
        _shiftOff += clampf((_dist2 < _shiftEndM ? 10.0f : 0.0f) - _shiftOff, -1.0f, 1.0f);
        _tgt = wrap180(_base - _shiftOff + _latOff + seaTrimDeg);
      }
      // END BOARD (campus_3: signboard on the right ~0.65 m before the end wall): after 70 %, 2 consistent right-side
      // line fits -> stay parallel to it at the distance first seen (no forced distance); when it ends, keep its direction
      if (useEndBoard && _dist2 >= gate && now - _aimMs >= 250) {
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
      if (_ds.update(s.imuOk(), yaw, _cruise(), now, l, r) == DriveStraight::FAIL) { if (_log) _log->event(MissionLog::EV_FAILSAFE); l = r = 0; _realign(s, _tgt, "heading failsafe - turning back"); }
      break;
    }
    case L3_FOLLOW:
    case L4_FOLLOW: {
      const bool l4 = _st == L4_FOLLOW;
      if (_frontM <= endWallM && level && _frontWall(s, endWallM + 0.3f)) {   // a wall right ahead: the corridor ends here
        if (!_endOn) { _endOn = true; _endMs = now; }
        if (now - _endMs >= 800) {
          if (l4 && _rampSeen && _log) _log->event(MissionLog::EV_AT_I);
          _finishRoute(l4 ? (_rampSeen ? "ARRIVED: top of the ramp, door / wall ahead - stopped (Block I)" : "leg 4: wall ahead - stopped")
                          : "leg 3: wall ahead - stopped");
          break;
        }
      } else _endOn = false;
      if (_guard(s, now, emergency)) break;
      if (l4 && _l4Ph == 2 && ((_entFrom >= 0 && _dist - _entFrom >= 1.5f) || (_corrFrom >= 0 && _dist - _corrFrom > 25.0f))) {
        if (_log) _log->event(MissionLog::EV_AT_I);
        _finishRoute(_entFrom >= 0 ? "ARRIVED: Block I (KARL entrance) - stopped" : "ARRIVED: 25 m along the railing corridor - stopped");
        break;
      }
      if (l4 && _edgeAssist(s, now, l, r, wheels)) break;      // v9: stuck at an edge -> push / back up / push
      if (_dist > leg3MaxM) { _finishRoute("maximum leg distance driven"); break; }
      if (fabsf(wrap180(_tgt - yaw)) > 18.0f && now - _rlMs > 1000) { _realign(s, _tgt, "off the wall line - turning parallel"); break; }
      _leg1(s, l, r);                               // left wall follower (leg 4 after the wall: _l4Walk)
      if (_st != L3_FOLLOW && _st != L4_FOLLOW) break;   // failsafe pause inside
      if (l4 && _l4Ph == 1 && _gWaitOn) {          // v10.3: step line close, not lined up with the ramp -> never enter
        l = r = 0;                                  // GentleDrive ramps the stop
        if (!_gWaitMs) { _gWaitMs = now; if (_log) _log->event(MissionLog::EV_GAP_WAIT);
          strlcpy(_msg, "step line ahead, not lined up with the ramp opening - waiting", sizeof(_msg)); }
        if (now - _gWaitMs > 20000) { _gWaitMs = 0; _ask(END_ASK, "Ramp opening not found or blocked. Line the rover up with the ramp (MANUAL), then GO."); }
        break;
      } else _gWaitMs = 0;
      if (l4 && _l4Ph >= 1) {                       // v9: walkway / railing / ramp - no "wall ended" stop
        if (_l4Ph == 3 && _dist - _l4PhStart > 6.0f) {
          l = r = 0; if (_log) _log->event(MissionLog::EV_AT_I);
          _finishRoute("ARRIVED: top of the ramp (6 m on the level after it) - stopped");
        }
        break;
      }
      if (_wallOk) { _l3Seen = true; _noWallFrom = -1; }
      else {
        if (_noWallFrom < 0) _noWallFrom = _dist;
        const bool ended = _l3Seen && _dist - _noWallFrom >= wallEndM, none = !_l3Seen && _dist >= 4.0f;
        if (l4 && (ended || none)) {                // v9: the wall after T3 ended -> walkway along the interlock
          _l4Ph = 1; _l4PhStart = _dist; _l4Dir = _l3Seen ? _wallDir : _expDir; _l4KeepSet = false; _railN = 0;
          _l4Flat = s.pitchDeg(); _l4ScanMs = 0;
          if (_log) _log->event(MissionLog::EV_L4_OPEN);
          snprintf(_msg, sizeof(_msg), "leg 4: wall ended - walkway on %.0f deg, looking for the railings", _l4Dir);
          Serial.printf("[mission] %s\n", _msg);
          break;
        }
        if (ended) {
          l = r = 0;
          if (!l4 && leg4) {                        // T3: building corner -> turn LEFT around it
            if (_log) _log->event(MissionLog::EV_T3_TURN);
            strlcpy(_msg, "arcade wall ended - T3: turning LEFT", sizeof(_msg));
            _turnNo = 3; _turnTries = 0; _startTurn(s, wrap180(_wallDir + 90.0f));
          } else _finishRoute("LEFT WALL ENDED - route done");
          break;
        }
        if (none) { l = r = 0; _finishRoute("no left wall found in 4 m - stopped"); break; }
      }
      break;
    }
    case HOLD: {
      // v8 (test1): the end wall needs the FLAT-WALL check here too - a 0.22 m ghost during a hold was taken as the end wall
      if (_leg == 2 && _dist2 >= 0.85f * leg2ExpM && _frontM <= endWallM && level && _frontWallWide(s, endWallM + 0.3f)) { _endWall(s); break; }
      if (_leg >= 3 && _frontM <= endWallM && level && _frontWall(s, endWallM + 0.3f)) { _finishRoute("wall ahead - stopped"); break; }
      // v10: at Block I (after the railing corridor) anything in the way ends the route - no retry into it
      if (_leg == 4 && _l4Ph >= 2) {
        if (_log) _log->event(MissionLog::EV_AT_I);
        _finishRoute("ARRIVED at Block I area: something in front - stopped");
        break;
      }
      // v10: vanished while closer than 0.45 m (not seen walking away) = still there, too close for the LiDAR
      const bool vanished = _hLastNear < 0.45f && !_hReceded;
      if (now - _stMs > 20000) {                    // v8: no pause - line up on the target heading and try again (3x)
        if (_blockN < 3 && !vanished) {
          _blockN++;
          if (_log) _log->event(MissionLog::EV_RETRY);
          _go(_prev); _realign(s, _tgt, "blocked 20 s - lining up and trying again");
        } else { _stMs = now; strlcpy(_msg, vanished ? "something RIGHT in front (too close to see) - waiting (STOP / manual)"
                                                   : "still blocked - waiting (STOP / manual to steer around)", sizeof(_msg)); }
        break;
      }
      const bool clear = _frontM > 1.0f && (!vanished || _frontFar >= 4 || _usMed() >= 100);
      if (clear) {
        if (!_clearOn) { _clearOn = true; _clearMs = now; }
        if (now - _clearMs >= 2000) {
          _blockN = 0;
          if (_log) _log->event(MissionLog::EV_HOLD_END);
          _ds.start(yaw, now); _ds.setTarget(_tgt); _go(_prev);
          // turned while waiting (someone moved it): pivot back first instead of tripping the 12 deg failsafe
          if ((_prev == L1_FOLLOW || _prev == L3_FOLLOW || _prev == L4_FOLLOW || _prev == L2_GO) && fabsf(wrap180(_tgt - yaw)) > 10.0f)
            _realign(s, _tgt, "after the wait - turning back");
        }
      } else _clearOn = false;
      break;
    }
    case MANUAL: {
      l = padL; r = padR;
      const bool pivot = (l > 0 && r < 0) || (l < 0 && r > 0);
      if (pivot) { const float d = r > 0 ? 1.0f : -1.0f; l = -0.5f * d; r = 0.5f * d; wheels = true; }
      // v8 CLIMB button (cobble -> interlock edge): while held, both wheels up to 0.35 (ramped in 0.3 s) for max 1.5 s,
      // then 0 until released; only with nothing within 0.35 m in front (confirmed obstacle, see _front)
      if (climb) {
        if (!_climbOn && _frontM >= 0.35f) { _climbOn = true; _climbStartMs = now; if (_log) _log->event(MissionLog::EV_CLIMB); }
        if (_climbOn) {
          const uint32_t t = now - _climbStartMs;
          const float c = t < 1500 ? 0.35f * fminf(1.0f, t / 300.0f) : 0.0f;
          l = r = (_frontM < 0.35f) ? 0.0f : c; wheels = true;
        }
      } else _climbOn = false;
      if (0.5f * (l + r) > 0 && _frontM < 0.35f) { l = r = 0; emergency = true; }
      break;
    }
    default: break;                                   // IDLE, ASK states, PAUSED, DONE: standing
  }

status:
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
  if (_leg == 4) { if (_l4Ph >= 1) _l4Walk(s, now); else _leg4Wall(s, now); }   // v7 wall / v9 walkway-railing-ramp
  else if (now - _wallScanMs >= 250) {              // one wall fit per LiDAR snapshot
    _wallScanMs = now;
    const WallFit w = WallFit::measure(s, true, _wallOk ? &_wall : nullptr);
    bool good = w.ok && fabsf(w.angleDeg) < 35.0f && w.rmsM < (_wallOk ? 0.08f : 0.06f);
    const float wy = wrap180(s.yawDeg() + w.angleDeg);       // wall direction in the gyro frame
    if (good && !_wallOk) {                                   // new wall: 3 consistent fits first
      if (_acqN && fabsf(wrap180(wy - _acqDir)) < 4.0f) _acqN++; else _acqN = 1;
      _acqDir = wy;
      if (_acqN < 3) good = false;
      else {
        _wallDir = wy; _wyN = _wyI = 0; _jumpN = 0; _drift1.newSegment();
        if (_dSetPending) { _dSet = clampf(w.distM, 0.7f, 2.2f); _dSetPending = false; }
      }
    } else if (good) {
      if (_wyN >= 3 && fabsf(wrap180(wy - _wallDir)) > 8.0f) {
        if (_jumpN && fabsf(wrap180(wy - _jumpDir)) < 4.0f) _jumpN++; else _jumpN = 1;
        _jumpDir = wy;
        if (_jumpN >= 3) { _wallDir = wy; _wyN = _wyI = 0; _jumpN = 0; _drift1.newSegment(); } else good = false;
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
      // v9: left wall back after the B gap (>= 3 m without it) = anchor of the turn-structure window
      if (!_wallOk && _leg == 1 && _lostAt >= 0 && _dist - _lostAt >= 3.0f && _dist >= 5.0f && _sLock < 0) {   // v10: any start
        _gapAnchor = _dist;
        float lo, hi; _structWin(lo, hi); uint8_t k = 0;               // candidates outside the new window are dropped
        for (uint8_t c = 0; c < _candN; c++) if (_cand[c].a >= lo && _cand[c].a <= hi) _cand[k++] = _cand[c];
        _candN = k;
        if (_log) _log->event(MissionLog::EV_GAP_ANCHOR);
        snprintf(_msg, sizeof(_msg), "B gap ended at %.1f m: turn structure expected at %.1f m", _dist, _dist + 5.35f);
      }
      _wallOk = true; _wallGoodMs = now; _wall = w;
      if (_leg == 1 && w.rmsM < 0.04f) _drift1.add(_mvT, wy);   // straight B wall: its drift in the gyro frame
      // stay PARALLEL (wall direction) + keep the distance: too far -> turn left, too close -> turn right
      if (_leg == 3) {
        // v8: leg 3 closes in to leg3WallM while driving (law of the v7 last leg: 20 deg/m, max 15 deg, target never
        // more than 10 deg ahead of the heading, max 2 deg per scan -> no failsafe, no overshoot in the simulation)
        const float yawNow = s.yawNowDeg();
        float want = wrap180(_wallDir + clampf(20.0f * (w.distM - _dSet), -15.0f, 15.0f));
        want = wrap180(yawNow + clampf(wrap180(want - yawNow), -10.0f, 10.0f));
        _tgt = wrap180(_tgt + clampf(wrap180(want - _tgt), -2.0f, 2.0f));
      } else {
        const float approach = clampf(10.0f * (w.distM - _dSet), -8.0f, 8.0f);
        _tgt = wrap180(_wallDir + approach);
      }
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
      _wallOk = false; _cN = 0; _acqN = 0; _jumpN = 0; _wyN = _wyI = 0; _drift1.newSegment();
      if (_leg == 1) _lostAt = _dist;                       // v9: start of a gap (same as the logged wall_lost)
      if (_log) _log->event(MissionLog::EV_WALL_LOST);     // gap / door: keep the last target heading (gyro)
    }
    _pillars(s);
  }
  _ds.setTarget(_tgt);
  if (_ds.update(s.imuOk(), s.yawNowDeg(), _cruise(), now, l, r) == DriveStraight::FAIL) {
    if (_log) _log->event(MissionLog::EV_FAILSAFE);
    l = r = 0;
    if (s.imuOk()) _realign(s, _tgt, "heading failsafe - turning back");   // v8: no pause
    else _pause(_ds.reason());                                            // IMU gone: cannot steer -> stop
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
  d.camRows = _tk; d.camConf = _tCycles; d.camOff = _cruise() * 100.0f;   // v9: speed_boost field = set forward speed d.camAng = _motionM < 0 ? -1.0f : _motionM * 100.0f;
  d.snapsSent = _shifts; d.snapsOk = _refType;
  if (_leg >= 2) d.crabDeg = _drift1.rate() * 60.0f;   // leg 2+: drift seen on leg 1 (logged only, not used)
  if (_st == L2_GO) { d.wallD = _refOff; d.wallAngDeg = _latOff; }   // v6: corridor offset (m) + side term (deg)
  if (_leg == 1) d.crabDeg = _sLock;                    // v9: locked turn-structure position (m along leg 1, -1 = none)
  if (_leg == 4) d.pillars = _l4Ph * 10 + (_eaPh > 9 ? 9 : _eaPh);
  d.trimLearned = _ds.learned(); d.corr = _ds.correction(); d.yawRate = _ds.yawRateDegS(); d.crabEst = _crabEst; d.boost = _boost; d.driveS = _ds.driveSeconds();   // v10.3 new-rover data   // v9: leg-4 phase x10 + edge-assist step
}

String RouteMission::_statusJson(const Sensors& s, const RoverLink& rover) {
  String j; j.reserve(1300);
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
  j += ",\"nearInten\":"; j += s.lidarNearInten(); j += ",\"link\":"; j += _linkWait ? 0 : 1;
  j += ",\"ref\":"; j += _refType; j += ",\"refOff\":"; j += String(_refOff, 2); j += ",\"lat\":"; j += String(_latOff, 1); j += ",\"drift\":"; j += String(_drift1.rate() * 60.0f, 1);
  j += ",\"driftOk\":"; j += _drift1.measured() ? 1 : 0;
  j += ",\"bp\":"; j += _bp; j += ",\"board\":"; j += String(_bdLockDir, 1); j += ",\"eb\":"; j += _ebOn ? 1 : 0;
  j += ",\"aim\":"; j += _aimOn ? 1 : 0; j += ",\"aimDeg\":"; j += String(_aimDeg, 1);
  j += ",\"tac\":\""; j += kTac[_tk < 5 ? _tk : 0];
  j += "\",\"tCycles\":"; j += _tCycles;
  j += ",\"shifts\":"; j += _shifts;
  j += ",\"boost\":"; j += String(_boost, 2);
  j += ",\"adj\":"; j += String(speedAdj, 2); j += ",\"outV\":"; j += String(_outV, 2);
  j += ",\"sLock\":"; j += String(_sLock, 1); j += ",\"anchor\":"; j += String(_gapAnchor, 1); j += ",\"cand\":"; j += _candN;
  j += ",\"l4ph\":"; j += _l4Ph; j += ",\"ea\":"; j += _eaPh; j += ",\"ramp\":"; j += _rampSeen ? 1 : 0;
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
  j += ",\"ppast\":"; j += String(pillarPastM, 2); j += ",\"rowoff\":"; j += String(rowOffsetM, 1);
  j += ",\"l4wall\":"; j += String(leg4WallM, 1); j += ",\"l3wall\":"; j += String(leg3WallM, 2);
  j += ",\"ninten\":"; j += (int)nearMinInten; j += ",\"kp\":"; j += String(seaKp, 1); j += ",\"ki\":"; j += String(seaKi, 2);
  j += ",\"kd\":"; j += String(seaKd, 1);
  j += ",\"spast\":"; j += String(structPastM, 2); j += ",\"l4speed\":"; j += String(l4Speed, 2); j += ",\"climbp\":"; j += String(climbPow, 2);
  j += ",\"bldoff\":"; j += String(bldOffsetM, 1); j += ",\"trim\":"; j += String(seaTrimDeg, 1); j += ",\"rows\":"; j += useRows ? 1 : 0; j += ",\"bside\":"; j += String(boardSideM, 2); j += ",\"boff\":"; j += String(boardOffsetDeg, 1); j += ",\"wend\":"; j += String(wallEndM, 1);
  j += "}}";
  return j;
}
