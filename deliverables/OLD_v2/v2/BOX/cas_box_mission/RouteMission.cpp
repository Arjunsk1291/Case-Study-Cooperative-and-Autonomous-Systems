#include "RouteMission.h"

static float wrap180(float a) { a = fmodf(a + 180.0f, 360.0f); if (a < 0) a += 360.0f; return a - 180.0f; }
static float clampf(float x, float lo, float hi) { return x < lo ? lo : (x > hi ? hi : x); }
static const char* kSt[] = {"ready", "LEG 1: following the left wall", "TURN? waiting for you", "turning right 90 deg",
                            "checking the turn", "looking for the strip (camera)", "OPEN SEA: waiting for you",
                            "aligning on the strip", "OPEN SEA: driving straight", "WAITING: something in front",
                            "OPEN SEA CROSSED", "MANUAL driving (logging)", "PAUSED", "finished"};
enum Cmd : uint8_t { C_NONE = 0, C_START, C_TURN, C_ON1M, C_MANUAL, C_RESUME, C_ALIGN, C_GO, C_L2HERE, C_FINISH, C_STOP };

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
    const char* names[] = {"", "start", "turn", "on1m", "manual", "resume", "align", "go", "l2here", "finish", "stop"};
    for (uint8_t i = 1; i <= C_STOP; i++) if (c == names[i]) _cmd = i;
    _aliveMs = millis();
    q->send(200, "application/json", "{\"ok\":true}");
  });
  server->on("/mis/set", HTTP_GET, [this](AsyncWebServerRequest* q) {
    auto f = [&](const char* k, float& v, float lo, float hi) { if (q->hasParam(k)) v = clampf(q->getParam(k)->value().toFloat(), lo, hi); };
    f("speed", speed, 0.12f, 0.30f); f("t1", t1AskM, 2.0f, 60.0f); f("l2", leg2ExpM, 5.0f, 80.0f);
    f("end", endWallM, 0.6f, 2.0f); f("px", pxPerM, 50.0f, 2000.0f); f("crab", crabPriorDeg, -12.0f, 12.0f);
    if (q->hasParam("cam")) camAssist = q->getParam("cam")->value() == "1";
    if (q->hasParam("auto")) autoL2 = q->getParam("auto")->value() == "1";
    if (q->hasParam("mode")) stripMode = q->getParam("mode")->value() == "1" ? 1 : 0;
    leg2MaxM = fmaxf(leg2MaxM, leg2ExpM * 1.4f);
    q->send(200, "application/json", "{\"ok\":true}");
  });
  return true;
}

void RouteMission::_go(uint8_t st) {
  _st = st; _stMs = millis(); _clearOn = _endOn = false;
  if (st == T1_TURN || st == L2_ALIGN) { _tPh = 0; _tStalls = 0; _tMs = _stMs; _tYaw = _prevYaw; }   // fresh turn
}

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

// TURN like the human driver (campus run 07.11.: a full pivot HELD for 10 s turned 7 deg and jammed; the teaching
// drives turned in 1-2 s full-power bursts). 0 BURST full pivot; no progress (< 4 deg) after 1.5 s -> 1 REST 0.6 s
// -> burst again; after 2 stalls -> 2 SWING: outer side drives, inner side stands (forward if there is room ahead,
// else backward). Released early by the coast (|rate| * 0.45 s). The caller times out.
void RouteMission::_turnReset(const Sensors& s) { _tPh = 0; _tStalls = 0; _tMs = millis(); _tYaw = s.yawNowDeg(); }

bool RouteMission::_pivot(const Sensors& s, float tgt, float& l, float& r) {
  const uint32_t now = millis();
  const float yaw = s.yawNowDeg(), e = wrap180(tgt - yaw);
  if (fabsf(e) <= fmaxf(3.0f, fabsf(_rate) * 0.45f)) { l = r = 0; return true; }
  const float d = e > 0 ? 1.0f : -1.0f;            // + = turn left
  const bool progress = fabsf(wrap180(yaw - _tYaw)) >= 4.0f;
  switch (_tPh) {
    case 0:                                        // burst
      l = -0.5f * d; r = 0.5f * d;
      if (now - _tMs >= 1500) {
        if (progress) { _tMs = now; _tYaw = yaw; }                  // turning: keep going
        else { _tStalls++; _tPh = 1; _tMs = now; if (_log) _log->event(MissionLog::EV_TURN_STALL); }
      }
      break;
    case 1:                                        // rest: let the wheels / gears relax
      l = r = 0;
      if (now - _tMs >= 600) { _tPh = _tStalls >= 2 ? 2 : 0; _tMs = now; _tYaw = yaw; }
      break;
    default: {                                     // swing turn
      const bool fwd = _frontM > 1.0f;
      if (d < 0) { l = fwd ? 0.5f : 0.0f; r = fwd ? 0.0f : -0.5f; }   // right turn
      else       { l = fwd ? 0.0f : -0.5f; r = fwd ? 0.5f : 0.0f; }    // left turn
      if (now - _tMs >= 2000) { if (progress) { _tMs = now; _tYaw = yaw; } else { _tPh = 1; _tMs = now; _tStalls = 0; } }
      break;
    }
  }
  return false;
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
  if (now - _wallMs >= 250) {                       // one wall fit per LiDAR snapshot
    _wallMs = now;
    const WallFit w = WallFit::measure(s, true, _wallOk ? &_wall : nullptr);
    bool good = w.ok && fabsf(w.angleDeg) < 30.0f && w.rmsM < 0.08f;
    if (good) {
      const float wy = wrap180(s.yawDeg() + w.angleDeg);       // wall direction in the gyro frame
      // median of the last 5 (relative to the current estimate: no wrap problems), drop jumps > 8 deg
      if (_wyN >= 3 && fabsf(wrap180(wy - _wallDir)) > 8.0f) good = false;
      else {
        _wyBuf[_wyI] = wrap180(wy - _wallDir); _wyI = (_wyI + 1) % 5; if (_wyN < 5) _wyN++;
        float t[5]; for (int i = 0; i < _wyN; i++) t[i] = _wyBuf[i];
        for (int i = 1; i < _wyN; i++) { float v = t[i]; int j = i - 1; while (j >= 0 && t[j] > v) { t[j + 1] = t[j]; j--; } t[j + 1] = v; }
        const float med = t[_wyN / 2];
        _wallDir = wrap180(_wallDir + med);
        for (int i = 0; i < _wyN; i++) _wyBuf[i] -= med;                     // keep relative to the new estimate
      }
    }
    if (good) {
      if (!_wallOk && _log) _log->event(MissionLog::EV_WALL_FOUND);
      _wallOk = true; _wall = w;
      const float approach = clampf(10.0f * (w.distM - _dSet), -8.0f, 8.0f);   // too far from the left wall -> turn left
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
      _wallMs = now;
    } else if (_wallOk && now - _wallMs > 1000) {
      _wallOk = false; _cN = 0;
      if (_log) _log->event(MissionLog::EV_WALL_LOST);     // gap / door: keep the last target heading (gyro)
    }
    _pillars(s);
  }
  _ds.setTarget(_tgt);
  if (_ds.update(s.imuOk(), s.yawNowDeg(), speed, now, l, r) == DriveStraight::FAIL) {
    if (_log) _log->event(MissionLog::EV_FAILSAFE);
    _pause(_ds.reason());
  }
}

void RouteMission::_snapStep(CamLink& cam) {
  const uint32_t now = millis();
  const uint8_t st = cam.snapState();
  if (st == CamLink::SNAP_WAIT) return;
  if (st == CamLink::SNAP_OK || st == CamLink::SNAP_FAIL) {
    const bool ok = st == CamLink::SNAP_OK && cam.snapResult().stripOk;
    if (ok && _snapOk < 3) { _sOff[_snapOk] = cam.snapResult().offX10 / 10.0f; _sAng[_snapOk] = cam.snapResult().angX10 / 10.0f; _snapOk++; }
    _snapGot++;
    if (_log) _log->event(ok ? MissionLog::EV_SNAP_OK : MissionLog::EV_SNAP_FAIL);
    cam.snapClear(); _snapGapMs = now;
    return;
  }
  if (_snapGot < _snapWant && now - _snapGapMs >= 200) cam.requestSnap(stripMode);   // not paired -> FAIL at once
}

static float median3(const float* v, uint8_t n) {
  if (n == 1) return v[0];
  if (n == 2) return 0.5f * (v[0] + v[1]);
  float a = v[0], b = v[1], c = v[2];
  return fmaxf(fminf(a, b), fminf(fmaxf(a, b), c));
}

void RouteMission::_startLeg2(const Sensors& s) {
  const float crabUse = _crabN >= 6 ? _crabEst : crabPriorDeg;
  _base = wrap180(s.yawNowDeg() - crabGain * crabUse);     // slides left (+) -> point a bit right
  _offM = (camAssist && _lookN >= 2) ? (_lookOff / pxPerM) : 0.0f;
  _mergeLeft = fabsf(_offM) > 0.03f ? 1.4f : 0.0f;
  _tgt = wrap180(_base - (_mergeLeft > 0 ? atanf(_offM / 1.4f) * RAD_TO_DEG : 0.0f));
  _leg = 2; _dist2 = 0; _nextCheck = 2.6f; _snapWant = _snapGot = _snapOk = 0;
  _ds.start(s.yawNowDeg(), millis()); _ds.setTarget(_tgt);
  if (_log) _log->event(MissionLog::EV_GO_L2);
  snprintf(_msg, sizeof(_msg), "open sea: crab %.1f deg (%s), offset merge %.0f cm", crabUse, _crabN >= 6 ? "measured" : "prior", _offM * 100);
  Serial.printf("[mission] leg 2: %s\n", _msg);
  _go(L2_GO);
}

void RouteMission::noteOutput(float l, float r, float dt) {
  _outL = l; _outR = r; _outV = 0.5f * (l + r);
  if (_outV > 0 && dt > 0 && dt < 0.2f) { const float d = _outV * 1.03f * dt; if (_leg == 2) _dist2 += d; else _dist += d; }
}

void RouteMission::update(const Sensors& s, RoverLink& rover, CamLink& cam, float padL, float padR, bool padActive,
                          float& l, float& r, bool& emergency) {
  const uint32_t now = millis();
  const float dt = _lastMs ? (now - _lastMs) / 1000.0f : 0; _lastMs = now;
  const float yaw = s.yawNowDeg();
  if (s.imuOk() && dt > 0 && dt < 0.2f) { const float d = wrap180(yaw - _prevYaw); if (fabsf(d) < 30) _rate += (d / dt - _rate) * (dt / (0.15f + dt)); }
  _prevYaw = yaw;
  _frontM = _front(s);

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
      _crabEst = 0; _crabN = 0; _cN = _cI = 0; _nextCrabM = 0; _askAt = t1AskM; _pitch0 = s.pitchDeg(); _lookN = 0;
      _ds.start(yaw, now); _ds.setTarget(_tgt);
      if (_log) { _log->start(); _log->event(MissionLog::EV_START); }
      snprintf(_msg, sizeof(_msg), "started: left wall at %.2f m", _dSet);
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
    const uint8_t back = (_prev == HOLD || _prev == PAUSED || _prev == MANUAL) ? (_leg == 2 ? L2_GO : L1_FOLLOW) : _prev;
    // 07.11.: after MANUAL the operator has corrected the heading -> that is the new straight line (before: the old
    // target was restored and the heading failsafe fired 5 times in a row)
    if (_st == MANUAL && back == L2_GO) { _base = yaw; _tgt = yaw; _mergeLeft = 0; }
    if (_st == MANUAL && back == L1_FOLLOW) { _tgt = yaw; }
    if (back == L1_FOLLOW || back == L2_GO) { _ds.start(yaw, now); _ds.setTarget(_tgt); }
    if (back == T1_TURN || back == L2_ALIGN) _turnTries = 0;
    _msg[0] = 0;
    _go(back);
  } else if (c == C_TURN && _st == T1_ASK) {
    if (_log) _log->event(MissionLog::EV_OP_TURN);
    _turnTgt = wrap180((_wallOk ? _wallDir : yaw) - 90.0f); _turnTries = 0;
    _go(T1_TURN);
  } else if (c == C_ON1M && _st == T1_ASK) {
    if (_log) _log->event(MissionLog::EV_OP_ON1M);
    _askAt = _dist + 1.0f; _ds.start(yaw, now); _ds.setTarget(_tgt);
    _go(L1_FOLLOW);
  } else if ((c == C_ALIGN || c == C_GO) && _st == L2_ASK) {
    if (c == C_ALIGN && _lookN >= 2 && fabsf(_lookAng) >= 3 && fabsf(_lookAng) <= 25) {
      if (_log) _log->event(MissionLog::EV_ALIGN);
      _turnTgt = wrap180(yaw - _lookAng); _turnTries = 0; _go(L2_ALIGN);
    } else _startLeg2(s);
  } else if (c == C_L2HERE && (_st == MANUAL || _st == PAUSED || _st == T1_ASK || _st == IDLE)) {
    if (_st == IDLE && _log) _log->start();
    _leg = 2; if (_dist < 2.0f) _pitch0 = s.pitchDeg();   // keep the leg-1 flat reference if leg 1 was driven
    _snapWant = 3; _snapGot = _snapOk = 0; _snapPurpose = 1; _snapGapMs = 0;
    _go(L2_LOOK);
  } else if (c == C_FINISH && _st != IDLE) {
    if (_log) { _log->event(MissionLog::EV_DONE); _log->stop(); }
    strlcpy(_msg, "finished - log saved", sizeof(_msg));
    _go(DONE);
  }

  // ---- safety in every moving state
  const bool moving = _st == L1_FOLLOW || _st == T1_TURN || _st == L2_ALIGN || _st == L2_GO;
  if (moving) {
    if (padActive) { _prev = _st; if (_log) _log->event(MissionLog::EV_MANUAL_ON); strlcpy(_msg, "you took over - manual (logging)", sizeof(_msg)); _go(MANUAL); }
    else if (now - _aliveMs > 1500) _pause("phone page lost (screen off / other app?)");
    else if (!rover.connected()) _pause("rover lost");
    else if (s.imuOk() && (fabsf(s.rollDeg()) > 15 || fabsf(s.pitchDeg() - _pitch0) > 18)) { emergency = true; _pause("tilt"); }
  }

  // pad touched while standing (ready / asking / paused / done): manual driving, logged
  if (padActive && (_st == IDLE || _st == DONE || _st == PAUSED || _st == T1_ASK || _st == L2_ASK || _st == END_ASK)) {
    if (_st == IDLE || _st == DONE) { _prev = IDLE; if (_log) _log->start(); } else if (_st != PAUSED) _prev = _st;
    if (_log) _log->event(MissionLog::EV_MANUAL_ON);
    strlcpy(_msg, "manual driving - everything is logged", sizeof(_msg));
    _go(MANUAL);
  }

  l = r = 0;
  switch (_st) {
    case L1_FOLLOW:
      if (_frontM < 0.45f) { emergency = true; _prev = L1_FOLLOW; if (_log) _log->event(MissionLog::EV_HOLD); _go(HOLD); break; }
      if (_near(holdM, now)) { _prev = L1_FOLLOW; if (_log) _log->event(MissionLog::EV_HOLD); _go(HOLD); break; }
      if (_dist >= _askAt) {
        if (_log) _log->event(MissionLog::EV_T1_ASK);
        char q[96]; snprintf(q, sizeof(q), "%.1f m driven, %u pillars counted. Is this the RIGHT TURN to the open area?", _dist, _pillarsN);
        _ask(T1_ASK, q); break;
      }
      _leg1(s, l, r);
      break;
    case T1_TURN:
      if (_pivot(s, _turnTgt, l, r)) _go(T1_SETTLE);
      else if (now - _stMs > 25000) _pause("turn failed (25 s) - turn it by hand, then Resume");
      break;
    case T1_SETTLE:
      if (now - _stMs >= 1000) {
        if (fabsf(wrap180(_turnTgt - yaw)) > 6.0f && _turnTries < 2) { _turnTries++; _go(T1_TURN); }
        else {
          if (_log) _log->event(MissionLog::EV_TURN_DONE);
          _leg = 2; _snapWant = 3; _snapGot = _snapOk = 0; _snapPurpose = 1; _snapGapMs = 0;   // pitch reference from leg 1 kept
          _go(L2_LOOK);
        }
      }
      break;
    case L2_LOOK:
      _snapStep(cam);
      if (_snapGot >= _snapWant) {
        _lookN = _snapOk;
        if (_snapOk) { _lookOff = median3(_sOff, _snapOk); _lookAng = median3(_sAng, _snapOk); }
        if (_log) _log->event(MissionLog::EV_LOOK);
        char q[96];
        if (_lookN) snprintf(q, sizeof(q), "Camera: strip in %u/3 pictures, angle %+.1f deg, offset %+.0f px. Align on it?", _lookN, _lookAng, _lookOff);
        else snprintf(q, sizeof(q), "Camera: no strip in 3 pictures (%s). Go straight on the gyro?", cam.paired() ? "paired" : "camera NOT paired");
        if (autoL2) {
          if (camAssist && _lookN >= 2 && fabsf(_lookAng) >= 3 && fabsf(_lookAng) <= 25) { if (_log) _log->event(MissionLog::EV_ALIGN); _turnTgt = wrap180(yaw - _lookAng); _turnTries = 0; _go(L2_ALIGN); }
          else _startLeg2(s);
        } else _ask(L2_ASK, q);
      }
      break;
    case L2_ALIGN:
      if (_turnTries == 0) { if (_pivot(s, _turnTgt, l, r)) { _turnTries = 1; _stMs = now; } else if (now - _stMs > 15000) _startLeg2(s); }
      else if (now - _stMs >= 800) _startLeg2(s);
      break;
    case L2_GO: {
      const float gate = 0.7f * leg2ExpM;
      // 07.11.: no "!_slope" here - the open sea lies ~4 deg lower than leg 1, so it counted as slope the whole time and
      // the end wall was never accepted. The ground filter in _front() already removes ramp-floor readings.
      if (_frontM < 0.45f) { emergency = true; if (_log) _log->event(MissionLog::EV_HOLD); _prev = L2_GO; _go(HOLD); break; }
      if (_dist2 >= gate && _frontM <= endWallM) {
        if (!_endOn) { _endOn = true; _endMs = now; }
        if (now - _endMs >= 500) { if (_log) _log->event(MissionLog::EV_END_WALL); _ask(END_ASK, "End wall reached: open sea crossed."); break; }
      } else _endOn = false;
      if (_near(_dist2 >= gate ? holdM * 0.6f : holdM, now)) { _prev = L2_GO; if (_log) _log->event(MissionLog::EV_HOLD); _go(HOLD); break; }
      if (_dist2 > leg2MaxM) { if (_log) _log->event(MissionLog::EV_L2_MAX); _ask(END_ASK, "Maximum open-sea distance driven, no end wall seen."); break; }
      if (_mergeLeft > 0) { _mergeLeft -= fmaxf(0.0f, _outV) * 1.03f * dt; if (_mergeLeft <= 0) _tgt = _base; }
      // camera check every 2.6 m: 2 snapshots; never stops the rover
      if (_snapWant == 0 && _dist2 >= _nextCheck) { _snapWant = 2; _snapGot = _snapOk = 0; _snapPurpose = 2; _snapGapMs = 0; }
      if (_snapWant) {
        _snapStep(cam);
        if (_snapGot >= _snapWant) {
          bool used = false;
          if (_snapOk) {
            const float a = median3(_sAng, _snapOk), o = median3(_sOff, _snapOk) / pxPerM;
            if (fabsf(a) < 18.0f && fabsf(o) < 0.7f) {
              _lastCorr = clampf(-(0.1f * a + 0.4f * atanf(o / 1.4f) * RAD_TO_DEG), -7.8f, 7.8f);
              if (camAssist) { _base = wrap180(_base + _lastCorr); if (_mergeLeft <= 0) _tgt = _base; used = true; }
            }
          }
          if (_log) _log->event(used ? MissionLog::EV_CHECK_OK : MissionLog::EV_CHECK_FAIL);
          _snapWant = 0; _nextCheck += 2.6f;
        }
      }
      _ds.setTarget(_tgt);
      if (_ds.update(s.imuOk(), yaw, speed, now, l, r) == DriveStraight::FAIL) { if (_log) _log->event(MissionLog::EV_FAILSAFE); _pause(_ds.reason()); }
      break;
    }
    case HOLD:
      if (_leg == 2 && _dist2 >= 0.7f * leg2ExpM && _frontM <= endWallM) {
        if (_log) _log->event(MissionLog::EV_END_WALL); _ask(END_ASK, "End wall reached: open sea crossed."); break;
      }
      if (_frontM > 1.0f) {
        if (!_clearOn) { _clearOn = true; _clearMs = now; }
        if (now - _clearMs >= 2000) { if (_log) _log->event(MissionLog::EV_HOLD_END); _ds.start(yaw, now); _ds.setTarget(_tgt); _go(_prev); }
      } else _clearOn = false;
      break;
    case MANUAL: {
      l = padL; r = padR;
      const bool pivot = (l > 0 && r < 0) || (l < 0 && r > 0);
      if (pivot) { const float d = r > 0 ? 1.0f : -1.0f; l = -0.5f * d; r = 0.5f * d; }
      if (0.5f * (l + r) > 0 && _frontM < 0.35f) { l = r = 0; emergency = true; }
      break;
    }
    default: break;                                   // IDLE, ASK states, PAUSED, DONE: standing
  }

  if (now - _statusMs >= 250) {
    _statusMs = now;
    String j = _statusJson(s, rover, cam);
    if (xSemaphoreTake(_mtx, pdMS_TO_TICKS(20)) == pdTRUE) { _status = j; xSemaphoreGive(_mtx); }
  }
}

void RouteMission::fillLog(MissionLog::Sample& d, const Sensors& s, const RoverLink& rover, const CamLink& cam) const {
  d.batteryV = rover.voltage(); d.rttMs = rover.rttMs();
  d.outL = _outL; d.outR = _outR;
  d.state = _st; d.leg = _leg; d.pillars = _pillarsN; d.dsStatus = _ds.status();
  d.tgtDeg = _tgt; d.errDeg = _ds.errorDeg(); d.crabDeg = _crabEst;
  d.wallD = _wallOk ? _wall.distM : 0; d.wallAngDeg = _wallOk ? _wall.angleDeg : 0;
  d.distM = _leg == 2 ? _dist2 : _dist; d.frontM = _frontM;
  d.camPaired = cam.paired(); d.manual = _st == MANUAL; d.hold = _st == HOLD; d.slope = _slope;
  const MsgData& m = cam.snapResult();
  d.snapOk = m.stripOk; d.camRows = m.rows; d.camConf = m.conf; d.camOff = m.offX10 / 10.0f; d.camAng = m.angX10 / 10.0f;
  d.snapsSent = (uint16_t)cam.snapsSent(); d.snapsOk = (uint16_t)cam.snapsOk();
}

String RouteMission::_statusJson(const Sensors& s, const RoverLink& rover, const CamLink& cam) {
  String j; j.reserve(1300);
  j += "{\"st\":"; j += _st;
  j += ",\"state\":\""; j += kSt[_st < 14 ? _st : 0];
  j += "\",\"leg\":"; j += _leg;
  j += ",\"msg\":\""; jsonEsc(j, _msg);
  j += "\",\"q\":\""; jsonEsc(j, (_st == T1_ASK || _st == L2_ASK || _st == END_ASK) ? _q : "");
  j += "\",\"prev\":\""; j += kSt[_prev < 14 ? _prev : 0];
  j += "\",\"dist\":"; j += String(_dist, 1);
  j += ",\"dist2\":"; j += String(_dist2, 1);
  j += ",\"askAt\":"; j += String(_askAt, 1);
  j += ",\"pillars\":"; j += _pillarsN;
  j += ",\"wallOk\":"; j += _wallOk ? 1 : 0;
  j += ",\"wallD\":"; j += String(_wallOk ? _wall.distM : 0, 2);
  j += ",\"dSet\":"; j += String(_dSet, 2);
  j += ",\"crab\":"; j += String(_crabEst, 1);
  j += ",\"crabN\":"; j += _crabN;
  j += ",\"yaw\":"; j += String(s.yawNowDeg(), 1);
  j += ",\"tgt\":"; j += String(_tgt, 1);
  j += ",\"err\":"; j += String(_ds.errorDeg(), 1);
  j += ",\"learned\":"; j += String(_ds.learned(), 3);
  j += ",\"front\":"; j += String(_frontM, 2);
  j += ",\"lidarF\":"; j += String(_lidarFront, 2);
  j += ",\"slope\":"; j += _slope ? 1 : 0;
  j += ",\"pitch\":"; j += String(s.pitchDeg() - _pitch0, 1);
  j += ",\"lookN\":"; j += _lookN;
  j += ",\"lookOff\":"; j += String(_lookOff, 1);
  j += ",\"lookAng\":"; j += String(_lookAng, 1);
  j += ",\"corr\":"; j += String(_lastCorr, 1);
  j += ",\"nextCheck\":"; j += String(_nextCheck, 1);
  j += ",\"camPaired\":"; j += cam.paired() ? 1 : 0;
  j += ",\"snaps\":"; j += cam.snapsSent(); j += ",\"snapsOk\":"; j += cam.snapsOk();
  j += ",\"snapRtt\":"; j += cam.snapRttMs();
  j += ",\"rover\":"; j += rover.connected() ? 1 : 0;
  j += ",\"batt\":"; j += String(rover.voltage(), 2);
  j += ",\"imu\":"; j += s.imuOk() ? 1 : 0;
  j += ",\"lidar\":"; j += s.lidarOk() ? 1 : 0;
  j += ",\"gps\":"; j += s.gpsFix() ? 1 : 0; j += ",\"sats\":"; j += s.sats();
  j += ",\"heap\":"; j += (unsigned)(ESP.getFreeHeap() / 1024);
  j += ",\"set\":{\"speed\":"; j += String(speed, 2); j += ",\"t1\":"; j += String(t1AskM, 1); j += ",\"l2\":"; j += String(leg2ExpM, 1);
  j += ",\"end\":"; j += String(endWallM, 2); j += ",\"px\":"; j += String(pxPerM, 0); j += ",\"crab\":"; j += String(crabPriorDeg, 1);
  j += ",\"cam\":"; j += camAssist ? 1 : 0; j += ",\"auto\":"; j += autoL2 ? 1 : 0; j += ",\"mode\":"; j += stripMode;
  j += "}}";
  return j;
}
