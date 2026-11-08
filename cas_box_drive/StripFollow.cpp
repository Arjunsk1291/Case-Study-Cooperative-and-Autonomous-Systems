#include "StripFollow.h"

static float wrap180(float a) { a = fmodf(a + 180.0f, 360.0f); if (a < 0) a += 360.0f; return a - 180.0f; }
static float clampf(float x, float lo, float hi) { return x < lo ? lo : (x > hi ? hi : x); }
static const char* kPh[] = {"ready", "aligning: turning onto the strip", "aligning: checking", "FOLLOWING the strip", "stopping"};
static const char* kWhat[] = {"align only", "follow only", "align + follow"};

static void jsonEsc(String& o, const char* s) {
  for (; *s; s++) { if (*s == '"' || *s == '\\') { o += '\\'; o += *s; } else if ((uint8_t)*s >= 0x20) o += *s; }
}

bool StripFollow::begin(AsyncWebServer* server) {
  _mtx = xSemaphoreCreateMutex();
  if (!server) return false;
  auto ok = [](AsyncWebServerRequest* q) { q->send(200, "application/json", "{\"ok\":true}"); };
  server->on("/follow/status", HTTP_GET, [this](AsyncWebServerRequest* q) {
    String j; if (xSemaphoreTake(_mtx, pdMS_TO_TICKS(50)) == pdTRUE) { j = _status; xSemaphoreGive(_mtx); } else j = "{}";
    q->send(200, "application/json", j);
  });
  server->on("/follow/alive", HTTP_GET, [this, ok](AsyncWebServerRequest* q) { _aliveMs = millis(); ok(q); });
  server->on("/follow/start", HTTP_GET, [this, ok](AsyncWebServerRequest* q) {
    _reqWhat = q->hasParam("what") ? (uint8_t)constrain(q->getParam("what")->value().toInt(), 0L, 2L) : 2;
    if (q->hasParam("spd")) _reqSpd = q->getParam("spd")->value().toFloat();
    if (q->hasParam("stop")) _reqStopM = q->getParam("stop")->value().toFloat();
    if (q->hasParam("max")) _reqMaxM = q->getParam("max")->value().toFloat();
    _aliveMs = millis(); _reqStart = true; ok(q);
  });
  server->on("/follow/stop", HTTP_GET, [this, ok](AsyncWebServerRequest* q) { _reqStop = true; ok(q); });
  server->on("/follow/zero", HTTP_GET, [this, ok](AsyncWebServerRequest* q) { _reqZero = true; ok(q); });
  server->on("/follow/log.csv", HTTP_GET, [this](AsyncWebServerRequest* q) {
    String c; if (xSemaphoreTake(_mtx, pdMS_TO_TICKS(300)) == pdTRUE) { c = _logCsv(); xSemaphoreGive(_mtx); }
    q->send(200, "text/csv", c);
  });
  server->on("/follow/runs.csv", HTTP_GET, [this](AsyncWebServerRequest* q) {
    String c; if (xSemaphoreTake(_mtx, pdMS_TO_TICKS(300)) == pdTRUE) { c = _runsCsv(); xSemaphoreGive(_mtx); }
    q->send(200, "text/csv", c);
  });
  return true;
}

void StripFollow::_start(const Sensors& s, RoverLink& rover, const CamLink& cam) {
  const char* no = nullptr;
  const uint8_t what = _reqWhat;
  if (!rover.connected()) no = "rover not connected";
  else if (!s.imuOk()) no = "IMU not ready";
  else if (!s.lidarOk()) no = "LiDAR not ready";
  else if (what != 1 && !(cam.fresh() && cam.data().stripOk)) no = "no strip in view (align needs the camera)";
  if (no) { snprintf(_msg, sizeof(_msg), "Not started: %s", no); return; }
  _what = what;
  speed = clampf(_reqSpd, 0.12f, 0.30f);
  stopWallM = clampf(_reqStopM, 0.35f, 1.5f);
  maxDistM = clampf(_reqMaxM, 0.5f, 20.0f);
  _dist = _distSinceStrip = 0; _seenN = _framesN = 0; _offAbsMax = 0; _tries = 0; _pivots = 0; _hard = false;
  _why[0] = 0; _msg[0] = 0;
  _runMs = _phaseMs = _lastMs = millis(); _lastStripMs = _runMs; _lastCamSeq = cam.data().h.seq; _lastSmpMs = 0;
  _prevYaw = s.yawNowDeg(); _rate = 0;
  if (xSemaphoreTake(_mtx, pdMS_TO_TICKS(50)) == pdTRUE) { _nSmp = 0; xSemaphoreGive(_mtx); }
  if (_what == 1) { _phase = FOLLOW; _yaw0 = _tgt = s.yawNowDeg(); _ds.start(_tgt, _runMs); }
  else { _phase = ALIGN_SETTLE; }          // first look at the strip, then turn
  Serial.printf("[follow] start: %s, speed %.2f, stop at %.2f m, max %.1f m\n", kWhat[_what], speed, stopWallM, maxDistM);
}

void StripFollow::_stop(const char* why, bool hard) {
  if (_phase == IDLE) return;
  if (_phase == STOPPING) { if (hard) _hard = true; return; }
  strlcpy(_why, why, sizeof(_why));
  _hard = hard;
  _phase = STOPPING;
  _phaseMs = millis();
}

void StripFollow::_finish() {
  Run r = {};
  r.n = ++_runNo; r.what = _what; r.spd = speed; r.distM = _dist; r.timeS = (millis() - _runMs) / 1000.0f;
  r.seenPct = _framesN ? 100.0f * _seenN / _framesN : 0; r.offAbsMax = _offAbsMax; r.pivots = _pivots;
  r.offEnd = NAN; r.frontEnd = NAN;
  strlcpy(r.why, _why, sizeof(r.why));
  // last logged sample = end values
  if (xSemaphoreTake(_mtx, pdMS_TO_TICKS(100)) == pdTRUE) {
    if (_nSmp) { r.frontEnd = _smp[_nSmp - 1].frontCm / 100.0f; if (_smp[_nSmp - 1].flags & 2) r.offEnd = _smp[_nSmp - 1].offX10 / 10.0f; }
    if (_nRuns == kRuns) { memmove(&_runs[0], &_runs[1], sizeof(Run) * (kRuns - 1)); _nRuns--; }
    _runs[_nRuns++] = r;
    xSemaphoreGive(_mtx);
  }
  Serial.printf("[follow] run %u end: %s | %.2f m in %.1f s, strip seen %.0f %%, max |offset| %.1f px, ahead at end %.2f m\n",
                r.n, r.why, r.distM, r.timeS, r.seenPct, r.offAbsMax, r.frontEnd);
  _phase = IDLE; _hard = false;
}

void StripFollow::update(const Sensors& s, RoverLink& rover, const CamLink& cam, float frontM, bool padActive,
                         float& l, float& r, bool& emergency) {
  const uint32_t now = millis();
  float dt = _lastMs ? (now - _lastMs) / 1000.0f : 0; _lastMs = now;
  if (dt > 0.1f) dt = 0.1f;
  const bool imuOk = s.imuOk();
  const float yaw = s.yawNowDeg();
  if (imuOk && dt > 0) { const float d = wrap180(yaw - _prevYaw); if (fabsf(d) < 30) _rate += (d / dt - _rate) * (dt / (0.15f + dt)); }
  _prevYaw = yaw;

  if (_reqStop) { _reqStop = false; if (active()) _stop("STOP pressed", false); }
  if (_reqStart) {
    _reqStart = false;
    if (blocked) strlcpy(_msg, "Not started: a trim run is active", sizeof(_msg));
    else if (!active()) _start(s, rover, cam);
  }
  if (_reqZero) {
    _reqZero = false;
    if (!active()) {
      if (cam.fresh() && cam.data().stripOk) { _offZero = cam.data().offX10 / 10.0f; snprintf(_msg, sizeof(_msg), "Centre set: %.1f px = on the strip", _offZero); }
      else strlcpy(_msg, "Centre NOT set: no strip in view", sizeof(_msg));
    }
  }

  // camera: a NEW message with a strip?
  const bool camNew = cam.data().h.seq != _lastCamSeq;
  if (camNew) _lastCamSeq = cam.data().h.seq;
  const bool stripNow = cam.fresh() && cam.data().stripOk;
  const float off = _offCorr(cam), ang = cam.data().angX10 / 10.0f;

  if (active()) {
    const uint32_t inPh = now - _phaseMs;
    if (_phase != STOPPING) {
      if (padActive) _stop("move button pressed", false);
      else if (now - _aliveMs > heartbeatMs) _stop("phone page lost (screen off / other app?)", false);
      else if (!rover.connected()) _stop("rover lost", false);
      else if (imuOk && (fabsf(s.rollDeg()) > tiltDeg || fabsf(s.pitchDeg()) > tiltDeg)) _stop("tilt", true);
      else if ((now - _runMs) / 1000.0f > maxTimeS) _stop("time limit", false);
      else if (_dist > maxDistM) _stop("distance limit", false);
    }
    float vT = 0;
    switch (_phase) {
      case ALIGN_SETTLE:                        // stand still 0.8 s, then decide from the camera
        l = r = 0;
        if (inPh >= 800) {
          if (!stripNow) { _stop("no strip in view", false); break; }
          if (fabsf(ang) <= alignDoneDeg || _tries >= 3) {
            if (_what == 0) { _stop(fabsf(ang) <= alignDoneDeg ? "aligned" : "align: 3 tries, still off", false); break; }
            _phase = FOLLOW; _phaseMs = now; _yaw0 = _tgt = yaw; _ds.start(_tgt, now);
            _lastStripMs = now; _distSinceStrip = 0;
          } else {
            _alignTgt = wrap180(yaw - ang);     // strip leans right (+) -> turn right (yaw -)
            _tries++; _pivots++; _phase = ALIGN_TURN; _phaseMs = now;
          }
        }
        break;
      case ALIGN_TURN: {                         // full pivot, gyro-closed, released early by the coast
        const float e = wrap180(_alignTgt - yaw);
        if (!imuOk) { _stop("IMU invalid", false); break; }
        if (fabsf(e) <= fmaxf(3.0f, fabsf(_rate) * 0.45f) || inPh > 6000) {
          if (inPh > 6000) Serial.println("[follow] pivot time-out");
          l = r = 0; _phase = ALIGN_SETTLE; _phaseMs = now;
        } else {
          const float d = e > 0 ? 1.0f : -1.0f;  // + = left
          l = -0.5f * d; r = 0.5f * d;
        }
        break;
      }
      case FOLLOW: {
        if (stripNow && camNew) {                // camera moves the target heading
          const float want = wrap180(yaw - (kAng * ang + kOff * off));
          float step = clampf(wrap180(want - _tgt), -tgtRateDegS * 0.1f, tgtRateDegS * 0.1f);   // per message (10/s)
          float t = wrap180(_tgt + step);
          const float fromStart = clampf(wrap180(t - _yaw0), -tgtMaxDeg, tgtMaxDeg);
          _tgt = wrap180(_yaw0 + fromStart);
          _ds.setTarget(_tgt);
        }
        if (camNew) { _framesN++; if (stripNow) { _seenN++; _offAbsMax = fmaxf(_offAbsMax, fabsf(off)); } }
        if (stripNow) { _lastStripMs = now; _distSinceStrip = 0; }
        const float lostS = (now - _lastStripMs) / 1000.0f;
        vT = (lostS > 1.0f) ? fminf(speed, 0.15f) : speed;
        if (lostS > 3.0f || _distSinceStrip > 0.8f) { _stop("strip lost", false); break; }
        if (frontM < hardM) { _stop("obstacle closer than 0.30 m", true); break; }
        if (frontM <= stopWallM) { _stop("reached the wall", false); break; }
        const auto st = _ds.update(imuOk, yaw, vT, now, l, r);
        if (st == DriveStraight::FAIL) _stop(_ds.reason(), false);
        break;
      }
      case STOPPING:
        l = r = 0;
        if (inPh >= 1500) _finish();
        break;
      default: break;
    }
    if (_phase == STOPPING || _phase == ALIGN_SETTLE) l = r = 0;
    if (_phase == FOLLOW && frontM < hardM) emergency = true;
    emergency = emergency || _hard;
    if (_outV > 0) { _dist += _outV * mPerUnit * dt; _distSinceStrip += _outV * mPerUnit * dt; }

    if (active() && now - _lastSmpMs >= 100) {   // 10 Hz log (RAM)
      _lastSmpMs = now;
      if (xSemaphoreTake(_mtx, 0) == pdTRUE) {
        if (_nSmp < kSmp) {
          Smp& m = _smp[_nSmp++];
          m.tDs = (now - _runMs) / 100; m.ph = _phase;
          m.flags = (cam.fresh() ? 1 : 0) | (stripNow ? 2 : 0) | (imuOk ? 4 : 0);
          m.rows = cam.data().rows;
          m.vX1000 = lroundf(_outV * 1000); m.lX1000 = lroundf(_outL * 1000); m.rX1000 = lroundf(_outR * 1000);
          m.yawX10 = lroundf(yaw * 10); m.tgtX10 = lroundf((_phase == ALIGN_TURN ? _alignTgt : _tgt) * 10);
          m.offX10 = lroundf(off * 10); m.angX10 = lroundf(ang * 10);
          m.frontCm = (uint16_t)lroundf(fminf(frontM, 9.9f) * 100);
        }
        xSemaphoreGive(_mtx);
      }
    }
  }

  if (now - _statusMs >= 250) {
    _statusMs = now;
    String j = _statusJson(s, cam, frontM);
    if (xSemaphoreTake(_mtx, pdMS_TO_TICKS(20)) == pdTRUE) { _status = j; xSemaphoreGive(_mtx); }
  }
}

String StripFollow::_statusJson(const Sensors& s, const CamLink& cam, float frontM) {
  String j;
  j.reserve(1200);
  j += "{\"ph\":"; j += _phase;
  j += ",\"phase\":\""; j += kPh[_phase < 5 ? _phase : 0];
  j += "\",\"what\":\""; j += kWhat[_what < 3 ? _what : 2];
  j += "\",\"t\":"; j += active() ? (millis() - _runMs) / 1000 : 0;
  j += ",\"why\":\""; jsonEsc(j, _why);
  j += "\",\"msg\":\""; jsonEsc(j, _msg);
  j += "\",\"dist\":"; j += String(_dist, 2);
  j += ",\"seen\":"; j += _framesN ? String(100.0f * _seenN / _framesN, 0) : String("0");
  j += ",\"front\":"; j += String(fminf(frontM, 9.9f), 2);
  j += ",\"stopM\":"; j += String(stopWallM, 2);
  j += ",\"spd\":"; j += String(speed, 2);
  j += ",\"maxM\":"; j += String(maxDistM, 1);
  j += ",\"yaw\":"; j += String(s.yawNowDeg(), 1);
  j += ",\"tgt\":"; j += String(_phase == ALIGN_TURN ? _alignTgt : _tgt, 1);
  j += ",\"zero\":"; j += String(_offZero, 1);
  j += ",\"offC\":"; j += String(_offCorr(cam), 1);
  j += ",\"ds\":\""; jsonEsc(j, _ds.reason());
  j += "\",\"runs\":[";
  const uint8_t first = _nRuns > 8 ? _nRuns - 8 : 0;
  for (uint8_t i = first; i < _nRuns; i++) {
    const Run& r = _runs[i];
    if (i > first) j += ',';
    j += '['; j += r.n; j += ",\""; j += kWhat[r.what < 3 ? r.what : 2]; j += "\","; j += String(r.distM, 2); j += ',';
    j += String(r.timeS, 1); j += ','; j += String(r.seenPct, 0); j += ','; j += String(r.offAbsMax, 1); j += ',';
    j += isnan(r.frontEnd) ? String("null") : String(r.frontEnd, 2); j += ",\""; jsonEsc(j, r.why); j += "\"]";
  }
  j += "]}";
  return j;
}

String StripFollow::_logCsv() {
  String c;
  c.reserve(80 + _nSmp * 70);
  c += "t_s,phase,cam_fresh,strip,rows,offset_px,angle_deg,yaw_deg,target_deg,v_out,l_out,r_out,front_m\n";
  for (uint16_t i = 0; i < _nSmp; i++) {
    const Smp& m = _smp[i];
    c += String(m.tDs / 10.0f, 1); c += ','; c += kPh[m.ph < 5 ? m.ph : 0]; c += ','; c += (m.flags & 1) ? 1 : 0; c += ',';
    c += (m.flags & 2) ? 1 : 0; c += ','; c += m.rows; c += ','; c += String(m.offX10 / 10.0f, 1); c += ',';
    c += String(m.angX10 / 10.0f, 1); c += ','; c += String(m.yawX10 / 10.0f, 1); c += ','; c += String(m.tgtX10 / 10.0f, 1); c += ',';
    c += String(m.vX1000 / 1000.0f, 3); c += ','; c += String(m.lX1000 / 1000.0f, 3); c += ','; c += String(m.rX1000 / 1000.0f, 3); c += ',';
    c += String(m.frontCm / 100.0f, 2); c += '\n';
  }
  return c;
}

String StripFollow::_runsCsv() {
  String c;
  c.reserve(120 + _nRuns * 110);
  c += "run,what,speed,dist_m,time_s,strip_seen_pct,max_abs_offset_px,offset_end_px,ahead_end_m,pivots,end\n";
  for (uint8_t i = 0; i < _nRuns; i++) {
    const Run& r = _runs[i];
    c += r.n; c += ','; c += kWhat[r.what < 3 ? r.what : 2]; c += ','; c += String(r.spd, 2); c += ','; c += String(r.distM, 2); c += ',';
    c += String(r.timeS, 1); c += ','; c += String(r.seenPct, 0); c += ','; c += String(r.offAbsMax, 1); c += ',';
    c += isnan(r.offEnd) ? String("") : String(r.offEnd, 1); c += ','; c += isnan(r.frontEnd) ? String("") : String(r.frontEnd, 2); c += ',';
    c += r.pivots; c += ",\""; c += r.why; c += "\"\n";
  }
  c += "# centre (zero) offset px: "; c += String(_offZero, 1); c += '\n';
  return c;
}
