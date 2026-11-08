#include "TrimCal.h"
#include "RoverTrim.h"

static float wrap180(float a) {
  a = fmodf(a + 180.0f, 360.0f);
  if (a < 0) a += 360.0f;
  return a - 180.0f;
}

static const char* kPhase[] = {"ready", "standing still (gyro check)", "speeding up", "settling", "MEASURING", "stopping"};
static const char* kKind[] = {"AUTO", "FIXED", "VERIFY", "CUSTOM"};
static const float kSpd[2] = {0.20f, 0.25f};

static void jsonEsc(String& o, const char* s) {
  for (; *s; s++) {
    if (*s == '"' || *s == '\\') { o += '\\'; o += *s; }
    else if ((uint8_t)*s >= 0x20) o += *s;
  }
}
static String num(float v, int d) { return isnan(v) ? String("null") : String(v, d); }

bool TrimCal::begin(AsyncWebServer* server) {
  _mtx = xSemaphoreCreateMutex();
  Preferences p;
  if (p.begin("trimcal", true)) {          // read-only: no flash write
    _saved[0] = p.getFloat("t20", NAN);
    _saved[1] = p.getFloat("t25", NAN);
    p.end();
  }
  for (int i = 0; i < 2; i++) _autoTrim[i] = isnan(_saved[i]) ? kTrimIndoor025 : _saved[i];   // start from the measured / saved value
  if (!server) return false;
  auto ok = [](AsyncWebServerRequest* req) { req->send(200, "application/json", "{\"ok\":true}"); };
  server->on("/trim/status", HTTP_GET, [this](AsyncWebServerRequest* req) {
    String j;
    if (xSemaphoreTake(_mtx, pdMS_TO_TICKS(50)) == pdTRUE) { j = _status; xSemaphoreGive(_mtx); }
    else j = "{}";
    req->send(200, "application/json", j);
  });
  server->on("/trim/alive", HTTP_GET, [this, ok](AsyncWebServerRequest* req) { _aliveMs = millis(); ok(req); });
  // /trim/start?kind=auto|fixed|verify|custom&spd=0.25[&trim=..]  (trim only for custom)
  server->on("/trim/start", HTTP_GET, [this, ok](AsyncWebServerRequest* req) {
    if (req->hasParam("spd")) _reqSpd = req->getParam("spd")->value().toFloat();
    if (req->hasParam("trim")) _reqTrim = req->getParam("trim")->value().toFloat();
    const String k = req->hasParam("kind") ? req->getParam("kind")->value() : String("auto");
    _reqKind = k == "fixed" ? FIXED : k == "verify" ? VERIFY : k == "custom" ? CUSTOM : AUTO;
    _aliveMs = millis();
    _reqStart = true;
    ok(req);
  });
  server->on("/trim/stop", HTTP_GET, [this, ok](AsyncWebServerRequest* req) { _reqStop = true; ok(req); });
  server->on("/trim/rate", HTTP_GET, [this, ok](AsyncWebServerRequest* req) { _reqRate = true; ok(req); });
  server->on("/trim/save", HTTP_GET, [this, ok](AsyncWebServerRequest* req) { _reqSave = true; ok(req); });
  server->on("/trim/clear", HTTP_GET, [this, ok](AsyncWebServerRequest* req) { _reqClear = true; ok(req); });
  server->on("/trim/results.csv", HTTP_GET, [this](AsyncWebServerRequest* req) {
    String c;
    if (xSemaphoreTake(_mtx, pdMS_TO_TICKS(200)) == pdTRUE) { c = _resultsCsv(); xSemaphoreGive(_mtx); }
    req->send(200, "text/csv", c);
  });
  server->on("/trim/last.csv", HTTP_GET, [this](AsyncWebServerRequest* req) {
    String c;
    if (xSemaphoreTake(_mtx, pdMS_TO_TICKS(200)) == pdTRUE) { c = _lastCsv(); xSemaphoreGive(_mtx); }
    req->send(200, "text/csv", c);
  });
  return true;
}

String TrimCal::phaseText() const { return kPhase[_phase < 6 ? _phase : 0]; }

// Free space ahead in m = LiDAR +-16 deg ONLY (9.9 = nothing within its range).
// 2026-11-07 fix: the front ultrasonic is NOT used for free space. It sits low and hears the floor:
// in logs 013-029, while the LiDAR saw nothing within 3 m, it read < 150 cm 62-74 % of the time
// (median 60-70 cm). v2 took that as "wall at 0.67 m" -> runs refused / ended after a few cm.
// The ultrasonic only covers what the LiDAR cannot see (closer than its 20 cm minimum): it counts as
// an obstacle only below usNearCm, continuously for usHoldMs (same limit as the team's SafetyGuard).
float TrimCal::_front(const Sensors& s) {
  float best = 9.9f;
  if (s.lidarOk()) {
    for (int k = -4; k < 4; k++) {
      const uint16_t cm = s.lidarCm((uint8_t)((k + Sensors::kLidarPoints) % Sensors::kLidarPoints));
      if (cm && cm / 100.0f < best) best = cm / 100.0f;
    }
  }
  _lidarFrontM = best;
  _usFrontCm = s.usCm(0);
  const bool usNear = _usFrontCm > 2 && _usFrontCm < usNearCm;
  const uint32_t now = millis();      // 2026-11-07: flag + start time ("now | 1" could lie 1 ms in the future)
  if (!usNear) _closeUs = false;
  else if (!_closeUs) { _closeUs = true; _closeUsMs = now; }
  if (usNear && now - _closeUsMs >= usHoldMs) best = fminf(best, _usFrontCm / 100.0f);
  return best;
}

// Suggested trim for one speed: straight line through (trim, drift) of the measured runs.
bool TrimCal::_suggest(float spd, float& t, int& nRuns, bool& fitted) const {
  double sx = 0, sy = 0, sxx = 0, sxy = 0; int k = 0; float xmin = 1, xmax = -1, lastX = 0, lastY = 0;
  for (uint8_t i = 0; i < _nRuns; i++) {
    const Run& r = _runs[i];
    if (fabsf(r.spd - spd) > 0.001f || r.measS < minMeasureS || isnan(r.omega)) continue;
    sx += r.trimMean; sy += r.omega; sxx += r.trimMean * r.trimMean; sxy += r.trimMean * r.omega; k++;
    xmin = fminf(xmin, r.trimMean); xmax = fmaxf(xmax, r.trimMean); lastX = r.trimMean; lastY = r.omega;
  }
  nRuns = k; fitted = false;
  if (k == 0) return false;
  if (k >= 2 && xmax - xmin >= 0.008f) {
    const double den = k * sxx - sx * sx;
    const double b = den > 1e-12 ? (k * sxy - sx * sy) / den : 0;
    const double a = (sy - b * sx) / k;
    if (b > 5.0) { t = constrain((float)(-a / b), -trimMax, trimMax); fitted = true; return true; }
  }
  t = constrain(lastX - lastY / (kTrimGainDegSPerUnit * spd / kTrimSpeed), -trimMax, trimMax);   // measured gain (log3)
  return true;
}

float TrimCal::_plannedTrim(uint8_t kind, int si, bool& ok, char* no, size_t n) const {
  ok = true;
  float t = 0; int k; bool fit;
  const bool have = _suggest(kSpd[si], t, k, fit);
  switch (kind) {
    case AUTO: return _autoTrim[si];
    case FIXED: {
      const float base = have ? t : _autoTrim[si];
      const float off[3] = {0, fixedStep, -fixedStep};
      return constrain(base + off[_fixedK[si] % 3], -trimMax, trimMax);
    }
    case VERIFY:
      if (!have || !fit) { ok = false; snprintf(no, n, "do step 2 first (needs a measured suggestion)"); }
      return t;
    default: return constrain((float)_reqTrim, -trimMax, trimMax);
  }
}

void TrimCal::_start(const Sensors& s, RoverLink& rover) {
  const float need = frontSoftM + 0.35f + 0.26f * (0.6f + minMeasureS);
  const char* no = nullptr;
  if (!rover.connected()) no = "rover not connected";
  else if (!s.imuOk()) no = "IMU not ready";
  else if (!s.lidarOk()) no = "LiDAR not ready";
  else if (_frontM < need) no = "not enough free space ahead";
  if (no) { snprintf(_msg, sizeof(_msg), "Not started: %s", no); return; }
  const int si = _si(_reqSpd);
  bool ok; char why[48] = "";
  const float t = _plannedTrim(_reqKind, si, ok, why, sizeof(why));
  if (!ok) { snprintf(_msg, sizeof(_msg), "Not started: %s", why); return; }
  _kind = _reqKind;
  _spd = kSpd[si];
  _trim = _trim0 = t;
  _batt = rover.voltage();
  _bN = _bT = _bY = _bTT = _bTY = 0;
  _mN = _mT = _mY = _mTT = _mTY = _mYY = 0;
  _trimSum = 0; _bias = 0; _dist = 0; _hard = false; _imuBadMs = 0;
  _why[0] = 0; _msg[0] = 0;
  _runMs = _phaseMs = millis();
  _lastSampleMs = 0;
  if (xSemaphoreTake(_mtx, pdMS_TO_TICKS(50)) == pdTRUE) { _nSmp = 0; xSemaphoreGive(_mtx); }
  _phase = BIAS;
  Serial.printf("[trim] start %s, speed %.2f, trim %.4f, battery %.2f V\n", kKind[_kind], _spd, _trim, _batt);
}

void TrimCal::_stopping(const char* why, bool hard) {
  if (_phase == IDLE) return;
  if (_phase == STOPPING) { if (hard) _hard = true; return; }
  strlcpy(_why, why, sizeof(_why));
  if (_phase == BIAS) { _finish(why); return; }      // standing anyway
  _hard = hard;
  _phase = STOPPING;
  _phaseMs = millis();
}

void TrimCal::_finish(const char* why) {
  if (why != _why) strlcpy(_why, why, sizeof(_why));
  Run r = {};
  r.n = ++_runNo; r.kind = _kind; r.spd = _spd; r.trim0 = _trim0; r.trim1 = _trim;
  r.bias = _bias; r.distM = _dist; r.batt = _batt;
  strlcpy(r.why, _why, sizeof(r.why));
  if (_mN >= 10) {
    const double den = _mN * _mTT - _mT * _mT;
    const double b = den > 1e-9 ? (_mN * _mTY - _mT * _mY) / den : 0;
    const double a = (_mY - b * _mT) / _mN;
    const double var = (_mYY - a * _mY - b * _mTY) / _mN;
    r.omega = (float)b - _bias;
    r.resid = var > 0 ? sqrtf((float)var) : 0;
    r.measS = (float)(_mT / _mN) * 2.0f;
    r.trimMean = (float)(_trimSum / _mN);
  } else {
    r.omega = NAN; r.resid = NAN; r.measS = 0; r.trimMean = _trim;
  }
  const int si = _si(_spd);
  const bool good = r.measS >= minMeasureS;
  if (good) {
    if (_kind == AUTO) { _autoTrim[si] = _trim; _autoN[si]++; }
    if (_kind == FIXED) _fixedK[si]++;
    if (_kind == VERIFY) { _verifyN[si]++; _verifyRate[si] = r.omega; _verifyTrim[si] = r.trimMean; }
  }
  if (xSemaphoreTake(_mtx, pdMS_TO_TICKS(100)) == pdTRUE) {
    if (_nRuns == kRuns) { memmove(&_runs[0], &_runs[1], sizeof(Run) * (kRuns - 1)); _nRuns--; }
    _runs[_nRuns++] = r;
    xSemaphoreGive(_mtx);
  }
  if (!good) snprintf(_msg, sizeof(_msg), "Run %u too short to count (%s) - run it again", r.n, r.why);
  Serial.printf("[trim] run %u %s done (%s): spd %.2f trim %.4f->%.4f mean %.4f | drift %.3f deg/s "
                "(bias %.3f) over %.1f s, ~%.2f m\n", r.n, kKind[r.kind], r.why, r.spd, r.trim0, r.trim1,
                r.trimMean, r.omega, r.bias, r.measS, r.distM);
  _phase = IDLE;
  _hard = false;
}

void TrimCal::update(const Sensors& s, RoverLink& rover, bool padActive, float& l, float& r, bool& emergency) {
  const uint32_t now = millis();
  float dt = _lastMs ? (now - _lastMs) / 1000.0f : 0.0f;
  _lastMs = now;
  if (dt > 0.1f) dt = 0.1f;

  if (s.imuOk()) {                                  // live yaw (unwrapped) + filtered yaw rate
    const float y = s.yawNowDeg();
    const float d = wrap180(y - _yawPrev);
    _yawPrev = y;
    if (dt > 0 && fabsf(d) < 30.0f) {
      _yawU += d;
      const float k = dt / (0.3f + dt);
      _rateF += k * (d / dt - _rateF);
    }
  }
  _frontM = _front(s);

  if (_reqStop) { _reqStop = false; if (active()) _stopping("STOP pressed", false); }
  if (_reqStart) { _reqStart = false; if (!active()) _start(s, rover); }
  if (_reqRate) { _reqRate = false; if (!active()) rover.requestSpeedRateRead(); }
  if (_reqClear) {
    _reqClear = false;
    if (!active() && xSemaphoreTake(_mtx, pdMS_TO_TICKS(50)) == pdTRUE) {
      _nRuns = 0; _nSmp = 0;
      xSemaphoreGive(_mtx);
      for (int i = 0; i < 2; i++) {
        _autoN[i] = _fixedK[i] = _verifyN[i] = 0; _verifyRate[i] = _verifyTrim[i] = NAN;
        _autoTrim[i] = isnan(_saved[i]) ? kTrimIndoor025 : _saved[i];
      }
      strlcpy(_msg, "Results cleared", sizeof(_msg));
    }
  }
  if (_reqSave) {
    _reqSave = false;
    if (!active()) {
      float t[2]; int k; bool fit[2] = {false, false}, have[2];
      for (int i = 0; i < 2; i++) have[i] = _suggest(kSpd[i], t[i], k, fit[i]) && fit[i];
      if (have[0] || have[1]) {
        Preferences p;
        if (p.begin("trimcal", false)) {            // the only flash write: 2 floats, on request
          if (have[0]) { p.putFloat("t20", t[0]); _saved[0] = t[0]; }
          if (have[1]) { p.putFloat("t25", t[1]); _saved[1] = t[1]; }
          p.end();
          strlcpy(_msg, "Saved in the box memory", sizeof(_msg));
        }
      } else {
        strlcpy(_msg, "Nothing to save yet (finish step 2)", sizeof(_msg));
      }
    }
  }

  if (active()) {
    const uint32_t inPhase = now - _phaseMs;
    const float tPh = inPhase / 1000.0f;
    if (_phase != STOPPING) {
      if (padActive) _stopping("move button pressed", false);
      else if (now - _aliveMs > heartbeatMs) _stopping("phone page lost (screen off / other app?)", false);
      else if (!rover.connected()) _stopping("rover lost", false);
      else if (s.imuOk() && (fabsf(s.rollDeg()) > tiltDeg || fabsf(s.pitchDeg()) > tiltDeg)) _stopping("tilt", true);
      else if (_phase >= RAMP && _frontM < frontHardM)
        _stopping(_lidarFrontM < frontHardM ? "LiDAR: obstacle closer than 0.40 m" : "ultrasonic: something at the nose (< 25 cm)", true);
      else if (_phase >= RAMP && _frontM < frontSoftM) _stopping("LiDAR: 0.70 m to the wall (end of free space)", false);
    }
    if (!s.imuOk()) {
      if (_imuBadMs == 0) _imuBadMs = now;
      else if (now - _imuBadMs > 500) _stopping("IMU invalid", false);
    } else {
      _imuBadMs = 0;
    }

    switch (_phase) {
      case BIAS:
        if (s.imuOk()) { _bN++; _bT += tPh; _bY += _yawU; _bTT += tPh * tPh; _bTY += tPh * _yawU; }
        if (inPhase >= 2000) {
          const double den = _bN * _bTT - _bT * _bT;
          _bias = (_bN > 20 && den > 1e-9) ? (float)((_bN * _bTY - _bT * _bY) / den) : 0.0f;
          _phase = RAMP; _phaseMs = now;
        }
        break;
      case RAMP:
        if (inPhase >= 2700) { _phase = SETTLE; _phaseMs = now; }
        break;
      case SETTLE:
        if (inPhase >= 600) { _phase = MEASURE; _phaseMs = now; _mN = _mT = _mY = _mTT = _mTY = _mYY = 0; _trimSum = 0; _y0 = _yawU; }
        break;
      case MEASURE:
        if (s.imuOk()) {
          const double y = _yawU - _y0;
          _mN++; _mT += tPh; _mY += y; _mTT += tPh * tPh; _mTY += tPh * y; _mYY += y * y;
          _trimSum += _trim;
        }
        if (_kind == AUTO && dt > 0) {
          const float G = kTrimGainDegSPerUnit * _spd / kTrimSpeed;   // measured (log3): 8.6 deg/s per unit at 0.25
          const float step = -(_rateF - _bias) / (G * autoTauS) * dt;
          _trim = constrain(_trim + constrain(step, -trimRate * dt, trimRate * dt), -trimMax, trimMax);
        }
        if (tPh >= measureS) _stopping("done", false);
        break;
      case STOPPING:
        if (inPhase >= 1500) _finish(_why);
        break;
      default: break;
    }

    if (_phase == RAMP || _phase == SETTLE || _phase == MEASURE) {
      l = _spd * (1.0f - _trim);
      r = _spd * (1.0f + _trim);
    } else if (active()) {
      l = r = 0;
    }
    emergency = emergency || _hard;
    if (_outV > 0) _dist += _outV * mPerUnit * dt;

    if (active() && now - _lastSampleMs >= 100) {   // 10 Hz samples (RAM only)
      _lastSampleMs = now;
      if (xSemaphoreTake(_mtx, 0) == pdTRUE) {
        if (_nSmp < kSamples) {
          Sample& m = _smp[_nSmp++];
          m.tDs = (uint16_t)((now - _runMs) / 100);
          m.phase = _phase;
          m.vX1000 = (int16_t)lroundf(_outV * 1000.0f);      // what really went to the rover
          m.trimX10000 = (int16_t)lroundf(_trim * 10000.0f);
          m.rateX100 = (int16_t)constrain(lroundf((_rateF - _bias) * 100.0f), -32000L, 32000L);
          m.yaw = _yawU;
          m.frontCm = (uint16_t)lroundf(fminf(_frontM, 9.9f) * 100.0f);
        }
        xSemaphoreGive(_mtx);
      }
    }
  }
  if (!active()) _batt = rover.voltage();

  if (now - _statusMs >= 250) {                     // status for the page, 4 x per second
    _statusMs = now;
    _rateReply = rover.speedRateReply();
    String j = _statusJson(s, rover);
    if (xSemaphoreTake(_mtx, pdMS_TO_TICKS(20)) == pdTRUE) { _status = j; xSemaphoreGive(_mtx); }
  }
}

String TrimCal::_statusJson(const Sensors& s, RoverLink& rover) {
  const float need = frontSoftM + 0.35f + 0.26f * (0.6f + measureS);
  const float needMin = frontSoftM + 0.35f + 0.26f * (0.6f + minMeasureS);
  String j;
  j.reserve(1700);
  j += "{\"ph\":"; j += _phase;
  j += ",\"phase\":\""; j += kPhase[_phase < 6 ? _phase : 0];
  j += "\",\"kind\":\""; j += kKind[_kind < 4 ? _kind : 0];
  j += "\",\"spd\":"; j += String(_spd, 2);
  j += ",\"t\":"; j += active() ? (millis() - _runMs) / 1000 : 0;
  j += ",\"why\":\""; jsonEsc(j, _why);
  j += "\",\"msg\":\""; jsonEsc(j, _msg);
  j += "\",\"trim\":"; j += String(_trim, 4);
  j += ",\"rate\":"; j += String(_rateF - _bias, 2);
  j += ",\"front\":"; j += String(fminf(_frontM, 9.9f), 2);
  j += ",\"need\":"; j += String(need, 1);
  j += ",\"needMin\":"; j += String(needMin, 1);
  j += ",\"batt\":"; j += String(_batt, 2);
  j += ",\"imu\":"; j += s.imuOk() ? 1 : 0;
  j += ",\"lidar\":"; j += s.lidarOk() ? 1 : 0;
  j += ",\"rover\":"; j += rover.connected() ? 1 : 0;
  j += ",\"pass\":"; j += String(passDegS, 2);
  j += ",\"sp\":[";
  for (int i = 0; i < 2; i++) {
    float t = 0; int k = 0; bool fit = false;
    const bool have = _suggest(kSpd[i], t, k, fit);
    bool ok; char no[48];
    const float nextFixed = _plannedTrim(FIXED, i, ok, no, sizeof(no));
    if (i) j += ',';
    j += "{\"auto\":"; j += _autoN[i];
    j += ",\"autoTrim\":"; j += String(_autoTrim[i], 4);
    j += ",\"fixedK\":"; j += _fixedK[i];
    j += ",\"nextFixed\":"; j += String(nextFixed, 4);
    j += ",\"sug\":"; j += have ? String(t, 4) : String("null");
    j += ",\"fit\":"; j += fit ? 1 : 0;
    j += ",\"n\":"; j += k;
    j += ",\"vN\":"; j += _verifyN[i];
    j += ",\"vRate\":"; j += num(_verifyRate[i], 2);
    j += ",\"vTrim\":"; j += num(_verifyTrim[i], 4);
    j += ",\"saved\":"; j += num(_saved[i], 4);
    j += '}';
  }
  j += "],\"runs\":[";
  const uint8_t first = _nRuns > 8 ? _nRuns - 8 : 0;
  for (uint8_t i = first; i < _nRuns; i++) {
    const Run& r = _runs[i];
    if (i > first) j += ',';
    j += '['; j += r.n; j += ','; j += String(r.spd, 2); j += ",\""; j += kKind[r.kind < 4 ? r.kind : 0];
    j += "\","; j += String(r.trimMean, 4); j += ','; j += num(r.omega, 2);
    j += ','; j += String(r.measS, 1); j += ",\""; jsonEsc(j, r.why); j += "\"]";
  }
  j += "],\"dbg\":{\"rtt\":"; j += String(rover.rttMs(), 0);
  j += ",\"ok\":"; j += rover.okCount(); j += ",\"fail\":"; j += rover.failCount();
  j += ",\"heap\":"; j += (unsigned)(ESP.getFreeHeap() / 1024);
  j += ",\"minheap\":"; j += (unsigned)(ESP.getMinFreeHeap() / 1024);
  j += ",\"us\":[";
  for (int i = 0; i < 5; i++) { if (i) j += ','; j += s.usCm(i); }
  j += "],\"lidarF\":"; j += String(fminf(_lidarFrontM, 9.9f), 2);
  j += ",\"usF\":"; j += _usFrontCm;
  j += ",\"roll\":"; j += String(s.rollDeg(), 1);
  j += ",\"pitch\":"; j += String(s.pitchDeg(), 1);
  j += ",\"yaw\":"; j += String(s.yawDeg(), 1);
  j += ",\"bias\":"; j += String(_bias, 3);
  j += ",\"up\":"; j += millis() / 1000;
  j += ",\"reset\":\""; jsonEsc(j, _reset.c_str());
  j += "\",\"rr\":\""; jsonEsc(j, _rateReply.c_str());
  j += "\"}}";
  return j;
}

String TrimCal::_resultsCsv() {
  String c;
  c.reserve(400 + _nRuns * 120);
  c += "run,speed,kind,trim_start,trim_end,trim_mean,drift_deg_s,resid_deg,gyro_bias_deg_s,measured_s,dist_m_est,battery_v,end\n";
  for (uint8_t i = 0; i < _nRuns; i++) {
    const Run& r = _runs[i];
    c += r.n; c += ','; c += String(r.spd, 2); c += ','; c += kKind[r.kind < 4 ? r.kind : 0];
    c += ','; c += String(r.trim0, 4); c += ','; c += String(r.trim1, 4); c += ','; c += String(r.trimMean, 4);
    c += ','; c += String(r.omega, 3); c += ','; c += String(r.resid, 2); c += ','; c += String(r.bias, 3);
    c += ','; c += String(r.measS, 1); c += ','; c += String(r.distM, 2); c += ','; c += String(r.batt, 2);
    c += ",\""; c += r.why; c += "\"\n";
  }
  c += "\n# summary\nspeed,suggested_trim,from_runs,fitted,verify_drift_deg_s,verify_trim,saved_trim\n";
  for (int i = 0; i < 2; i++) {
    float t = 0; int k = 0; bool fit = false;
    const bool have = _suggest(kSpd[i], t, k, fit);
    c += String(kSpd[i], 2); c += ','; c += have ? String(t, 4) : String(""); c += ','; c += k;
    c += ','; c += fit ? "yes" : "no"; c += ','; c += isnan(_verifyRate[i]) ? String("") : String(_verifyRate[i], 3);
    c += ','; c += isnan(_verifyTrim[i]) ? String("") : String(_verifyTrim[i], 4);
    c += ','; c += isnan(_saved[i]) ? String("") : String(_saved[i], 4); c += '\n';
  }
  c += "# reset reason: "; c += _reset; c += "\n# rover T:139 reply: "; c += _rateReply; c += '\n';
  return c;
}

String TrimCal::_lastCsv() {
  String c;
  c.reserve(60 + _nSmp * 44);
  c += "t_s,phase,v_out,trim,drift_deg_s,yaw_deg,front_m\n";
  for (uint16_t i = 0; i < _nSmp; i++) {
    const Sample& m = _smp[i];
    c += String(m.tDs / 10.0f, 1); c += ','; c += kPhase[m.phase < 6 ? m.phase : 0]; c += ',';
    c += String(m.vX1000 / 1000.0f, 3); c += ','; c += String(m.trimX10000 / 10000.0f, 4); c += ',';
    c += String(m.rateX100 / 100.0f, 2); c += ','; c += String(m.yaw, 2); c += ','; c += String(m.frontCm / 100.0f, 2);
    c += '\n';
  }
  return c;
}
