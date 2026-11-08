#include "MissionLog.h"
#include <LittleFS.h>
#include <vector>

static int8_t toI8(float v) { const long x = lroundf(v * 200.0f); return (int8_t)(x < -127 ? -127 : (x > 127 ? 127 : x)); }
static uint16_t toU16(long v) { return (uint16_t)(v < 0 ? 0 : (v > 65535 ? 65535 : v)); }
static int16_t toI16(float v) { const long x = lroundf(v); return (int16_t)(x < -32767 ? -32767 : (x > 32767 ? 32767 : x)); }

bool MissionLog::begin(AsyncWebServer* server) {
  _fsOk = LittleFS.begin(true);   // the data partition; formatted once on first use
  if (_fsOk) {
    _usedKB = LittleFS.usedBytes() / 1024; _totalKB = LittleFS.totalBytes() / 1024;
    Serial.printf("[log] LittleFS %u / %u kB used\n", (unsigned)_usedKB, (unsigned)_totalKB);
    _refreshList();
  } else {
    Serial.println("[log] LittleFS mount failed - logging disabled (partition scheme must be Default)");
  }
  if (!server) return _fsOk;
  auto ok = [](AsyncWebServerRequest* q) { q->send(200, "application/json", "{\"ok\":true}"); };
  server->on("/log/status", HTTP_GET, [this](AsyncWebServerRequest* q) { q->send(200, "application/json", statusJson()); });
  server->on("/log/start", HTTP_GET, [this, ok](AsyncWebServerRequest* q) { _reqStart = true; ok(q); });
  server->on("/log/stop", HTTP_GET, [this, ok](AsyncWebServerRequest* q) { _reqStop = true; ok(q); });
  server->on("/log/clear", HTTP_GET, [this, ok](AsyncWebServerRequest* q) { _reqClear = true; ok(q); });
  server->on("/log/mark", HTTP_GET, [this, ok](AsyncWebServerRequest* q) { _userMarks = _userMarks + 1; ok(q); });
  server->on("/log/file", HTTP_GET, [this](AsyncWebServerRequest* q) {
    if (!_fsOk || !q->hasParam("name")) { q->send(400, "text/plain", "name missing"); return; }
    const String n = q->getParam("name")->value();
    if (!_validName(n)) { q->send(400, "text/plain", "bad name"); return; }
    if (_active && (n == _fileName || n.substring(4) == String(_fileName).substring(4))) { q->send(409, "text/plain", "stop the log first"); return; }
    if (!LittleFS.exists("/" + n)) { q->send(404, "text/plain", "not found"); return; }
    q->send(LittleFS, "/" + n, "application/octet-stream", true);
  });
  return _fsOk;
}

void MissionLog::event(uint8_t ev) {
  const uint8_t next = (_evHead + 1) % 16;
  if (next == _evTail) return;                 // queue full: drop (16 events within 4 s is not expected)
  _evQ[_evHead] = ev; _evHead = next;
}

void MissionLog::update(const Sensors& s, const Sample& d) {
  if (!_fsOk) return;
  if (_reqClear) { _reqClear = false; if (!_active) _clear(); }
  if (_reqStart) { _reqStart = false; if (!_active) _start(); }
  if (_reqStop) { _reqStop = false; if (_active) _stop(); }
  if (_userMarks != _seenMarks) { _seenMarks = _userMarks; event(EV_MARK); }
  if (!_active) { _evTail = _evHead; return; }  // not recording: events are not kept
  const uint32_t now = millis();
  if (now - _lastRecMs < kPeriodMs) return;
  _lastRecMs += kPeriodMs;
  if (now - _lastRecMs > kPeriodMs) _lastRecMs = now;
  uint8_t ev = EV_NONE;
  if (_evTail != _evHead) { ev = _evQ[_evTail]; _evTail = (_evTail + 1) % 16; }
  _fill(_buf[_bufN++], s, d, ev);
  _records = _records + 1;
  if (_bufN >= kBuf) _flush();
}

void MissionLog::_fill(Rec& r, const Sensors& s, const Sample& d, uint8_t ev) {
  r.ms = millis();
  // GPS: the Sensors snapshot (last valid fix). Note: the NEO-6M saw 0 satellites in logs 025-029.
  Sensors& sm = const_cast<Sensors&>(s);
  r.latE7 = (int32_t)llround(sm.lat() * 1e7); r.lonE7 = (int32_t)llround(sm.lon() * 1e7);
  r.hdopX100 = toU16(lroundf(sm.hdop() * 100)); r.sats = (uint8_t)constrain(sm.sats(), 0, 255);
  r.flags = (s.gpsFix() ? 1 : 0) | (s.imuOk() ? 2 : 0) | (s.lidarOk() ? 4 : 0) | (d.camPaired ? 8 : 0) | (d.manual ? 16 : 0) |
            (d.hold ? 32 : 0) | (d.slope ? 64 : 0) | (d.snapOk ? 128 : 0);
  r.yawX100 = toI16(s.yawNowDeg() * 100); r.pitchX100 = toI16(s.pitchDeg() * 100); r.rollX100 = toI16(s.rollDeg() * 100);
  r.state = d.state; r.leg = d.leg; r.event = ev; r.pillars = d.pillars;
  r.cmdL = toI8(d.cmdL); r.cmdR = toI8(d.cmdR); r.outL = toI8(d.outL); r.outR = toI8(d.outR);
  r.rttX10ms = (uint8_t)constrain(lroundf(d.rttMs / 10.0f), 0L, 255L);
  r.batteryX100 = toU16(lroundf(d.batteryV * 100));
  for (uint8_t i = 0; i < 5; i++) r.usCm[i] = toU16(s.usCm(i));
  for (uint8_t i = 0; i < Sensors::kLidarPoints; i++) r.lidarCm[i] = s.lidarCm(i);
  r.tgtX100 = toI16(d.tgtDeg * 100); r.errX100 = toI16(d.errDeg * 100); r.crabX100 = toI16(d.crabDeg * 100);
  r.wallDcm = toU16(lroundf(d.wallD * 100)); r.wallAngX10 = toI16(d.wallAngDeg * 10);
  r.distCm = toU16(lroundf(d.distM * 100)); r.frontCm = toU16(lroundf(d.frontM * 100));
  r.camRows = d.camRows; r.camConf = d.camConf; r.camOffX10 = toI16(d.camOff * 10); r.camAngX10 = toI16(d.camAng * 10);
  r.snapsSent = d.snapsSent; r.snapsOk = d.snapsOk;
  r.loopMaxX4ms = (uint8_t)constrain(lroundf(d.loopMaxMs / 4.0f), 0L, 255L); r.dsStatus = d.dsStatus;
}

void MissionLog::_start() {
  int next = 1;
  fs::File root = LittleFS.open("/");
  for (fs::File f = root.openNextFile(); f; f = root.openNextFile()) {
    const String n = f.name();
    if (_validName(n)) next = max(next, (int)n.substring(4, 7).toInt() + 1);
  }
  root.close();
  if (next > 999) { strlcpy(_error, "999 files - delete some", sizeof(_error)); return; }
  if (LittleFS.totalBytes() - LittleFS.usedBytes() < 2 * kMinFreeBytes) { strlcpy(_error, "flash full - delete logs", sizeof(_error)); return; }
  snprintf(_fileName, sizeof(_fileName), "mis_%03d.bin", next);
  _file = LittleFS.open(String("/") + _fileName, "w");
  if (!_file) { strlcpy(_error, "cannot create file", sizeof(_error)); return; }
  Header h; memcpy(h.magic, "CASLOG", 6); h.version = 4; h.lidarPoints = Sensors::kLidarPoints;
  h.recordSize = sizeof(Rec); h.periodMs = kPeriodMs; h.startMs = millis();
  _file.write((const uint8_t*)&h, sizeof(h));
  char imgName[20]; snprintf(imgName, sizeof(imgName), "/img_%03d.bin", next);
  _imgFile = LittleFS.open(imgName, "w"); _imgN = 0;
  _error[0] = 0; _records = 0; _bufN = 0; _evTail = _evHead;
  _lastRecMs = millis() - kPeriodMs;
  _active = true;
  Serial.printf("[log] start %s (record %u bytes)\n", _fileName, (unsigned)sizeof(Rec));
}

void MissionLog::saveImage(const uint8_t* jpg, uint16_t len, uint16_t snapId, uint8_t ok, uint8_t rows, int16_t offX10, int16_t angX10) {
  if (!_active || !_imgFile || !len) return;
  if (LittleFS.totalBytes() - LittleFS.usedBytes() < 2 * kMinFreeBytes) return;    // keep room for the drive log
#pragma pack(push, 1)
  struct { char m[4]; uint32_t rec, ms; uint16_t id; uint8_t ok, rows; int16_t off, ang; uint16_t len; } h =
      {{'I', 'M', 'G', '1'}, (uint32_t)_records, millis(), snapId, ok, rows, offX10, angX10, len};
#pragma pack(pop)
  _imgFile.write((const uint8_t*)&h, sizeof(h));
  _imgFile.write(jpg, len);
  _imgFile.flush();
  _imgN++;
}

void MissionLog::_stop() {
  _flush();
  if (_file) _file.close();
  if (_imgFile) _imgFile.close();
  _active = false;
  _usedKB = LittleFS.usedBytes() / 1024;
  _refreshList();
  Serial.printf("[log] stop %s, %u records\n", _fileName, (unsigned)_records);
}

void MissionLog::_flush() {
  if (!_bufN || !_file) { _bufN = 0; return; }
  const size_t want = _bufN * sizeof(Rec), got = _file.write((const uint8_t*)_buf, want);
  _bufN = 0;
  _file.flush();
  _usedKB = LittleFS.usedBytes() / 1024;
  if (got != want) strlcpy(_error, "write failed (flash full?)", sizeof(_error));
  else if (LittleFS.totalBytes() - LittleFS.usedBytes() < kMinFreeBytes) strlcpy(_error, "flash full - log stopped", sizeof(_error));
  else return;
  _file.close(); _active = false; _refreshList();
  Serial.printf("[log] %s\n", _error);
}

void MissionLog::_clear() {
  std::vector<String> names;
  fs::File root = LittleFS.open("/");
  for (fs::File f = root.openNextFile(); f; f = root.openNextFile()) {
    const String n = f.name();
    if (_validName(n) || (n.startsWith("log_") && n.endsWith(".bin"))) names.push_back(n);   // also the team's old logs
  }
  root.close();
  for (const String& n : names) LittleFS.remove("/" + n);
  _error[0] = 0; _fileName[0] = 0;
  _usedKB = LittleFS.usedBytes() / 1024;
  _refreshList();
  Serial.printf("[log] deleted %u files\n", (unsigned)names.size());
}

void MissionLog::_refreshList() {      // loop task only (no flash access from the web task)
  String j; bool first = true;
  fs::File root = LittleFS.open("/");
  for (fs::File f = root.openNextFile(); f; f = root.openNextFile()) {
    const String n = f.name();
    if (!_validName(n)) continue;
    if (j.length() > sizeof(_filesJson) - 40) break;
    if (!first) j += ',';
    first = false;
    j += "[\""; j += n; j += "\","; j += (uint32_t)((f.size() + 1023) / 1024); j += ']';
  }
  root.close();
  portENTER_CRITICAL(&_mux); strlcpy(_filesJson, j.c_str(), sizeof(_filesJson)); portEXIT_CRITICAL(&_mux);
}

String MissionLog::statusJson() const {
  char files[sizeof(_filesJson)];
  portENTER_CRITICAL(&_mux); memcpy(files, _filesJson, sizeof(files)); portEXIT_CRITICAL(&_mux);
  String j; j.reserve(560);
  j += "{\"fs\":"; j += _fsOk ? 1 : 0;
  j += ",\"active\":"; j += _active ? 1 : 0;
  j += ",\"file\":\""; j += _fileName;
  j += "\",\"recs\":"; j += (uint32_t)_records;
  j += ",\"usedKB\":"; j += (uint32_t)_usedKB;
  j += ",\"totalKB\":"; j += (uint32_t)_totalKB;
  j += ",\"imgs\":"; j += _imgN;
  j += ",\"err\":\""; j += _error;
  j += "\",\"files\":["; j += files; j += "]}";
  return j;
}

bool MissionLog::_validName(const String& n) {   // mis_NNN.bin (drive log) or img_NNN.bin (camera pictures)
  if (n.length() != 11 || !(n.startsWith("mis_") || n.startsWith("img_")) || !n.endsWith(".bin")) return false;
  for (int i = 4; i < 7; i++) if (!isDigit(n[i])) return false;
  return true;
}
