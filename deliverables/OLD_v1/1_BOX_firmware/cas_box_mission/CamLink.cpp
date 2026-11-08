#include "CamLink.h"
#include <WiFi.h>
#include <esp_wifi.h>
#include <esp_now.h>

CamLink camLink;

// ---- receive queue: the ESP-NOW callback (Wi-Fi task) only copies, loop() handles
struct Rx { uint8_t mac[6]; uint8_t len; int8_t rssi; uint32_t us; uint8_t d[40]; };
static const int kQ = 32;
static Rx q[kQ];
static volatile uint16_t qHead = 0, qTail = 0;
static volatile uint32_t qDrop = 0;
static portMUX_TYPE qMux = portMUX_INITIALIZER_UNLOCKED;

static void onRecv(const esp_now_recv_info_t* info, const uint8_t* d, int len) {
  if (len <= 0 || len > 40) return;
  portENTER_CRITICAL_ISR(&qMux);
  const uint16_t next = (qHead + 1) % kQ;
  if (next == qTail) { qDrop++; portEXIT_CRITICAL_ISR(&qMux); return; }
  Rx& r = q[qHead];
  memcpy(r.mac, info->src_addr, 6); r.len = len; r.rssi = info->rx_ctrl ? info->rx_ctrl->rssi : 0; r.us = micros();
  memcpy(r.d, d, len);
  qHead = next;
  portEXIT_CRITICAL_ISR(&qMux);
}

bool CamLink::begin(AsyncWebServer* server) {
  if (esp_now_init() != ESP_OK) { Serial.println("[cam] ESP-NOW init FAILED"); return false; }
  esp_now_register_recv_cb(onRecv);
  Serial.printf("[cam] ESP-NOW ready on channel %d, waiting for the camera\n", WiFi.channel());
  _winStartMs = _lastTickMs = millis();
  if (server) {
    server->on("/cam/status", HTTP_GET, [this](AsyncWebServerRequest* req) { req->send(200, "application/json", statusJson()); });
    server->on("/cam/mode", HTTP_GET, [this](AsyncWebServerRequest* req) {
      _reqMode = (req->hasParam("m") && req->getParam("m")->value() == "1") ? 1 : 0;
      req->send(200, "application/json", "{\"ok\":true}");
    });
  }
  return true;
}

void CamLink::_send(const void* m, size_t n) { if (_paired) esp_now_send(_mac, (const uint8_t*)m, n); }

bool CamLink::requestSnap(uint8_t mode) {
  _snapReqMs = millis(); _snapsSent++;
  if (!_paired) { _snap = SNAP_FAIL; return false; }
  MsgReq q; q.h = {kMagic, kVersion, MSG_REQ, ++_seq}; q.mode = mode ? 1 : 0;
  _snap = SNAP_WAIT;
  _send(&q, sizeof(q));
  return true;
}

void CamLink::setMode(uint8_t m) {
  _mode = m ? 1 : 0;
  MsgCfg c; c.h = {kMagic, kVersion, MSG_CFG, ++_seq}; c.rateHz = _rate; c.mode = _mode;
  _send(&c, sizeof(c));
}

bool CamLink::fresh() const {
  return _paired && _lastDataMs != 0 && millis() - _lastDataMs <= kStaleMs && _last.camOk;
}

void CamLink::_handle(const uint8_t* mac, const uint8_t* d, int len, int8_t rssi, uint32_t us) {
  const uint8_t t = linkType(d, len);
  if (t == MSG_HELLO && linkValid(d, len, MSG_HELLO, sizeof(MsgHello))) {
    if (_paired && memcmp(mac, _mac, 6) != 0) return;            // paired with another camera: ignore
    if (!_paired) {
      memcpy(_mac, mac, 6);
      esp_now_peer_info_t p = {};
      memcpy(p.peer_addr, _mac, 6); p.channel = 0; p.ifidx = WIFI_IF_AP; p.encrypt = false;
      if (!esp_now_is_peer_exist(_mac)) esp_now_add_peer(&p);
      _paired = true;
      Serial.printf("[cam] paired with camera %02X:%02X:%02X:%02X:%02X:%02X\n", _mac[0], _mac[1], _mac[2], _mac[3], _mac[4], _mac[5]);
    }
    _lastSeq = 0;                                                  // HELLO = camera (re)started: numbering from 1
    MsgAck a; a.h = {kMagic, kVersion, MSG_ACK, ++_seq}; a.rateHz = _rate; a.mode = _mode;
    _send(&a, sizeof(a));
    return;
  }
  if (!_paired || memcmp(mac, _mac, 6) != 0) return;
  if (t == MSG_DATA && linkValid(d, len, MSG_DATA, sizeof(MsgData))) {
    const MsgData* m = (const MsgData*)d;
    if (_lastSeq && m->h.seq + 50 < _lastSeq) { _lastSeq = 0; _w.restarts++; }
    if (_lastSeq && m->h.seq <= _lastSeq) { _w.dup++; return; }
    if (_lastSeq && m->h.seq - _lastSeq < 1000) _w.lost += m->h.seq - _lastSeq - 1;
    _lastSeq = m->h.seq;
    if (_lastDataUs) { const uint32_t g = (us - _lastDataUs) / 1000; if (g > _w.gapMaxMs) _w.gapMaxMs = g; }
    _lastDataUs = us; _lastDataMs = millis();
    _w.rx++; _w.rssiSum += rssi; _w.rssiN++;
    _last = *m;
    if (_snap == SNAP_WAIT) { _snapRes = *m; _snap = SNAP_OK; _snapRtt = millis() - _snapReqMs; _snapsOk++; }
  } else if (t == MSG_PONG && linkValid(d, len, MSG_PONG, sizeof(MsgPong))) {
    _w.pongs++;
    const uint32_t rtt = us - ((const MsgPong*)d)->boxUs;
    if (rtt < 2000000) { _w.rttN++; _w.rttSumUs += rtt; _w.rttMinUs = min(_w.rttMinUs, rtt); _w.rttMaxUs = max(_w.rttMaxUs, rtt); }
  }
}

void CamLink::update() {
  const uint32_t now = millis();
  for (;;) {
    Rx r;
    portENTER_CRITICAL(&qMux);
    if (qTail == qHead) { portEXIT_CRITICAL(&qMux); break; }
    r = q[qTail]; qTail = (qTail + 1) % kQ;
    portEXIT_CRITICAL(&qMux);
    _handle(r.mac, r.d, r.len, r.rssi, r.us);
  }
  if (_reqMode != 255) { setMode(_reqMode); _reqMode = 255; }
  if (_snap == SNAP_WAIT && now - _snapReqMs > snapTimeoutMs) _snap = SNAP_FAIL;   // no answer: give up, never wait longer
  if (_paired && now - _lastPingMs >= 500) {                       // round-trip test, 2 per second
    _lastPingMs = now;
    MsgPing m; m.h = {kMagic, kVersion, MSG_PING, ++_seq}; m.boxUs = micros();
    _send(&m, sizeof(m)); _w.pings++;
  }
  if (!fresh()) _w.staleMs += now - _lastTickMs;
  _lastTickMs = now;

  if (now - _winStartMs >= 2000) {                                  // close the 2 s window
    const float win = (now - _winStartMs) / 1000.0f;
    _winStartMs = now;
    _done = _w; _w = Win(); _winNo++;
    Pub p;
    p.paired = _paired; p.fresh = fresh(); p.d = _last;
    p.rxPerS = _done.rx / win; p.lost = _done.lost; p.gapMaxMs = _done.gapMaxMs;
    p.rttAvgMs = _done.rttN ? _done.rttSumUs / 1000.0f / _done.rttN : 0;
    p.rttMaxMs = _done.rttN ? _done.rttMaxUs / 1000.0f : 0;
    p.rssi = _done.rssiN ? _done.rssiSum / _done.rssiN : 0;
    p.mode = _mode;
    portENTER_CRITICAL(&_mux); _pub = p; portEXIT_CRITICAL(&_mux);
  } else {                                                          // keep paired / fresh / latest result live
    portENTER_CRITICAL(&_mux); _pub.paired = _paired; _pub.fresh = fresh(); _pub.d = _last; portEXIT_CRITICAL(&_mux);
  }
}

String CamLink::statusJson() const {
  Pub p;
  portENTER_CRITICAL(&_mux); p = _pub; portEXIT_CRITICAL(&_mux);
  String j;
  j.reserve(320);
  j += "{\"paired\":"; j += p.paired ? 1 : 0;
  j += ",\"fresh\":"; j += p.fresh ? 1 : 0;
  j += ",\"camOk\":"; j += p.d.camOk;
  j += ",\"strip\":"; j += p.d.stripOk;
  j += ",\"rows\":"; j += p.d.rows;
  j += ",\"off\":"; j += String(p.d.offX10 / 10.0f, 1);
  j += ",\"ang\":"; j += String(p.d.angX10 / 10.0f, 1);
  j += ",\"bright\":"; j += p.d.bright;
  j += ",\"resets\":"; j += p.d.camResets;
  j += ",\"rx\":"; j += String(p.rxPerS, 1);
  j += ",\"lost\":"; j += p.lost;
  j += ",\"gap\":"; j += p.gapMaxMs;
  j += ",\"rtt\":"; j += String(p.rttAvgMs, 1);
  j += ",\"rttMax\":"; j += String(p.rttMaxMs, 1);
  j += ",\"rssi\":"; j += p.rssi;
  j += ",\"mode\":"; j += p.mode;
  j += ",\"qdrop\":"; j += (unsigned)qDrop;
  j += "}";
  return j;
}
