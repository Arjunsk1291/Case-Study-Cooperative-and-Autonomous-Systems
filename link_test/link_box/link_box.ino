// =============================================================================================
//  link_box - box side of the camera -> box link test (2026-11-07). Board: ESP32 Dev Module.
// =============================================================================================
//  - opens the box's normal access point (CAS-Box-04, channel 11, same transmit power steps as the team
//    program) so the test runs exactly in the situation of the real box: AP + ESP-NOW on one radio
//  - answers the camera's HELLO (pairs with the first camera, ignores others), PINGs it twice a second
//  - prints link statistics every 2 s as one JSON line: received per second, lost, round-trip time,
//    largest gap, signal strength, staleness, the camera's strip result
//  - NO sensors, NO LiDAR motor, NO rover link in this test (less current on laptop USB power)
//  - serial commands: 1 / 2 / 5 = 10 / 20 / 50 messages per second, t = tactile (wide) strip mode,
//    b = belt (narrow) mode, r = reset statistics, u = forget the camera (pair again)
// =============================================================================================
#include <Arduino.h>
#include <WiFi.h>
#include <esp_wifi.h>
#include <esp_now.h>
#include "LinkProto.h"

const int BOX_ID = 4;
#define WIFI_TX_START WIFI_POWER_8_5dBm
#define WIFI_TX_RUN WIFI_POWER_15dBm

// ---- receive queue (callback -> loop)
struct Rx { uint8_t mac[6]; uint8_t len; int8_t rssi; uint32_t us; uint8_t d[40]; };
static const int kQ = 32;
static Rx q[kQ];
static volatile uint16_t qHead = 0, qTail = 0, qDrop = 0;
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

static bool paired = false;
static uint8_t camMac[6];
static uint8_t rate = 10, mode = 0;
static uint32_t seqNo = 0;
static void fillHdr(MsgHdr& h, uint8_t type) { h.magic = kMagic; h.version = kVersion; h.type = type; h.seq = ++seqNo; }

// ---- statistics (one window = 2 s, plus totals)
struct Stats {
  uint32_t rx = 0, lost = 0, dup = 0, pings = 0, pongs = 0, staleMs = 0, restarts = 0;
  uint32_t gapMaxMs = 0; int rssiSum = 0, rssiN = 0;
  uint32_t rtt[64]; uint8_t nRtt = 0;
} W, Tot;
static uint32_t lastSeq = 0, lastDataMs = 0, lastDataUs = 0, staleSince = 0;
static MsgData last = {};
static uint32_t pingUs[8]; static uint32_t pingSeq[8];

static void resetWin() { W = Stats(); }

static void sendCfg() {
  if (!paired) return;
  MsgCfg m; fillHdr(m.h, MSG_CFG); m.rateHz = rate; m.mode = mode;
  esp_now_send(camMac, (const uint8_t*)&m, sizeof(m));
}

static void handle(const Rx& r, uint32_t nowMs) {
  const uint8_t t = linkType(r.d, r.len);
  if (t == MSG_HELLO && linkValid(r.d, r.len, MSG_HELLO, sizeof(MsgHello))) {
    if (paired && memcmp(r.mac, camMac, 6) != 0) return;          // already paired with another camera
    // 2026-11-07 fix: a HELLO from OUR camera = it restarted -> its numbering starts again at 1.
    // (Before: the box kept the old last number and dropped every new message as a "duplicate".)
    lastSeq = 0;
    if (!paired) {
      memcpy(camMac, r.mac, 6);
      esp_now_peer_info_t p = {};
      memcpy(p.peer_addr, camMac, 6); p.channel = 0; p.ifidx = WIFI_IF_AP; p.encrypt = false;
      if (!esp_now_is_peer_exist(camMac)) esp_now_add_peer(&p);
      paired = true; lastSeq = 0;
      Serial.printf("[link] paired with camera %02X:%02X:%02X:%02X:%02X:%02X (camera %s)\n", camMac[0], camMac[1], camMac[2],
                    camMac[3], camMac[4], camMac[5], ((const MsgHello*)r.d)->camOk ? "ok" : "FAILED");
    }
    MsgAck a; fillHdr(a.h, MSG_ACK); a.rateHz = rate; a.mode = mode;
    esp_now_send(camMac, (const uint8_t*)&a, sizeof(a));
    return;
  }
  if (!paired || memcmp(r.mac, camMac, 6) != 0) return;            // only our camera
  if (t == MSG_DATA && linkValid(r.d, r.len, MSG_DATA, sizeof(MsgData))) {
    const MsgData* m = (const MsgData*)r.d;
    if (lastSeq && m->h.seq + 50 < lastSeq) { lastSeq = 0; W.restarts++; }   // big jump back = camera restarted
    if (lastSeq && m->h.seq <= lastSeq) { W.dup++; return; }   // DATA has its own numbering on the camera
    if (lastSeq && m->h.seq - lastSeq < 1000) W.lost += m->h.seq - lastSeq - 1;   // (a big jump = camera restarted)
    lastSeq = m->h.seq;
    if (lastDataUs) { const uint32_t g = (r.us - lastDataUs) / 1000; if (g > W.gapMaxMs) W.gapMaxMs = g; }
    lastDataUs = r.us; lastDataMs = nowMs;
    W.rx++; W.rssiSum += r.rssi; W.rssiN++;
    last = *m;
  } else if (t == MSG_PONG && linkValid(r.d, r.len, MSG_PONG, sizeof(MsgPong))) {
    const MsgPong* m = (const MsgPong*)r.d;
    W.pongs++;
    const uint32_t rtt = r.us - m->boxUs;
    if (rtt < 2000000 && W.nRtt < 64) W.rtt[W.nRtt++] = rtt;
  }
}

void setup() {
  Serial.begin(115200);
  delay(200);
  Serial.printf("\n[box] link_box, reset reason %d\n", (int)esp_reset_reason());
  char ssid[16], pass[16];
  snprintf(ssid, sizeof(ssid), "CAS-Box-%02d", BOX_ID);
  snprintf(pass, sizeof(pass), "CASbox%02d", BOX_ID);
  WiFi.mode(WIFI_AP);
  WiFi.setTxPower(WIFI_TX_START);
  WiFi.softAPConfig(IPAddress(192, 168, 10 + BOX_ID, 1), IPAddress(192, 168, 10 + BOX_ID, 1), IPAddress(255, 255, 255, 0));
  WiFi.softAP(ssid, pass, kLinkChannel);
  WiFi.setSleep(false);
  WiFi.setTxPower(WIFI_TX_RUN);
  if (esp_now_init() != ESP_OK) Serial.println("[link] ESP-NOW init FAILED");
  esp_now_register_recv_cb(onRecv);
  Serial.printf("[link] AP %s on channel %d, AP MAC %s, waiting for the camera\n", ssid, WiFi.channel(), WiFi.softAPmacAddress().c_str());
}

void loop() {
  const uint32_t nowMs = millis();
  for (;;) {                                                        // drain the receive queue
    Rx r;
    portENTER_CRITICAL(&qMux);
    if (qTail == qHead) { portEXIT_CRITICAL(&qMux); break; }
    r = q[qTail]; qTail = (qTail + 1) % kQ;
    portEXIT_CRITICAL(&qMux);
    handle(r, nowMs);
  }

  static uint32_t lastPing = 0;
  if (paired && nowMs - lastPing >= 500) {                          // round-trip test, 2 per second
    lastPing = nowMs;
    MsgPing m; fillHdr(m.h, MSG_PING); m.boxUs = micros();
    if (esp_now_send(camMac, (const uint8_t*)&m, sizeof(m)) == ESP_OK) W.pings++;
  }

  const bool stale = !paired || lastDataMs == 0 || nowMs - lastDataMs > kStaleMs;   // fail-safe flag
  static uint32_t lastTick = 0;
  if (stale) W.staleMs += nowMs - lastTick;
  lastTick = nowMs;

  while (Serial.available()) {
    const char c = Serial.read();
    if (c == '1') { rate = 10; sendCfg(); }
    else if (c == '2') { rate = 20; sendCfg(); }
    else if (c == '5') { rate = 50; sendCfg(); }
    else if (c == 't') { mode = 0; sendCfg(); }
    else if (c == 'b') { mode = 1; sendCfg(); }
    else if (c == 'r') { resetWin(); Tot = Stats(); qDrop = 0; }
    else if (c == 'u') { if (paired && esp_now_is_peer_exist(camMac)) esp_now_del_peer(camMac); paired = false; lastSeq = 0; Serial.println("[link] camera forgotten"); }
  }

  static uint32_t lastRep = 0;
  if (nowMs - lastRep >= 2000) {
    const float win = (nowMs - lastRep) / 1000.0f;
    lastRep = nowMs;
    uint32_t s[64]; const int n = W.nRtt; memcpy(s, W.rtt, n * 4);
    for (int i = 1; i < n; i++) { uint32_t v = s[i]; int j = i - 1; while (j >= 0 && s[j] > v) { s[j + 1] = s[j]; j--; } s[j + 1] = v; }
    uint64_t sum = 0; for (int i = 0; i < n; i++) sum += s[i];
    const float lostPct = (W.rx + W.lost) ? 100.0f * W.lost / (W.rx + W.lost) : 0;
    Serial.printf("{\"t\":%.1f,\"paired\":%d,\"rate_set\":%u,\"mode\":\"%s\",\"rx_per_s\":%.1f,\"lost\":%u,\"lost_pct\":%.1f,\"dup\":%u,\"cam_restart\":%u,"
                  "\"gap_max_ms\":%u,\"stale_ms\":%u,\"pings\":%u,\"pongs\":%u,\"rtt_ms\":{\"min\":%.2f,\"avg\":%.2f,\"p95\":%.2f,\"max\":%.2f},"
                  "\"rssi\":%d,\"q_drop\":%u,\"cam\":{\"ok\":%u,\"resets\":%u,\"strip\":%u,\"rows\":%u,\"off_px\":%.1f,\"ang_deg\":%.1f,"
                  "\"proc_us\":%u,\"cap_us\":%u,\"bright\":%u},\"heap_kB\":%u,\"stations\":%d}\n",
                  nowMs / 1000.0f, paired, rate, mode ? "belt" : "tactile", W.rx / win, W.lost, lostPct, W.dup, W.restarts, W.gapMaxMs, W.staleMs,
                  W.pings, W.pongs, n ? s[0] / 1000.0f : 0, n ? sum / 1000.0f / n : 0, n ? s[(n * 95) / 100 < n ? (n * 95) / 100 : n - 1] / 1000.0f : 0,
                  n ? s[n - 1] / 1000.0f : 0, W.rssiN ? W.rssiSum / W.rssiN : 0, (unsigned)qDrop, last.camOk, last.camResets, last.stripOk,
                  last.rows, last.offX10 / 10.0f, last.angX10 / 10.0f, last.procUs, last.capUs, last.bright,
                  (unsigned)(ESP.getFreeHeap() / 1024), WiFi.softAPgetStationNum());
    Tot.rx += W.rx; Tot.lost += W.lost; Tot.pongs += W.pongs;
    resetWin();
  }
  delay(1);
}
