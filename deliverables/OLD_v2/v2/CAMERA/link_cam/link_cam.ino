// =============================================================================================
//  link_cam - ESP32-CAM side of the camera -> box link test (2026-11-07). Board: AI Thinker ESP32-CAM.
// =============================================================================================
//  - finds the box by itself (HELLO broadcast on channel 11, the box answers ACK), then sends the strip
//    result to the box only (DATA ~30 bytes): 10 per second, or - rate 0 - ONE per request (MSG_REQ),
//    answers the box's PINGs (round-trip time). Each picture: grab, 4-row detector, buffer handed back.
//  - no images over the air, no access point, no connection to the box's Wi-Fi: ESP-NOW only
//  - transmit power at the minimum (~2 dBm): the box is centimetres away
//  - camera image is ROTATED 90 deg in the box: the detector reads columns of the sensor image = rows of
//    the upright image (rotated 90 deg clockwise: 120 px wide, 160 px tall, the floor is at the bottom)
//  - fail-safe: box silent for 3 s -> search again; frame grab fails 5x -> camera power-cycle + re-init
//  - USB serial (115200): status every 2 s; 'f' = one raw frame as text (debug only)
// =============================================================================================
#include <Arduino.h>
#include <WiFi.h>
#include <esp_wifi.h>
#include <esp_now.h>
#include "esp_camera.h"
#include "img_converters.h"   // frame2jpg: the snapshot picture for the box (logged, shown on the phone)
#include "LinkProto.h"

// AI-Thinker pin map
#define PWDN_GPIO_NUM 32
#define RESET_GPIO_NUM -1
#define XCLK_GPIO_NUM 0
#define SIOD_GPIO_NUM 26
#define SIOC_GPIO_NUM 27
#define Y9_GPIO_NUM 35
#define Y8_GPIO_NUM 34
#define Y7_GPIO_NUM 39
#define Y6_GPIO_NUM 36
#define Y5_GPIO_NUM 21
#define Y4_GPIO_NUM 19
#define Y3_GPIO_NUM 18
#define Y2_GPIO_NUM 5
#define VSYNC_GPIO_NUM 25
#define HREF_GPIO_NUM 23
#define PCLK_GPIO_NUM 22
#define FLASH_LED_GPIO 4

static const uint8_t kBroadcast[6] = {0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF};
static const int SW = 160, SH = 120;          // sensor image (sideways)
static const int VW = 120;                    // upright image width = sensor height
static const int kVRows[4] = {152, 140, 128, 116};   // upright rows near the bottom = ground closest first

// ---- link state (callback -> loop)
static volatile bool gGotAck = false, gGotPing = false, gGotCfg = false, gGotReq = false;
static volatile uint32_t gPingBoxUs = 0, gLastBoxMs = 0;
static volatile uint8_t gRate = 10, gMode = 0, gNewRate = 10, gNewMode = 0;
static uint8_t gBoxMac[6];
static volatile bool gBoxMacNew = false;
static volatile bool paired = false;
static uint32_t seqNo[8] = {0};             // one numbering per message type (box counts DATA gaps)

struct Hit { bool ok; float x; int w; };
static uint8_t* jpg = nullptr; static size_t jpgLen = 0;   // JPEG of the current on-demand snapshot    // (up here: Arduino puts function prototypes before the first function)

// ---- camera state
static bool camOk = false;
static uint16_t camResets = 0;
static uint8_t grabFails = 0;

static void onRecv(const esp_now_recv_info_t* info, const uint8_t* d, int len) {
  const uint8_t t = linkType(d, len);
  if (t == MSG_ACK && linkValid(d, len, MSG_ACK, sizeof(MsgAck))) {
    const MsgAck* a = (const MsgAck*)d;
    memcpy(gBoxMac, info->src_addr, 6);
    gNewRate = a->rateHz; gNewMode = a->mode;
    gBoxMacNew = true; gGotAck = true; gLastBoxMs = millis();
  } else if (paired && memcmp(info->src_addr, gBoxMac, 6) == 0) {   // only from our box
    if (t == MSG_PING && linkValid(d, len, MSG_PING, sizeof(MsgPing))) {
      gPingBoxUs = ((const MsgPing*)d)->boxUs; gGotPing = true; gLastBoxMs = millis();
    } else if (t == MSG_CFG && linkValid(d, len, MSG_CFG, sizeof(MsgCfg))) {
      gNewRate = ((const MsgCfg*)d)->rateHz; gNewMode = ((const MsgCfg*)d)->mode; gGotCfg = true; gLastBoxMs = millis();
    } else if (t == MSG_REQ && linkValid(d, len, MSG_REQ, sizeof(MsgReq))) {   // on demand: one snapshot
      gNewMode = ((const MsgReq*)d)->mode; gGotReq = true; gLastBoxMs = millis();
    }
  }
}

static void fillHdr(MsgHdr& h, uint8_t type) { h.magic = kMagic; h.version = kVersion; h.type = type; h.seq = ++seqNo[type & 7]; }

// ---------------- camera ----------------
static bool camInit(int xclk) {
  camera_config_t c = {};
  c.ledc_channel = LEDC_CHANNEL_0; c.ledc_timer = LEDC_TIMER_0;
  c.pin_d0 = Y2_GPIO_NUM; c.pin_d1 = Y3_GPIO_NUM; c.pin_d2 = Y4_GPIO_NUM; c.pin_d3 = Y5_GPIO_NUM;
  c.pin_d4 = Y6_GPIO_NUM; c.pin_d5 = Y7_GPIO_NUM; c.pin_d6 = Y8_GPIO_NUM; c.pin_d7 = Y9_GPIO_NUM;
  c.pin_xclk = XCLK_GPIO_NUM; c.pin_pclk = PCLK_GPIO_NUM; c.pin_vsync = VSYNC_GPIO_NUM;
  c.pin_href = HREF_GPIO_NUM; c.pin_sccb_sda = SIOD_GPIO_NUM; c.pin_sccb_scl = SIOC_GPIO_NUM;
  c.pin_pwdn = PWDN_GPIO_NUM; c.pin_reset = RESET_GPIO_NUM;
  c.xclk_freq_hz = xclk;
  c.pixel_format = PIXFORMAT_GRAYSCALE;
  c.frame_size = FRAMESIZE_QQVGA;
  c.fb_count = psramFound() ? 2 : 1;
  c.fb_location = psramFound() ? CAMERA_FB_IN_PSRAM : CAMERA_FB_IN_DRAM;
  c.grab_mode = psramFound() ? CAMERA_GRAB_LATEST : CAMERA_GRAB_WHEN_EMPTY;
  const esp_err_t e = esp_camera_init(&c);
  if (e != ESP_OK) esp_camera_deinit();
  return e == ESP_OK;
}

// power-cycle the sensor (PWDN) and start it; up to 3 attempts, 20 then 10 MHz
static bool camStart() {
  for (int k = 0; k < 3; k++) {
    pinMode(PWDN_GPIO_NUM, OUTPUT);
    digitalWrite(PWDN_GPIO_NUM, HIGH); delay(200);
    digitalWrite(PWDN_GPIO_NUM, LOW); delay(300);
    if (camInit(k == 0 ? 20000000 : 10000000)) {
      for (int i = 0; i < 4; i++) { camera_fb_t* fb = esp_camera_fb_get(); if (fb) esp_camera_fb_return(fb); delay(60); }
      Serial.printf("[cam] camera OK (attempt %d)\n", k + 1);
      return true;
    }
    Serial.printf("[cam] camera start attempt %d failed\n", k + 1);
    delay(500);
  }
  return false;
}

// ---------------- strip detector on the UPRIGHT image ----------------
// upright (rotated 90 deg clockwise) pixel (x, y), x 0..119, y 0..159  =  sensor pixel (row 119 - x, col y)
static inline uint8_t up(const uint8_t* s, int x, int y) { return s[(SH - 1 - x) * SW + y]; }

static Hit scanRow(const uint8_t* img, int y, int kMinW, int kMaxW) {
  Hit h = {false, 0, 0};
  int16_t s[VW], g[VW];
  int acc = 0;
  for (int x = 0; x < VW; x++) {                       // 3 rows averaged + box filter 5 px
    acc += up(img, x, y - 1) + up(img, x, y) + up(img, x, y + 1);
    if (x >= 5) acc -= up(img, x - 5, y - 1) + up(img, x - 5, y) + up(img, x - 5, y + 1);
    s[x] = (x >= 4) ? acc / 15 : up(img, x, y);
  }
  long sumAbs = 0;
  for (int x = 0; x < VW; x++) { g[x] = (x >= 2 && x < VW - 2) ? (int16_t)(s[x + 2] - s[x - 2]) : 0; sumAbs += abs(g[x]); }
  const int thr = max(6, (int)(2.5f * sumAbs / VW));
  int best = 0;
  for (int a = 4; a < VW - 4; a++) {
    if (g[a] > -thr || g[a - 1] < g[a]) continue;
    for (int b = a + kMinW; b < min(VW - 4, a + kMaxW); b++) {
      if (g[b] < thr || g[b + 1] > g[b]) continue;
      const int contrast = min(s[max(0, a - 3)], s[min(VW - 1, b + 3)]) - s[(a + b) / 2];
      if (contrast < 15) continue;
      const int st = min(-g[a], (int)g[b]);
      if (st > best) { best = st; h = {true, (a + b) / 2.0f, b - a}; }
    }
  }
  if (h.ok && (h.x < 8 || h.x > VW - 8)) h.ok = false;   // at the image edge: shadows / kerbs
  return h;
}

static void sendFrameText(const camera_fb_t* fb) {
  Serial.printf("FRAME %d %d\n", fb->width, fb->height);
  static const char* hex = "0123456789ABCDEF";
  char line[2 * 160 + 1];
  for (int y = 0; y < (int)fb->height; y++) {
    const uint8_t* p = fb->buf + y * fb->width;
    for (int x = 0; x < (int)fb->width; x++) { line[2 * x] = hex[p[x] >> 4]; line[2 * x + 1] = hex[p[x] & 15]; }
    line[2 * fb->width] = 0;
    Serial.println(line);
  }
  Serial.println("END");
}

void setup() {
  pinMode(FLASH_LED_GPIO, OUTPUT);
  digitalWrite(FLASH_LED_GPIO, LOW);
  Serial.begin(115200);
  delay(300);
  Serial.printf("\n[cam] link_cam, reset reason %d, PSRAM %s\n", (int)esp_reset_reason(), psramFound() ? "yes" : "NO");
  camOk = camStart();                                   // camera first (before the radio: smaller current peak)

  WiFi.mode(WIFI_STA);                                  // radio for ESP-NOW only, no connection
  WiFi.disconnect();
  esp_wifi_set_ps(WIFI_PS_NONE);                        // always listening: PINGs answered at once
  esp_wifi_set_channel(kLinkChannel, WIFI_SECOND_CHAN_NONE);
  esp_wifi_set_max_tx_power(8);                         // 8 x 0.25 dBm = 2 dBm (minimum useful)
  if (esp_now_init() != ESP_OK) Serial.println("[link] ESP-NOW init FAILED");
  esp_now_register_recv_cb(onRecv);
  esp_now_peer_info_t p = {};
  memcpy(p.peer_addr, kBroadcast, 6); p.channel = kLinkChannel; p.ifidx = WIFI_IF_STA; p.encrypt = false;
  esp_now_add_peer(&p);
  Serial.printf("[link] my MAC %s, channel %d, searching for the box\n", WiFi.macAddress().c_str(), kLinkChannel);
}

void loop() {
  const uint32_t now = millis();

  // ---- link events from the callback
  if (gBoxMacNew) {
    gBoxMacNew = false;
    if (!esp_now_is_peer_exist(gBoxMac)) {
      esp_now_peer_info_t p = {};
      memcpy(p.peer_addr, gBoxMac, 6); p.channel = kLinkChannel; p.ifidx = WIFI_IF_STA; p.encrypt = false;
      esp_now_add_peer(&p);
    }
    if (!paired) Serial.printf("[link] paired with box %02X:%02X:%02X:%02X:%02X:%02X\n",
                               gBoxMac[0], gBoxMac[1], gBoxMac[2], gBoxMac[3], gBoxMac[4], gBoxMac[5]);
    paired = true;
  }
  if (gGotAck || gGotCfg) {
    gGotAck = gGotCfg = false;
    gRate = constrain((int)gNewRate, 0, 50); gMode = gNewMode;   // 0 = on demand (no stream)
  }
  if (gGotPing && paired) {                             // answer the round-trip test at once
    gGotPing = false;
    MsgPong m; fillHdr(m.h, MSG_PONG); m.boxUs = gPingBoxUs; m.camMs = now;
    esp_now_send(gBoxMac, (const uint8_t*)&m, sizeof(m));
  }
  if (paired && now - gLastBoxMs > kLostMs) {           // fail-safe: box gone -> search again
    paired = false;
    if (esp_now_is_peer_exist(gBoxMac)) esp_now_del_peer(gBoxMac);
    Serial.println("[link] box silent for 3 s - searching again");
  }
  static uint32_t lastHello = 0;
  if (!paired && now - lastHello >= 1000) {
    lastHello = now;
    MsgHello m; fillHdr(m.h, MSG_HELLO); m.upMs = now; m.camOk = camOk;
    esp_now_send(kBroadcast, (const uint8_t*)&m, sizeof(m));
  }

  while (Serial.available()) {
    if (Serial.read() == 'f' && camOk) { camera_fb_t* fb = esp_camera_fb_get(); if (fb) { sendFrameText(fb); esp_camera_fb_return(fb); } }
  }

  // ---- one frame -> one DATA message, gRate per second
  static uint32_t lastData = 0;
  const bool streamDue = gRate > 0 && now - lastData >= 1000u / gRate;
  const bool reqDue = gGotReq && paired;
  if (streamDue || reqDue) {
    lastData = now;
    if (reqDue) { gGotReq = false; gMode = gNewMode; }
    MsgData m = {};
    fillHdr(m.h, MSG_DATA);
    m.camMs = now; m.camResets = camResets;
    if (camOk) {
      const uint32_t t0 = micros();
      camera_fb_t* fb = esp_camera_fb_get();
      const uint32_t t1 = micros();
      if (!fb) {
        if (++grabFails >= 5) {                          // watchdog: sensor stopped -> restart it
          Serial.println("[cam] 5 grabs failed - restarting the camera");
          esp_camera_deinit(); camResets++; grabFails = 0;
          camOk = camStart();
        }
      } else {
        grabFails = 0;
        // 2026-11-07 (campus run): wide strip may cover most of the 120 px image near the rover -> up to 100 px
        const int kMinW = gMode ? 3 : 12, kMaxW = gMode ? 15 : 100;
        Hit hit[4]; int n = 0; float sx = 0, sy = 0, sxx = 0, sxy = 0;
        for (int i = 0; i < 4; i++) {
          hit[i] = scanRow(fb->buf, kVRows[i], kMinW, kMaxW);
          if (hit[i].ok) { const float y = kVRows[i]; n++; sx += y; sy += hit[i].x; sxx += y * y; sxy += y * hit[i].x; }
        }
        long mean = 0;
        for (int i = 0; i < SW * SH; i += 97) mean += fb->buf[i];
        mean /= (SW * SH + 96) / 97;
        if (reqDue) {                                    // on demand: keep the picture as JPEG for the box
          if (jpg) { free(jpg); jpg = nullptr; }
          if (!frame2jpg(fb, 50, &jpg, &jpgLen)) { jpg = nullptr; jpgLen = 0; }
        }
        esp_camera_fb_return(fb);                        // frame buffer handed back at once
        bool lineOk = false; float off = 0, ang = 0;
        // on demand each picture is judged ALONE (the box votes over its pictures): 2 rows are enough;
        // the stream keeps 3 rows + "2 of the last 3 frames"
        if (n >= (gRate == 0 ? 2 : 3)) {
          const float den = n * sxx - sx * sx, b = den > 1e-3f ? (n * sxy - sx * sy) / den : 0, a = (sy - b * sx) / n;
          float resid = 0;
          for (int i = 0; i < 4; i++) if (hit[i].ok) resid = fmaxf(resid, fabsf(hit[i].x - (a + b * kVRows[i])));
          lineOk = resid <= 8;
          off = (a + b * 159) - VW / 2.0f;
          ang = -atanf(b) * RAD_TO_DEG;
        }
        static uint8_t hist = 0;
        hist = ((hist << 1) | (lineOk ? 1 : 0)) & 7;
        m.camOk = 1;
        m.stripOk = gRate == 0 ? lineOk : (lineOk && __builtin_popcount(hist) >= 2);
        m.rows = n; m.conf = m.stripOk ? n * 25 : 0;
        m.offX10 = (int16_t)lroundf(off * 10); m.angX10 = (int16_t)lroundf(ang * 10);
        m.procUs = (uint16_t)min<uint32_t>(65535, micros() - t1); m.capUs = (uint16_t)min<uint32_t>(65535, t1 - t0);
        m.bright = (uint8_t)mean;
      }
    }
    if (paired) esp_now_send(gBoxMac, (const uint8_t*)&m, sizeof(m));
    // the picture: JPEG (~2-4 kB) in 200-byte chunks right after its result. snapId = this DATA's number.
    if (reqDue && paired && jpg && jpgLen > 0 && jpgLen < 16000) {
      MsgImg c;
      for (uint16_t off = 0; off < jpgLen; off += kImgChunk) {
        fillHdr(c.h, MSG_IMG);
        c.snapId = (uint16_t)m.h.seq; c.total = (uint16_t)jpgLen; c.off = off;
        c.len = (uint8_t)min<size_t>(kImgChunk, jpgLen - off);
        memcpy(c.d, jpg + off, c.len);
        for (int tries = 0; tries < 5 && esp_now_send(gBoxMac, (const uint8_t*)&c, sizeof(c)) != ESP_OK; tries++) delay(3);
        delay(4);                                        // do not flood the radio (~15 chunks = ~60 ms)
      }
    }
    if (jpg) { free(jpg); jpg = nullptr; jpgLen = 0; }   // nothing piles up: one picture at a time
  }

  static uint32_t lastLog = 0;
  if (now - lastLog >= 2000) {
    lastLog = now;
    Serial.printf("[cam] %s, camera %s (resets %u), rate %u Hz (0 = on demand), mode %s, heap %u kB\n", paired ? "PAIRED" : "searching",
                  camOk ? "ok" : "FAILED", camResets, gRate, gMode ? "belt" : "tactile", (unsigned)(ESP.getFreeHeap() / 1024));
  }
  delay(1);
}
