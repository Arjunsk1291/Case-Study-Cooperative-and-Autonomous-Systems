// =============================================================================================
//  cam_strip_bench - AI-Thinker ESP32-CAM bench test: tactile strip detection, NO Wi-Fi (2026-11-06)
// =============================================================================================
//
//  WHY: first test of the camera with the smallest possible risk and load. The radio is never
//  started (no Wi-Fi, no Bluetooth), the flash LED (GPIO4) is held OFF. Everything goes over USB.
//
//  WHAT IT DOES, ~10 times per second:
//    1. grabs a 160 x 120 GRAYSCALE frame (the OV2640 delivers grayscale directly: no JPEG decode)
//    2. "row-scan edge" detector (a light-weight 1-D cousin of Canny, see below) on 4 rows in the
//       lower half of the image (the ground 0.5 - 2 m ahead)
//    3. prints one JSON line: strip offset, strip angle, confidence, and the timing / memory used
//
//  DETECTOR (per row): smooth the row (box filter 5 px) -> gradient g[x] = s[x+2] - s[x-2]
//    -> the strip is a DARK BAND: a strong falling edge (bright -> dark) followed, 6..60 px to the
//       right, by a strong rising edge (dark -> bright), and the inside is darker than both sides.
//    Edge thresholds adapt to the row (k x mean |gradient|), so it works in shade and in sun.
//    A straight line through the row centres gives the offset at the bottom and the angle.
//    Cost: 4 rows x 160 px - a few thousand operations, well under 1 ms. A full-frame Canny on
//    the same frame would need ~19 200 px x (blur + Sobel + NMS + hysteresis) and extra buffers.
//
//  SERIAL COMMANDS (115200 baud):
//    f   send ONE frame as text (header "FRAME 160 120", then 120 lines of hex) - to see where the
//        camera looks. The laptop turns it back into an image.
//    e   one frame + the on-chip "Canny-lite" edge image of it + timing (see edgeOnChip)
//    p   pause / resume the JSON lines
//
//  Board: "AI Thinker ESP32-CAM" (esp32:esp32:esp32cam), PSRAM enabled by the board definition.
// =============================================================================================

#include <Arduino.h>
#include "esp_camera.h"
#include <Wire.h>

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

static const int W = 160, H = 120;
static const int kRows[4] = {112, 100, 88, 76};   // lower half: ground nearest to the rover first
static const int kMinW = 18, kMaxW = 75;           // strip width in px (narrower = paving joints; tuned on the route video)
static const float kEdgeK = 2.5f;                  // edge must be k x the row's mean |gradient|
static const uint8_t kMinContrast = 15;            // band must be this much darker than its sides
static const int kBorder = 12;                     // centres closer to the image edge: shadows / kerbs, ignored
static const float kMaxResidPx = 10;               // the row centres must lie on one straight line

bool camOk = false, paused = false;
uint32_t seq = 0;

struct RowHit { bool ok; float x; int width; int contrast; int strength; };

// One row: find the best dark band. Uses only stack buffers (2 x 160 int16).
static RowHit scanRow(const uint8_t* img, int y) {
  RowHit h = {false, 0, 0, 0, 0};
  const uint8_t* p0 = img + (y - 1) * W;           // 3 image rows averaged (less texture)
  const uint8_t* p1 = img + y * W;
  const uint8_t* p2 = img + (y + 1) * W;
  int16_t s[W], g[W];
  int acc = 0;
  for (int x = 0; x < W; x++) {                    // box filter 5 px (running sum)
    acc += p0[x] + p1[x] + p2[x];
    if (x >= 5) acc -= p0[x - 5] + p1[x - 5] + p2[x - 5];
    s[x] = (x >= 4) ? acc / 15 : p1[x];
  }
  long sumAbs = 0;
  for (int x = 0; x < W; x++) {
    g[x] = (x >= 2 && x < W - 2) ? (int16_t)(s[x + 2] - s[x - 2]) : 0;
    sumAbs += abs(g[x]);
  }
  const int thr = max(6, (int)(kEdgeK * sumAbs / W));
  int best = 0;
  for (int a = 4; a < W - 4; a++) {
    if (g[a] > -thr) continue;                     // falling edge = left side of the dark band
    if (a > 4 && g[a - 1] < g[a]) continue;        // local minimum only
    for (int b = a + kMinW; b < min(W - 4, a + kMaxW); b++) {
      if (g[b] < thr) continue;                    // rising edge = right side
      if (g[b + 1] > g[b]) continue;               // local maximum only
      const int inside = s[(a + b) / 2];
      const int left = s[max(0, a - 4)], right = s[min(W - 1, b + 4)];
      const int contrast = min(left, right) - inside;
      if (contrast < kMinContrast) continue;
      const int strength = min(-g[a], (int)g[b]);
      if (strength > best) { best = strength; h = {true, (a + b) / 2.0f, b - a, contrast, strength}; }
    }
  }
  return h;
}

static void sendFrame(const camera_fb_t* fb) {
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

// ---- diagnostics (2026-11-07): the R04 firmware reported "Camera probe failed (SCCB addr 0x30)" and
//      "psram=no". This start-up tells WHY: camera power-cycle, control-bus scan, retries.
static void scanSccb() {
  // The OV2640 answers on its control bus (SCCB = I2C, SDA 26 / SCL 27) only with a clock on XCLK.
  // Give it 10 MHz on GPIO0 for the scan (LEDC), then release everything for esp_camera_init().
  ledcAttach(XCLK_GPIO_NUM, 10000000, 1);
  ledcWrite(XCLK_GPIO_NUM, 1);
  delay(20);
  Wire.begin(SIOD_GPIO_NUM, SIOC_GPIO_NUM, 100000);
  int found = 0;
  for (uint8_t a = 1; a < 127; a++) {
    Wire.beginTransmission(a);
    if (Wire.endTransmission() == 0) { Serial.printf("[diag] control bus: device at 0x%02X%s\n", a, a == 0x30 ? " (OV2640 camera)" : ""); found++; }
  }
  if (!found) Serial.println("[diag] control bus: NO device answers (camera not powered, ribbon loose, or camera dead)");
  Wire.end();
  ledcDetach(XCLK_GPIO_NUM);
  pinMode(SIOD_GPIO_NUM, INPUT); pinMode(SIOC_GPIO_NUM, INPUT);
}

static bool initCamera(int xclkHz) {
  camera_config_t c = {};
  c.ledc_channel = LEDC_CHANNEL_0; c.ledc_timer = LEDC_TIMER_0;
  c.pin_d0 = Y2_GPIO_NUM; c.pin_d1 = Y3_GPIO_NUM; c.pin_d2 = Y4_GPIO_NUM; c.pin_d3 = Y5_GPIO_NUM;
  c.pin_d4 = Y6_GPIO_NUM; c.pin_d5 = Y7_GPIO_NUM; c.pin_d6 = Y8_GPIO_NUM; c.pin_d7 = Y9_GPIO_NUM;
  c.pin_xclk = XCLK_GPIO_NUM; c.pin_pclk = PCLK_GPIO_NUM; c.pin_vsync = VSYNC_GPIO_NUM;
  c.pin_href = HREF_GPIO_NUM; c.pin_sccb_sda = SIOD_GPIO_NUM; c.pin_sccb_scl = SIOC_GPIO_NUM;
  c.pin_pwdn = PWDN_GPIO_NUM; c.pin_reset = RESET_GPIO_NUM;
  c.xclk_freq_hz = xclkHz;
  c.pixel_format = PIXFORMAT_GRAYSCALE;            // 1 byte per pixel, no JPEG
  c.frame_size = FRAMESIZE_QQVGA;                  // 160 x 120 = 19 200 bytes per frame
  c.fb_count = psramFound() ? 2 : 1;               // without PSRAM: one frame in normal RAM (fits easily)
  c.fb_location = psramFound() ? CAMERA_FB_IN_PSRAM : CAMERA_FB_IN_DRAM;
  c.grab_mode = psramFound() ? CAMERA_GRAB_LATEST : CAMERA_GRAB_WHEN_EMPTY;
  const esp_err_t e = esp_camera_init(&c);
  Serial.printf("[diag] camera init at XCLK %d MHz: %s (0x%x), heap %u kB\n", xclkHz / 1000000,
                e == ESP_OK ? "OK" : "FAILED", (unsigned)e, (unsigned)(ESP.getFreeHeap() / 1024));
  if (e != ESP_OK) esp_camera_deinit();
  return e == ESP_OK;
}

// ---- 'e': full-frame "Canny-lite" ON THE ESP32 (2026-11-07), to measure what it really costs.
//   1 blur 3x3   2 Sobel gx, gy -> magnitude (|gx| + |gy|) / 4   3 thin edges (keep a pixel only if it is
//   the maximum across the edge: left/right or up/down)   4 two thresholds (strong = 3 x mean, weak =
//   half of that; a weak pixel counts if a strong one touches it). Integer math, 3 buffers of 19.2 kB.
static uint8_t* gBlur = nullptr; static uint8_t* gMag = nullptr; static uint8_t* gOut = nullptr;
static void edgeOnChip(const camera_fb_t* fb) {
  const int w = fb->width, h = fb->height, n = w * h;
  if (!gBlur) { gBlur = (uint8_t*)malloc(n); gMag = (uint8_t*)malloc(n); gOut = (uint8_t*)malloc(n); }
  if (!gBlur || !gMag || !gOut) { Serial.println("{\"err\":\"no RAM for edge buffers\"}"); return; }
  const uint8_t* im = fb->buf;
  const uint32_t t0 = micros();
  memset(gBlur, 0, n); memset(gMag, 0, n); memset(gOut, 0, n);
  for (int y = 1; y < h - 1; y++)                                    // 1 blur
    for (int x = 1; x < w - 1; x++) {
      const uint8_t* p = im + y * w + x;
      gBlur[y * w + x] = (p[-w - 1] + p[-w] + p[-w + 1] + p[-1] + p[0] + p[1] + p[w - 1] + p[w] + p[w + 1]) / 9;
    }
  const uint32_t t1 = micros();
  uint32_t sum = 0;
  for (int y = 2; y < h - 2; y++)                                    // 2 Sobel
    for (int x = 2; x < w - 2; x++) {
      const uint8_t* p = gBlur + y * w + x;
      const int gx = (p[-w + 1] + 2 * p[1] + p[w + 1]) - (p[-w - 1] + 2 * p[-1] + p[w - 1]);
      const int gy = (p[w - 1] + 2 * p[w] + p[w + 1]) - (p[-w - 1] + 2 * p[-w] + p[-w + 1]);
      int m = (abs(gx) + abs(gy)) / 4; if (m > 127) m = 127;
      gMag[y * w + x] = (uint8_t)(m | (abs(gx) >= abs(gy) ? 0x80 : 0));   // bit 7: edge runs up/down
      sum += m;
    }
  const uint32_t t2 = micros();
  const int mean = sum / ((w - 4) * (h - 4));
  const int hi = max(12, 3 * mean), lo = hi / 2;
  for (int y = 3; y < h - 3; y++)                                    // 3 thin + 4 thresholds (strong)
    for (int x = 3; x < w - 3; x++) {
      const int i = y * w + x, m = gMag[i] & 127;
      if (m < lo) continue;
      const bool vert = gMag[i] & 0x80;
      const int a = gMag[vert ? i - 1 : i - w] & 127, b = gMag[vert ? i + 1 : i + w] & 127;
      if (m < a || m < b) continue;
      gOut[i] = m >= hi ? 255 : 128;
    }
  int edges = 0;
  for (int y = 3; y < h - 3; y++)                                    // weak kept only next to strong
    for (int x = 3; x < w - 3; x++) {
      const int i = y * w + x;
      if (gOut[i] == 128) {
        const bool touch = gOut[i - 1] == 255 || gOut[i + 1] == 255 || gOut[i - w] == 255 || gOut[i + w] == 255 ||
                           gOut[i - w - 1] == 255 || gOut[i - w + 1] == 255 || gOut[i + w - 1] == 255 || gOut[i + w + 1] == 255;
        gOut[i] = touch ? 255 : 0;
      }
      if (gOut[i] == 255) edges++;
    }
  const uint32_t t3 = micros();
  Serial.printf("{\"edge_on_chip\":1,\"blur_us\":%u,\"sobel_us\":%u,\"thin_threshold_us\":%u,\"total_us\":%u,\"edge_px\":%d,\"mean_grad\":%d}\n",
                (unsigned)(t1 - t0), (unsigned)(t2 - t1), (unsigned)(t3 - t2), (unsigned)(t3 - t0), edges, mean);
  camera_fb_t e = *fb; e.buf = gOut;                                 // send the edge image like a frame
  sendFrame(&e);
}

void setup() {
  pinMode(FLASH_LED_GPIO, OUTPUT);
  digitalWrite(FLASH_LED_GPIO, LOW);               // flash LED off (it is very bright and draws current)
  Serial.begin(115200);
  delay(300);
  Serial.printf("\n[cam] strip bench + diagnostics, NO Wi-Fi. reset reason %d (1=power on, 15=brownout)\n",
                (int)esp_reset_reason());
  Serial.printf("[diag] PSRAM: %s, size %u kB | heap %u kB | flash %u kB | CPU %u MHz\n",
                psramFound() ? "FOUND" : "NOT FOUND", (unsigned)(ESP.getPsramSize() / 1024),
                (unsigned)(ESP.getFreeHeap() / 1024), (unsigned)(ESP.getFlashChipSize() / 1024),
                (unsigned)getCpuFrequencyMhz());

  pinMode(PWDN_GPIO_NUM, OUTPUT);                  // camera power-down pin: off 200 ms, then on
  digitalWrite(PWDN_GPIO_NUM, HIGH);
  delay(200);
  digitalWrite(PWDN_GPIO_NUM, LOW);
  delay(200);
  scanSccb();

  camOk = initCamera(20000000);
  if (!camOk) { delay(300); camOk = initCamera(10000000); }   // a weak supply / long ribbon sometimes needs a slower clock
  if (camOk) {
    sensor_t* sn = esp_camera_sensor_get();
    if (sn) Serial.printf("[diag] sensor id: PID 0x%02X (0x26 = OV2640, 0x36 = OV3660, 0x56 = OV5640)\n", sn->id.PID);
    for (int i = 0; i < 4; i++) { camera_fb_t* fb = esp_camera_fb_get(); if (fb) esp_camera_fb_return(fb); delay(60); }
    Serial.println("[diag] camera READY - send 'f' for a frame");
  } else {
    Serial.println("[diag] camera NOT available. Likely: supply too weak / wrong voltage, ribbon not seated, or camera module faulty.");
  }
}

void loop() {
  while (Serial.available()) {
    const char ch = Serial.read();
    if (ch == 'p') paused = !paused;
    if (ch == 'f' && camOk) {
      camera_fb_t* fb = esp_camera_fb_get();
      if (fb) { sendFrame(fb); esp_camera_fb_return(fb); }
    }
    if (ch == 'e' && camOk) {                    // raw frame, then the on-chip edge image of the SAME frame
      camera_fb_t* fb = esp_camera_fb_get();
      if (fb) { sendFrame(fb); edgeOnChip(fb); esp_camera_fb_return(fb); }
    }
  }
  if (!camOk) {
    static uint32_t last = 0;
    if (millis() - last > 3000) { last = millis(); Serial.println("{\"err\":\"camera init failed\"}"); }
    delay(10);
    return;
  }
  static uint32_t lastMs = 0;
  if (millis() - lastMs < 100) { delay(2); return; }   // ~10 per second
  lastMs = millis();

  const uint32_t t0 = micros();
  camera_fb_t* fb = esp_camera_fb_get();
  const uint32_t t1 = micros();
  if (!fb) { Serial.println("{\"err\":\"no frame\"}"); return; }
  RowHit hit[4];
  int n = 0; float sx = 0, sy = 0, sxx = 0, sxy = 0; long mean = 0;
  for (int i = 0; i < 4; i++) {
    hit[i] = scanRow(fb->buf, kRows[i]);
    if (hit[i].ok && (hit[i].x < kBorder || hit[i].x > W - kBorder)) hit[i].ok = false;
    if (hit[i].ok) { const float y = kRows[i]; n++; sx += y; sy += hit[i].x; sxx += y * y; sxy += y * hit[i].x; }
  }
  for (int i = 0; i < W * H; i += 97) mean += fb->buf[i];     // rough brightness (exposure check)
  mean /= (W * H + 96) / 97;
  esp_camera_fb_return(fb);
  const uint32_t t2 = micros();

  float offPx = 0, angDeg = 0, resid = 99;
  bool lineOk = false;
  if (n >= 3) {                                    // at least 3 of 4 rows, on one straight line
    const float den = n * sxx - sx * sx;
    const float b = den > 1e-3f ? (n * sxy - sx * sy) / den : 0;    // x = a + b * y
    const float a = (sy - b * sx) / n;
    resid = 0;
    for (int i = 0; i < 4; i++) if (hit[i].ok) resid = fmaxf(resid, fabsf(hit[i].x - (a + b * kRows[i])));
    lineOk = resid <= kMaxResidPx;
    offPx = (a + b * (H - 1)) - W / 2.0f;                          // + = strip right of centre
    angDeg = -atanf(b) * RAD_TO_DEG;                               // + = strip leans to the right ahead
  }
  static uint8_t histBits = 0;                     // last 3 frames: line found?
  histBits = ((histBits << 1) | (lineOk ? 1 : 0)) & 7;
  const bool stripOk = lineOk && __builtin_popcount(histBits) >= 2;   // seen in 2 of the last 3 frames
  const float conf = stripOk ? n / 4.0f : 0.0f;
  seq++;
  if (paused) return;
  Serial.printf("{\"seq\":%u,\"ok\":%d,\"rows\":%d,\"resid\":%.1f,\"off_px\":%.1f,\"ang_deg\":%.1f,\"conf\":%.2f,\"w\":[%d,%d,%d,%d],"
                "\"bright\":%ld,\"t_cap_us\":%u,\"t_proc_us\":%u,\"heap_kB\":%u,\"psram_kB\":%u}\n",
                (unsigned)seq, stripOk ? 1 : 0, n, resid, offPx, angDeg, conf, hit[0].width, hit[1].width, hit[2].width, hit[3].width,
                mean, (unsigned)(t1 - t0), (unsigned)(t2 - t1), (unsigned)(ESP.getFreeHeap() / 1024),
                (unsigned)(ESP.getFreePsram() / 1024));
}
