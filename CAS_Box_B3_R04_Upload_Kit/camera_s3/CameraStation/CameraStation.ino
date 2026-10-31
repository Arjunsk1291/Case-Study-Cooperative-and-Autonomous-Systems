/*
 * CAS Box B3 / Rover R04
 * Board: AI-Thinker ESP32-CAM (OV2640)
 *
 * Camera joins CAS-Box-04 as a Wi-Fi station. The main ESP32 is the only live hub;
 * it requests and relays bounded JPEG snapshots to the phone. The laptop is offline-only.
 *
 * The phone requests individual snapshots through the main ESP32. No laptop upload
 * or continuous camera upload runs on this board.
 *
 * Low-RAM configuration based on the measured board diagnostics:
 * QVGA 320x240, JPEG quality 12, 20 MHz XCLK, one frame buffer in internal DRAM.
 * PSRAM was reported as not detected. No image-processing buffers are allocated here.
 */

#include <Arduino.h>
#include <WiFi.h>
#include <HTTPClient.h>
#include <WebServer.h>
#include <esp_camera.h>
#include <esp_heap_caps.h>
#include "camera_user_config.h"

#if !defined(CAS_BOX_IP)
#error "Set CAS_BOX_IP in camera_user_config.h."
#endif

// AI-Thinker ESP32-CAM / OV2640 wiring — do not reuse camera GPIOs.
#define PWDN_GPIO_NUM     32
#define RESET_GPIO_NUM    -1
#define XCLK_GPIO_NUM      0
#define SIOD_GPIO_NUM     26
#define SIOC_GPIO_NUM     27
#define Y9_GPIO_NUM       35
#define Y8_GPIO_NUM       34
#define Y7_GPIO_NUM       39
#define Y6_GPIO_NUM       36
#define Y5_GPIO_NUM       21
#define Y4_GPIO_NUM       19
#define Y3_GPIO_NUM       18
#define Y2_GPIO_NUM        5
#define VSYNC_GPIO_NUM    25
#define HREF_GPIO_NUM     23
#define PCLK_GPIO_NUM     22

static constexpr uint32_t WIFI_RETRY_MS = 5000;
static constexpr uint32_t HEARTBEAT_MS = 1000;
static constexpr size_t MAX_JPEG_BYTES = 300 * 1024;
static WebServer cameraServer(80);
static bool cameraReady = false;

// camera_fb_t is normal RAM. WebServer::send_P() expects PROGMEM and corrupts
// RAM JPEGs; use the stream overload with an explicit byte count instead.
class FrameBufferStream : public Stream {
public:
  FrameBufferStream(const uint8_t* data, size_t length) : _data(data), _length(length) {}
  int available() override { return _position < _length ? (int)(_length - _position) : 0; }
  int read() override { return _position < _length ? _data[_position++] : -1; }
  int peek() override { return _position < _length ? _data[_position] : -1; }
  size_t write(uint8_t) override { return 0; }
  void flush() override {}
private:
  const uint8_t* _data;
  size_t _length;
  size_t _position = 0;
};

static void cameraSnapshot() {
  camera_fb_t* fb = esp_camera_fb_get();
  if (!fb || !fb->buf || fb->len < 4 || fb->len > MAX_JPEG_BYTES || fb->format != PIXFORMAT_JPEG) {
    if (fb) esp_camera_fb_return(fb);
    cameraServer.send(503, "application/json", "{\"ok\":false,\"reason\":\"capture_failed_or_too_large\"}");
    return;
  }
  cameraServer.sendHeader("X-Camera-Board-Id", CAS_CAMERA_BOARD_ID);
  cameraServer.sendHeader("X-Frame-Sequence", String(millis()));
  FrameBufferStream stream(fb->buf, fb->len);
  cameraServer.send(200, "image/jpeg", stream, fb->len);
  esp_camera_fb_return(fb);
}

static void cameraStatus() {
  String body = "{\"ok\":true,\"board_id\":\"" + String(CAS_CAMERA_BOARD_ID) +
    "\",\"firmware\":\"" + String(CAS_CAMERA_FIRMWARE_VERSION) +
    "\",\"camera_ready\":true,\"width\":320,\"height\":240,\"jpeg_quality\":12,\"free_heap\":" +
    String(ESP.getFreeHeap()) + "}";
  cameraServer.send(200, "application/json", body);
}

static void registerWithMainBox() {
  if (WiFi.status() != WL_CONNECTED) return;
  HTTPClient http; http.setTimeout(800);
  const String url = String("http://") + CAS_BOX_IP + "/camera/register?ip=" + WiFi.localIP().toString();
  if (http.begin(url)) { http.GET(); http.end(); }
}

/* Legacy laptop JSON/upload helpers removed: camera is a snapshot server only.
static String jsonString(const String& body, const char* key) {
  const String needle = String("\"") + key + "\":";
  int p = body.indexOf(needle);
  if (p < 0) return "";
  p += needle.length();
  while (p < (int)body.length() && body[p] == ' ') ++p;
  if (p >= (int)body.length() || body[p] != '"') return "";
  const int end = body.indexOf('"', p + 1);
  return end < 0 ? "" : body.substring(p + 1, end);
}

static int jsonInt(const String& body, const char* key, int fallback = 0) {
  const String needle = String("\"") + key + "\":";
  int p = body.indexOf(needle);
  if (p < 0) return fallback;
  p += needle.length();
  while (p < (int)body.length() && body[p] == ' ') ++p;
  return body.substring(p).toInt();
}

static bool jsonBool(const String& body, const char* key) {
  const String needle = String("\"") + key + "\":";
  int p = body.indexOf(needle);
  if (p < 0) return false;
  p += needle.length();
  while (p < (int)body.length() && body[p] == ' ') ++p;
  return body.startsWith("true", p);
}

*/

static bool connectBox() {
  if (WiFi.status() == WL_CONNECTED) return true;
  WiFi.mode(WIFI_STA);
  WiFi.setSleep(false);
  WiFi.setHostname("R04-Camera");
  WiFi.begin(CAS_BOX_SSID, CAS_BOX_PASSWORD);

  const uint32_t started = millis();
  while (WiFi.status() != WL_CONNECTED && millis() - started < 10000) {
    delay(150); // camera board only; main ESP32/motor loop is unaffected
  }
  if (WiFi.status() == WL_CONNECTED) {
    Serial.printf("[wifi] connected to %s | camera IP %s | box %s\n",
                  CAS_BOX_SSID, WiFi.localIP().toString().c_str(), CAS_BOX_IP);
    return true;
  }
  Serial.printf("[wifi] cannot join %s; retrying in %lu ms\n",
                CAS_BOX_SSID, (unsigned long)WIFI_RETRY_MS);
  return false;
}

/* Removed legacy laptop heartbeat/session/JPEG upload path.
static bool postHeartbeat(bool cameraReady, const char* lastError, uint32_t frameSeq) {
  if (WiFi.status() != WL_CONNECTED) return false;

  const bool psram = psramFound();
  const uint32_t freeHeap = ESP.getFreeHeap();
  const uint32_t minHeap = ESP.getMinFreeHeap();
  const uint32_t largestBlock = heap_caps_get_largest_free_block(MALLOC_CAP_8BIT);

  String body;
  body.reserve(420);
  body = "{";
  body += "\"board_id\":\""; body += CAS_CAMERA_BOARD_ID; body += "\",";
  body += "\"firmware\":\""; body += CAS_CAMERA_FIRMWARE_VERSION; body += "\",";
  body += "\"camera_ready\":"; body += cameraReady ? "true" : "false"; body += ",";
  body += "\"width\":320,\"height\":240,\"jpeg_quality\":12,\"xclk_hz\":20000000,";
  body += "\"psram_detected\":"; body += psram ? "true" : "false"; body += ",";
  body += "\"free_heap\":"; body += freeHeap; body += ",";
  body += "\"min_free_heap\":"; body += minHeap; body += ",";
  body += "\"largest_free_block\":"; body += largestBlock; body += ",";
  body += "\"wifi_rssi\":"; body += WiFi.RSSI(); body += ",";
  body += "\"wifi_ip\":\""; body += WiFi.localIP().toString(); body += "\",";
  body += "\"frame_sequence\":"; body += frameSeq; body += ",";
  body += "\"last_error\":\""; body += lastError ? lastError : ""; body += "\"}";

  HTTPClient http;
  http.setTimeout(1200);
  if (!http.begin(baseUrl() + "/api/camera/heartbeat")) return false;
  http.addHeader("Content-Type", "application/json");
  http.addHeader("X-Camera-Token", CAS_CAMERA_TOKEN);
  const int code = http.POST((uint8_t*)body.c_str(), body.length());
  http.end();
  return code == 200;
}

static bool pollSurveySession(String& sessionId, String& captureEventId, int& framesRemaining) {
  if (WiFi.status() != WL_CONNECTED) return false;

  HTTPClient http;
  http.setTimeout(1200);
  if (!http.begin(baseUrl() + "/api/camera/session")) return false;
  http.addHeader("X-Camera-Token", CAS_CAMERA_TOKEN);
  const int code = http.GET();
  if (code != 200) {
    http.end();
    return false;
  }
  const String body = http.getString();
  http.end();

  if (!jsonBool(body, "active")) {
    sessionId = "";
    captureEventId = "";
    framesRemaining = 0;
    return true;
  }

  sessionId = jsonString(body, "session_id");
  captureEventId = "";
  framesRemaining = 0;
  const int capturePos = body.indexOf("\"capture\":{");
  if (capturePos >= 0) {
    const String captureJson = body.substring(capturePos);
    captureEventId = jsonString(captureJson, "event_id");
    framesRemaining = jsonInt(captureJson, "frames_remaining", 0);
  }
  return !sessionId.isEmpty();
}

static bool postJpeg(camera_fb_t* fb, bool preview, const String& sessionId,
                     const String& captureEventId, uint32_t frameSeq) {
  if (!fb || !fb->buf || fb->len < 4 || fb->len > MAX_JPEG_BYTES ||
      fb->format != PIXFORMAT_JPEG) {
    Serial.printf("[capture] invalid/oversized JPEG; bytes=%u\n", fb ? (unsigned)fb->len : 0);
    return false;
  }

  HTTPClient http;
  http.setTimeout(HTTP_TIMEOUT_MS);
  const String route = preview ? "/api/camera/preview" : "/api/camera/frame";
  if (!http.begin(baseUrl() + route)) return false;
  http.addHeader("Content-Type", "image/jpeg");
  http.addHeader("X-Camera-Token", CAS_CAMERA_TOKEN);
  http.addHeader("X-Camera-Board-Id", CAS_CAMERA_BOARD_ID);
  http.addHeader("X-Frame-Id", String(CAS_CAMERA_BOARD_ID) + "_" + String(frameSeq));
  http.addHeader("X-Frame-Sequence", String(frameSeq));
  http.addHeader("X-Camera-Monotonic-Ms", String(millis()));
  if (!preview) {
    http.addHeader("X-Session-Id", sessionId);
    if (!captureEventId.isEmpty()) http.addHeader("X-Capture-Event-Id", captureEventId);
  }

  // Send the existing framebuffer directly. No JPEG String/copy is created.
  const int code = http.POST(fb->buf, fb->len);
  if (code == 200) {
    Serial.printf("[%s] frame %lu stored | %u bytes\n",
                  preview ? "preview" : "survey", (unsigned long)frameSeq, (unsigned)fb->len);
  } else {
    Serial.printf("[%s] upload failed HTTP %d | frame %lu\n",
                  preview ? "preview" : "survey", code, (unsigned long)frameSeq);
  }
  http.end();
  return code == 200;
}

*/

static bool initCamera() {
  camera_config_t config = {};
  config.ledc_channel = LEDC_CHANNEL_0;
  config.ledc_timer = LEDC_TIMER_0;
  config.pin_d0 = Y2_GPIO_NUM;
  config.pin_d1 = Y3_GPIO_NUM;
  config.pin_d2 = Y4_GPIO_NUM;
  config.pin_d3 = Y5_GPIO_NUM;
  config.pin_d4 = Y6_GPIO_NUM;
  config.pin_d5 = Y7_GPIO_NUM;
  config.pin_d6 = Y8_GPIO_NUM;
  config.pin_d7 = Y9_GPIO_NUM;
  config.pin_xclk = XCLK_GPIO_NUM;
  config.pin_pclk = PCLK_GPIO_NUM;
  config.pin_vsync = VSYNC_GPIO_NUM;
  config.pin_href = HREF_GPIO_NUM;
  config.pin_sccb_sda = SIOD_GPIO_NUM;
  config.pin_sccb_scl = SIOC_GPIO_NUM;
  config.pin_pwdn = PWDN_GPIO_NUM;
  config.pin_reset = RESET_GPIO_NUM;
  config.xclk_freq_hz = 20000000;
  config.pixel_format = PIXFORMAT_JPEG;
  config.frame_size = FRAMESIZE_QVGA;
  config.jpeg_quality = 12;
  const bool hasPsram = psramFound();
  config.fb_count = hasPsram ? 2 : 1;
  config.fb_location = hasPsram ? CAMERA_FB_IN_PSRAM : CAMERA_FB_IN_DRAM;
  config.grab_mode = hasPsram ? CAMERA_GRAB_LATEST : CAMERA_GRAB_WHEN_EMPTY;

  const esp_err_t err = esp_camera_init(&config);
  if (err != ESP_OK) {
    Serial.printf("[camera] init failed: 0x%x\n", (unsigned)err);
    return false;
  }
  sensor_t* sensor = esp_camera_sensor_get();
  if (sensor) {
    sensor->set_framesize(sensor, FRAMESIZE_QVGA);
    sensor->set_quality(sensor, 12);
    sensor->set_exposure_ctrl(sensor, 1);
    sensor->set_gain_ctrl(sensor, 1);
    sensor->set_aec2(sensor, 1);
  }
  // Discard initial frames while exposure and gain settle.
  for (uint8_t i = 0; i < 3; ++i) {
    camera_fb_t* warm = esp_camera_fb_get();
    if (warm) esp_camera_fb_return(warm);
    delay(40);
  }
  Serial.printf("[camera] initialized AI-Thinker OV2640 | QVGA | JPEG q12 | XCLK 20 MHz | PSRAM %s\n",
                psramFound() ? "yes" : "no");
  return true;
}

void setup() {
  Serial.begin(115200);
  delay(250);
  Serial.println();
  Serial.println("CAS Box B3 / Rover R04 — AI-Thinker ESP32-CAM");

  cameraReady = initCamera();
  if (!cameraReady) {
    Serial.println("[fatal] camera initialization failed; reboot after checking power/camera ribbon.");
  }
  connectBox();
  cameraServer.on("/snapshot", HTTP_GET, cameraSnapshot);
  cameraServer.on("/status", HTTP_GET, cameraStatus);
  cameraServer.begin();

  Serial.printf("[memory] free=%lu min=%lu largest=%lu psram=%s flash=%lu\n",
                (unsigned long)ESP.getFreeHeap(),
                (unsigned long)ESP.getMinFreeHeap(),
                (unsigned long)heap_caps_get_largest_free_block(MALLOC_CAP_8BIT),
                psramFound() ? "yes" : "no",
                (unsigned long)ESP.getFlashChipSize());
}

void loop() {
  static uint32_t nextWifiRetry = 0;
  static uint32_t lastRegistration = 0;

  const uint32_t now = millis();

  if (WiFi.status() != WL_CONNECTED) {
    if (now >= nextWifiRetry) {
      nextWifiRetry = now + WIFI_RETRY_MS;
      connectBox();
    }
    delay(5);
    return;
  }

  // Phone-led operation: serve on-demand snapshots and register with the main ESP32.
  cameraServer.handleClient();
  if (now - lastRegistration >= 5000) { lastRegistration = now; registerWithMainBox(); }
  delay(2);
}
