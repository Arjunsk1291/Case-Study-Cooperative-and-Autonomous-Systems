/*
 * CAS Box B3 / R07 — ESP32-S3 camera station
 *
 * Architecture:
 *   ESP32-S3 camera --Wi-Fi STA--> CAS-Box-07 AP
 *   ESP32-S3 camera --HTTP JPEG POST--> laptop FastAPI /api/camera/frame
 *
 * Large JPEG payloads never pass through the main ESP32. No motor command is sent by this sketch.
 * This sketch intentionally refuses to compile until camera_board_config.h contains the real
 * ESP32-S3 camera pin map and laptop AP-side IP/token. No camera GPIO pins are guessed here.
 *
 * Required camera_board_config.h contract:
 *   #define CAS_CAMERA_BOARD_PINMAP_CONFIGURED 1
 *   #define CAS_CAMERA_BOARD_ID "camera-s3-b3"
 *   #define CAS_CAMERA_LAPTOP_HOST "192.168.17.X"  // laptop IP assigned by CAS-Box-07 AP
 *   #define CAS_CAMERA_TOKEN "same token as CAS_CAMERA_TOKEN env on laptop"
 *   bool configureCasCamera(camera_config_t& config); // fill the exact board pin map
 */

#include <Arduino.h>
#include <WiFi.h>
#include <HTTPClient.h>
#include "esp_camera.h"
#include "camera_board_config.h"

#if !defined(CAS_CAMERA_BOARD_PINMAP_CONFIGURED) || CAS_CAMERA_BOARD_PINMAP_CONFIGURED != 1
#error "Set CAS_CAMERA_BOARD_PINMAP_CONFIGURED=1 only after adding the real ESP32-S3 camera pin map."
#endif
#ifndef CAS_CAMERA_BOARD_ID
#error "Define CAS_CAMERA_BOARD_ID in camera_board_config.h."
#endif
#ifndef CAS_CAMERA_LAPTOP_HOST
#error "Define CAS_CAMERA_LAPTOP_HOST to the laptop's CAS-Box-07 AP-side IP."
#endif
#ifndef CAS_CAMERA_TOKEN
#error "Define CAS_CAMERA_TOKEN; it must match CAS_CAMERA_TOKEN on the laptop."
#endif

static constexpr char kBoxSsid[] = "CAS-Box-07";
static constexpr char kBoxPassword[] = "CASbox07";
static constexpr uint16_t kLaptopPort = 8000;
static constexpr uint32_t kSessionPollMs = 500;
static constexpr uint32_t kCapturePeriodMs = 500; // initial target 2 fps; tune after measuring transfer
static constexpr uint32_t kHttpTimeoutMs = 2500;
static constexpr size_t kMaxJpegBytes = 3 * 1024 * 1024;

static String laptopBaseUrl() {
  return String("http://") + CAS_CAMERA_LAPTOP_HOST + ":" + String(kLaptopPort);
}

static String jsonString(const String& json, const char* key) {
  const String needle = String("\"") + key + "\":";
  int p = json.indexOf(needle);
  if (p < 0) return "";
  p += needle.length();
  while (p < (int)json.length() && json[p] == ' ') ++p;
  if (p >= (int)json.length() || json[p] != '"') return "";
  const int end = json.indexOf('"', p + 1);
  if (end < 0) return "";
  return json.substring(p + 1, end);
}

static int jsonInt(const String& json, const char* key, int fallback = 0) {
  const String needle = String("\"") + key + "\":";
  int p = json.indexOf(needle);
  if (p < 0) return fallback;
  p += needle.length();
  while (p < (int)json.length() && json[p] == ' ') ++p;
  return json.substring(p).toInt();
}

static bool jsonTrue(const String& json, const char* key) {
  const String needle = String("\"") + key + "\":";
  int p = json.indexOf(needle);
  if (p < 0) return false;
  p += needle.length();
  while (p < (int)json.length() && json[p] == ' ') ++p;
  return json.startsWith("true", p);
}

static bool connectBox() {
  if (WiFi.status() == WL_CONNECTED) return true;
  WiFi.mode(WIFI_STA);
  WiFi.setSleep(false);
  WiFi.begin(kBoxSsid, kBoxPassword);
  const uint32_t start = millis();
  while (WiFi.status() != WL_CONNECTED && millis() - start < 12000) {
    delay(150);
  }
  if (WiFi.status() == WL_CONNECTED) {
    Serial.printf("[camera] joined %s, laptop URL %s\n", kBoxSsid, laptopBaseUrl().c_str());
    return true;
  }
  Serial.println("[camera] box AP join failed; retrying");
  return false;
}

static bool getCameraSession(String& sessionId, String& captureEventId, int& framesRemaining) {
  HTTPClient http;
  http.setTimeout(kHttpTimeoutMs);
  if (!http.begin(laptopBaseUrl() + "/api/camera/session")) return false;
  http.addHeader("X-Camera-Token", CAS_CAMERA_TOKEN);
  const int code = http.GET();
  if (code != 200) {
    Serial.printf("[camera] session poll HTTP %d\n", code);
    http.end();
    return false;
  }
  const String body = http.getString();
  http.end();
  if (!jsonTrue(body, "active")) return false;
  sessionId = jsonString(body, "session_id");
  captureEventId = "";
  framesRemaining = 0;
  const int cap = body.indexOf("\"capture\":{");
  if (cap >= 0) {
    const String tail = body.substring(cap);
    captureEventId = jsonString(tail, "event_id");
    framesRemaining = jsonInt(tail, "frames_remaining", 0);
  }
  return !sessionId.isEmpty();
}

static bool uploadFrame(camera_fb_t* fb, const String& sessionId,
                        const String& captureEventId, uint32_t frameSequence) {
  if (!fb || !fb->buf || fb->len < 4 || fb->len > kMaxJpegBytes) {
    Serial.println("[camera] invalid/oversize framebuffer; dropped");
    return false;
  }
  if (fb->format != PIXFORMAT_JPEG) {
    Serial.println("[camera] framebuffer is not JPEG; dropped");
    return false;
  }
  HTTPClient http;
  http.setTimeout(kHttpTimeoutMs);
  if (!http.begin(laptopBaseUrl() + "/api/camera/frame")) return false;
  http.addHeader("Content-Type", "image/jpeg");
  http.addHeader("X-Camera-Token", CAS_CAMERA_TOKEN);
  http.addHeader("X-Session-Id", sessionId);
  http.addHeader("X-Frame-Id", String(CAS_CAMERA_BOARD_ID) + "_" + String(frameSequence));
  http.addHeader("X-Frame-Sequence", String(frameSequence));
  http.addHeader("X-Camera-Board-Id", CAS_CAMERA_BOARD_ID);
  http.addHeader("X-Camera-Monotonic-Ms", String(millis()));
  if (!captureEventId.isEmpty()) http.addHeader("X-Capture-Event-Id", captureEventId);
  // HTTPClient sends the camera framebuffer directly; no full JPEG String/JSON copy is made.
  const int code = http.POST(fb->buf, fb->len);
  if (code == 200) {
    Serial.printf("[camera] frame %lu stored (%u bytes)\n",
                  (unsigned long)frameSequence, (unsigned)fb->len);
  } else {
    Serial.printf("[camera] upload failed: HTTP %d (frame %lu)\n",
                  code, (unsigned long)frameSequence);
  }
  http.end();
  return code == 200;
}

void setup() {
  Serial.begin(115200);
  delay(200);
  Serial.printf("[camera] board %s\n", CAS_CAMERA_BOARD_ID);
  camera_config_t cameraConfig = {};
  if (!configureCasCamera(cameraConfig)) {
    Serial.println("[camera] board camera configuration failed; no capture started");
    while (true) delay(1000);
  }
  cameraConfig.pixel_format = PIXFORMAT_JPEG;
  if (cameraConfig.jpeg_quality == 0) cameraConfig.jpeg_quality = 14;
  if (cameraConfig.fb_count < 1) cameraConfig.fb_count = 2;
  cameraConfig.grab_mode = CAMERA_GRAB_LATEST;
  const esp_err_t err = esp_camera_init(&cameraConfig);
  if (err != ESP_OK) {
    Serial.printf("[camera] esp_camera_init failed: 0x%x\n", (unsigned)err);
    while (true) delay(1000);
  }
  connectBox();
}

void loop() {
  static uint32_t lastPollMs = 0, lastFrameMs = 0, frameSequence = 0;
  static String activeSession, captureEventId;
  static int captureFramesRemaining = 0;
  if (WiFi.status() != WL_CONNECTED) {
    connectBox();
    delay(50);
    return;
  }
  const uint32_t now = millis();
  if (now - lastPollMs >= kSessionPollMs) {
    lastPollMs = now;
    String sessionId, eventId;
    int remaining = 0;
    if (getCameraSession(sessionId, eventId, remaining)) {
      activeSession = sessionId;
      if (!eventId.isEmpty()) {
        captureEventId = eventId;
        captureFramesRemaining = remaining;
      }
    } else {
      activeSession = "";
      captureEventId = "";
      captureFramesRemaining = 0;
    }
  }
  if (activeSession.isEmpty() || now - lastFrameMs < kCapturePeriodMs) {
    delay(5);
    return;
  }
  lastFrameMs = now;
  camera_fb_t* fb = esp_camera_fb_get();
  if (!fb) {
    Serial.println("[camera] framebuffer unavailable");
    delay(5);
    return;
  }
  ++frameSequence;
  const bool ok = uploadFrame(fb, activeSession, captureEventId, frameSequence);
  esp_camera_fb_return(fb);
  if (ok && captureFramesRemaining > 0) {
    --captureFramesRemaining;
    if (captureFramesRemaining == 0) captureEventId = "";
  }
  delay(2);
}
