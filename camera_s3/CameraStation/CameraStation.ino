/*
 * CAS Box B3 / Rover R04 — AI-Thinker ESP32-CAM (OV2640)
 *
 * Network architecture:
 *   phone ───────┐
 *                ├── Wi-Fi AP CAS-Box-04 / main ESP32 (192.168.14.1)
 *   camera ──────┘       ↑ capture requests + camera health metadata
 *
 * Original JPEGs are saved directly to the AI-Thinker board's microSD card.
 * The main ESP32 receives no JPEG payloads and does not allocate image buffers.
 * The laptop is not part of the live robot network or control loop.
 *
 * Requires a FAT32 microSD card in the AI-Thinker ESP32-CAM card slot.
 * SD is mounted in 1-bit mode to avoid camera GPIO conflicts.
 * Camera settings: QVGA 320x240 JPEG, q12, 20 MHz XCLK, 1 DRAM framebuffer.
 */

#include <Arduino.h>
#include <WiFi.h>
#include <HTTPClient.h>
#include <WebServer.h>
#include <FS.h>
#include <SD_MMC.h>
#include <esp_camera.h>
#include <esp_heap_caps.h>
#include "camera_user_config.h"

#if !defined(CAS_BOX_HOST) || !defined(CAS_BOX_SSID) || !defined(CAS_BOX_PASSWORD)
#error "Set CAS_BOX_HOST, CAS_BOX_SSID and CAS_BOX_PASSWORD in camera_user_config.h."
#endif

// AI-Thinker ESP32-CAM / OV2640 pin map.
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

static constexpr uint16_t BOX_PORT = 80;
static constexpr uint16_t ARCHIVE_PORT = 81;
static constexpr uint32_t WIFI_RETRY_MS = 5000;
static constexpr uint32_t HEARTBEAT_MS = 1000;
static constexpr uint32_t COMMAND_POLL_MS = 500;
static constexpr uint32_t BURST_INTERVAL_MS = 450;
static constexpr uint32_t HTTP_TIMEOUT_MS = 900;
static constexpr size_t MAX_JPEG_BYTES = 300 * 1024;
static constexpr uint32_t MIN_FREE_SD_BYTES = 512 * 1024;

static WebServer archiveServer(ARCHIVE_PORT);
static bool archiveServerStarted = false;
static bool cameraReady = false;
static bool sdReady = false;
static uint32_t frameSequence = 0;
static uint32_t storedImages = 0;
static uint32_t lastHeartbeat = 0;
static uint32_t lastCommandPoll = 0;
static uint32_t nextCaptureAt = 0;
static uint32_t lastMemoryLog = 0;
static String activeEventId;
static uint8_t framesRemaining = 0;
static uint8_t activeRequestedFrames = 0;
static uint8_t activeSavedFrames = 0;
static char lastError[48] = "none";

static String boxBaseUrl() {
  return String("http://") + CAS_BOX_HOST;
}

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

static bool connectBox() {
  if (WiFi.status() == WL_CONNECTED) return true;
  WiFi.mode(WIFI_STA);
  WiFi.setSleep(false);
  WiFi.setHostname("R04-Camera");
  WiFi.begin(CAS_BOX_SSID, CAS_BOX_PASSWORD);
  const uint32_t started = millis();
  while (WiFi.status() != WL_CONNECTED && millis() - started < 10000) {
    delay(100);  // Only blocks the camera board, never the main ESP32.
  }
  if (WiFi.status() == WL_CONNECTED) {
    Serial.printf("[wifi] connected to %s | camera IP %s | main box %s\n",
                  CAS_BOX_SSID, WiFi.localIP().toString().c_str(), boxBaseUrl().c_str());
    return true;
  }
  Serial.printf("[wifi] cannot join %s; will retry\n", CAS_BOX_SSID);
  return false;
}

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
  config.fb_count = 1;
  config.fb_location = CAMERA_FB_IN_DRAM;
  config.grab_mode = CAMERA_GRAB_WHEN_EMPTY;

  const esp_err_t err = esp_camera_init(&config);
  if (err != ESP_OK) {
    Serial.printf("[camera] init failed: 0x%x\n", (unsigned)err);
    return false;
  }
  sensor_t* sensor = esp_camera_sensor_get();
  if (sensor) {
    sensor->set_framesize(sensor, FRAMESIZE_QVGA);
    sensor->set_quality(sensor, 12);
  }
  Serial.printf("[camera] AI-Thinker OV2640 ready | QVGA | q12 | 20 MHz | PSRAM %s\n",
                psramFound() ? "yes" : "no");
  return true;
}

static uint32_t countStoredImages() {
  if (!sdReady) return 0;
  File dir = SD_MMC.open("/captures");
  if (!dir || !dir.isDirectory()) return 0;
  uint32_t count = 0;
  File f = dir.openNextFile();
  while (f) {
    if (!f.isDirectory()) {
      String name = String(f.name());
      if (name.endsWith(".jpg")) ++count;
    }
    f.close();
    f = dir.openNextFile();
  }
  dir.close();
  return count;
}

static bool initStorage() {
  // AI-Thinker onboard microSD: 1-bit mode uses GPIO 2/14/15 and avoids 4-bit bus conflicts.
  if (!SD_MMC.begin("/sdcard", true)) {
    Serial.println("[sd] mount failed; insert a FAT32 microSD card and reboot");
    return false;
  }
  if (SD_MMC.cardType() == CARD_NONE) {
    Serial.println("[sd] no card detected");
    SD_MMC.end();
    return false;
  }
  if (!SD_MMC.exists("/captures") && !SD_MMC.mkdir("/captures")) {
    Serial.println("[sd] cannot create /captures");
    SD_MMC.end();
    return false;
  }
  sdReady = true;
  storedImages = countStoredImages();
  Serial.printf("[sd] ready | total=%lu MB used=%lu MB images=%lu\n",
                (unsigned long)(SD_MMC.totalBytes() / 1024 / 1024),
                (unsigned long)(SD_MMC.usedBytes() / 1024 / 1024),
                (unsigned long)storedImages);
  return true;
}

static bool saveFrame(camera_fb_t* fb, const String& eventId) {
  if (!sdReady || !fb || !fb->buf || fb->len < 4 ||
      fb->len > MAX_JPEG_BYTES || fb->format != PIXFORMAT_JPEG) {
    strncpy(lastError, "invalid_frame_or_sd", sizeof(lastError) - 1);
    return false;
  }
  const uint64_t total = SD_MMC.totalBytes();
  const uint64_t used = SD_MMC.usedBytes();
  if (total <= used || (total - used) < MIN_FREE_SD_BYTES + fb->len) {
    strncpy(lastError, "sd_low_space", sizeof(lastError) - 1);
    return false;
  }
  if (fb->buf[0] != 0xFF || fb->buf[1] != 0xD8 ||
      fb->buf[fb->len - 2] != 0xFF || fb->buf[fb->len - 1] != 0xD9) {
    strncpy(lastError, "jpeg_marker_invalid", sizeof(lastError) - 1);
    return false;
  }

  char filename[80];
  snprintf(filename, sizeof(filename), "/captures/R04_%06lu_%s.jpg",
           (unsigned long)frameSequence, eventId.c_str());
  File out = SD_MMC.open(filename, FILE_WRITE);
  if (!out) {
    strncpy(lastError, "sd_open_failed", sizeof(lastError) - 1);
    return false;
  }
  const size_t written = out.write(fb->buf, fb->len);
  out.flush();
  out.close();
  if (written != fb->len) {
    SD_MMC.remove(filename);
    strncpy(lastError, "sd_short_write", sizeof(lastError) - 1);
    return false;
  }

  File manifest = SD_MMC.open("/captures/manifest.csv", FILE_APPEND);
  if (manifest) {
    if (manifest.size() == 0) manifest.println("frame_sequence,event_id,filename,bytes,camera_device_ms");
    manifest.printf("%lu,%s,%s,%u,%lu\n", (unsigned long)frameSequence,
                    eventId.c_str(), filename + 10, (unsigned)fb->len,
                    (unsigned long)millis());
    manifest.flush();
    manifest.close();
  }
  ++storedImages;
  strncpy(lastError, "none", sizeof(lastError) - 1);
  Serial.printf("[capture] saved %s | %u bytes | event=%s\n",
                filename, (unsigned)fb->len, eventId.c_str());
  return true;
}

static bool boxGet(const String& path, String* responseBody = nullptr) {
  if (WiFi.status() != WL_CONNECTED) return false;
  HTTPClient http;
  http.setTimeout(HTTP_TIMEOUT_MS);
  if (!http.begin(boxBaseUrl() + path)) return false;
  const int code = http.GET();
  if (responseBody && code == 200) *responseBody = http.getString();
  http.end();
  return code == 200;
}

static void sendHeartbeat() {
  if (WiFi.status() != WL_CONNECTED) return;
  String path = "/camera/heartbeat?ready=" + String(cameraReady ? 1 : 0) +
                "&sd=" + String(sdReady ? 1 : 0) +
                "&ip=" + WiFi.localIP().toString() +
                "&frames=" + String(storedImages) +
                "&seq=" + String(frameSequence) +
                "&heap=" + String(ESP.getFreeHeap()) +
                "&err=" + String(lastError);
  if (!boxGet(path)) Serial.println("[heartbeat] main ESP32 did not respond");
}

static void acknowledgeEvent() {
  if (activeEventId.isEmpty()) return;
  String path = "/camera/ack?event_id=" + activeEventId;
  boxGet(path);
  Serial.printf("[capture] event %s complete; %u frames requested/attempted\n",
                activeEventId.c_str(), (unsigned)0);
  activeEventId = "";
  Serial.printf(", saved=%u/%u\\n", (unsigned)activeSavedFrames, (unsigned)activeRequestedFrames);
  activeEventId = "";
  framesRemaining = 0;
  activeRequestedFrames = activeSavedFrames = 0;
}

static void pollCaptureCommand() {
  String body;
  if (!boxGet("/camera/command", &body)) return;
  if (!jsonBool(body, "capture")) return;
  const String eventId = jsonString(body, "event_id");
  const int requested = constrain(jsonInt(body, "frames", 3), 1, 5);
  if (eventId.isEmpty() || eventId == activeEventId) return;
  activeEventId = eventId;
  framesRemaining = (uint8_t)requested;
  activeRequestedFrames = (uint8_t)requested;
  activeSavedFrames = 0;
  nextCaptureAt = millis();
  Serial.printf("[capture] new request event=%s frames=%u\n",
                activeEventId.c_str(), (unsigned)framesRemaining);
}

static void handleArchiveRoot() {
  String html;
  html.reserve(1800);
  html += "<!doctype html><html><head><meta name='viewport' content='width=device-width,initial-scale=1'>";
  html += "<title>R04 camera archive</title><style>body{font:16px system-ui;max-width:760px;margin:24px auto;padding:0 14px;background:#101722;color:#eef}a{color:#79baff}li{margin:10px 0}.muted{color:#9aa}</style></head><body>";
  html += "<h2>R04 camera archive</h2><p>Original QVGA JPEG files stored on this camera's microSD card.</p>";
  html += "<p>Stored images: " + String(storedImages) + "</p>";
  if (!sdReady) {
    html += "<p>microSD unavailable. Insert a FAT32 card and reboot the camera.</p>";
  } else {
    html += "<p><a href='/manifest'>Download manifest.csv</a></p><ul>";
    File dir = SD_MMC.open("/captures");
    File f = dir ? dir.openNextFile() : File();
    while (f) {
      if (!f.isDirectory()) {
        String full = String(f.name());
        String name = full.substring(full.lastIndexOf('/') + 1);
        if (name.endsWith(".jpg")) {
          html += "<li><a href='/file?name=" + name + "' download>" + name + "</a> (" + String(f.size()) + " bytes)</li>";
        }
      }
      f.close();
      f = dir.openNextFile();
    }
    if (dir) dir.close();
    html += "</ul>";
  }
  html += "</body></html>";
  archiveServer.send(200, "text/html; charset=utf-8", html);
}

static void handleArchiveFile() {
  if (!sdReady || !archiveServer.hasArg("name")) {
    archiveServer.send(400, "text/plain", "Missing file name or SD card unavailable");
    return;
  }
  const String name = archiveServer.arg("name");
  if (!name.startsWith("R04_") || !name.endsWith(".jpg") ||
      name.indexOf('/') >= 0 || name.indexOf('\\') >= 0 || name.indexOf("..") >= 0) {
    archiveServer.send(400, "text/plain", "Invalid file name");
    return;
  }
  const String path = "/captures/" + name;
  File file = SD_MMC.open(path, FILE_READ);
  if (!file || file.isDirectory()) {
    archiveServer.send(404, "text/plain", "Image not found");
    return;
  }
  archiveServer.sendHeader("Content-Disposition", "attachment; filename=\"" + name + "\"");
  archiveServer.sendHeader("Cache-Control", "no-store");
  archiveServer.streamFile(file, "image/jpeg");
  file.close();
}

static void handleManifest() {
  if (!sdReady) {
    archiveServer.send(503, "text/plain", "microSD unavailable");
    return;
  }
  File file = SD_MMC.open("/captures/manifest.csv", FILE_READ);
  if (!file) {
    archiveServer.send(404, "text/plain", "No captures yet");
    return;
  }
  archiveServer.sendHeader("Content-Disposition", "attachment; filename=\"R04_camera_manifest.csv\"");
  archiveServer.streamFile(file, "text/csv");
  file.close();
}

static void beginArchiveServer() {
  if (archiveServerStarted) return;
  archiveServer.on("/", HTTP_GET, handleArchiveRoot);
  archiveServer.on("/file", HTTP_GET, handleArchiveFile);
  archiveServer.on("/manifest", HTTP_GET, handleManifest);
  archiveServer.on("/api/status", HTTP_GET, []() {
    String body = "{\"camera_ready\":";
    body += cameraReady ? "true" : "false";
    body += ",\"sd_ready\":";
    body += sdReady ? "true" : "false";
    body += ",\"stored_images\":" + String(storedImages) + "}";
    archiveServer.send(200, "application/json", body);
  });
  archiveServer.onNotFound([]() { archiveServer.send(404, "text/plain", "Not found"); });
  archiveServer.begin();
  archiveServerStarted = true;
  Serial.printf("[archive] image files served at http://%s:%u/\n",
                WiFi.localIP().toString().c_str(), ARCHIVE_PORT);
}

void setup() {
  Serial.begin(115200);
  delay(250);
  Serial.println();
  Serial.println("CAS Box B3 / Rover R04 — camera storage node");

  cameraReady = initCamera();
  if (!cameraReady) strncpy(lastError, "camera_init_failed", sizeof(lastError) - 1);
  sdReady = initStorage();
  if (!sdReady && cameraReady) strncpy(lastError, "sd_unavailable", sizeof(lastError) - 1);

  connectBox();
  if (WiFi.status() == WL_CONNECTED) beginArchiveServer();

  Serial.printf("[memory] free=%lu min=%lu largest=%lu psram=%s flash=%lu\n",
                (unsigned long)ESP.getFreeHeap(),
                (unsigned long)ESP.getMinFreeHeap(),
                (unsigned long)heap_caps_get_largest_free_block(MALLOC_CAP_8BIT),
                psramFound() ? "yes" : "no",
                (unsigned long)ESP.getFlashChipSize());
}

void loop() {
  static uint32_t nextWifiRetry = 0;
  if (WiFi.status() != WL_CONNECTED) {
    if (millis() >= nextWifiRetry) {
      nextWifiRetry = millis() + WIFI_RETRY_MS;
      connectBox();
      if (WiFi.status() == WL_CONNECTED) beginArchiveServer();
    }
    delay(5);
    return;
  }

  archiveServer.handleClient();
  const uint32_t now = millis();

  static uint32_t nextInitRetry = 0;
  if ((!cameraReady || !sdReady) && now >= nextInitRetry) {
    nextInitRetry = now + 10000;
    if (!cameraReady) cameraReady = initCamera();
    if (!sdReady) sdReady = initStorage();
    if (cameraReady && sdReady) strncpy(lastError, "none", sizeof(lastError) - 1);
  }
  if (now - lastHeartbeat >= HEARTBEAT_MS) {
    lastHeartbeat = now;
    sendHeartbeat();
  }
  if (now - lastCommandPoll >= COMMAND_POLL_MS) {
    lastCommandPoll = now;
    pollCaptureCommand();
  }

  if (cameraReady && sdReady && framesRemaining > 0 && now >= nextCaptureAt) {
    nextCaptureAt = now + BURST_INTERVAL_MS;
    camera_fb_t* fb = esp_camera_fb_get();
    ++frameSequence;
    if (!fb) {
      strncpy(lastError, "frame_capture_failed", sizeof(lastError) - 1);
      Serial.println("[capture] camera framebuffer unavailable");
    } else {
      if (saveFrame(fb, activeEventId)) ++activeSavedFrames;
      esp_camera_fb_return(fb);  // Release single framebuffer immediately after SD write.
    }
    --framesRemaining;
    if (framesRemaining == 0) acknowledgeEvent();
  }

  if (now - lastMemoryLog >= 10000) {
    lastMemoryLog = now;
    Serial.printf("[memory] free=%lu min=%lu largest=%lu psram=%s wifi_rssi=%d seq=%lu stored=%lu\n",
                  (unsigned long)ESP.getFreeHeap(),
                  (unsigned long)ESP.getMinFreeHeap(),
                  (unsigned long)heap_caps_get_largest_free_block(MALLOC_CAP_8BIT),
                  psramFound() ? "yes" : "no", WiFi.RSSI(),
                  (unsigned long)frameSequence, (unsigned long)storedImages);
  }
  delay(2);
}
