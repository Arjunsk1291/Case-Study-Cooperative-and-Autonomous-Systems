// =============================================================================================
//  cas_box_trimcal - WHEEL TRIM CALIBRATION for the drift of the WAVE ROVER (2026-11-06)
//  Based on the CAS Box starter program v1.0 and the team's field-tested box files (prakash_v2).
// =============================================================================================
//
//  WHAT IT DOES
//    Finds the left/right wheel TRIM that makes the rover drive straight without the gyro having
//    to correct all the time. See TrimCal.h for one run. The result is hard-coded in the real
//    program afterwards.
//      left = v * (1 - trim)   right = v * (1 + trim)
//
//  WHAT IS IN / OUT compared with the navigation program
//    in : dedicated page (TrimPage.h) with hold-to-move buttons, RoverLink, Sensors (LiDAR, ultrasonic, IMU, GPS), OLED,
//         GentleDrive (all commands ramped - gearbox rule), TrimCal (calibration runs, 3 guided steps)
//    out: route following, mission, DriveLog (no flash writing while driving)
//
//  SAFETY / BEHAVIOUR (stop rules of the starter program are kept)
//    - move buttons: release = stop (ramped), page closed / no update for 0.4 s = stop
//    - RoverLink deadman (0.5 s) and the rover's own 3 s stop are unchanged
//    - every command passes GentleDrive: no jumps. Pivots on the pad use the full 0.50 (the rover
//      does not turn below ~0.46), ramped up in ~0.6 s
//    - moving forward stops (ramped) when something is closer than 0.40 m ahead (LiDAR/ultrasonic)
//    - after a reset nothing starts by itself: a run only starts from a step button on the page
//    - NO heading hold (no Kethan PD) here on purpose: the calibration must SEE the drift
//    - NOTHING is written to the rover. Its stored speed ratio can only be READ (T:139).
//    - The only flash write is "Save trim in the box" (2 numbers in NVS) - on request.
//
//  FLASH: Arduino IDE / arduino-cli, board "ESP32 Dev Module", esp32 core 3.3.x, partition
//    "Default". Do NOT use "Erase All Flash" (not needed here). Upload speed 115200 if the
//    normal speed gives errors. BOX_ID below = your kit (4).
// =============================================================================================

#include "WebHMI.h"
#include "RoverLink.h"
#include "Sensors.h"
#include "DisplayManager.h"
#include "GentleDrive.h"
#include "TrimCal.h"
#include "TrimPage.h"
#include <esp_wifi.h>
#include <esp_system.h>

// ---- YOUR KIT ------------------------------------------------------------------------------
const int BOX_ID = 4;                      // 1..10 = kit number on the box
// Wi-Fi transmit power: low at start (smallest current spike, brownout), moderate while running.
#define WIFI_TX_START WIFI_POWER_8_5dBm
#define WIFI_TX_RUN WIFI_POWER_15dBm

// ---- settings --------------------------------------------------------------------------------
const float MAX_SPEED = 0.30f;            // drive pad maximum (slider)
const float PIVOT_CMD = 0.50f;            // pad turn keys: full pivot (the rover stalls below ~0.46)
const uint32_t PAD_TIMEOUT_MS = 400;      // no update from the phone for this long -> stop
const int LIDAR_OFFSET_DEG = 90;          // rotates the LiDAR view so that "front" is up
const float PAD_FRONT_STOP_M = 0.40f;     // pad forward: ramped stop when something is this close

WebHMI hmi;
RoverLink rover;
Sensors sensors;
DisplayManager disp;
GentleDrive gentle;
TrimCal trimcal;
bool oledReady = false;
char boxSsid[16], boxPassword[16];
String resetReason;

static String readResetReason() {
  switch (esp_reset_reason()) {
    case ESP_RST_POWERON: return "power on";
    case ESP_RST_BROWNOUT: return "BROWNOUT (supply voltage dropped)";
    case ESP_RST_PANIC: return "CRASH (panic)";
    case ESP_RST_INT_WDT: case ESP_RST_TASK_WDT: case ESP_RST_WDT: return "WATCHDOG";
    case ESP_RST_SW: return "software restart";
    case ESP_RST_EXT: return "reset button";
    default: return "reset code " + String((int)esp_reset_reason());
  }
}

void updateOled() {
  static uint32_t lastTry = 0;
  if (!oledReady) {
    if (millis() - lastTry < 5000) return;
    lastTry = millis();
    if (!i2cPresent(0x3C)) return;
    oledReady = disp.begin();
  }
  String ip = WiFi.softAPIP().toString();
  String r = rover.connected() ? "Rover " + String(rover.voltage(), 1) + "V" : "Rover: search";
  disp.show(boxSsid, ip.c_str(), r.c_str());
}

void setup() {
  Serial.begin(115200);
  delay(200);
  resetReason = readResetReason();
  Serial.printf("[boot] trimcal, reset reason: %s\n", resetReason.c_str());

  snprintf(boxSsid, sizeof(boxSsid), "CAS-Box-%02d", BOX_ID);
  snprintf(boxPassword, sizeof(boxPassword), "CASbox%02d", BOX_ID);
  const uint8_t channel = (BOX_ID % 2) ? 1 : 11;

  hmi.setTitle(String("CAS Box ") + BOX_ID + " - trim");
  hmi.enableDrivePad(MAX_SPEED);         // limits /drive (the page's "Move rover" buttons)
  hmi.setCustomPage(kTrimPage);          // the dedicated drift-trim page instead of the standard one
  hmi.setStateText("Starting ...");
  WiFi.mode(WIFI_AP);
  WiFi.setTxPower(WIFI_TX_START);
  hmi.begin(boxSsid, boxPassword, IPAddress(192, 168, 10 + BOX_ID, 1), channel);
  wifi_config_t apCfg;                    // DTIM 1: the rover wakes at every beacon (less delay)
  if (esp_wifi_get_config(WIFI_IF_AP, &apCfg) == ESP_OK) {
    apCfg.ap.dtim_period = 1;
    esp_wifi_set_config(WIFI_IF_AP, &apCfg);
  }
  WiFi.setTxPower(WIFI_TX_RUN);

  trimcal.setResetReason(resetReason);
  trimcal.begin(hmi.server());            // /trim/... routes (the page itself: TrimPage.h)

  Serial.println("[boot] web ready, starting sensors (LiDAR motor) in 0.5 s");
  delay(500);                             // let the supply recover from the Wi-Fi start first
  sensors.begin(LIDAR_OFFSET_DEG);
  rover.begin(100);                       // one command every 100 ms
  Serial.println("[boot] setup done");
}

void loop() {
  hmi.tick();
  sensors.update();

  // ---- drive pad (manual: reposition the rover between runs)
  float l, r;
  uint32_t age;
  hmi.getDrive(l, r, age);
  if (age > PAD_TIMEOUT_MS) l = r = 0;
  const bool padActive = (l != 0 || r != 0);
  bool emergency = false;
  if (padActive) {
    const bool pivot = (l > 0 && r < 0) || (l < 0 && r > 0);
    if (pivot) {                                           // turn on the spot: full pivot only
      const float d = r > 0 ? 1.0f : -1.0f;
      l = -d * PIVOT_CMD; r = d * PIVOT_CMD;
    }
    if (0.5f * (l + r) > 0 && trimcal.frontM() < PAD_FRONT_STOP_M) {   // forward into something
      l = r = 0;
      emergency = true;
    }
  }

  // ---- calibration run (overrides the pad; a pad key during a run stops the run)
  trimcal.update(sensors, rover, padActive, l, r, emergency);

  gentle.apply(l, r, emergency);          // every command ramped, no jumps (gearbox rule)
  rover.setDrive(l, r);
  trimcal.noteOutput(l, r);               // what really went to the rover (log, distance)

  // ---- OLED (the page gets everything from /trim/status)
  const uint32_t now = millis();
  static uint32_t lastOled = 0;
  if (now - lastOled >= 1000) {
    lastOled = now;
    updateOled();
  }

  static uint32_t lastRep = 0;
  if (now - lastRep >= 10000) {
    lastRep = now;
    Serial.printf("[sys] heap %u kB (min %u kB), rover %s %.2f V, rtt %.0f ms, ok %u fail %u, trim %.4f\n",
                  (unsigned)(ESP.getFreeHeap() / 1024), (unsigned)(ESP.getMinFreeHeap() / 1024),
                  rover.connected() ? "ok" : "searching", rover.voltage(), rover.rttMs(),
                  (unsigned)rover.okCount(), (unsigned)rover.failCount(), trimcal.trim());
  }
  delay(2);
}
