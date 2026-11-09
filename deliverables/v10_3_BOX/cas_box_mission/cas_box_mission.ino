// =============================================================================================
//  cas_box_mission v10.3 - AUTONOMOUS mission B -> right turn -> open sea -> leg 3 -> T3 -> leg 4 to Block I, with full
//  logging (2026-11-07 evening). Box ESP32 ("ESP32 Dev Module", partition Default). NO camera any more.
// =============================================================================================
//  Phone: Wi-Fi CAS-Box-04 / CASbox04 -> http://192.168.14.1  (mission dashboard)
//  Kept from the tested programs: gentle ramps for every command (GentleDrive), pivots only at 0.50,
//  steady 10 Hz rover link with 3 decimals, ultrasonic pings 50 ms apart, checked IMU reads, measured
//  trim + Kethan's heading hold + failsafe (DriveStraight). Turns: per-wheel ramps (GentleDrive turn mode).
//  Stop rules of the starter program kept: page heartbeat, RoverLink deadman 0.5 s, rover's own 3 s stop.
//  The rover firmware is not changed and nothing is written to the rover.
// =============================================================================================
#include "WebHMI.h"
#include "RoverLink.h"
#include "Sensors.h"
#include "DisplayManager.h"
#include "GentleDrive.h"
#include "MissionLog.h"
#include "RouteMission.h"
#include "MissionPage.h"
#include <esp_wifi.h>
#include <esp_system.h>

const int BOX_ID = 4;
#define WIFI_TX_START WIFI_POWER_8_5dBm
#define WIFI_TX_RUN WIFI_POWER_15dBm
const float MAX_SPEED = 0.30f;
const uint32_t PAD_TIMEOUT_MS = 400;
// 07.11. (campus run + log_027, two days, two drivers): the LiDAR sits 7 deg rotated in the box. With 90 every wall
// looked 7 deg turned -> "parallel" was 7 deg off (wall distance grew 1.55 -> 2.23 m on leg 1; the team's "crab -7.6").
// Rotating the bearings by 7 deg makes the apparent sideways slide 0.1 deg on both days. 90 - 7 = 83.
const int LIDAR_OFFSET_DEG = 83;

WebHMI hmi;
RoverLink rover;
Sensors sensors;
DisplayManager disp;
GentleDrive gentle;
MissionLog mlog;
RouteMission mission;
bool oledReady = false;
char boxSsid[16], boxPassword[16];

static const char* resetText() {
  switch (esp_reset_reason()) {
    case ESP_RST_POWERON: return "power on";
    case ESP_RST_BROWNOUT: return "BROWNOUT";
    case ESP_RST_PANIC: return "CRASH";
    case ESP_RST_INT_WDT: case ESP_RST_TASK_WDT: case ESP_RST_WDT: return "WATCHDOG";
    case ESP_RST_SW: return "software";
    case ESP_RST_EXT: return "reset button";
    default: return "other";
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
  String r = rover.connected() ? "Rover " + String(rover.voltage(), 1) + "V" : "Rover: search";
  disp.show(boxSsid, WiFi.softAPIP().toString().c_str(), r.c_str());
}

void setup() {
  Serial.begin(115200);
  delay(200);
  Serial.printf("[boot] cas_box_mission, reset: %s\n", resetText());
  snprintf(boxSsid, sizeof(boxSsid), "CAS-Box-%02d", BOX_ID);
  snprintf(boxPassword, sizeof(boxPassword), "CASbox%02d", BOX_ID);
  const uint8_t channel = (BOX_ID % 2) ? 1 : 11;
  hmi.setTitle(String("CAS Box ") + BOX_ID + " - mission");
  hmi.enableDrivePad(MAX_SPEED);                 // limits /drive (manual buttons)
  hmi.setCustomPage(kMissionPage);
  WiFi.mode(WIFI_AP);
  WiFi.setTxPower(WIFI_TX_START);
  hmi.begin(boxSsid, boxPassword, IPAddress(192, 168, 10 + BOX_ID, 1), channel);
  wifi_config_t apCfg;
  if (esp_wifi_get_config(WIFI_IF_AP, &apCfg) == ESP_OK) { apCfg.ap.dtim_period = 1; esp_wifi_set_config(WIFI_IF_AP, &apCfg); }
  WiFi.setTxPower(WIFI_TX_RUN);
  mlog.begin(hmi.server());
  mission.begin(hmi.server(), &mlog);
  Serial.println("[boot] web ready, starting sensors (LiDAR motor) in 0.5 s");
  delay(500);
  sensors.begin(LIDAR_OFFSET_DEG);
  rover.begin(100);
  Serial.println("[boot] setup done");
}

void loop() {
  const uint32_t t0 = micros();
  hmi.tick();
  sensors.update();

  float padL, padR; uint32_t age;
  hmi.getDrive(padL, padR, age);
  if (age > PAD_TIMEOUT_MS) padL = padR = 0;
  const bool padActive = padL != 0 || padR != 0;

  sensors.setLidarNearMinInten((uint8_t)mission.nearMinInten);   // v8: LiDAR near-point intensity filter (0 = off)
  float l = 0, r = 0; bool emergency = false, wheels = false;
  mission.update(sensors, rover, padL, padR, padActive, l, r, emergency, wheels);
  const float cmdL = l, cmdR = r;
  gentle.apply(l, r, emergency, wheels);         // every command ramped (gearbox rule); turns: per wheel
  rover.setDrive(l, r);
  static uint32_t lastUs = 0;
  const float dt = lastUs ? (micros() - lastUs) / 1e6f : 0; lastUs = micros();
  mission.noteOutput(l, r, dt);

  static float loopMaxMs = 0;
  MissionLog::Sample smp;
  mission.fillLog(smp, sensors, rover);
  smp.cmdL = cmdL; smp.cmdR = cmdR; smp.outL = l; smp.outR = r; smp.loopMaxMs = loopMaxMs;
  mlog.update(sensors, smp);

  const uint32_t now = millis();
  static uint32_t lastOled = 0, lastRep = 0, lastWin = 0;
  if (now - lastOled >= 1000) { lastOled = now; updateOled(); }
  loopMaxMs = fmaxf(loopMaxMs * 0.999f, (micros() - t0) / 1000.0f);
  if (now - lastRep >= 10000) {
    lastRep = now;
    Serial.printf("[sys] heap %u kB (min %u), rover %s %.2f V, loop max %.1f ms\n",
                  (unsigned)(ESP.getFreeHeap() / 1024), (unsigned)(ESP.getMinFreeHeap() / 1024), rover.connected() ? "ok" : "searching",
                  rover.voltage(), loopMaxMs);
  }
  (void)lastWin;
  delay(2);
}
