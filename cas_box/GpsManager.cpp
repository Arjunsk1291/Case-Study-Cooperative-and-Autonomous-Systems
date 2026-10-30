#include "GpsManager.h"

static const int kRxPin = 16;  // ESP32 RX2 <- GPS TX
static const int kTxPin = 17;  // ESP32 TX2 -> GPS RX
static const uint32_t kBaud = 9600;
static const uint32_t kMaxFixAgeMs = 3000;

void GpsManager::begin() { Serial2.begin(kBaud, SERIAL_8N1, kRxPin, kTxPin); }

void GpsManager::poll() {
  while (Serial2.available()) _gps.encode(Serial2.read());
}

bool GpsManager::hasFix() { return _gps.location.isValid() && _gps.location.age() < kMaxFixAgeMs; }
double GpsManager::latDeg() { return _gps.location.lat(); }
double GpsManager::lonDeg() { return _gps.location.lng(); }
int GpsManager::satellites() { return _gps.satellites.isValid() ? (int)_gps.satellites.value() : 0; }
float GpsManager::hdop() { return _gps.hdop.isValid() ? (float)_gps.hdop.hdop() : 0.0f; }
uint32_t GpsManager::charsReceived() { return _gps.charsProcessed(); }

uint32_t GpsManager::locationAgeMs() { return _gps.location.isValid() ? _gps.location.age() : UINT32_MAX; }
bool GpsManager::altitudeValid() { return _gps.altitude.isValid(); }
double GpsManager::altitudeM() { return _gps.altitude.meters(); }
bool GpsManager::speedValid() { return _gps.speed.isValid(); }
double GpsManager::speedKmph() { return _gps.speed.kmph(); }
bool GpsManager::courseValid() { return _gps.course.isValid(); }
double GpsManager::courseDeg() { return _gps.course.deg(); }
bool GpsManager::dateTimeValid() { return _gps.date.isValid() && _gps.time.isValid(); }
int GpsManager::year() { return _gps.date.year(); }
int GpsManager::month() { return _gps.date.month(); }
int GpsManager::day() { return _gps.date.day(); }
int GpsManager::hour() { return _gps.time.hour(); }
int GpsManager::minute() { return _gps.time.minute(); }
int GpsManager::second() { return _gps.time.second(); }
int GpsManager::centisecond() { return _gps.time.centisecond(); }
