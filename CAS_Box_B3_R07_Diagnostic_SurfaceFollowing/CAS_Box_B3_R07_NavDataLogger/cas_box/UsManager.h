#pragma once
// Five HC-SR04 sensors. Measurements run on a dedicated FreeRTOS task.
// Echo pins must be protected by the project's 5 V -> 3.3 V voltage dividers.
#include <Arduino.h>

class UsManager {
public:
  struct Pins {
    int trigFront = 32, echoFront = 25;
    int trigLeft = 4, echoLeft = 18;
    int trigRight = 13, echoRight = 19;
    int trigBack = 5, echoBack = 14;
    int trigFrontAngled = 23, echoFrontAngled = 26;
  };

  static constexpr uint8_t kSensorCount = 5;
  static constexpr uint8_t kFilterWindow = 5;

  struct Reading {
    int rawCm;             // latest echo in cm; -1 means no echo on latest attempt
    int filteredCm;        // median of recent valid readings; maxDistanceCm if none yet
    bool latestValid;      // whether the most recent attempt got a valid echo
    uint32_t ageMs;        // age of most recent valid echo; UINT32_MAX if never valid
    uint32_t timeoutCount; // cumulative no-echo/out-of-range attempts
  };

  void begin(int maxDistanceCm, uint32_t timeoutUs);
  Reading reading(uint8_t index) const;

  // Compatibility getters used by the existing controller: a failed latest attempt
  // returns maxDistanceCm, so callers do not mistake an old value for a current one.
  int readFrontCm() const { return _safeCm(0); }
  int readLeftCm() const { return _safeCm(1); }
  int readRightCm() const { return _safeCm(2); }
  int readBackCm() const { return _safeCm(3); }
  int readFrontAngledCm() const { return _safeCm(4); }

private:
  int _safeCm(uint8_t index) const;
  int _measureCm(int trigPin, int echoPin);
  static void _taskEntry(void* self);
  void _taskLoop();
  void _publish(uint8_t index, int rawCm, bool valid);
  int _median(uint8_t index) const;

  Pins _pins;
  int _maxDistanceCm = 300;
  uint32_t _timeoutUs = 17500;
  TaskHandle_t _task = nullptr;

  // Sensor task writes these; reads/writes of each scalar are atomic on ESP32.
  volatile int _rawCm[kSensorCount] = {-1, -1, -1, -1, -1};
  volatile int _filteredCm[kSensorCount] = {300, 300, 300, 300, 300};
  volatile bool _latestValid[kSensorCount] = {false, false, false, false, false};
  volatile uint32_t _lastGoodMs[kSensorCount] = {0, 0, 0, 0, 0};
  volatile uint32_t _timeoutCount[kSensorCount] = {0, 0, 0, 0, 0};
  int _history[kSensorCount][kFilterWindow] = {};
  uint8_t _historyCount[kSensorCount] = {0, 0, 0, 0, 0};
  uint8_t _historyNext[kSensorCount] = {0, 0, 0, 0, 0};
};
