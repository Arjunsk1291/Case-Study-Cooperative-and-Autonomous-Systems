#include "UsManager.h"

// HC-SR04 echoes can linger. A 25 ms quiet gap is a conservative starting point for
// reducing cross-talk between separate modules. Tune only after bench testing.
static constexpr uint32_t kPauseBetweenSensorsMs = 25;

void UsManager::begin(int maxDistanceCm, uint32_t timeoutUs) {
  if (_task) return;
  _maxDistanceCm = maxDistanceCm;
  _timeoutUs = timeoutUs;
  for (uint8_t i = 0; i < kSensorCount; ++i) {
    _rawCm[i] = -1;
    _filteredCm[i] = _maxDistanceCm;
    _latestValid[i] = false;
    _lastGoodMs[i] = 0;
    _timeoutCount[i] = 0;
    _historyCount[i] = _historyNext[i] = 0;
  }

  const int pins[5][2] = {{_pins.trigFront, _pins.echoFront}, {_pins.trigLeft, _pins.echoLeft},
                          {_pins.trigRight, _pins.echoRight}, {_pins.trigBack, _pins.echoBack},
                          {_pins.trigFrontAngled, _pins.echoFrontAngled}};
  for (const auto& p : pins) {
    pinMode(p[0], OUTPUT);
    digitalWrite(p[0], LOW);
    pinMode(p[1], INPUT);
  }
  // Keep the existing task/core arrangement so the web/rover loop is not blocked by pulseIn().
  xTaskCreatePinnedToCore(_taskEntry, "ultrasonic", 4096, this, 1, &_task, 0);
}

int UsManager::_measureCm(int trigPin, int echoPin) {
  digitalWrite(trigPin, LOW);
  delayMicroseconds(3);
  digitalWrite(trigPin, HIGH);
  delayMicroseconds(10);
  digitalWrite(trigPin, LOW);
  const uint32_t us = pulseIn(echoPin, HIGH, _timeoutUs);
  if (us == 0) return -1;
  const int cm = static_cast<int>((us + 29) / 58); // rounded cm; ~58 us per cm
  if (cm <= 0 || cm > _maxDistanceCm) return -1;
  return cm;
}

int UsManager::_median(uint8_t index) const {
  const uint8_t count = _historyCount[index];
  if (!count) return _maxDistanceCm;
  int sorted[kFilterWindow];
  for (uint8_t i = 0; i < count; ++i) sorted[i] = _history[index][i];
  for (uint8_t i = 1; i < count; ++i) {
    const int value = sorted[i];
    int j = i - 1;
    while (j >= 0 && sorted[j] > value) { sorted[j + 1] = sorted[j]; --j; }
    sorted[j + 1] = value;
  }
  return sorted[count / 2];
}

void UsManager::_publish(uint8_t index, int rawCm, bool valid) {
  _rawCm[index] = rawCm;
  _latestValid[index] = valid;
  if (valid) {
    _history[index][_historyNext[index]] = rawCm;
    _historyNext[index] = (_historyNext[index] + 1) % kFilterWindow;
    if (_historyCount[index] < kFilterWindow) ++_historyCount[index];
    _filteredCm[index] = _median(index);
    _lastGoodMs[index] = millis();
  } else {
    ++_timeoutCount[index];
  }
}

UsManager::Reading UsManager::reading(uint8_t index) const {
  Reading r;
  if (index >= kSensorCount) {
    r.rawCm = -1; r.filteredCm = _maxDistanceCm; r.latestValid = false;
    r.ageMs = UINT32_MAX; r.timeoutCount = 0;
    return r;
  }
  r.rawCm = _rawCm[index];
  r.filteredCm = _filteredCm[index];
  r.latestValid = _latestValid[index];
  const uint32_t last = _lastGoodMs[index];
  r.ageMs = last == 0 ? UINT32_MAX : millis() - last;
  r.timeoutCount = _timeoutCount[index];
  return r;
}

int UsManager::_safeCm(uint8_t index) const {
  const Reading r = reading(index);
  return r.latestValid ? r.filteredCm : _maxDistanceCm;
}

void UsManager::_taskEntry(void* self) { static_cast<UsManager*>(self)->_taskLoop(); }

void UsManager::_taskLoop() {
  for (;;) {
    { const int v = _measureCm(_pins.trigFront, _pins.echoFront); _publish(0, v, v >= 0); }
    vTaskDelay(pdMS_TO_TICKS(kPauseBetweenSensorsMs));
    { const int v = _measureCm(_pins.trigLeft, _pins.echoLeft); _publish(1, v, v >= 0); }
    vTaskDelay(pdMS_TO_TICKS(kPauseBetweenSensorsMs));
    { const int v = _measureCm(_pins.trigRight, _pins.echoRight); _publish(2, v, v >= 0); }
    vTaskDelay(pdMS_TO_TICKS(kPauseBetweenSensorsMs));
    { const int v = _measureCm(_pins.trigBack, _pins.echoBack); _publish(3, v, v >= 0); }
    vTaskDelay(pdMS_TO_TICKS(kPauseBetweenSensorsMs));
    { const int v = _measureCm(_pins.trigFrontAngled, _pins.echoFrontAngled); _publish(4, v, v >= 0); }
    vTaskDelay(pdMS_TO_TICKS(kPauseBetweenSensorsMs));
  }
}
