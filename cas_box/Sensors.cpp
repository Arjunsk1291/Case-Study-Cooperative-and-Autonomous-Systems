#include "Sensors.h"
#include <Wire.h>

static const uint8_t kImuAddr = 0x28;
static const uint32_t kSnapshotMs = 250;
static const uint32_t kImuRetryMs = 5000;

bool i2cPresent(uint8_t addr) {
  Wire.beginTransmission(addr);
  return Wire.endTransmission() == 0;
}

void Sensors::begin(int lidarOffsetDeg) {
  _lidarOffsetDeg = lidarOffsetDeg;
  Wire.begin(21, 22);                 // I2C: IMU + OLED

  _us.begin(kUsMaxCm, 17500);         // 17.5 ms echo timeout ~ 3 m

  LidarScan::Config lc;               // GPIO27/33, 230400 baud, filtering see LidarScan.h
  _lidar.begin(Serial1, lc);

  _gps.begin();                       // Serial2, GPIO16/17, 9600 baud
}

void Sensors::update() {
  _lidar.update();  // drain both UARTs every pass
  _gps.poll();
  if (_imuReady) {
    _imu.update();
    // Keep turn-feedback yaw as fresh as the IMU's 100 ms update cadence. The web snapshot
    // remains 250 ms, but the motor controller must not wait for the UI snapshot to see yaw.
    _yaw = _imu.yawDeg();
    _pitch = _imu.pitchDeg();
    _roll = _imu.rollDeg();
    _imu.calibration(_cal[0], _cal[1], _cal[2], _cal[3]);
    _imuRaw = _imu.rawMeasurements();
    _imuAgeMs = _imu.ageMs();
  } else {
    _startImuIfPresent();
  }

  const uint32_t now = millis();
  if (now - _lastSnapMs < kSnapshotMs) return;
  _lastSnapMs = now;

  for (uint8_t i = 0; i < 5; ++i) {
    const UsManager::Reading r = _us.reading(i);
    _usRawCm[i] = r.rawCm;
    _usFilteredCm[i] = r.filteredCm;
    _usValid[i] = r.latestValid;
    _usAgeMs[i] = r.ageMs;
    _usTimeouts[i] = r.timeoutCount;
    // Preserve the existing controller API: an unsuccessful latest ping means max range.
    _usCm[i] = r.latestValid ? r.filteredCm : kUsMaxCm;
  }
  _snapshotLidar();
  _snapshotGps();
  ++_snapshotSeq;
}

// The IMU start blocks ~3.5 s once (north alignment); RoverLink's deadman stops the rover meanwhile.
void Sensors::_startImuIfPresent() {
  const uint32_t now = millis();
  if (now - _lastImuTryMs < kImuRetryMs) return;
  _lastImuTryMs = now;
  if (!i2cPresent(kImuAddr)) return;
  Serial.println("[sensors] BNO055 found, starting IMU (~3.5 s)");
  _imuReady = _imu.begin();
}

// Packet rate, checksum errors and the 90-point view (closest reliable bin per 4 deg).
void Sensors::_snapshotLidar() {
  const uint32_t now = millis();
  if (now - _lastPpsMs >= 1000) {
    const uint32_t packets = _lidar.packets();
    _lidarPps = (packets - _lastPackets) * 1000.0f / (now - _lastPpsMs);
    _lastPackets = packets;
    _lastPpsMs = now;
  }
  _lidarOk = _lidarPps > 1;
  _lidarCrcPct = _lidar.crcErrorPct();

  const int degPerPoint = 360 / kLidarPoints;
  for (uint8_t i = 0; i < kLidarPoints; i++) _lidarCm[i] = 0;
  for (int bin = 0; bin < 360; bin++) {
    const uint16_t mm = _lidar.distanceMm(bin);
    if (!mm) continue;
    const int deg = ((bin + _lidarOffsetDeg) % 360 + 360) % 360;
    const uint8_t i = deg / degPerPoint;
    const uint16_t cm = mm / 10;
    if (_lidarCm[i] == 0 || cm < _lidarCm[i]) _lidarCm[i] = cm;
  }
}

void Sensors::_snapshotGps() {
  _gpsFix = _gps.hasFix();
  if (_gpsFix) {
    _lat = _gps.latDeg();
    _lon = _gps.lonDeg();
  }
  _sats = _gps.satellites();
  _hdop = _gps.hdop();
  _gpsChars = _gps.charsReceived();
  _gpsAgeMs = _gps.locationAgeMs();
  _gpsAltValid = _gps.altitudeValid();
  _gpsAltitudeM = _gpsAltValid ? _gps.altitudeM() : 0;
  _gpsSpeedValid = _gps.speedValid();
  _gpsSpeedKmph = _gpsSpeedValid ? _gps.speedKmph() : 0;
  _gpsCourseValid = _gps.courseValid();
  _gpsCourseDeg = _gpsCourseValid ? _gps.courseDeg() : 0;
  _gpsUtcValid = _gps.dateTimeValid();
  if (_gpsUtcValid) {
    _gpsYear = _gps.year(); _gpsMonth = _gps.month(); _gpsDay = _gps.day();
    _gpsHour = _gps.hour(); _gpsMinute = _gps.minute(); _gpsSecond = _gps.second();
    _gpsCentisecond = _gps.centisecond();
  }
}

// {"us":[5 x cm],"usRaw":[5],"usFiltered":[5],"usValid":[5],"usAgeMs":[5],"usTimeouts":[5],"usMax":.., ...
//  "gps":{fix,sats,hdop,lat,lon,chars}}
String Sensors::toJson() const {
  String j;
  j.reserve(4200);
  j += "{\"snapshot_device_ms\":"; j += _lastSnapMs;
  j += ",\"snapshot_seq\":"; j += _snapshotSeq;
  j += ",\"us\":[";
  for (int i = 0; i < 5; i++) { if (i) j += ','; j += _usCm[i]; }
  j += "],\"usMax\":"; j += kUsMaxCm;
  j += ",\"usRaw\":[";
  for (int i = 0; i < 5; i++) { if (i) j += ','; j += _usRawCm[i]; }
  j += "],\"usFiltered\":[";
  for (int i = 0; i < 5; i++) { if (i) j += ','; j += _usFilteredCm[i]; }
  j += "],\"usValid\":[";
  for (int i = 0; i < 5; i++) { if (i) j += ','; j += _usValid[i] ? "true" : "false"; }
  j += "],\"usAgeMs\":[";
  for (int i = 0; i < 5; i++) { if (i) j += ','; if (_usAgeMs[i] == UINT32_MAX) j += -1; else j += _usAgeMs[i]; }
  j += "],\"usTimeouts\":[";
  for (int i = 0; i < 5; i++) { if (i) j += ','; j += _usTimeouts[i]; }
  j += "],\"lidar\":{\"ok\":"; j += _lidarOk ? "true" : "false";
  j += ",\"pps\":"; j += String(_lidarPps, 0);
  j += ",\"crc\":"; j += String(_lidarCrcPct, 1);
  j += ",\"d\":[";
  for (int i = 0; i < kLidarPoints; i++) { if (i) j += ','; j += _lidarCm[i]; }
  j += "],\"age_ms\":"; j += (_lidar.lastPacketMs() ? (int32_t)(millis() - _lidar.lastPacketMs()) : -1);
  j += ",\"filtered_1deg_mm\":[";
  for (int i = 0; i < 360; ++i) { if (i) j += ','; j += _lidar.distanceMm(i); }
  j += "]},\"imu\":{\"ok\":"; j += _imuReady ? "true" : "false";
  j += ",\"age_ms\":"; j += (_imuAgeMs == UINT32_MAX ? -1 : (int32_t)_imuAgeMs);
  j += ",\"raw_valid\":"; j += _imuRaw.valid ? "true" : "false";
  j += ",\"accel_m_s2\":["; j += String(_imuRaw.accelX, 4); j += ','; j += String(_imuRaw.accelY, 4); j += ','; j += String(_imuRaw.accelZ, 4); j += ']';
  j += ",\"gyro_rad_s\":["; j += String(_imuRaw.gyroX, 5); j += ','; j += String(_imuRaw.gyroY, 5); j += ','; j += String(_imuRaw.gyroZ, 5); j += ']';
  j += ",\"mag_uT\":["; j += String(_imuRaw.magX, 3); j += ','; j += String(_imuRaw.magY, 3); j += ','; j += String(_imuRaw.magZ, 3); j += ']';
  j += ",\"temperature_c\":"; j += _imuRaw.temperatureC;
  j += ",\"yaw\":"; j += String(_yaw, 1);
  j += ",\"pitch\":"; j += String(_pitch, 1);
  j += ",\"roll\":"; j += String(_roll, 1);
  j += ",\"cal\":["; j += _cal[0]; j += ','; j += _cal[1]; j += ','; j += _cal[2]; j += ','; j += _cal[3];
  j += "]},\"gps\":{\"fix\":"; j += _gpsFix ? "true" : "false";
  j += ",\"sats\":"; j += _sats;
  j += ",\"hdop\":"; j += String(_hdop, 1);
  j += ",\"lat\":"; j += String(_lat, 6);
  j += ",\"lon\":"; j += String(_lon, 6);
  j += ",\"age_ms\":"; j += (_gpsAgeMs == UINT32_MAX ? -1 : (int32_t)_gpsAgeMs);
  j += ",\"altitude_valid\":"; j += _gpsAltValid ? "true" : "false";
  j += ",\"altitude_m\":"; j += String(_gpsAltitudeM, 2);
  j += ",\"speed_valid\":"; j += _gpsSpeedValid ? "true" : "false";
  j += ",\"speed_kmph\":"; j += String(_gpsSpeedKmph, 2);
  j += ",\"course_valid\":"; j += _gpsCourseValid ? "true" : "false";
  j += ",\"course_deg\":"; j += String(_gpsCourseDeg, 2);
  j += ",\"utc_valid\":"; j += _gpsUtcValid ? "true" : "false";
  j += ",\"utc\":\"";
  char utc[32] = {0};
  if (_gpsUtcValid) snprintf(utc, sizeof(utc), "%04d-%02d-%02dT%02d:%02d:%02d.%02dZ", _gpsYear, _gpsMonth, _gpsDay, _gpsHour, _gpsMinute, _gpsSecond, _gpsCentisecond);
  j += utc; j += "\",\"chars\":"; j += _gpsChars;
  j += "}}";
  return j;
}
