#pragma once
// CamLink - box side of the ESP32-CAM link (ESP-NOW), 2026-11-07. Bench-tested in link_test/ (0 lost at
// 10/s, recovers from camera power loss and box restart). Same protocol: LinkProto.h.
//
//   camLink.begin(server);   // AFTER the access point is up (same radio, channel 11)
//   camLink.update();        // every loop pass: handles HELLO / DATA / PONG, pings the camera 2 x per s
//   if (camLink.fresh()) { const MsgData& d = camLink.data(); ... }   // fresh = data younger than 0.3 s
//
// FAIL-SAFE: fresh() is false when the camera is not paired, its data is older than kStaleMs or it reports
// the sensor not running - then NOTHING may steer on the camera (the gyro heading hold continues).
// Routes: /cam/status (JSON for the page), /cam/mode?m=0|1 (0 = tactile strip, 1 = belt / narrow).
// Memory: receive queue 32 x 48 B, statistics ~100 B. Nothing is written to flash.
#include <Arduino.h>
#include <ESPAsyncWebServer.h>
#include "LinkProto.h"

class CamLink {
public:
  bool begin(AsyncWebServer* server);
  void update();
  bool paired() const { return _paired; }
  bool fresh() const;
  const MsgData& data() const { return _last; }
  uint8_t mode() const { return _mode; }
  void setMode(uint8_t m);
  String statusJson() const;       // for the page (/cam/status), from the copies (web task)

  // ON DEMAND (mission): rate 0 = the camera sends nothing until asked.
  //   requestSnap(mode) -> one picture is taken on the camera -> snapState() becomes SNAP_OK (result in
  //   snapResult()) or SNAP_FAIL after snapTimeoutMs (no pairing / no answer). Never blocks.
  enum SnapState : uint8_t { SNAP_IDLE = 0, SNAP_WAIT, SNAP_OK, SNAP_FAIL };
  bool requestSnap(uint8_t mode);  // false = not paired (state -> SNAP_FAIL at once)
  uint8_t snapState() const { return _snap; }
  const MsgData& snapResult() const { return _snapRes; }
  void snapClear() { _snap = SNAP_IDLE; }
  uint32_t snapRttMs() const { return _snapRtt; }
  uint32_t snapTimeoutMs = 500;
  uint32_t snapsSent() const { return _snapsSent; }
  uint32_t snapsOk() const { return _snapsOk; }

  // statistics of the last finished 2 s window (loop only); windowNo() counts up when a new one is ready
  struct Win { uint32_t rx = 0, lost = 0, dup = 0, restarts = 0, pings = 0, pongs = 0, gapMaxMs = 0, staleMs = 0;
               uint32_t rttMinUs = 0xFFFFFFFF, rttMaxUs = 0; uint64_t rttSumUs = 0; uint32_t rttN = 0; int rssiSum = 0, rssiN = 0; };
  const Win& lastWindow() const { return _done; }
  uint32_t windowNo() const { return _winNo; }

private:
  void _handle(const uint8_t* mac, const uint8_t* d, int len, int8_t rssi, uint32_t us);
  void _send(const void* m, size_t n);

  bool _paired = false;
  uint8_t _mac[6] = {0};
  uint8_t _mode = 0, _rate = 0;     // 0 = on demand (mission). 10 = stream (link test)
  volatile uint8_t _snap = SNAP_IDLE;
  MsgData _snapRes = {};
  uint32_t _snapReqMs = 0, _snapRtt = 0, _snapsSent = 0, _snapsOk = 0;
  volatile uint8_t _reqMode = 255;
  uint32_t _seq = 0, _lastSeq = 0, _lastDataMs = 0, _lastDataUs = 0, _lastPingMs = 0, _lastTickMs = 0;
  MsgData _last = {};
  Win _w, _done;
  uint32_t _winNo = 0, _winStartMs = 0;
  // copies for the web task
  mutable portMUX_TYPE _mux = portMUX_INITIALIZER_UNLOCKED;
  struct Pub { bool paired, fresh; MsgData d; float rxPerS; uint32_t lost, gapMaxMs; float rttAvgMs, rttMaxMs; int rssi; uint8_t mode; } _pub = {};
};

extern CamLink camLink;
