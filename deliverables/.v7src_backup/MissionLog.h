#pragma once
// MissionLog - records the whole drive to the box flash (LittleFS, the 1.4 MB data partition), 2026-11-07.
// Based on the team's DriveLog (proven on this box): 4 records per second, written in blocks of 16 (every
// 4 s, one ~4 kB write), a power cut loses at most the last 4 s. ~1 kB/s -> ~20 minutes fit.
// The partition is SEPARATE from the program: logging can never fill the program memory or the RAM.
// Format v4 ("CASLOG" header, then records), decoded by tools/decode_mission_log.py:
//   time, GPS (lat/lon/hdop/sats - kept, v3 had dropped them), IMU yaw/pitch/roll, wheel command and output,
//   battery, rover round trip, 5 ultrasonics, LiDAR 90 x 4 deg, AND mission state, leg, event, pillars,
//   target heading, heading error, crab estimate, wall distance + angle, leg distance, free space ahead,
//   camera snapshot result (rows, offset, angle), snapshot counters, loop time.
// Events (one per record, queued so none is lost): see enum Ev.
#include <Arduino.h>
#include <FS.h>
#include <ESPAsyncWebServer.h>
#include "Sensors.h"

class MissionLog {
public:
  enum Ev : uint8_t {
    EV_NONE = 0, EV_START = 1, EV_PILLAR = 2, EV_T1_ASK = 3, EV_OP_TURN = 4, EV_OP_ON1M = 5, EV_MANUAL_ON = 6,
    EV_MANUAL_OFF = 7, EV_TURN_DONE = 8, EV_LOOK = 9, EV_ALIGN = 10, EV_GO_L2 = 11, EV_CHECK_OK = 12, EV_CHECK_FAIL = 13,
    EV_HOLD = 14, EV_HOLD_END = 15, EV_END_WALL = 16, EV_PAUSE = 17, EV_RESUME = 18, EV_MARK = 19, EV_L2_MAX = 20,
    EV_FAILSAFE = 21, EV_RIGHT_OPEN = 22, EV_WALL_LOST = 23, EV_WALL_FOUND = 24, EV_SNAP_OK = 25, EV_SNAP_FAIL = 26,
    EV_DONE = 27, EV_TURN_STALL = 28, EV_IMG = 29, EV_L3_START = 30, EV_WALL_END = 31, EV_TURN_PILLAR = 32, EV_AIM_DONE = 33,
    EV_REALIGN = 34, EV_BOARD_SEEN = 35, EV_BOARD_LOCK = 36, EV_END_BOARD = 37, EV_T3_TURN = 38, EV_L4_START = 39,
    EV_FRONT_IS_WALL = 40, EV_REF_REAR = 41, EV_REF_FRONT = 42, EV_REF_BOARD = 43,
    EV_REF_ROW = 44, EV_REF_BLD = 45, EV_L4_WALL = 46,
    EV_P_STOP = 47, EV_P_PHONE = 48, EV_P_ROVER = 49, EV_P_TILT = 50, EV_P_TURN = 51, EV_P_BLOCKED = 52, EV_P_DRIVE = 53
  };
  struct Sample {                 // filled by the mission every loop pass
    float cmdL = 0, cmdR = 0, outL = 0, outR = 0, batteryV = 0, rttMs = 0;
    uint8_t state = 0, leg = 0, pillars = 0, dsStatus = 0;
    float tgtDeg = 0, errDeg = 0, crabDeg = 0, wallD = 0, wallAngDeg = 0, distM = 0, frontM = 9.9f;
    bool camPaired = false, snapOk = false, manual = false, hold = false, slope = false;
    uint8_t camRows = 0, camConf = 0; float camOff = 0, camAng = 0;
    uint16_t snapsSent = 0, snapsOk = 0;
    float loopMaxMs = 0;
  };

  static const uint16_t kPeriodMs = 250;
  static const uint8_t kBuf = 16;
  static const uint32_t kMinFreeBytes = 32 * 1024;

  bool begin(AsyncWebServer* server);
  void update(const Sensors& s, const Sample& d);
  void event(uint8_t ev);                    // queue an event (loop task)
  void start() { _reqStart = true; }
  void stop() { _reqStop = true; }
  bool active() const { return _active; }
  // camera picture of a snapshot -> img_NNN.bin (same NNN as mis_NNN.bin). Per picture: "IMG1", record index (u32),
  // ms (u32), snapId (u16), strip ok (u8), rows (u8), offset px x10 (i16), angle deg x10 (i16), JPEG length (u16), JPEG.
  void saveImage(const uint8_t* jpg, uint16_t len, uint16_t snapId, uint8_t ok, uint8_t rows, int16_t offX10, int16_t angX10);
  uint16_t imagesSaved() const { return _imgN; }
  String statusJson() const;                 // web task: copies only

#pragma pack(push, 1)
  struct Header { char magic[6]; uint8_t version, lidarPoints; uint16_t recordSize, periodMs; uint32_t startMs; };
  struct Rec {
    uint32_t ms;
    int32_t latE7, lonE7; uint16_t hdopX100; uint8_t sats;
    uint8_t flags;              // 0 GPS fix, 1 IMU ok, 2 LiDAR ok, 3 camera paired, 4 manual, 5 hold, 6 on slope, 7 snapshot ok
    int16_t yawX100, pitchX100, rollX100;
    uint8_t state, leg, event, pillars;
    int8_t cmdL, cmdR, outL, outR;  // x 200
    uint8_t rttX10ms; uint16_t batteryX100;
    uint16_t usCm[5];
    uint16_t lidarCm[Sensors::kLidarPoints];
    int16_t tgtX100, errX100, crabX100;
    uint16_t wallDcm; int16_t wallAngX10; uint16_t distCm, frontCm;
    uint8_t camRows, camConf; int16_t camOffX10, camAngX10; uint16_t snapsSent, snapsOk;
    uint8_t loopMaxX4ms, dsStatus;
  };
#pragma pack(pop)
  // must match tools/decode_mission_log.py ('<IiiHBBhhhBBBBbbbbBH5H90HhhhHhHHBBhhHHBB' = 249 bytes)
  static_assert(sizeof(Rec) == 249, "MissionLog::Rec layout changed - update tools/decode_mission_log.py");
  static_assert(sizeof(Header) == 16, "MissionLog::Header layout");

private:
  void _start(); void _stop(); void _flush(); void _clear(); void _refreshList();
  void _fill(Rec& r, const Sensors& s, const Sample& d, uint8_t ev);
  static bool _validName(const String& n);

  bool _fsOk = false;
  volatile bool _reqStart = false, _reqStop = false, _reqClear = false;
  volatile bool _active = false;
  volatile uint32_t _records = 0, _usedKB = 0, _totalKB = 0;
  char _fileName[20] = "", _error[40] = "", _filesJson[420] = "";
  mutable portMUX_TYPE _mux = portMUX_INITIALIZER_UNLOCKED;
  fs::File _file, _imgFile;
  uint16_t _imgN = 0;
  uint32_t _lastRecMs = 0;
  Rec _buf[kBuf]; uint8_t _bufN = 0;
  uint8_t _evQ[16]; uint8_t _evHead = 0, _evTail = 0;
  volatile uint16_t _userMarks = 0; uint16_t _seenMarks = 0;
};
