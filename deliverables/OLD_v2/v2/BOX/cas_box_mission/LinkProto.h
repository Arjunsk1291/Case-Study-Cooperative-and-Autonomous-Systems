#pragma once
// LinkProto - ESP-NOW messages between the ESP32-CAM and the box (2026-11-07). Copy into both sketches.
//
// WHY ESP-NOW: one small radio frame, no connection, no IP / TCP, ~1-3 ms. Works next to the box's own
// Wi-Fi access point as long as both use the SAME CHANNEL (kit 4: channel 11). No images over the air:
// the camera sends only its result (~30 bytes).
//
// HANDSHAKE
//   camera  --HELLO (broadcast, 1 per s)-->  box          "I am camera <id>"
//   box     --ACK (to the camera's MAC)-->   camera       "I am the box, talk only to me"
//   camera  --DATA (to the box, 10 per s)--> box          strip result, sequence number, timing
//   box     --PING (2 per s)-->              camera  --PONG--> box   round-trip time
//   box     --CFG-->                         camera       message rate (0 = on demand) / strip width mode
//   box     --REQ-->                         camera  --DATA--> box   one snapshot on demand
// FAIL-SAFE
//   box:    DATA older than kStaleMs -> "no camera" (navigation falls back to the gyro)
//   camera: no ACK / PING from the box for kLostMs -> back to HELLO (search again)
//   both:   messages with a wrong magic / version / size or from an unknown MAC are ignored
#include <stdint.h>

static const uint8_t kLinkChannel = 11;       // = box access point channel (BOX_ID 4: even -> 11)
static const uint16_t kMagic = 0xCA54;        // "CAS"
static const uint8_t kVersion = 1;
static const uint32_t kStaleMs = 300;         // box: older camera data does not count
static const uint32_t kLostMs = 3000;         // camera: box silent this long -> search again

enum MsgType : uint8_t { MSG_HELLO = 1, MSG_ACK = 2, MSG_DATA = 3, MSG_PING = 4, MSG_PONG = 5, MSG_CFG = 6, MSG_REQ = 7, MSG_IMG = 8 };
// ON DEMAND (2026-11-07): rateHz = 0 in ACK / CFG -> the camera sends NO stream, it waits for MSG_REQ and
// answers each one with exactly one MSG_DATA (one picture taken, processed, buffer handed back at once).

#pragma pack(push, 1)
struct MsgHdr {
  uint16_t magic;
  uint8_t version;
  uint8_t type;
  uint32_t seq;
};
struct MsgHello { MsgHdr h; uint32_t upMs; uint8_t camOk; };
struct MsgAck { MsgHdr h; uint8_t rateHz; uint8_t mode; };
struct MsgData {
  MsgHdr h;
  uint32_t camMs;       // camera clock when the frame was taken
  uint8_t camOk;        // 1 = sensor running
  uint8_t stripOk;      // 1 = strip found (3 of 4 rows on one line, 2 of the last 3 frames)
  uint8_t rows;         // rows with a hit (0..4)
  uint8_t conf;         // confidence x 100
  int16_t offX10;       // strip offset at the bottom of the (rotated) image, px x 10, + = right of centre
  int16_t angX10;       // strip angle, deg x 10, + = leans right ahead
  uint16_t procUs;      // detector time on the camera
  uint16_t capUs;       // frame grab time
  uint16_t camResets;   // camera re-inits since start (watchdog)
  uint8_t bright;       // mean brightness 0..255 (exposure check)
};
struct MsgPing { MsgHdr h; uint32_t boxUs; };
struct MsgPong { MsgHdr h; uint32_t boxUs; uint32_t camMs; };
struct MsgCfg { MsgHdr h; uint8_t rateHz; uint8_t mode; };   // mode 0 = tactile strip (wide), 1 = belt (narrow)
struct MsgReq { MsgHdr h; uint8_t mode; };                    // "take one snapshot now" (answer: one MSG_DATA)
// the picture of an on-demand snapshot, JPEG in chunks (sent right after its MSG_DATA): snapId = DATA seq & 0xFFFF
static const uint8_t kImgChunk = 200;
struct MsgImg { MsgHdr h; uint16_t snapId, total, off; uint8_t len; uint8_t d[kImgChunk]; };
#pragma pack(pop)

inline bool linkValid(const uint8_t* d, int len, uint8_t type, int size) {
  if (len != size) return false;
  const MsgHdr* h = (const MsgHdr*)d;
  return h->magic == kMagic && h->version == kVersion && h->type == type;
}
inline uint8_t linkType(const uint8_t* d, int len) {
  if (len < (int)sizeof(MsgHdr)) return 0;
  const MsgHdr* h = (const MsgHdr*)d;
  return (h->magic == kMagic && h->version == kVersion) ? h->type : 0;
}
