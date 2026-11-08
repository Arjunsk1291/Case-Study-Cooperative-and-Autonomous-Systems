#pragma once
// LineRef - wall references for the open sea + gyro drift estimate (v5, 2026-11-07 after campus4). Header-only.
//
// sectorFit(): robust straight-line fit (RANSAC over point pairs, then least squares) on the LiDAR points within
//   +-halfDeg of centerDeg (0 = front, 180 = behind, clockwise), range rMin..rMax. Accepted: >= 6 inliers (0.12 m),
//   >= minLen long, rms <= 0.07 m. Out: line angle in the rover frame (deg, + = the line turns to the RIGHT going
//   forward) and its distance. ~30 points -> ~13000 cheap operations, every 250 ms.
//   Campus logs: the B facade BEHIND the rover is seen for the first 7-10 m of the open sea (3-4 deg from the
//   leg-1 wall), the end wall AHEAD from ~10 m out (parallel to B within 1-3 deg).
//
// DriftEst: the BNO055 in IMUPLUS mode drifts while DRIVING (standing still: 0.00 deg in all logs). A straight
//   wall's direction measured in the gyro frame should be constant -> its slope over driving time = the drift.
#include <Arduino.h>
#include <math.h>
#include "Sensors.h"
#include "WallFit.h"

struct LineRef {
  static float wrap180(float a) { a = fmodf(a + 180.0f, 360.0f); if (a < 0) a += 360.0f; return a - 180.0f; }

  static bool sectorFit(const Sensors& s, float centerDeg, float halfDeg, float rMin, float rMax, float minLen,
                        float& angDeg, float& distM) {
    if (!s.lidarOk()) return false;
    float px[Sensors::kLidarPoints], py[Sensors::kLidarPoints];
    int m = 0;
    for (int i = 0; i < Sensors::kLidarPoints; i++) {
      const uint16_t cm = s.lidarCm(i);
      if (!cm || cm < rMin * 100 || cm > rMax * 100) continue;
      const float a = i * 4 + 2.0f;
      const float o = fabsf(fmodf(a - centerDeg + 540.0f, 360.0f) - 180.0f);
      if (o > halfDeg) continue;
      const float d = cm / 100.0f, ar = a * DEG_TO_RAD;
      px[m] = d * cosf(ar); py[m] = d * sinf(ar); m++;
    }
    if (m < 6) return false;
    int bestN = 0; float bnx = 0, bny = 0, bc = 0;
    for (int i = 0; i < m; i++) {
      for (int j = i + 1; j < m; j++) {
        const float dx = px[j] - px[i], dy = py[j] - py[i], n = sqrtf(dx * dx + dy * dy);
        if (n < 0.5f) continue;
        const float nx = -dy / n, ny = dx / n, c = nx * px[i] + ny * py[i];
        int k = 0;
        for (int q = 0; q < m; q++) if (fabsf(nx * px[q] + ny * py[q] - c) < 0.12f) k++;
        if (k > bestN) { bestN = k; bnx = nx; bny = ny; bc = c; }
      }
    }
    if (bestN < 6) return false;
    float qx[Sensors::kLidarPoints], qy[Sensors::kLidarPoints];
    int k = 0;
    for (int q = 0; q < m; q++) if (fabsf(bnx * px[q] + bny * py[q] - bc) < 0.12f) { qx[k] = px[q]; qy[k] = py[q]; k++; }
    float mx, my, ux, uy;
    if (!WallFit::fitLine(qx, qy, k, mx, my, ux, uy)) return false;
    float sMin = 1e9f, sMax = -1e9f, sq = 0;
    for (int q = 0; q < k; q++) {
      const float al = qx[q] * ux + qy[q] * uy;
      sMin = fminf(sMin, al); sMax = fmaxf(sMax, al);
      const float r = -uy * (qx[q] - mx) + ux * (qy[q] - my); sq += r * r;
    }
    if (sMax - sMin < minLen || sqrtf(sq / k) > 0.07f) return false;
    angDeg = atan2f(uy, ux) * RAD_TO_DEG;
    distM = fabsf(-uy * mx + ux * my);
    return true;
  }

  // heading (gyro frame) at right angles to a line seen at rover-frame angle lineAng, the one closest to expect
  static float perpHeading(float yaw, float lineAng, float expect) {
    const float ld = wrap180(yaw - lineAng);
    const float a = wrap180(ld + 90.0f), b = wrap180(ld - 90.0f);
    return fabsf(wrap180(a - expect)) <= fabsf(wrap180(b - expect)) ? a : b;
  }
};

class DriftEst {
  // v5b (replayed on campus_2/3/4): a straight wall followed for >= 15 s of driving gives the drift with 0.3 deg
  // scatter: +4.7, +5.8, +5.3, +6.0, +3.6 deg/min (one outlier -5.7 on a not-straight wall section). So:
  // running least squares per wall stretch; a stretch counts if >= 15 s and within 6 deg/min of the prior
  // (+5 deg/min); result = span-weighted mean of the stretches with the prior (weight 10 s); clamp -2..+9 deg/min.
public:
  void reset() { _W = 0; _R = 0; newSegment(); }
  void newSegment() { _finish(); _n = 0; _st = _sd = _stt = _std = 0; _t0 = 0; }
  void add(float tS, float dirDeg) {
    if (_n == 0) { _ref = dirDeg; _t0 = tS; }
    const float t = tS - _t0, d = LineRef::wrap180(dirDeg - _ref);
    _n++; _st += t; _sd += d; _stt += t * t; _std += t * d; _tLast = t;
  }
  float rate() const {                                // deg per driving second
    float W = _W + kPriorW, R = _R + kPrior * kPriorW, r;
    if (_segRate(r) && fabsf(r - kPrior) <= kOutlier) { W += _tLast; R += r * _tLast; }   // current stretch, provisional
    return fmaxf(kMin, fminf(kMax, R / W));
  }
  bool measured() const { float r; return _W > 0 || (_segRate(r) && fabsf(r - kPrior) <= kOutlier); }
private:
  static constexpr float kPrior = 5.0f / 60.0f, kPriorW = 10.0f, kOutlier = 6.0f / 60.0f;
  static constexpr float kMin = -2.0f / 60.0f, kMax = 9.0f / 60.0f;
  bool _segRate(float& r) const {
    if (_n < 40 || _tLast < 15.0f) return false;
    const float den = _n * _stt - _st * _st;
    if (den <= 0) return false;
    r = (_n * _std - _st * _sd) / den;
    return true;
  }
  void _finish() { float r; if (_segRate(r) && fabsf(r - kPrior) <= kOutlier) { _W += _tLast; _R += r * _tLast; } }
  float _W = 0, _R = 0, _ref = 0, _t0 = 0, _tLast = 0;
  double _st = 0, _sd = 0, _stt = 0, _std = 0;
  int _n = 0;
};
