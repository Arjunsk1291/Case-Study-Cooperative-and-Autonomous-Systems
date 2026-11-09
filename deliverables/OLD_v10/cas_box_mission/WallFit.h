#pragma once
// WallFit - one LiDAR wall-line fit on the left or right side (the team's WallFollow::measure, unchanged
// logic, tested on log_003: d = 1.52 m +- 3.5 cm, fit error ~1 cm). Header-only.
//   side frame: x forward, y towards the wall. angleDeg = 0 parallel, < 0 nose turned TOWARDS the wall.
#include <Arduino.h>
#include <math.h>
#include "Sensors.h"

struct WallFit {
  bool ok; float distM, angleDeg, rmsM; uint8_t points;

  static bool fitLine(const float* x, const float* y, int n, float& mx, float& my, float& ux, float& uy) {
    if (n < 2) return false;
    mx = my = 0;
    for (int i = 0; i < n; i++) { mx += x[i]; my += y[i]; }
    mx /= n; my /= n;
    float sxx = 0, syy = 0, sxy = 0;
    for (int i = 0; i < n; i++) { const float dx = x[i] - mx, dy = y[i] - my; sxx += dx * dx; syy += dy * dy; sxy += dx * dy; }
    const float phi = 0.5f * atan2f(2.0f * sxy, sxx - syy);
    ux = cosf(phi); uy = sinf(phi);
    if (ux < 0) { ux = -ux; uy = -uy; }
    return true;
  }

  // left = true: wall on the left. prior: last good fit (tracking through door gaps) or nullptr.
  static WallFit measure(const Sensors& s, bool left, const WallFit* prior) {
    WallFit w = {false, 0, 0, 0, 0};
    if (!s.lidarOk()) return w;
    const int n = Sensors::kLidarPoints;
    const float degPer = 360.0f / n, center = left ? 270.0f : 90.0f;
    float px[Sensors::kLidarPoints], py[Sensors::kLidarPoints], off[Sensors::kLidarPoints];
    int m = 0;
    for (int i = 0; i < n; i++) {
      const uint16_t cm = s.lidarCm(i);
      if (cm < 20 || cm > 300) continue;
      const float a = i * degPer + degPer / 2;
      const float o = fabsf(fmodf(a - center + 540.0f, 360.0f) - 180.0f);
      if (o > 60.0f) continue;
      const float r = cm / 100.0f, ar = a * DEG_TO_RAD;
      px[m] = r * cosf(ar);
      py[m] = r * sinf(ar) * (left ? -1.0f : 1.0f);
      off[m] = o;
      m++;
    }
    float qx[Sensors::kLidarPoints], qy[Sensors::kLidarPoints];
    int k = 0;
    float mx, my, ux, uy;
    if (prior && prior->ok) {
      const float th = prior->angleDeg * DEG_TO_RAD, nx = -sinf(th), ny = cosf(th);
      for (int i = 0; i < m; i++) if (fabsf(nx * px[i] + ny * py[i] - prior->distM) < 0.15f) { qx[k] = px[i]; qy[k] = py[i]; k++; }
      if (k < 5) return w;
    } else {
      for (int i = 0; i < m; i++) if (off[i] <= 20.0f) { qx[k] = px[i]; qy[k] = py[i]; k++; }
      if (k < 4) return w;
    }
    if (!fitLine(qx, qy, k, mx, my, ux, uy)) return w;
    for (int it = 0; it < 2; it++) {
      k = 0;
      for (int i = 0; i < m; i++) if (fabsf(-uy * (px[i] - mx) + ux * (py[i] - my)) < 0.10f) { qx[k] = px[i]; qy[k] = py[i]; k++; }
      if (k < 5) return w;
      if (!fitLine(qx, qy, k, mx, my, ux, uy)) return w;
    }
    float sMin = 1e9f, sMax = -1e9f, sq = 0, xMax = -1e9f;
    for (int i = 0; i < k; i++) {
      xMax = fmaxf(xMax, qx[i]);
      const float along = qx[i] * ux + qy[i] * uy;
      sMin = fminf(sMin, along); sMax = fmaxf(sMax, along);
      const float res = -uy * (qx[i] - mx) + ux * (qy[i] - my);
      sq += res * res;
    }
    if (sMax - sMin < 0.6f) return w;
    if (xMax < -0.1f) return w;
    w.ok = true;
    w.distM = fabsf(-uy * mx + ux * my);
    w.angleDeg = atan2f(uy, ux) * RAD_TO_DEG;
    w.rmsM = sqrtf(sq / k);
    w.points = (uint8_t)k;
    return w;
  }
};
