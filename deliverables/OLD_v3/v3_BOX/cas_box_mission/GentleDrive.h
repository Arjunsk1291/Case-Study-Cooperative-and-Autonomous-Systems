#pragma once
// GentleDrive - the LAST stage before rover.setDrive(): every change of the wheel commands is gradual
// (gearbox rule of the team, 2026-11-06). Works on speed v = (l + r) / 2 and turn w = (r - l) / 2.
//
//   speed up     from standstill: first step vStart (0.05), then linear to the target in riseS (2.5 s)
//   slow down    normal: vSoft per second (0.25 -> 0 in ~0.8 s)
//                emergency (obstacle very close, tilt): vHard per second (0.25 -> 0 in ~0.3 s) -
//                faster, but still a ramp, NEVER a jump to 0
//   reverse      always passes through 0 first
//   turn         rises wUp per second (0 -> 0.50 in ~0.6 s: the rover only turns at full pivot),
//                falls wDown per second
//
//   TURN MODE (wheels = true, 07.11. campus_2): each wheel ramped ON ITS OWN - up wheelUp per second (0 -> 0.50 in
//                0.25 s: the rover only turns with a quick full-power push), down wheelDown per second, through 0 on a
//                change of direction. Without it a swing turn (0.50 / 0) came out as 0.42 / -0.07 (the v/w split mixed
//                the wheels) and a pivot needed 0.6 s to reach full power.
//
//   gentle.apply(l, r, emergency, wheels);   // l / r in: target, out: what may go to the rover now
#include <Arduino.h>
#include <math.h>

class GentleDrive {
public:
  float vStart = 0.05f;   // first step from standstill
  float riseS = 2.5f;     // vStart -> target in this time
  float vSoft = 0.30f;    // normal slow-down / stop, per second
  float vHard = 0.80f;    // emergency slow-down, per second (still a ramp)
  float wUp = 0.80f;      // turn rise per second
  float wDown = 1.00f;    // turn fall per second
  float wheelUp = 5.0f;   // turn mode: one wheel rises per second (0 -> 0.5 in 0.1 s = Kethan's step to full power)
  float wheelDown = 1.0f; // turn mode: one wheel falls per second

  void apply(float& l, float& r, bool emergency, bool wheels = false) {
    const uint32_t now = millis();
    float dt = _ms ? (now - _ms) / 1000.0f : 0.0f;
    _ms = now;
    if (dt > 0.1f) dt = 0.1f;
    if (wheels) {                                   // turn mode: per wheel
      float cl = _v - _w, cr = _v + _w;             // what goes out now (continuous with the v/w mode)
      _wheel(cl, constrain(l, -0.5f, 0.5f), dt, emergency);
      _wheel(cr, constrain(r, -0.5f, 0.5f), dt, emergency);
      _v = 0.5f * (cl + cr); _w = 0.5f * (cr - cl);
      if (fabsf(_v) < 0.002f && l == 0 && r == 0) _v = 0;
      if (fabsf(_w) < 0.002f && l == 0 && r == 0) _w = 0;
      l = fabsf(cl) < 0.005f ? 0 : cl;
      r = fabsf(cr) < 0.005f ? 0 : cr;
      return;
    }
    float vT = constrain(0.5f * (l + r), -0.5f, 0.5f);
    float wT = constrain(0.5f * (r - l), -0.5f, 0.5f);

    // speed
    if (_v != 0 && vT != 0 && (_v > 0) != (vT > 0)) vT = 0;   // reverse: through 0 first
    if (fabsf(vT) > fabsf(_v)) {
      if (_v == 0) _v = vT > 0 ? fminf(vStart, vT) : fmaxf(-vStart, vT);
      const float rate = fmaxf(0.02f, (fabsf(vT) - vStart) / riseS);
      _v += constrain(vT - _v, -rate * dt, rate * dt);
    } else {
      const float rate = emergency ? vHard : vSoft;
      _v += constrain(vT - _v, -rate * dt, rate * dt);
    }
    if (fabsf(_v) < 0.002f && vT == 0) _v = 0;

    // turn
    float wt = wT;
    if (_w != 0 && wt != 0 && (_w > 0) != (wt > 0)) wt = 0;   // other direction: through 0
    const float wr = (fabsf(wt) > fabsf(_w) ? wUp : (emergency ? 2.0f * wDown : wDown)) * dt;
    _w += constrain(wt - _w, -wr, wr);
    if (fabsf(_w) < 0.002f && wT == 0) _w = 0;

    l = constrain(_v - _w, -0.5f, 0.5f);
    r = constrain(_v + _w, -0.5f, 0.5f);
    if (fabsf(l) < 0.005f) l = 0;
    if (fabsf(r) < 0.005f) r = 0;
  }

  float v() const { return _v; }
  float w() const { return _w; }
  bool still() const { return _v == 0 && _w == 0; }

private:
  void _wheel(float& c, float t, float dt, bool emergency) {
    if (c != 0 && t != 0 && (c > 0) != (t > 0)) t = 0;           // other direction: through 0
    const float rate = (fabsf(t) > fabsf(c) ? wheelUp : (emergency ? 2.0f * wheelDown : wheelDown)) * dt;
    c += constrain(t - c, -rate, rate);
  }
  float _v = 0, _w = 0;
  uint32_t _ms = 0;
};
