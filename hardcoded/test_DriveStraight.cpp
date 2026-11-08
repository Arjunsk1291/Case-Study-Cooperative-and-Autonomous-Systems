// Laptop test of DriveStraight.h:  g++ -std=c++17 -O2 test_DriveStraight.cpp -o t && ./t
#include "DriveStraight.h"
#include <cstdio>
#include <random>
// plant: yaw rate = d0 + 34.4 * (r - l) / 2  (measured in log3: 8.6 deg/s per unit trim at v 0.25), 0.25 s lag
struct Res { int failMs; const char* why; float maxErr, rmsErr, learned; };
Res sim(float d0, float simS, float kickAtS, float kickDeg, float imuOffAtS, float imuOffS, unsigned seed=1) {
  DriveStraight ds; std::mt19937 rng(seed); std::normal_distribution<float> nz(0, 0.3f);
  float yaw = 0, rate = 0; ds.start(0, 0); Res R{-1, "", 0, 0, 0}; double se = 0; int n = 0; bool kicked = false;
  for (int ms = 10; ms <= simS * 1000; ms += 10) {
    float l, r; const bool imuOk = !(ms >= imuOffAtS * 1000 && ms < (imuOffAtS + imuOffS) * 1000);
    float meas = roundf((yaw) * 16) / 16;                      // BNO055: 1/16 deg steps
    auto st = ds.update(imuOk, meas, 0.25f, ms, l, r);
    if (st == DriveStraight::FAIL) { R.failMs = ms; R.why = ds.reason(); break; }
    const float want = d0 + 34.4f * (r - l) / 2 + nz(rng);
    rate += (want - rate) * 0.01f / 0.25f;
    yaw += rate * 0.01f;
    if (!kicked && kickAtS > 0 && ms >= kickAtS * 1000) { yaw += kickDeg; kicked = true; }
    if (ms > 3000) { R.maxErr = fmaxf(R.maxErr, fabsf(ds.errorDeg())); se += ds.errorDeg() * ds.errorDeg(); n++; }
  }
  R.rmsErr = n ? sqrt(se / n) : 0; R.learned = ds.learned(); return R;
}
void show(const char* name, Res r) {
  if (r.failMs >= 0) printf("%-62s -> FAILSAFE at %.1f s: %s\n", name, r.failMs / 1000.0, r.why);
  else printf("%-62s -> ok, heading error rms %.2f deg, max %.2f deg, learned trim %+.3f\n", name, r.rmsErr, r.maxErr, r.learned);
}
int main() {
  show("A indoor tiles (equal wheels +0.40 deg/s) + trim -0.046, 60 s", sim(0.40f, 60, 0, 0, -1, 0));
  show("B outdoor-like ground push -2.0 deg/s, 120 s", sim(-2.0f, 120, 0, 0, -1, 0));
  show("C strong ground push -3.0 deg/s, 120 s", sim(-3.0f, 120, 0, 0, -1, 0));
  show("D impossible push -8.0 deg/s (beyond the wheels' authority)", sim(-8.0f, 120, 0, 0, -1, 0));
  show("E 8 deg knock at 30 s (bump / kerb)", sim(0.0f, 60, 30, 8, -1, 0));
  show("F 25 deg knock at 30 s (hit something)", sim(0.0f, 60, 30, 25, -1, 0));
  show("G IMU gives no data for 0.3 s at 20 s", sim(0.0f, 60, 0, 0, 20, 0.3f));
  show("H IMU gives no data for 1 s at 20 s", sim(0.0f, 60, 0, 0, 20, 1.0f));
}
