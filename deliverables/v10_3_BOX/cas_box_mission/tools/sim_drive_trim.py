#!/usr/bin/env python3
"""sim_drive_trim.py - a NEW rover (different gearbox / motors): does the heading hold + learned trim (DriveStraight.h) cope with its natural drift?
Python mirror of DriveStraight (same constants as the header). Yaw model from the kit-04 trim calibration: yaw rate (deg/s, + = left)
= d0 + 8.58 * (trim_total + c / v) * (v / 0.25), plus a slow random walk of the drift (+-0.3 deg/s: stones, battery) and gyro noise.
Reports per natural drift d0: max |heading error| after 10 s, steady error, learned trim, failsafe? (saturated 5 s / > 12 deg for 2 s)."""
import math, random, sys

class DS:
    def __init__(s, learn_max=0.20, tau=15.0, tau_fast=None, trim0=-0.046):
        s.kP, s.kD, s.cmax, s.cfrac, s.crate, s.dead = 0.045, 0.0015, 0.10, 0.40, 0.30, 0.5
        s.learn_max, s.tau, s.tau_fast, s.trim = learn_max, tau, tau_fast, trim0
        s.learned = 0.0; s.c = 0.0; s.rate = 0.0; s.prev = 0.0; s.target = 0.0; s.drive_s = 0.0; s.sat = 0.0; s.errT = 0.0; s.fail = ''
    def update(s, yaw, v, dt):
        d = yaw - s.prev; s.prev = yaw
        keep = 0.7 ** (dt / 0.02); s.rate = keep * s.rate + (1 - keep) * d / dt
        err = s.target - yaw; ae = abs(err)
        limit = min(s.cmax, s.cfrac * abs(v))
        want = max(-limit, min(limit, (s.kP * err if ae > s.dead else 0.0) - s.kD * s.rate))
        s.c += max(-s.crate * dt, min(s.crate * dt, want - s.c)); s.c = max(-limit, min(limit, s.c))
        if v > 0.1 and ae < 6.0:
            tau = s.tau if s.tau_fast is None else s.tau_fast + (s.tau - s.tau_fast) * min(1.0, s.drive_s / 40.0)
            move = (s.c / v) * dt / tau
            s.learned = max(-s.learn_max, min(s.learn_max, s.learned + move)); s.c -= move * v
        if v > 0.1: s.drive_s += dt
        if limit > 0 and abs(s.c) >= 0.98 * limit: s.sat += dt
        else: s.sat = 0
        s.errT = s.errT + dt if ae > 12 else 0
        if s.sat >= 5 and not s.fail: s.fail = 'saturated'
        if s.errT >= 2 and not s.fail: s.fail = 'off >12 deg'
        t = s.trim + s.learned
        return v * (1 - t) - s.c, v * (1 + t) + s.c

def run(d0, learn_max, tau, tau_fast, v=0.22, T=60, seed=1, trim0=-0.046):
    rng = random.Random(seed); ds = DS(learn_max, tau, tau_fast, trim0); yaw = 0.0; dt = 0.05; drift = d0; maxe = 0; errs = []
    for k in range(int(T / dt)):
        drift += rng.gauss(0, 0.02) * math.sqrt(dt) * 3
        l, r = ds.update(yaw + rng.gauss(0, 0.05), v, dt)
        v_eq = (r - l) / 2 / v      # effective trim incl. correction: r-l = 2 v (t + c/v)
        yaw += (drift + 8.58 * v_eq * (v / 0.25)) * dt
        e = abs(ds.target - yaw); maxe = max(maxe, e) if k * dt > 10 else maxe
        if k * dt > 20: errs.append(e)
    return maxe, sum(errs) / len(errs), ds.learned, ds.fail

if __name__ == '__main__':
    print('natural drift (deg/s, + left, equal wheels incl. the fixed trim -0.046) | v10.3 now (max 0.20, tau 15) vs new (max 0.35, tau 6 -> 15)')
    for d0 in (-4, -3, -2, -1, 0, 1, 2, 3, 4):
        a = [run(d0, 0.20, 15.0, None, seed=s) for s in range(5)]; b = [run(d0, 0.35, 15.0, 6.0, seed=s) for s in range(5)]
        f = lambda R: f"max err {max(x[0] for x in R):5.1f} deg | mean err 20-60 s {sum(x[1] for x in R)/5:4.2f} | learned {sum(x[2] for x in R)/5:+.2f} | fails {sum(1 for x in R if x[3])}/5"
        print(f"{d0:+d}: old {f(a)}   ||   new {f(b)}")
