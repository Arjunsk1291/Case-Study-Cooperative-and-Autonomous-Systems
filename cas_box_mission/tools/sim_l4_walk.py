#!/usr/bin/env python3
"""sim_l4_walk.py - leg-4 WALKWAY (phase 1, after the left wall ends) in closed loop: does the rover keep its distance from the benches, the hydrant and the
mud edge on its left? campus_10.3 (rover 7): the walkway heading was 11.7 deg too far LEFT (stale wall direction) and the left objects (0.5-1.3 m) were below the
1.0 m minimum of the edge fit -> it drove into the mud side.
World (walkway along +y, rover starts at y = 0 where the wall ended, x to the right): benches 1.6 m x 0.4 m at lateral distance Lb on the left every 3.6 m,
a hydrant 0.3 m at Lb, optional second row; true walkway direction = yaw 0. Rover 0.30 m/s, yaw follows the target (<= 3 deg/s), small gyro noise.
OLD = v10.3 as flashed on rover 7: heading = stale wall direction (bias), left edge fit 1.0-3.0 m (>= 5 points), keep clamp 1.3-2.4 m, no clearance.
NEW = heading = median wall direction (bias ~0.3 deg), edge fit 0.4-4.5 m (>= 4 points), keep clamp 1.3-4.5 m, hard clearance 0.9 m (steer right up to 8 deg).
Metrics at y = 20 m: lateral offset (left = negative), closest approach to any left object, 'mud' = more than 1.5 m left of the start track.
  python3 sim_l4_walk.py [--quick]"""
import sys, math, random; sys.path.insert(0, '.')
import fwmirror as F
from sim_open_sea import ray
KEEP_V12 = 2.1     # v12: taught left distance (c102_3); None = v11 (first distance seen)

def w180(a): return (a + 180) % 360 - 180
def box(x0, y0, x1, y1): return [((x0, y0), (x1, y0)), ((x1, y0), (x1, y1)), ((x1, y1), (x0, y1)), ((x0, y1), (x0, y0))]

def world(Lb, rng, hydrant=True, second=False):
    S = []
    for k, y in enumerate((3.0, 6.6, 10.2, 13.8, 17.4, 21.0, 24.6)):
        S += box(-Lb - 0.4, y, -Lb, y + 1.6)
    if hydrant: S += box(-Lb - 0.15, 8.4 + rng.uniform(-0.3, 0.3), -Lb + 0.15, 8.7 + rng.uniform(-0.3, 0.3))
    if second: S += box(-Lb - 2.2, 5.0, -Lb - 1.8, 22.0)                               # a second row (wall of the building) farther left
    S += [((3.5, -2.0), (3.5, 30.0))]                                                    # something right (railing / far edge), 3.5 m
    return S

def scan(x, y, th_cw, S, rng):
    """90 beams, bin i at (i*4+2) deg clockwise from the heading; th_cw = heading clockwise from +y (rad)"""
    L = []
    for i in range(90):
        a = th_cw + math.radians(i * 4 + 2); d = ray(x, y, math.pi / 2 - a, S, rmax=12.0)
        L.append(0 if d is None or rng.random() < 0.12 else max(35, int(round((d + rng.gauss(0, 0.02)) * 100))))
    return L

def run(Lb, bias, x0, new, seed, hydrant=True, second=False, half_deg=None, trace=None):
    """new: 0 = OLD, 1 = NEW (median heading + wider edge fit + clearance), 2 = NEW rule but the stale (biased) heading kept (tests the clearance alone)"""
    stale = new == 2; new = bool(new)
    rng = random.Random(seed); S = world(Lb, rng, hydrant, second)
    x, y = x0, 0.0; yaw = float(rng.gauss(0, 1.0))                 # yaw (deg, counter-clockwise +), th_cw = -yaw
    l4dir = bias if (not new or stale) else rng.gauss(0.0, 0.3)   # the walkway direction estimate (true = 0)
    tgt = l4dir; keep = None; minc = 9.0; t = 0.0
    cfg = dict(rmin=0.4, rmax=4.5, tol=0.12, minN=4, minLen=0.6, minPair=0.3, klo=1.3, khi=4.5) if new else dict(rmin=1.0, rmax=3.0, tol=0.10, minN=5, minLen=0.8, minPair=0.4, klo=1.3, khi=2.4)
    while y < 20.0 and t < 120:
        L = scan(x, y, math.radians(-yaw), S, rng)
        want = l4dir
        fit = F.row_fit(L, 270, 60 if new else 40, cfg['rmin'], cfg['rmax'], cfg['tol'], cfg['minN'], cfg['minLen'], cfg['minPair'])
        if fit and fit[1] < 0:
            d = w180(yaw - fit[0])
            if abs(w180(d - l4dir)) <= 12.0:
                if keep is None: keep = (KEEP_V12 if (new and KEEP_V12) else max(cfg['klo'], min(cfg['khi'], -fit[1])))
                l4dir = w180(l4dir + max(-0.5, min(0.5, w180(d - l4dir))))
                want = w180(l4dir + max(-6.0, min(6.0, 10.0 * (-fit[1] - keep))))
        if new:                                                       # hard clearance
            minL = 9.0
            for i in range(55, 81):
                cm = L[i]
                if not cm or cm < 35: continue
                a = math.radians(i * 4 + 2); sd = -cm / 100 * math.sin(a); f = cm / 100 * math.cos(a)
                if abs(f) <= 0.9 and sd >= 0.3: minL = min(minL, sd)
            if minL < 0.9: want = w180(want - max(0.0, min(8.0, 15.0 * (0.9 - minL))))
        want = yaw + max(-10.0, min(10.0, w180(want - yaw))); tgt = w180(tgt + max(-2.0, min(2.0, w180(want - tgt))))
        for _ in range(5):                                           # 5 x 50 ms
            e = w180(tgt - yaw); yaw += max(-0.15, min(0.15, 0.8 * e * 0.05)) + rng.gauss(0, 0.01)
            th = math.radians(-yaw); x += 0.30 * 0.05 * math.sin(th); y += 0.30 * 0.05 * math.cos(th)
        t += 0.25
        if trace is not None: trace.append((x, y))
        for (ax, ay), (bx, by) in S:                                  # closest approach to a LEFT object
            if ax > 0.5: continue
            dx, dy = bx - ax, by - ay; L2 = dx * dx + dy * dy; u = 0 if L2 == 0 else max(0, min(1, ((x - ax) * dx + (y - ay) * dy) / L2))
            minc = min(minc, math.hypot(x - ax - u * dx, y - ay - u * dy))
    return x, minc

def main():
    quick = '--quick' in sys.argv; n = 3 if quick else 6
    LBS = (0.8, 1.2, 1.8, 2.5, 3.4) if not quick else (1.2, 2.5)
    print('walkway, 20 m after the wall end. lateral offset at 20 m (left = negative), closest approach to a left object; "mud" = more than 1.5 m left')
    for bias in (12.0, 6.0, 0.0, -6.0, -12.0):
        for label, new in (('OLD v10.3', 0), ('NEW', 1), ('NEW, stale dir', 2)):
            R = []
            for Lb in LBS:
                for x0 in (-0.3, 0.0, 0.3):
                    for s in range(n): R.append(run(Lb, bias, x0, new, 100 + s, hydrant=(s % 2 == 0), second=(s % 3 == 0)))
            xs = sorted(r[0] for r in R); cs = sorted(r[1] for r in R); m = len(R)
            print(f"bias {bias:+5.1f} deg {label:14s}: runs {m:3d} | offset median {xs[m//2]:+.2f} m (worst left {xs[0]:+.2f}) | mud {sum(1 for r in R if r[0] < -1.5)} | closest median {cs[m//2]:.2f} m (min {cs[0]:.2f}) | closer than 0.5 m {sum(1 for r in R if r[1] < 0.5)}", flush=True)

if __name__ == '__main__': main()
