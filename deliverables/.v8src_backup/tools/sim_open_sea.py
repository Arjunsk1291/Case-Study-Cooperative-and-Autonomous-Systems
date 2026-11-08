#!/usr/bin/env python3
"""sim_open_sea.py - closed-loop simulation of the open-sea crossing with the firmware's reference logic
(fwmirror.OpenSea), for comparing parameter sets (V6 = flashed, V7 = proposed) before flashing.

World (measured in campus_2..6 logs): B facade behind; LEFT landmark row (tree-grate edges + trunks + stakes +
posts) 6.65 m from the strip at 9.5-24 m; left building front at 3 m (28-37 m); end wall at 40 m; random people.
LiDAR: 90 bins of 4 deg, 2 cm noise, 12 % dropouts, the left side blinded on a share of scans (calibrated to the
real row lock rate ~50 %). Rover: heading loop like DriveStraight (max 3 deg/s rolling), gyro bias, optional
sideways slide (slanted cobbles) and row-direction wobble (what campus_6 showed: +-5 deg).

  python3 sim_open_sea.py            # prints the comparison table
"""
import math, random, sys, os
sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
import fwmirror as F
w180 = F.w180


def ray(px, py, ang, segs, rmax=12.0):
    dx, dy = math.cos(ang), math.sin(ang); best = None
    for (ax, ay), (bx, by) in segs:
        ex, ey = bx - ax, by - ay; den = dx * ey - dy * ex
        if abs(den) < 1e-9: continue
        t = ((ax - px) * ey - (ay - py) * ex) / den; u = ((ax - px) * dy - (ay - py) * dx) / den
        if t > 0.05 and 0 <= u <= 1 and t <= rmax and (best is None or t < best): best = t
    return best


def scan(x, y, th, segs, rng):
    L = []
    for i in range(90):
        a = math.radians(i * 4 + 2); d = ray(x, y, th - a, segs)
        L.append(0 if d is None or rng.random() < 0.12 else max(20, int(round((d + rng.gauss(0, 0.02)) * 100))))
    return L


def world(rng, rowoff=6.65, people=3, missing=0.0, wobble=0.0, rot=0.0):
    """rover crosses SOUTH from y=0 along x=0; LEFT of a south-going rover = +x. rot (deg, + = the strip turns to the
    rover's LEFT): the strip, the row, the building and the end wall are rotated against the B wall (campus_6: ~+3..+5)"""
    base = [((-25, 0), (-3, 0)), ((-1.5, 0), (25, 0))]
    for x in (-12, -7.5, 6, 10.5): base.append(((x - 0.25, -2.6), (x + 0.25, -2.6)))
    segs = []
    y = -9.5
    while y > -24:
        xo = rowoff + rng.uniform(-wobble, wobble)                       # irregular row (planters not in line)
        if rng.random() >= missing:
            segs.append(((xo, y + 0.6), (xo, y - 0.6)))
            for dx, dy in ((0.6, 0), (0.95, 0.2), (0.3, 0.25)):
                segs.append(((xo + dx - 0.08, y + dy), (xo + dx + 0.08, y + dy)))
        if rng.random() >= missing: segs.append(((xo - 0.05, y - 2.0), (xo + 0.05, y - 2.0)))
        y -= 4.0
    segs.append(((3.0, -28), (3.0, -37)))
    segs.append(((-8, -40), (8, -40)))
    for _ in range(people):
        px, py = rng.uniform(-4, 9), rng.uniform(-35, -5); segs.append(((px - 0.2, py), (px + 0.2, py)))
    c, sn = math.cos(math.radians(rot)), math.sin(math.radians(rot))
    R = lambda p: (p[0] * c - p[1] * sn, p[0] * sn + p[1] * c)        # rotate about the start (0, 0)
    return base + [(R(a), R(b)) for a, b in segs]


def run(seed, preset, bias_dpm=0.0, turn_err=4.0, x0=0.0, slide=0.0, blind=0.3, rowoff_true=6.65, rot=0.0, **wk):
    rng = random.Random(seed); segs = world(rng, rowoff=rowoff_true, rot=rot, **wk)
    ux, uy = math.sin(math.radians(rot)), -math.cos(math.radians(rot))   # strip direction (south rotated by rot)
    lat = lambda x, y: x * uy - y * ux                                     # signed distance from the strip
    th = math.radians(-90 + turn_err); x, y = x0, -1.0; drift = 0.0
    yaw = lambda: w180(math.degrees(th) + drift)
    C = F.OpenSea(-90.0, **dict(preset, leg2=38.0))
    v = 0.22; dt = 0.05; t = 0; last = -1; d2 = 0; tgt = -90.0; xs = {}; maxx = 0
    while t < 500:
        if t - last >= 0.25:
            last = t; L = scan(x, y, th, segs, rng)
            if blind and rng.random() < blind: L = [0 if 50 <= i <= 85 else c for i, c in enumerate(L)]
            tgt = C.step(L, yaw(), d2, 0.25)
        e = w180(tgt - yaw()); th += math.radians(max(-3, min(3, 0.8 * e)) * dt)
        drift += bias_dpm / 60 * dt
        x += v * math.cos(th) * dt + slide * v * math.cos(th - math.pi / 2) * dt
        y += v * math.sin(th) * dt + slide * v * math.sin(th - math.pi / 2) * dt
        d2 += v * dt; t += dt
        along = x * ux + y * uy
        if along > 12: maxx = max(maxx, abs(lat(x, y)))
        for mark in (24, 37):
            if mark not in xs and along >= mark: xs[mark] = lat(x, y)
        if along >= 38.5: break
    return abs(xs.get(24, 99)), abs(lat(x, y)), maxx


def batch(label, preset, n=12, **kw):
    r = [run(500 + k, preset, **kw) for k in range(n)]
    q = lambda v, p: sorted(v)[min(n - 1, int(p * n))]
    a24 = [a for a, b, c in r]; aE = [b for a, b, c in r]
    return f"{label:44s} 24 m: med {q(a24, .5):.2f} 90% {q(a24, .9):.2f} | end: med {q(aE, .5):.2f} 90% {q(aE, .9):.2f} m"


CASES = [("nominal", {}), ("gyro bias +6/min", dict(bias_dpm=6)), ("gyro bias -6/min", dict(bias_dpm=-6)),
         ("start 0.7 m right of strip", dict(x0=-0.7)), ("start 0.7 m left of strip", dict(x0=0.7)),
         ("3 % slide (cobble slant)", dict(slide=0.03)), ("5 % slide", dict(slide=0.05)),
         ("row wobble +-0.4 m (campus_6 dir noise)", dict(wobble=0.4)),
         ("worst mix", dict(bias_dpm=-6, turn_err=8, x0=-0.7, slide=0.03, missing=0.4))]

if __name__ == "__main__":
    for name, kw in CASES:
        for tag, P in (("V6", F.OpenSea.V6), ("V7", F.OpenSea.V7)):
            print(batch(f"{tag} {name}", P, **kw))
