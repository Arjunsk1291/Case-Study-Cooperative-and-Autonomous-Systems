#!/usr/bin/env python3
"""fwmirror.py - Python mirror of the box firmware's LiDAR geometry and open-sea reference logic.

Used by the replay / benchmark / simulation tools so that what is tested on the laptop is exactly what the ESP32
does. Keep it in step with WallFit.h, LineRef.h and RouteMission.cpp (case L2_GO).

Scan format: list of 90 ranges in cm, index i = bearing (i*4+2) deg clockwise from the front (0 = no return).
Rover frame: x forward, y to the RIGHT. Yaw frame: + = turning left (counter-clockwise), like the BNO055 output.
"""
import math


def w180(a):
    return (a + 180.0) % 360.0 - 180.0


def clamp(x, a, b):
    return max(a, min(b, x))


def fitline(P):
    """least-squares line through points: centre (mx, my), direction (ux, uy) with ux >= 0 (WallFit::fitLine)"""
    n = len(P)
    mx = sum(p[0] for p in P) / n; my = sum(p[1] for p in P) / n
    sxx = sum((p[0] - mx) ** 2 for p in P); syy = sum((p[1] - my) ** 2 for p in P)
    sxy = sum((p[0] - mx) * (p[1] - my) for p in P)
    phi = 0.5 * math.atan2(2 * sxy, sxx - syy)
    ux, uy = math.cos(phi), math.sin(phi)
    if ux < 0: ux, uy = -ux, -uy
    return mx, my, ux, uy


# ---------------------------------------------------------------------------------------- WallFit.h
def wallfit(L, left, prior=None):
    """WallFit::measure. Returns (dist, angleDeg, rms, n, length) or None. Side frame: y towards the wall,
    angle < 0 = nose turned towards the wall. prior = (dist, angleDeg) of the last good fit."""
    pts = []
    center = 270 if left else 90
    for i in range(90):
        cm = L[i]
        if cm < 20 or cm > 300: continue
        a = i * 4 + 2; o = abs((a - center + 540) % 360 - 180)
        if o > 60: continue
        r = cm / 100; ar = math.radians(a)
        pts.append((r * math.cos(ar), r * math.sin(ar) * (-1 if left else 1), o))
    if prior:
        th = math.radians(prior[1]); nx, ny = -math.sin(th), math.cos(th)
        q = [(x, y) for x, y, o in pts if abs(nx * x + ny * y - prior[0]) < 0.15]
        if len(q) < 5: return None
    else:
        q = [(x, y) for x, y, o in pts if o <= 20]
        if len(q) < 4: return None
    mx, my, ux, uy = fitline(q)
    for _ in range(2):
        q = [(x, y) for x, y, o in pts if abs(-uy * (x - mx) + ux * (y - my)) < 0.10]
        if len(q) < 5: return None
        mx, my, ux, uy = fitline(q)
    al = [x * ux + y * uy for x, y in q]
    if max(al) - min(al) < 0.6: return None
    if max(x for x, y in q) < -0.1: return None
    rms = math.sqrt(sum((-uy * (x - mx) + ux * (y - my)) ** 2 for x, y in q) / len(q))
    return abs(-uy * mx + ux * my), math.degrees(math.atan2(uy, ux)), rms, len(q), max(al) - min(al)


# ---------------------------------------------------------------------------------------- LineRef.h
def _pts(L, center, half, rmin, rmax):
    P = []
    for i in range(90):
        cm = L[i]
        if not cm or cm < rmin * 100 or cm > rmax * 100: continue
        a = i * 4 + 2; o = abs((a - center + 540) % 360 - 180)
        if o > half: continue
        d = cm / 100; P.append((d * math.cos(math.radians(a)), d * math.sin(math.radians(a))))
    return P


def _ransac(P, tol, minpair):
    best, bl = 0, None
    for i in range(len(P)):
        for j in range(i + 1, len(P)):
            dx = P[j][0] - P[i][0]; dy = P[j][1] - P[i][1]; n = math.hypot(dx, dy)
            if n < minpair: continue
            nx, ny = -dy / n, dx / n; c = nx * P[i][0] + ny * P[i][1]
            k = sum(1 for p in P if abs(nx * p[0] + ny * p[1] - c) < tol)
            if k > best: best, bl = k, (nx, ny, c)
    return best, bl


def row_fit(L, center, half, rmin, rmax, tol, min_n, min_len, min_pair):
    """LineRef::rowFit. Returns (angleDeg rover frame, + = turns right going forward; signed offset, + = right;
    n; length) or None."""
    P = _pts(L, center, half, rmin, rmax)
    if len(P) < min_n: return None
    best, bl = _ransac(P, tol, min_pair)
    if best < min_n: return None
    nx, ny, c = bl; Q = [p for p in P if abs(nx * p[0] + ny * p[1] - c) < tol]
    mx, my, ux, uy = fitline(Q); al = [x * ux + y * uy for x, y in Q]
    rms = math.sqrt(sum((-uy * (x - mx) + ux * (y - my)) ** 2 for x, y in Q) / len(Q))
    if max(al) - min(al) < min_len or rms > 0.5 * tol: return None
    return math.degrees(math.atan2(uy, ux)), -uy * mx + ux * my, len(Q), max(al) - min(al)


def sector_fit(L, center, half, rmin, rmax, min_len):
    """LineRef::sectorFit. Returns (angleDeg, |distance|) or None."""
    P = _pts(L, center, half, rmin, rmax)
    if len(P) < 6: return None
    best, bl = _ransac(P, 0.12, 0.5)
    if best < 6: return None
    nx, ny, c = bl; Q = [p for p in P if abs(nx * p[0] + ny * p[1] - c) < 0.12]
    mx, my, ux, uy = fitline(Q); al = [x * ux + y * uy for x, y in Q]
    rms = math.sqrt(sum((-uy * (x - mx) + ux * (y - my)) ** 2 for x, y in Q) / len(Q))
    if max(al) - min(al) < min_len or rms > 0.07: return None
    return math.degrees(math.atan2(uy, ux)), abs(-uy * mx + ux * my)


def perp(yaw, ang, expect):
    """LineRef::perpHeading"""
    ld = w180(yaw - ang); a = w180(ld + 90); b = w180(ld - 90)
    return a if abs(w180(a - expect)) <= abs(w180(b - expect)) else b


# ---------------------------------------------------------------------------------------- RouteMission L2_GO
class OpenSea:
    """Mirror of the open-sea reference logic. P = parameters (defaults = current firmware). step() every 0.25 s:
    scan, yaw (deg), d2 (m driven in the open sea), dt (s) -> target heading (yaw frame)."""
    # V6 = firmware v6 exactly as flashed for campus_6; V7 = proposed (each change justified in INSTRUCTIONS v7)
    V6 = dict(row_moves_base=True, keep_i=False, row_cap=180.0, row_rmax=9.0, bld_max=4.5, leg2=36.0, rowoff=6.5, bldoff=3.0, row_from=10.0, row_to=26.0, row_min=4.5, row_max=8.5,
              rear_from=0.0, rear_min_dist=0.0, kp=10.0, ki=0.0, kd=0.0, lat_max=8.0, i_max=4.0, dir_med=5, dir_step=0.5)
    V7 = dict(V6, rowoff=6.65, rear_from=2.0, rear_min_dist=2.5, ki=1.5, kd=20.0, dir_med=9, dir_step=0.25, row_cap=5.0,
              row_moves_base=False, keep_i=True, row_to=24.5, row_min=4.0, row_max=9.5, row_rmax=11.0, bld_max=6.0)
    DEFAULT = V6
    NAMES = {0: "gyro", 1: "rear_wall", 2: "front_wall", 3: "board", 4: "left_row", 5: "left_building"}

    def __init__(s, base, **kw):
        s.P = dict(OpenSea.DEFAULT); s.P.update(kw)
        s.base = base; s.kind = 0; s.buf = []; s.lat = 0.0; s.type = 0; s.front0 = None
        s.off = 0.0; s.ei = 0.0; s.last_e = None; s.row0 = None

    def step(s, L, yaw, d2, dt=0.25):
        P = s.P; h = None; typ = 0; lat_want = 0.0
        row_dir = lambda a: (lambda d: w180(d + 180) if abs(w180(d - s.base)) > 90 else d)(w180(yaw - a))
        remain = P['leg2'] - d2
        if P['row_from'] <= d2 <= P['row_to']:
            r = row_fit(L, 270, 70, 1.5, P['row_rmax'], 0.25, 5, 4.0, 1.0)
            if r and r[1] < 0:
                d = row_dir(r[0]); off = -r[1]
                if abs(w180(d - s.base)) <= 8 and P['row_min'] <= off <= P['row_max']:
                    h = d if P['row_moves_base'] else None; typ = 4; s.off = off
                    e = off - P['rowoff']                                   # + = too far from the row (right of path)
                    s.ei = clamp(s.ei + e * dt, -P['i_max'] / max(P['ki'], 1e-6), P['i_max'] / max(P['ki'], 1e-6))
                    # D: rate of the side error from the heading relative to the row: pointing left of the row by
                    # rel deg -> the rover closes in on the (left) row at v*sin(rel) -> e falls. v = 0.22 m/s.
                    rel = w180(yaw - d)                                     # + = pointing left of the row direction
                    e_dot = -0.22 * math.sin(math.radians(rel))
                    lat_want = clamp(P['kp'] * e + P['ki'] * s.ei + P['kd'] * e_dot, -P['lat_max'], P['lat_max'])
        if not typ and 26 <= d2 <= 40:
            r = row_fit(L, 270, 70, 1.5, 7.0, 0.25, 5, 4.0, 1.0)
            if r and r[1] < 0:
                d = row_dir(r[0]); off = -r[1]
                if abs(w180(d - s.base)) <= 8 and 2.0 <= off <= P['bld_max']:
                    h = d if P['row_moves_base'] else None; typ = 5; s.off = off; lat_want = clamp(10 * (off - P['bldoff']), -6, 6)
        if not typ and P['rear_from'] <= d2 < 14:
            r = sector_fit(L, 180, 50, 1.0, 12.0, 1.5)
            if r and P['rear_min_dist'] <= r[1] <= d2 + 6:
                hh = perp(yaw, r[0], s.base)
                if abs(w180(hh - s.base)) <= 12: h = hh; typ = 1
        if not typ and d2 > 0.5 * P['leg2']:
            r = sector_fit(L, 0, 40, 2.5, 12.0, 1.5)
            if r and remain - 6 <= r[1] <= remain + 8:
                hh = perp(yaw, r[0], s.base)
                if abs(w180(hh - s.base)) <= 10: h = hh; typ = 2
        if typ == 2 and P['keep_i']: s.ei = 0.0                       # v7: the end wall fixes the direction ->
        if typ and h is not None:                                       # the kept integral would correct it twice
            if typ != s.kind:
                s.buf = []; s.kind = typ
                if typ == 2 and s.front0 is None: s.front0 = s.base
            if typ in (4, 5): s.front0 = None
            s.buf.append(w180(h - s.base))
            nmed = P['dir_med'] if typ in (4, 5) else 5
            s.buf = s.buf[-nmed:]
            lim = P['dir_step'] if typ in (4, 5) else 0.5
            st = clamp(sorted(s.buf)[len(s.buf) // 2], -lim, lim); s.base = w180(s.base + st)
            if typ == 2: s.base = w180(s.front0 + clamp(w180(s.base - s.front0), -8, 8))
            if typ in (4, 5):                                           # v7: the row may turn the path direction
                if s.row0 is None: s.row0 = s.base                       # max row_cap deg from where it was first seen
                s.base = w180(s.row0 + clamp(w180(s.base - s.row0), -P['row_cap'], P['row_cap']))
            s.buf = [b - st for b in s.buf]
        s.type = typ
        # side term: P+D only while a row is seen; the I part (a constant sideways pull) is kept after it (v7)
        hold = P['ki'] * s.ei if P['keep_i'] else 0.0
        s.lat += clamp((lat_want if typ in (4, 5) else hold) - s.lat, -0.5, 0.5)
        return w180(s.base + s.lat)


class Leg4:
    """Mirror of RouteMission::_leg4Wall (v7): forward-left line, +-15 deg of the expected direction, close in to
    dset (20 deg/m, max 15 deg), target max 10 deg ahead of the heading and max 2 deg per scan."""
    def __init__(s, exp_dir, dset=1.0):
        s.exp = exp_dir; s.wdir = exp_dir; s.ok = False; s.good_t = 0.0; s.tgt = exp_dir; s.dset = dset; s.d = None

    def step(s, L, yaw, t):
        want = s.tgt; good = False
        r = row_fit(L, 300, 45, 0.5, 5.0, 0.12, 5, 1.0, 0.5)
        if r and r[1] < 0:
            d = w180(yaw - r[0])
            if abs(w180(d - s.exp)) > 90: d = w180(d + 180)
            if abs(w180(d - s.exp)) <= 15:
                good = True; s.ok = True; s.good_t = t
                s.wdir = w180(s.wdir + clamp(w180(d - s.wdir), -1, 1)); s.d = -r[1]
                want = w180(s.wdir + clamp(20 * (-r[1] - s.dset), -15, 15))
        if not good and s.ok and t - s.good_t > 1.0: s.ok = False
        if not good and not s.ok: want = s.exp
        want = w180(yaw + clamp(w180(want - yaw), -10, 10))
        s.tgt = w180(s.tgt + clamp(w180(want - s.tgt), -2, 2))
        return s.tgt


def front_v8(L, us_hist, min_cm=35):
    """Mirror of RouteMission::_front (v8.1, without the slope ground filter): LiDAR +-16 deg, points closer than
    min_cm dropped (LidarScan minMm 350); a near (< 0.6 m) obstacle counts only if the FRONT ultrasonic agrees
    (median of the last 3 readings < 50 cm and <= LiDAR + 15 cm) or it is wide (>= 4 neighbouring beams within
    15 cm). us_hist: list of front ultrasonic readings (cm), newest last. Returns (front distance m, ghost_ignored)."""
    import statistics
    b = lambda k: (lambda c: c if c >= min_cm else 0)(L[k % 90])
    best, kb = 9.9, 0
    for k in range(-4, 4):
        c = b(k)
        if c and c / 100 < best: best, kb = c / 100, k
    ghost = False
    if best < 0.6:
        c0 = int(best * 100); run = 1
        for d in (-1, 1):
            k = kb + d
            while -7 <= k <= 6 and b(k) and b(k) <= c0 + 15: run += 1; k += d
        usm = statistics.median(us_hist[-3:]) if len(us_hist) >= 3 else 300
        if not ((2 < usm < 50 and usm <= c0 + 15) or run >= 4):
            ghost = True
            best = min([b(k) / 100 for k in range(-4, 4) if b(k) and b(k) / 100 >= 0.6] or [9.9])
    return best, ghost
