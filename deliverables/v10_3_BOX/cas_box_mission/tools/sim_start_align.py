#!/usr/bin/env python3
"""sim_start_align.py - v10.3 START ALIGNMENT (position first, then the open sea) in closed loop.

Scene (sim frame of sim_open_sea.py: south = -90 deg, LEFT of the south-going rover = +x, strip at x = 0):
  the THD sign pillar stands 1.3 m to the RIGHT of the strip (x = -1.3), board behind it, one more pole farther right,
  and (option) a person next to it. The rover starts at x = +lateness (a late turn = left of the strip, the real logs:
  right object 2.0 / 2.9 m in campus_10 / c102_3, 0.9-1.6 m in every good run).
Controller = mirror of RouteMission::_alignStart (StartAlign below):
  1. stop, measure the nearest object on the right (x 0.3-4.5 m right, y -2.5..+1.0 m, >= 35 cm), median of 5 scans
  2. d in [0.85, 1.75] m -> NOTHING (good turn, the check layer only logs)
  3. else e = d - 1.3: pivot 30 deg (right if e > 0), roll min(|e| / sin 30, 1.6 m) at 0.18, pivot back, measure again;
     max 3 rounds / 60 s; not seen -> nothing
Then the unchanged open sea (sim_open_sea.run) starts from the resulting lateral offset and heading error.
  python3 sim_start_align.py            # table: baseline (v10.2) vs v10.3 alignment, many seeds and disturbances
"""
import sys, math, random; sys.path.insert(0, '.')
import sim_open_sea as S, fwmirror as F

TARGET, BAND_LO, BAND_HI, MAX_ROLL, MAX_ROUNDS = 1.3, 0.85, 1.75, 1.6, 3


def start_world(rng, person=False, clutter=True):
    segs = [((-25, 0), (25, 0))]                                   # B wall behind, the rover starts 1.5 m from it (y = -1.5)
    py = -1.5 + rng.uniform(-0.4, 0.4)                              # the pillar is abeam +-0.4 m after the turn
    segs += [((-1.3, py + .2), (-1.7, py + .2)), ((-1.7, py + .2), (-1.7, py - .2)), ((-1.7, py - .2), (-1.3, py - .2)), ((-1.3, py - .2), (-1.3, py + .2))]  # pillar
    segs += [((-1.5, py), (-2.6, py))]                              # board
    if clutter: segs += [((-6.0, -3.0), (-5.6, -3.0)), ((-5.6, -3.0), (-5.6, -3.4)), ((-6.0, -3.4), (-6.0, -3.0))]
    if person:
        qx, qy = rng.uniform(-2.8, -1.9), rng.uniform(-2.5, -0.5); segs += [((qx - .2, qy - .2), (qx + .2, qy - .2)), ((qx + .2, qy - .2), (qx + .2, qy + .2)), ((qx - .2, qy + .2), (qx + .2, qy + .2)), ((qx - .2, qy + .2), (qx - .2, qy - .2))]
    return segs


def right_pts(L):
    P = []
    for i in range(90):
        cm = L[i]
        if not cm or cm < 35: continue
        a = math.radians(i * 4 + 2); P.append((cm / 100 * math.sin(a), cm / 100 * math.cos(a)))
    return P


def window_pts(L, track=None):
    """points of one scan that may be the sign: first look 0.3-4.5 m right, abeam y -1.0..+1.0 m; following (track = where the
    pillar must be after a roll): within 0.6 m of it. At most 24 per scan (the box keeps the same)."""
    P = right_pts(L)
    if track is None: C = [(x, y) for x, y in P if 0.3 <= x <= 4.5 and -1.0 < y < 1.0]
    else: C = [(x, y) for x, y in P if math.hypot(x - track[0], y - track[1]) <= 0.6 and x > 0.3]
    return sorted(C)[:24]


def vote(scans, rad=0.15, need=4):
    """the rover STANDS STILL while it measures, so a real object is in the same place in every scan; snow flakes, ghosts and a person
    walking by are not. Nearest point that has a point within 0.15 m in >= 4 of the last 8 scans; its position = mean of those points."""
    best = None
    for sc in scans:
        for (x, y) in sc:
            sup = []
            for sc2 in scans:
                near = [(x2, y2) for (x2, y2) in sc2 if math.hypot(x2 - x, y2 - y) <= rad]
                if near: sup.append(min(near, key=lambda p: math.hypot(p[0] - x, p[1] - y)))
            if len(sup) >= need:
                m = (sum(p[0] for p in sup) / len(sup), sum(p[1] for p in sup) / len(sup))
                if best is None or m[0] < best[0] - 0.05: best = m
    return best


class StartAlign:
    def __init__(s): s.round = 0; s.t = 0.0; s.scans = []; s.track = None; s.last_e = 9.9

    def measure(s, L):
        s.scans = (s.scans + [window_pts(L, s.track)])[-8:]
        if len(s.scans) < 8: return None
        return vote(s.scans)

    def decide(s, d):
        """-> ('ok', 0) | ('move', e) ; e > 0 = move RIGHT (toward the pillar)"""
        if BAND_LO <= d <= BAND_HI: return 'ok', 0.0
        if s.round >= MAX_ROUNDS or s.t > 60: return 'giveup', 0.0
        return 'move', d - TARGET


def align(rng, x0, slip=0.0, person=False, dropout=0.12, pivot_sd=3.0):
    """returns (x after, heading error deg, seconds, rounds, moved)"""
    segs = start_world(rng, person); x, y, th = x0, -1.5, math.radians(-90); A = StartAlign(); t = 0.0; moved = False
    herr = 0.0
    for _ in range(40):
        # measure: 5 scans (2 s) standing still
        for _k in range(32):
            L = S.scan(x, y, th, segs, rng); t += 0.25
            L = [0 if (c and rng.random() < dropout - 0.12) else c for c in L] if dropout > 0.12 else L
            m = A.measure(L)
            if m is not None: break
        if m is None: return x, herr, t, A.round, moved
        A.t = t
        what, e = A.decide(m[0])
        if what == 'move' and A.round >= 1 and abs(e) > A.last_e + 0.15: what = 'giveup'     # the last move did not improve: stop
        if what != 'move': return x, herr, t, A.round, moved
        A.last_e = abs(e)
        moved = True; A.round += 1; A.scans = []
        a = math.radians(30) * (1 if e > 0 else -1)                        # right = th decreases
        roll = min(abs(e) / math.sin(math.radians(30)), MAX_ROLL)
        th1 = th - a + math.radians(rng.gauss(0, pivot_sd)); t += 3.0     # pivot (full power, ~3 s with the ramps)
        step = roll * (1 + rng.gauss(0, 0.05)); t += step / 0.18
        dx, dy = step * math.cos(th1), step * math.sin(th1)
        x += dx; y += dy
        # the pillar in the NEW rover frame (odometry: the rover moved right by -dx and forward by -dy, then turned back)
        A.track = (m[0] + dx, m[1] - (-dy))
        x += slip * step * rng.choice((-1, 1))                             # slip sideways
        err = rng.gauss(0, pivot_sd); th = math.radians(-90) + math.radians(err); herr = err; t += 3.0
    return x, herr, t, A.round, moved


def baseline_run(seed, x0, turn_err=None, **kw):
    return S.run(seed, S.F.OpenSea.V7, x0=x0, turn_err=4.0 if turn_err is None else turn_err, **kw)


def full(seed, lateness, mode, person=False, **kw):
    rng = random.Random(seed * 7 + 1)
    if mode == 'v10.2':   x, herr, t, rounds, moved = lateness, rng.gauss(4, 3), 0, 0, False
    else:                 x, herr, t, rounds, moved = align(rng, lateness, person=person, slip=kw.pop('align_slip', 0.0))
    a24, aend, mx = S.run(seed, S.F.OpenSea.V7, x0=x, turn_err=herr if mode != 'v10.2' else herr, **kw)
    return dict(x=x, moved=moved, rounds=rounds, t=t, a24=a24, aend=aend)


DIST = {'clean': dict(), 'sparse row': dict(missing=0.6, blind=0.5), 'sparse+slide 3%': dict(missing=0.6, blind=0.5, slide=0.03),
        'sparse+gyro -3/min': dict(missing=0.6, blind=0.5, bias_dpm=-3)}


def q(v, p): v = sorted(v); return v[min(len(v) - 1, int(p * len(v)))]


def main():
    quick = '--quick' in sys.argv
    n = 3 if quick else 6
    LAT = [-0.6, -0.3, 0.0, 0.2, 0.4, 0.7, 1.0, 1.4, 1.8] if not quick else [-0.3, 0.0, 0.7, 1.4]
    if '--dist' not in sys.argv: pass
    print('GOOD band = right object 0.85-1.75 m  (= lateness -0.45..+0.45 m from the pillar line 1.3 m)')
    print('\n== A. untouched check: starts inside the band must not move (people / clutter / dropouts on)')
    cnt = 0; tot = 0
    for lat in (-0.4, -0.2, 0.0, 0.2, 0.4):
        for sd in range(20):
            r = full(100 + sd, lat, 'v10.3', person=(sd % 4 == 0)); tot += 1; cnt += (not r['moved'])
    print(f'   starts inside the band: {tot}, left untouched: {cnt} ({100 * cnt / tot:.0f} %)')
    print('\n== B. late starts: lateral offset after the alignment and error at the end wall (median / 90 %)')
    only = sys.argv[sys.argv.index('--dist') + 1] if '--dist' in sys.argv else None
    for dn, kw in DIST.items():
        if only and dn != only: continue
        print(f'-- disturbance: {dn}')
        for lat in LAT:
            row = {}
            for mode in ('v10.2', 'v10.3'):
                R = [full(300 + k, lat, mode, person=(k % 3 == 0), **kw) for k in range(n)]
                row[mode] = R
            f = lambda R, key: (q([abs(r[key]) for r in R], .5), q([abs(r[key]) for r in R], .9))
            a, b = row['v10.2'], row['v10.3']
            print(f"   late {lat:+.1f} m | start off: v10.2 {f(a,'x')[0]:.2f}  v10.3 {f(b,'x')[0]:.2f} (90% {f(b,'x')[1]:.2f}) | end: v10.2 {f(a,'aend')[0]:.2f}/{f(a,'aend')[1]:.2f}  v10.3 {f(b,'aend')[0]:.2f}/{f(b,'aend')[1]:.2f} | rounds {sum(r['rounds'] for r in b) / len(b):.1f} time {sum(r['t'] for r in b) / len(b):.0f} s", flush=True)


def stress():
    """harsh conditions: snow-like noise in every scan (8 % random near returns, 25 % dropouts, 20 % ghosts), pivot error 6 deg,
    slip 8 %, a person beside the sign in half of the runs. Starts inside the band must stay untouched; late starts must
    never end WORSE than doing nothing."""
    S.ENV = dict(precip=0.08, drop=0.25, ghost=0.2)
    res = {'in-band untouched': [0, 0], 'late: start off after (<= 0.5 m)': [0, 0], 'late: worse than before': [0, 0]}
    wrong = []
    for lat in (-0.4, -0.2, 0.0, 0.2, 0.4):
        for sd in range(24):
            rng = random.Random(900 + sd); x, herr, t, rounds, moved = align(rng, lat, person=(sd % 2 == 0), slip=0.08, pivot_sd=6.0)
            res['in-band untouched'][1] += 1; res['in-band untouched'][0] += (not moved)
            if moved: wrong.append((lat, sd, round(x, 2)))
    for lat in (0.7, 1.0, 1.4, 1.8, 2.2, -0.7):
        for sd in range(24):
            rng = random.Random(1500 + sd); x, herr, t, rounds, moved = align(rng, lat, person=(sd % 2 == 0), slip=0.08, pivot_sd=6.0)
            res['late: start off after (<= 0.5 m)'][1] += 1; res['late: start off after (<= 0.5 m)'][0] += (abs(x) <= 0.5)
            res['late: worse than before'][1] += 1; res['late: worse than before'][0] += (abs(x) > abs(lat) + 0.1)
    S.ENV = {}
    for k, (a, b) in res.items(): print(f'   {k}: {a} of {b}')
    print('   moved although in band:', wrong[:8])


if __name__ == '__main__':
    if '--stress' in sys.argv: stress()
    else: main()
