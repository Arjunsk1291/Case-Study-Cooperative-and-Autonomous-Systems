#!/usr/bin/env python3
"""sim_people.py - PEOPLE near the turn pillar: injects simulated people into the REAL leg-1 LiDAR scans of the
recorded runs and replays the v9 turn decision (fwmirror.TurnV9) and, for comparison, the v8 rule (old pillar
landmark with bins to 162 deg from turnAt - 5 m).

People = circles (radius 0.15-0.30 m = slim person .. person with bag), ray-cast into the 90 LiDAR bins (they also
hide what is behind them). Leg-1 frame: along = odometry metres, lateral = metres to the RIGHT of the rover track.

Reference = the turn v9 makes on the clean scans of the same run. PASS = turn within +-0.5 m of it.
A turn > 2 m early = AT THE WRONG PILLAR (pillars are ~4.5-5 m apart) - must never happen.

  python3 sim_people.py            # all scenarios, summary table
  python3 sim_people.py --worst    # also list the worst cases
"""
import math, random, sys, os
sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
import fwmirror as F
import replay_turn as RT

RUNS = ['campus2', 'campus3', 'campus5', 'campus6', 'test1', 'test2', 'log8']


class Person:
    def __init__(s, a, lat, r=0.22, va=0.0, vl=0.0, follow=None):
        """a/lat: start position (leg-1 frame), va/vl: m/s; follow = (df, ds): stays at that offset from the rover"""
        s.a0, s.l0, s.r, s.va, s.vl, s.follow = a, lat, r, va, vl, follow

    def pos(s, t, dist):
        if s.follow: return dist + s.follow[0], s.follow[1]
        return s.a0 + s.va * t, s.l0 + s.vl * t


def inject(L, dist, rel, people, t):
    """people into one scan (rover at 'dist', heading rel deg to the leg-1 direction, + = left)"""
    out = list(L)
    for p in people:
        pa, pl = p.pos(t, dist)
        f, sd = pa - dist, pl
        rr = math.radians(rel)                                       # leg frame -> rover frame (rover rel deg LEFT)
        cf, cs = f * math.cos(rr) - sd * math.sin(rr), f * math.sin(rr) + sd * math.cos(rr)
        for i in range(90):
            a = math.radians(i * 4 + 2); ux, uy = math.cos(a), math.sin(a)
            tc = cf * ux + cs * uy
            if tc <= 0: continue
            d2 = cf * cf + cs * cs - tc * tc
            if d2 > p.r * p.r: continue
            hit = tc - math.sqrt(p.r * p.r - d2)
            if hit < 0.35: continue                                    # LidarScan minMm 350
            cm = int(hit * 100)
            if out[i] == 0 or cm < out[i]: out[i] = cm
    return out


def replay(scans, people, turn_at=20.0, old=False):
    tv = F.TurnV9(turn_at=turn_at)
    edges = []
    d0 = scans[0][1]
    for k, (L, d, rel, anc) in enumerate(scans):
        t = k * 0.25
        if anc is not None and tv.anchor is None: tv.set_anchor(anc)
        Lp = inject(L, d, rel, people, t) if people else L
        if old:                                                        # v8: old landmark from turnAt - 5 m, bins to 162
            if d >= turn_at - 5:
                lm = F.landmark(Lp, max_deg=162)
                if lm: edges.append(d - lm[0]); edges = edges[-7:]
            if edges and d - sorted(edges)[len(edges) // 2] >= 0.40: return d
            if d >= turn_at + 2.5: return d
            continue
        res = tv.step(Lp, d, rel)
        if res: return res[1]
    # log ended before the turn: the rover WILL turn at the structure it has chosen (lock + structPast)
    if tv.lock is not None: return tv.lock + tv.sp
    return None


def scenarios(T, rng):
    """T = reference turn position of the run. Yields (group, people)"""
    S = T - 0.60                                                       # ~ the structure / turn pillar
    P2 = S - 4.8                                                       # ~ the pillar before
    for ref, nm in ((S, 'turn pillar'), (P2, 'pillar before')):
        for da in [x * 0.5 for x in range(-6, 5)]:
            for lat in (0.6, 1.0, 1.5, 2.0, 2.5, 3.0, 3.5, 4.0):
                for r in (0.15, 0.3):
                    yield f"1 standing person near the {nm}", [Person(ref + da, lat, r)]
        for da in (-2, -1, -0.5, 0, 0.5, 1, 2):                         # groups side by side (a row across)
            for lat in (0.7, 1.5, 2.2):
                for n in (2, 3, 5):
                    yield f"group of {n} across near the {nm}", [Person(ref + da, lat + 0.5 * k, 0.22) for k in range(n)]
                    yield f"group of {n} along near the {nm}", [Person(ref + da + 0.5 * k, lat, 0.22) for k in range(n)]
    for da in (1.0, 2.0, 3.0, 4.0, 5.0):                               # people BEYOND the turn pillar (open area)
        for lat in (0.8, 1.5, 2.5):
            for n in (1, 3, 5):
                yield "group BEYOND the turn pillar (open area)", [Person(S + da, lat + 0.5 * k, 0.22) for k in range(n)]
    for lat in (0.6, 0.9, 1.5, 2.0, 2.5, 3.0):
        for df in (-2.0, -1.0, 0.0, 1.0, 2.0):
            yield "person walking along WITH the rover", [Person(0, 0, 0.22, follow=(df, lat))]
        yield "2 people walking along with the rover", [Person(0, 0, 0.22, follow=(0.5, lat)), Person(0, 0, 0.22, follow=(0.5, lat + 0.6))]
        for a0 in (T - 2, T + 2, T + 6):
            yield "person walking towards the rover", [Person(a0, lat, 0.22, va=-1.2)]
        yield "person walking slowly ahead of the rover", [Person(T - 8, lat, 0.22, va=0.15)]
    for df, ds in ((-1.5, 0.8), (-1.2, 0.6), (-2.0, 1.0), (-1.0, 1.2), (-0.5, 0.9)):
        yield "FOLLOWER behind-right (log8 case)", [Person(0, 0, 0.25, follow=(df, ds))]
    for a0 in [T - 6 + 0.5 * k for k in range(18)]:
        for v in (1.0, -1.0):
            yield "person crossing (walks across)", [Person(a0, 0.5 if v > 0 else 5.0, 0.22, vl=v)]
    for k in range(60):
        ppl = []
        for _ in range(8):
            mv = rng.random() < 0.5
            ppl.append(Person(rng.uniform(T - 9, T + 3), rng.uniform(0.6, 5.0), rng.uniform(0.15, 0.3),
                              va=rng.uniform(-1.3, 1.3) if mv else 0, vl=rng.uniform(-0.6, 0.6) if mv else 0))
        yield "crowd of 8 (half walking)", ppl


def main():
    worst = '--worst' in sys.argv
    rng = random.Random(7)
    stats = {}; bad = []
    for run in RUNS:
        scans, Treal = RT.leg1(run)
        _, res0, Tref = RT.run(scans)
        if Tref is None: print(f"({run}: no structure in the clean log - skipped)"); continue
        sc = [x for x in scans if x[1] >= 8.0]                         # the grid only needs the last ~12 m
        for grp, ppl in scenarios(Tref, rng):
            for ver in ('v9', 'v8'):
                d = replay(sc, ppl, old=(ver == 'v8'))
                err = (d - Tref) if d is not None else None     # None: not turned before the log ends = LATE
                st = stats.setdefault((grp, ver), [0, 0, 0, 0])
                st[0] += 1
                if err is not None and abs(err) <= 0.5: st[1] += 1
                if err is not None and err < -2.0: st[2] += 1
                if err is None: st[3] += 1
                if ver == 'v9' and (err is None or abs(err) > 0.5):
                    bad.append((99 if err is None else abs(err), run, grp, None if err is None else round(err, 2),
                                [(round(p.a0 - Tref, 1), round(p.l0, 1), p.follow) for p in ppl][:3]))
    groups = []
    for g, v in stats:
        if g not in groups: groups.append(g)
    print("ok = turn within +-0.5 m of the clean turn; early = > 2 m early (WRONG pillar); late = no structure chosen when the log ends")
    print(f"{'scenario':44s} {'cases':>6s} | {'v9 ok':>7s} {'early':>6s} {'late':>5s} | {'v8 ok':>7s} {'early':>6s} {'late':>5s}")
    tot = {'v9': [0, 0, 0, 0], 'v8': [0, 0, 0, 0]}
    for g in groups:
        a, b = stats[(g, 'v9')], stats[(g, 'v8')]
        for ver, s in (('v9', a), ('v8', b)):
            for q in range(4): tot[ver][q] += s[q]
        print(f"{g:44s} {a[0]:6d} | {100 * a[1] / a[0]:6.1f}% {a[2]:6d} {a[3]:5d} | {100 * b[1] / b[0]:6.1f}% {b[2]:6d} {b[3]:5d}")
    for ver in ('v9', 'v8'):
        n, ok, wp, late = tot[ver]
        print(f"TOTAL {ver}: {n} cases, turn within +-0.5 m: {100 * ok / n:.1f} %, WRONG (earlier) pillar: {wp}, late/none: {late}")
    if worst:
        for b in sorted(bad, reverse=True)[:25]: print('  ', b)


if __name__ == '__main__':
    main()
