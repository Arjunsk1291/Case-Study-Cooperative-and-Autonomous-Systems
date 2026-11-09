#!/usr/bin/env python3
"""sim_open_sea.py - closed-loop simulation of the open-sea crossing with the firmware's reference logic
(fwmirror.OpenSea), for comparing parameter sets (V6 = flashed, V7 = proposed) before flashing.

World (measured in campus_2..6 logs): B facade behind; LEFT landmark row (tree-grate edges + trunks + stakes +
posts) 6.65 m from the strip at 9.5-24 m; left building front at 3 m (28-37 m); end wall at 40 m; random people.
LiDAR: 90 bins of 4 deg, 2 cm noise, 12 % dropouts, the left side blinded on a share of scans (calibrated to the
real row lock rate ~50 %). Rover: heading loop like DriveStraight (max 3 deg/s rolling), gyro bias, optional
sideways slide (slanted cobbles) and row-direction wobble (what campus_6 showed: +-5 deg).

  python3 sim_open_sea.py            # prints the comparison table + the crowd table
  python3 sim_open_sea.py --crowd    # only people / crowds (v9)
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


END_INC, END_DEC, END_TRIG = 2, 1, 6      # v10 leaky end-wall counter (firmware: _endCnt)
ENV = {}          # v10: environment noise applied to every simulated scan (see ENVS below)


def env_noise(L, rng):
    """daylight ghosts (campus_6: 15-21 % of scans, 1-2 beams at 20-34 cm, mostly right/front), low-sun dropouts,
    rain / snow (random near returns 0.35-3 m, extra dropouts)"""
    L = list(L)
    if ENV.get('ghost') and rng.random() < ENV['ghost']:
        for _ in range(rng.choice((1, 1, 2))):
            i = rng.choice([rng.randrange(0, 20), rng.randrange(60, 90), rng.randrange(20, 40)]); L[i] = rng.randint(20, 34)
    if ENV.get('sun') and rng.random() < ENV['sun']:                 # low sun: a 40 deg wedge goes blind
        c = rng.randrange(90)
        for k in range(-5, 6): L[(c + k) % 90] = 0
    if ENV.get('precip'):                                            # per beam: random near return / dropout
        for i in range(90):
            q = rng.random()
            if q < ENV['precip']: L[i] = rng.randint(35, 300)
            elif q < ENV['precip'] + ENV.get('drop', 0): L[i] = 0
    return L


def scan(x, y, th, segs, rng):
    L = []
    for i in range(90):
        a = math.radians(i * 4 + 2); d = ray(x, y, th - a, segs)
        L.append(0 if d is None or rng.random() < 0.12 else max(20, int(round((d + rng.gauss(0, 0.02)) * 100))))
    L = [0 if 0 < c < 35 else c for c in L]                          # LidarScan minMm 350 (real point filter)
    return env_noise(L, rng) if ENV else L


def world(rng, rowoff=6.65, people=3, missing=0.0, wobble=0.0, rot=0.0, fake=None, nobwall=False, rowgap=None):
    """rover crosses SOUTH from y=0 along x=0; LEFT of a south-going rover = +x. rot (deg, + = the strip turns to the
    rover's LEFT): the strip, the row, the building and the end wall are rotated against the B wall (campus_6: ~+3..+5)"""
    base = [] if nobwall else [((-25, 0), (-3, 0)), ((-1.5, 0), (25, 0))]
    for x in (-12, -7.5, 6, 10.5): base.append(((x - 0.25, -2.6), (x + 0.25, -2.6)))
    segs = []
    y = -9.5
    while y > -24:
        xo = rowoff + rng.uniform(-wobble, wobble)                       # irregular row (planters not in line)
        if rng.random() >= missing and not (rowgap and rowgap[0] <= -y <= rowgap[1]):
            segs.append(((xo, y + 0.6), (xo, y - 0.6)))
            for dx, dy in ((0.6, 0), (0.95, 0.2), (0.3, 0.25)):
                segs.append(((xo + dx - 0.08, y + dy), (xo + dx + 0.08, y + dy)))
        if rng.random() >= missing: segs.append(((xo - 0.05, y - 2.0), (xo + 0.05, y - 2.0)))
        y -= 4.0
    if fake: segs.append(((fake[0], fake[1]), (fake[0], fake[2])))      # a false straight line (bike rack, parked cars, hoarding)
    segs.append(((3.0, -28), (3.0, -37)))
    segs.append(((-8, -40), (8, -40)))
    for _ in range(people):
        px, py = rng.uniform(-4, 9), rng.uniform(-35, -5); segs.append(((px - 0.2, py), (px + 0.2, py)))
    c, sn = math.cos(math.radians(rot)), math.sin(math.radians(rot))
    R = lambda p: (p[0] * c - p[1] * sn, p[0] * sn + p[1] * c)        # rotate about the start (0, 0)
    return base + [(R(a), R(b)) for a, b in segs]


def run(seed, preset, bias_dpm=0.0, turn_err=4.0, x0=0.0, slide=0.0, blind=0.3, rowoff_true=6.65, rot=0.0, jump=None, **wk):
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
        if jump and abs(t - jump[0]) < dt / 2: drift += jump[1]            # IMU glitch: sudden heading jump (magnetic spot / reset)
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



# ------------------------------------------------------------------------------------------------ v9: PEOPLE / CROWDS
# People as 0.4 m squares (4 segments), standing or walking (straight, bouncing inside the area). The rover follows
# the firmware's people rule: something within holdM (0.6 m, +-16 deg) for 0.5 s -> STOP and WAIT (never swerves),
# go on after 2 s clear (> 1.0 m). End wall = firmware rule (after 70 %, front <= 1.5 m, flat-wall check, level).
def person_segs(x, y, r=0.2):
    return [((x - r, y - r), (x + r, y - r)), ((x + r, y - r), (x + r, y + r)), ((x + r, y + r), (x - r, y + r)), ((x - r, y + r), (x - r, y - r))]


def crowd(rng, kind):
    """people as [x, y, vx, vy, r] in the world of world(): rover crosses SOUTH (y 0 -> -40) along x = 0"""
    P = []
    if kind == 'nobody':
        pass
    elif kind == 'standing 12':
        for _ in range(12): P.append([rng.uniform(-3, 8), rng.uniform(-36, -3), 0, 0, rng.uniform(0.15, 0.3)])
    elif kind == '1 person standing in the path (last 30 %)':
        P.append([rng.uniform(-0.3, 0.3), rng.uniform(-36, -28), 0, 0, rng.uniform(0.18, 0.28)])
    elif kind == 'group of 3 standing in the path (last 30 %)':
        y = rng.uniform(-36, -28); x = rng.uniform(-0.4, 0.4)
        for k in range(3): P.append([x - 0.45 + 0.45 * k, y, 0, 0, 0.22])
    elif kind == 'walking 20':
        for _ in range(20):
            a = rng.uniform(0, 2 * math.pi); v = rng.uniform(0.8, 1.5)
            P.append([rng.uniform(-4, 9), rng.uniform(-36, -3), v * math.cos(a), v * math.sin(a), 0.22])
    elif kind == 'group across the path at 15 m':
        for k in range(6): P.append([-1.2 + 0.45 * k, -15.0, 0, 0, 0.22])
    elif kind == 'crowd at the tree row (hides it)':
        for _ in range(15): P.append([rng.uniform(4.0, 6.2), rng.uniform(-24, -10), 0, rng.choice((0, 0.6, -0.6)), 0.22])
    elif kind == 'row of people across at 30 m (fake end wall?)':
        for k in range(8): P.append([-1.6 + 0.42 * k, -30.0, 0, 0, 0.2])
    elif kind == 'people standing at the store poles (2-4 m left)':
        for _ in range(8): P.append([rng.uniform(1.5, 4.5), rng.uniform(-30, -10), 0, 0, rng.uniform(0.18, 0.28)])
    elif kind == 'people walking near the store poles':
        for _ in range(10):
            a = rng.uniform(0, 2 * math.pi); v = rng.uniform(0.8, 1.5)
            P.append([rng.uniform(0.5, 4.5), rng.uniform(-30, -10), v * math.cos(a), v * math.sin(a), 0.22])
    elif kind == 'mix of 20 at the poles (10 stand, 10 walk)':
        for _ in range(10): P.append([rng.uniform(1.0, 4.5), rng.uniform(-30, -10), 0, 0, 0.22])
        for _ in range(10):
            a = rng.uniform(0, 2 * math.pi); v = rng.uniform(0.8, 1.5)
            P.append([rng.uniform(-1, 4.5), rng.uniform(-30, -10), v * math.cos(a), v * math.sin(a), 0.22])
    elif kind == 'group of 5 in the path at 33-36 m':
        y = rng.uniform(-36, -33); x = rng.uniform(-0.4, 0.4)
        for k in range(5): P.append([x - 0.9 + 0.45 * k, y, 0, 0, 0.22])
    elif kind == '8 shoulder to shoulder across at 34 m':
        for k in range(8): P.append([-1.6 + 0.42 * k, -34.0, 0, 0, 0.2])
    elif kind == 'person 0.8 m in front of the end wall':
        P.append([rng.uniform(-0.3, 0.3), -39.2, 0, 0, 0.22])
    elif kind == 'pillar / pole 0.3 m in the path at 34 m (never moves)':
        P.append([0.0, -34.0, 0, 0, 0.15, 'fixed'])
    elif kind == 'people walking along with the rover':
        for dx in (-1.0, 1.2, 2.0): P.append([dx, -1.5, 0, -0.22, 0.22])
    return P


def run_crowd(seed, kind, preset, leave_s=30.0, wide=True):
    rng = random.Random(seed); segs0 = world(rng, people=0)
    ppl = crowd(rng, kind)
    th = math.radians(-90 + 4.0); x, y = 0.0, -1.0
    C = F.OpenSea(-90.0, **dict(preset, leg2=38.0))
    v0 = 0.22; dt = 0.05; t = 0; last = -1; d2 = 0; tgt = -90.0; maxlat = 0
    first_hold = None; end_cnt = 0
    hold = False; near_t = None; clear_t = None; holds = 0; hold_s = 0.0; false_end = False; end_ok = False; end_t = None
    while t < 900:
        for p in ppl:                                                   # people move; a blocking group leaves after leave_s
            p[0] += p[2] * dt; p[1] += p[3] * dt
            if p[0] < -5 or p[0] > 10: p[2] = -p[2]
            if p[1] < -38 or p[1] > -1: p[3] = -p[3]
            if first_hold is not None and t > first_hold + leave_s and p[2] == 0 and p[3] == 0 and len(p) < 6 and abs(p[0] - x) < 1.5:
                p[0] += (1.0 if p[0] >= x else -1.0) * dt               # people in the way walk off ~30 s after the stop
        if t - last >= 0.25:
            last = t
            segs = segs0 + [s for p in ppl for s in person_segs(p[0], p[1], p[4])]
            L = scan(x, y, th, segs, rng)
            if not hold: tgt = C.step(L, math.degrees(th), d2, 0.25)
            # firmware _front(): near (< 0.6 m) LiDAR obstacles need >= 4 neighbouring beams or the front ultrasonic
            # (< 50 cm); the simulated ultrasonic hears people within 1 m in the +-8 deg cone, else the floor/nothing (300)
            us = 300
            for p in ppl:
                dx, dy = p[0] - x, p[1] - y; f = dx * math.cos(th) + dy * math.sin(th); sd = -dx * math.sin(th) + dy * math.cos(th)
                if 0.05 < f < 0.5 and abs(sd) < 0.15 + 0.2: us = min(us, int(f * 100))
            fr = F.front_v8(L, [us, us, us])[0]
            # end wall rule
            good = d2 >= 0.85 * 38 and fr <= 1.5 and (F.front_wall_wide(L, 1.8) if wide else F.front_wall(L, 1.8))
            if good: end_cnt = min(END_TRIG + 2, end_cnt + END_INC)           # v10 leaky counter (firmware: +2 / -1, trigger 6)
            elif end_cnt > 0: end_cnt = max(0, end_cnt - END_DEC)
            if end_cnt >= END_TRIG:
                if -y < 40 - 1.5 - 0.6: false_end = True
                else: end_ok = True
                break
            # people rule: stop and wait, never swerve
            if not hold:
                if fr < 0.6:
                    near_t = t if near_t is None else near_t
                    if t - near_t >= 0.5:
                        hold = True; holds += 1; clear_t = None
                        if first_hold is None: first_hold = t
                else: near_t = None
            else:
                if fr > 1.0:
                    clear_t = t if clear_t is None else clear_t
                    if t - clear_t >= 2.0: hold = False; near_t = None
                else: clear_t = None
        v = 0.0 if hold else v0
        if hold: hold_s += dt
        e = w180(tgt - math.degrees(th)); th += math.radians(max(-3, min(3, 0.8 * e)) * dt) if not hold else 0
        x += v * math.cos(th) * dt; y += v * math.sin(th) * dt; d2 += v * dt; t += dt
        if -y > 12: maxlat = max(maxlat, abs(x))
    return dict(end_ok=end_ok, false_end=false_end, lat_end=abs(x), maxlat=maxlat, holds=holds, hold_s=hold_s, t=t)


CROWDS = ['standing 12', '1 person standing in the path (last 30 %)', 'group of 3 standing in the path (last 30 %)', 'walking 20', 'group across the path at 15 m', 'crowd at the tree row (hides it)',
          'row of people across at 30 m (fake end wall?)', 'people walking along with the rover']


def crowd_table(n=20):
    for wide in (False, True):
        print("END WALL CHECK: " + ("v9 flat AND WIDE (>= 1.1 m to each side)" if wide else "v8 flat only"))
        _crowd_table(n, wide)


def _crowd_table(n, wide):
    print(f"{'crowd scenario':48s} {'reached end':>11s} {'false end':>9s} {'side err end med/max':>21s} {'holds':>6s} {'wait s':>7s}")
    for kind in CROWDS:
        R = [run_crowd(900 + k, kind, F.OpenSea.V7, wide=wide) for k in range(n)]
        le = sorted(r['lat_end'] for r in R if r['end_ok'])
        print(f"{kind:48s} {sum(r['end_ok'] for r in R):5d}/{n:<5d} {sum(r['false_end'] for r in R):9d} "
              f"{(le[len(le) // 2] if le else 0):9.2f} / {(le[-1] if le else 0):5.2f} m {sum(r['holds'] for r in R) / n:6.1f} {sum(r['hold_s'] for r in R) / n:7.1f}")


ENVS = [("clear night (reference)", {}, {}),
        ("daylight ghosts 20 % of scans", dict(ghost=0.20), {}),
        ("low sun: 40 deg blind wedge, 30 % of scans", dict(sun=0.30, ghost=0.15), {}),
        ("falling snow: 8 % random near returns + 25 % lost beams", dict(precip=0.08, drop=0.25, ghost=0.10), {}),
        ("snowy ground: 8 % sideways slip + gyro drift -6 deg/min", {}, dict(slide=0.08, bias_dpm=-6)),
        ("worst: snow + slip + drift + sun", dict(precip=0.08, drop=0.25, ghost=0.2, sun=0.3), dict(slide=0.05, bias_dpm=6))]


def env_table(n=12):
    global ENV
    print(f"{'environment (no people)':56s} {'reached end':>11s} {'false end':>9s} {'false holds':>11s} {'side err at end med/max':>24s}")
    for name, env, kw in ENVS:
        ENV = env; R = [run_crowd(1300 + k, 'nobody', F.OpenSea.V7) for k in range(n)]
        le = sorted(r['lat_end'] for r in R if r['end_ok'])
        sl = [run(700 + k, F.OpenSea.V7, **kw) for k in range(n)] if kw else None
        extra = f"  [slip run: end err med {sorted(b for a, b, c in sl)[n // 2]:.2f} max {max(b for a, b, c in sl):.2f}]" if sl else ""
        print(f"{name:56s} {sum(r['end_ok'] for r in R):5d}/{n:<5d} {sum(r['false_end'] for r in R):9d} {sum(r['holds'] for r in R):11d} "
              f"{(le[len(le)//2] if le else 0):10.2f} / {(le[-1] if le else 0):5.2f} m{extra}")
    ENV = {}


if __name__ == "__main__":
    if "--tune" in sys.argv:
        import itertools
        inc, dec, trig = [int(v) for v in sys.argv[sys.argv.index("--tune") + 1:][:3]]
        END_INC, END_DEC, END_TRIG = inc, dec, trig
        out = []
        for kind, env in (("1 person standing in the path (last 30 %)", {}), ("group of 3 standing in the path (last 30 %)", {}), ("walking 20", {}),
                          ("nobody", dict(precip=0.03, drop=0.10)), ("nobody", dict(precip=0.08, drop=0.25, ghost=0.10)), ("nobody", dict(precip=0.08, drop=0.25, ghost=0.2, sun=0.3))):
            ENV = env; R = [run_crowd(1500 + k, kind, F.OpenSea.V7) for k in range(20)]
            out.append(f"{kind[:22]:22s} {'rain' if env else '':5s} reached {sum(r['end_ok'] for r in R):2d}/20 false {sum(r['false_end'] for r in R):2d}")
        print(f"inc {inc} dec {dec} trig {trig}: " + " | ".join(out), flush=True); sys.exit()
    if "--cases" in sys.argv:
        for name, kw in CASES:
            for tag, P in (("V6", F.OpenSea.V6), ("V7", F.OpenSea.V7)):
                print(batch(f"{tag} {name}", P, **kw), flush=True)
        sys.exit()
    if "--faults" in sys.argv:
        FAULTS = [("nominal", {}),
                  ("row missing completely (trees/poles hidden)", dict(missing=1.0)),
                  ("row gap 14-20 m (cars / a stall in front)", dict(rowgap=(14, 20))),
                  ("row 0.7 m closer than taught (6.0 m)", dict(rowoff_true=6.0)),
                  ("row 0.7 m farther than taught (7.3 m)", dict(rowoff_true=7.3)),
                  ("false line 4.5 m left, 12-22 m (bike rack)", dict(fake=(4.5, -12, -22))),
                  ("false line 8.5 m left, 10-24 m (hoarding)", dict(fake=(8.5, -10, -24))),
                  ("B wall behind hidden (people at the sign)", dict(nobwall=True)),
                  ("heading glitch +10 deg at 10 s", dict(jump=(10, 10))),
                  ("heading glitch -15 deg at 30 s", dict(jump=(30, -15))),
                  ("gyro +6/min and row gone", dict(bias_dpm=6, missing=1.0)),
                  ("worst: bias -6, glitch, slide, wobble, row gap", dict(bias_dpm=-6, jump=(20, 8), slide=0.03, wobble=0.4, rowgap=(12, 18)))]
        for name, kw in FAULTS:
            for tag, P in ((("V7", F.OpenSea.V7),) if "--v7only" in sys.argv else (("V6", F.OpenSea.V6), ("V7", F.OpenSea.V7))):
                print(batch(f"{tag} {name}", P, n=20, **kw), flush=True)
        sys.exit()
    if "--nearest" in sys.argv:
        KINDS = ['nobody', 'people standing at the store poles (2-4 m left)', 'people walking near the store poles', 'mix of 20 at the poles (10 stand, 10 walk)',
                 '1 person standing in the path (last 30 %)', 'group of 5 in the path at 33-36 m', '8 shoulder to shoulder across at 34 m',
                 'person 0.8 m in front of the end wall', 'pillar / pole 0.3 m in the path at 34 m (never moves)', 'walking 20']
        n = 20
        print(f"{'scenario (20 starts)':55s} {'rule':8s} {'reached':>7s} {'FALSE end':>9s} {'holds':>6s} {'wait s':>7s} {'side end med':>12s}")
        for kind in KINDS:
            for tag, near in (('median', False), ('nearest', True)):
                F.WIDE_NEAREST = near
                R = [run_crowd(2100 + k, kind, F.OpenSea.V7) for k in range(n)]
                lat = sorted(r['lat_end'] for r in R)[n // 2]
                print(f"{kind[:55]:55s} {tag:8s} {sum(r['end_ok'] for r in R):4d}/{n} {sum(r['false_end'] for r in R):9d} {sum(r['holds'] for r in R):6d} {sum(r['hold_s'] for r in R) / n:7.1f} {lat:12.2f}", flush=True)
        sys.exit()
    if "--env" in sys.argv: env_table(); sys.exit()
    if "--crowd" in sys.argv: crowd_table(); sys.exit()
    for name, kw in CASES:
        for tag, P in (("V6", F.OpenSea.V6), ("V7", F.OpenSea.V7)):
            print(batch(f"{tag} {name}", P, **kw))
    print(); crowd_table()
