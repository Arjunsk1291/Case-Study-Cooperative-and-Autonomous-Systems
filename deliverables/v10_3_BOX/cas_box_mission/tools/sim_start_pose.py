#!/usr/bin/env python3
"""sim_start_pose.py - v10.3 START POSE: "Start open sea HERE" with the rover placed ANYWHERE in the start area (about 3 x 3 paving blocks)
and facing ANY direction (the wall, left, right, front). Mirror of RouteMission::_alignStart phases 10-11 (orient) + the pillar alignment.
Scene (sim frame: south = -90 deg = the open sea, left = +x, B wall = the line y = 0 behind the start):
  the B wall (cement structure) behind: from x = -3.5 (its END, on the right) to +25; a gap-free line
  the sign pillar + board 1.3 m right of the strip; a second pole; the structure AHEAD-LEFT (3 m left, 2-3.6 m ahead)
Controller:
  1. stand still, 8 scans: dominant straight line within 0.6-4 m (>= 8 points, length >= 2 m, rms <= 6 cm) = the B wall;
     bearing of its foot point (deg clockwise from the front) -> the heading that faces AWAY from it = bearing + 180
  2. |turn| <= 8 deg: ok (nothing); else pivot in place (full power) to it, measure again (max 2 rounds)
  3. then sim_start_align.StartAlign (pillar 0.85-1.75 m = untouched, else pivot-roll-pivot back), wide window (placed anywhere)
Success = heading within 8 deg of B-perpendicular, lateral offset <= 0.5 m (the band edge: 0.85-1.75 m from the pillar is left alone), no contact with a wall/pole/structure.
  python3 sim_start_pose.py [--quick] [--stress]"""
import sys, math, random; sys.path.insert(0, '.')
import sim_open_sea as S, sim_start_align as A

TURN_OK = 8.0


def world(rng, person=False):
    segs = [((-3.5, 0), (25, 0))]                                              # B wall: ends 3.5 m right of the strip
    py = -rng.uniform(1.5, 2.8)                                                # real logs: the pillar is 1.5-2.8 m beyond the wall plane (mostly 2.7)
    segs += [((-1.3, py + .2), (-1.7, py + .2)), ((-1.7, py + .2), (-1.7, py - .2)), ((-1.7, py - .2), (-1.3, py - .2)), ((-1.3, py - .2), (-1.3, py + .2))]
    segs += [((-1.5, py), (-2.6, py))]                                         # board
    segs += [((-6.0, -3.0), (-5.6, -3.0)), ((-5.6, -3.0), (-5.6, -3.4)), ((-6.0, -3.4), (-6.0, -3.0))]
    sy = -2.8 + rng.uniform(-0.6, 0.6)
    segs += [((2.8, sy), (3.2, sy)), ((3.2, sy), (3.2, sy - 1.4)), ((3.2, sy - 1.4), (2.8, sy - 1.4)), ((2.8, sy - 1.4), (2.8, sy))]   # structure ahead-left
    if person:
        qx, qy = rng.uniform(-3.0, 3.0), rng.uniform(-3.5, -0.8); segs += [((qx - .2, qy - .2), (qx + .2, qy - .2)), ((qx + .2, qy - .2), (qx + .2, qy + .2)), ((qx - .2, qy + .2), (qx + .2, qy + .2)), ((qx - .2, qy + .2), (qx - .2, qy - .2))]
    return segs


def wall_line(L):
    """RANSAC over the 90 beams (rover frame x fwd, y right). -> (bearing deg clockwise of the foot point, dist, length) or None"""
    P = []
    for i in range(90):
        cm = L[i]
        if not cm or cm < 60 or cm > 400: continue
        a = math.radians(i * 4 + 2); P.append((cm / 100 * math.cos(a), cm / 100 * math.sin(a)))
    best = (0, None)
    for i in range(len(P)):
        for j in range(i + 1, len(P)):
            dx, dy = P[j][0] - P[i][0], P[j][1] - P[i][1]; n = math.hypot(dx, dy)
            if n < 0.5: continue
            nx, ny = -dy / n, dx / n; c = nx * P[i][0] + ny * P[i][1]
            k = sum(1 for p in P if abs(nx * p[0] + ny * p[1] - c) < 0.10)
            if k > best[0]: best = (k, (nx, ny, c))
    if best[0] < 8: return None
    nx, ny, c = best[1]; Q = [p for p in P if abs(nx * p[0] + ny * p[1] - c) < 0.10]
    mx, my = sum(p[0] for p in Q) / len(Q), sum(p[1] for p in Q) / len(Q)
    sxx = sum((p[0] - mx) ** 2 for p in Q); syy = sum((p[1] - my) ** 2 for p in Q); sxy = sum((p[0] - mx) * (p[1] - my) for p in Q)
    th = 0.5 * math.atan2(2 * sxy, sxx - syy); ux, uy = math.cos(th), math.sin(th)
    al = [(p[0] - mx) * ux + (p[1] - my) * uy for p in Q]; rms = math.sqrt(sum((-uy * (p[0] - mx) + ux * (p[1] - my)) ** 2 for p in Q) / len(Q))
    if max(al) - min(al) < 2.0 or rms > 0.06: return None
    d = -uy * mx + ux * my                                                      # signed distance of the line from the rover
    fx, fy = -uy * d, ux * d                                                    # foot point
    return math.degrees(math.atan2(fy, fx)), abs(d), max(al) - min(al)


def w180(a): return (a + 180) % 360 - 180


def measure_wall(scans):
    r = [wall_line(L) for L in scans]; r = [x for x in r if x]
    if len(r) < 5: return None
    b = sorted(x[0] for x in r); m = b[len(b) // 2]
    ok = [x for x in r if abs(w180(x[0] - m)) <= 8]
    if len(ok) < 5: return None
    return m, sorted(x[1] for x in ok)[len(ok) // 2]


def place(seed, person=False, slip=0.0, pivot_sd=3.0, dropout=0.12, hdg=None):
    rng = random.Random(seed); segs = world(rng, person)
    while True:
        x = rng.uniform(-1.0, 2.0); y = rng.uniform(-3.0, -1.0)
        if min(A.min_dist(segs, x, y) for _ in (0,)) > 0.5: break
    th = math.radians(rng.uniform(0, 360)) if hdg is None else math.radians(hdg)
    t = 0.0; hits = []; base = -math.pi / 2; why = ''
    A.WIDE = True; A.MAX_ROUNDS = 4
    for rnd in range(3):
        scans = [S.scan(x, y, th, segs, rng) for _ in range(8)]; t += 2.0
        m = measure_wall(scans)
        if m is None: why = 'no wall seen'; break
        turn = w180(m[0] + 180)                                               # deg clockwise to face away from the wall
        if abs(turn) <= TURN_OK: break
        if rnd == 2: why = 'still off'; break
        th -= math.radians(turn + rng.gauss(0, pivot_sd)); t += 3.0 + abs(turn) / 60
    A.YMIN = -(m[1] if m else 1.0) + 0.4 if m else -2.6
    herr0 = math.degrees(w180(th - base) if False else ((th - base + math.pi) % (2 * math.pi) - math.pi))
    xa, herr, ta, rounds, moved = A.align(rng, x, slip=slip, person=person, dropout=dropout, pivot_sd=pivot_sd, y0=y, th0=th, segs=segs, hits=hits)
    A.WIDE = False; A.YMIN = -1.0; A.MAX_ROUNDS = 3
    hdg_err = herr if moved else herr0
    return dict(x=xa, herr=hdg_err, herr0=herr0, t=t + ta, hit=bool(hits), moved=moved, why=why, th0=math.degrees(th))


def main():
    quick = '--quick' in sys.argv; n = 60 if quick else 300
    S.ENV = dict(precip=0.08, drop=0.25, ghost=0.2) if '--stress' in sys.argv else {}
    kw = dict(slip=0.08, pivot_sd=6.0, dropout=0.25) if '--stress' in sys.argv else {}
    R = [place(1000 + k, person=(k % 3 == 0), **kw) for k in range(n)]
    ok = [r for r in R if abs(r['herr0'] if not r['moved'] else r['herr']) <= 8 and abs(r['x']) <= 0.5 and not r['hit']]
    print(f"{'STRESS ' if '--stress' in sys.argv else ''}placements: {n} (random pose x -1..+2 m, 1-3 m from the wall, ANY heading 0-360 deg)")
    print(f"  success (heading <= 8 deg, lateral <= 0.5 m, no contact): {len(ok)} of {n}")
    print(f"  heading error after the orient step: median {sorted(abs(r['herr0']) for r in R)[n//2]:.1f} deg, 90 % {sorted(abs(r['herr0']) for r in R)[int(.9*n)]:.1f} deg")
    print(f"  lateral offset at the end: median {sorted(abs(r['x']) for r in R)[n//2]:.2f} m, 90 % {sorted(abs(r['x']) for r in R)[int(.9*n)]:.2f} m | contacts {sum(r['hit'] for r in R)} | time median {sorted(r['t'] for r in R)[n//2]:.0f} s")
    bad = [(round(r['th0']), round(r['x'], 2), round(r['herr0'], 1), r['why'], r['hit']) for r in R if r not in ok]
    print('  failures (heading0, x, herr, why, hit):', bad[:12])
    S.ENV = {}


if __name__ == '__main__': main()
