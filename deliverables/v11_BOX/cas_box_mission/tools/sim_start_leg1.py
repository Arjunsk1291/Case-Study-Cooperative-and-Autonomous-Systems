#!/usr/bin/env python3
"""sim_start_leg1.py - MISSION START at the B facade (photos: Log_run/campus_10.3/start positions). The rover is placed anywhere in the start area (0.7-2.7 m from the
facade, 0-3 m along it) facing ANY way (0-360 deg: the facade, backwards, the sea, the pool, diagonal). Mirror of RouteMission::_startOrient: 8 scans standing still, the facade =
the dominant straight line within 0.6-4 m (sim_start_pose.wall_line); turn = bearing + 90 deg clockwise puts it on the LEFT; <= 8 deg -> start; else pivot in place (max 2).
World (x right, leg 1 = +y, the facade on the left at x = 0): glass door (no LiDAR return) y -0.5..1.5, window recess 0.15 m at y 3.5..5, pillars on the right (x = 2.3) every 8 m,
people (0.4 m squares) near. Success: heading within 10 deg of the leg-1 direction (wall on the left, parallel), no contact. Time includes the pivots (full power).
  python3 sim_start_leg1.py [--stress]"""
import sys, math, random; sys.path.insert(0, '.')
import sim_open_sea as S, sim_start_pose as P
def box(x0, y0, x1, y1): return [((x0, y0), (x1, y0)), ((x1, y0), (x1, y1)), ((x1, y1), (x0, y1)), ((x0, y1), (x0, y0))]
def world(rng, person):
    W = [((0.0, -12.0), (0.0, -0.5)), ((0.0, 1.5), (0.0, 3.5)), ((0.0, 3.5), (0.15, 3.5)), ((0.15, 3.5), (0.15, 5.0)), ((0.15, 5.0), (0.0, 5.0)), ((0.0, 5.0), (0.0, 30.0))]
    for y in (6.0, 14.0, 22.0): W += box(2.1, y, 2.5, y + 0.4)                       # pillars on the right
    if person:
        qx, qy = rng.uniform(1.0, 4.5), rng.uniform(-1.0, 4.0); W += box(qx - .2, qy - .2, qx + .2, qy + .2)
    return W
def trial(seed, person, slip=0.0, pivot_sd=3.0):
    rng = random.Random(seed); W = world(rng, person)
    x = rng.uniform(0.7, 2.7); y = rng.uniform(0.0, 3.0); th = math.radians(rng.uniform(0, 360)); t = 0.0; why = ''; th0 = th
    for rnd in range(3):
        scans = [S.scan(x, y, th, W, rng) for _ in range(8)]; t += 2.0
        m = P.measure_wall(scans)
        if m is None: why = 'no wall'; break
        turn = P.w180(m[0] + 90.0)
        if abs(turn) <= 8.0: break
        if rnd == 2: why = 'still off'; break
        th -= math.radians(turn + rng.gauss(0, pivot_sd)); t += 3.0 + abs(turn) / 60
    herr = (math.degrees(th) - 90.0 + 180) % 360 - 180
    return dict(herr=herr, why=why, t=t, x0=x, y0=y, th0=th0, th1=th)
def main():
    st = '--stress' in sys.argv
    if st: S.ENV = dict(precip=0.08, drop=0.25, ghost=0.2)
    kw = dict(slip=0.08, pivot_sd=6.0) if st else {}
    n = 300; R = [trial(2000 + k, person=(k % 3 == 0), **kw) for k in range(n)]
    ok = [r for r in R if abs(r['herr']) <= 10 and not r['why']]
    e = sorted(abs(r['herr']) for r in R)
    print(f"{'STRESS ' if st else ''}start at the facade: {n} placements (0.7-2.7 m from the wall, ANY heading 0-360 deg, people, glass door gap, window recess)")
    print(f"  wall on the left, parallel within 10 deg: {len(ok)} of {n} | heading error median {e[n//2]:.1f} deg, 90 % {e[int(.9*n)]:.1f} deg | time median {sorted(r['t'] for r in R)[n//2]:.0f} s | 'no wall': {sum(1 for r in R if r['why']=='no wall')}, 'still off': {sum(1 for r in R if r['why']=='still off')}")
    bad = [(round(r['herr'], 1), r['why'], round(r['x0'], 1)) for r in R if r not in ok]; print('  failures (herr, why, distance from the wall):', bad[:10])
    S.ENV = {}
if __name__ == '__main__': main()
