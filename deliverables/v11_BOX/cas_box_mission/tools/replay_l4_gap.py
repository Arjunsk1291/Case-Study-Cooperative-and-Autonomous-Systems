#!/usr/bin/env python3
"""replay_l4_gap.py - v10.3 leg-4 RAMP OPENING finder on real logs (RouteMission::_l4Gap mirror).
The ramp to Block I is a ~2.5-3 m opening in a CROSS LINE (step edge / step lights / low wall) between the KARL facade
(left) and the railing (right). Per scan (rover frame, x right, y forward):
  cross line: points 2.5-10 m ahead; histogram of y in 0.25 m bins over x -5..+5; the line = the nearest y bin with
              points spread over >= 2.0 m of x (both sides of the opening together)
  opening:    along that line, x intervals without line points (|y - yL| <= 0.6) whose beams go >= 1.5 m beyond it
              (or no return); width 2.2-4.0 m; chosen = the LEFTMOST one within +-4 m (the stairs right of it look open too)
  out:        (yL, x centre, width)
Prints per run: first scan with an opening, centre/lateral error over distance, and where it would stop if not inside."""
import sys, math; sys.path.insert(0, '.')
import decode_mission_log as D

RUNS = {'c102_2 (ok)': 'campus_10.2/mis_001 (2).bin', 'c102_3 (ok)': 'campus_10.2/mis_001 (3).bin',
        'c102_4 (HIT steps)': 'campus_10.2/mis_001 (4).bin', 'campus_9 (v9)': 'campus_9/mis_001.bin'}


def pts(L, rot=0.0):
    """scan points; rot = rover heading rel. to the walkway (deg, + = right) -> points in the WALKWAY frame"""
    P = []
    for i in range(90):
        cm = L[i]
        if not cm or cm < 35: continue
        a = math.radians(i * 4 + 2 + rot); d = cm / 100
        P.append((d * math.sin(a), d * math.cos(a)))
    return P


def find_gap(L, ymin=2.5, ymax=10.0, xspan=2.0, rot=0.0):
    P = [(x, y) for x, y in pts(L, rot) if abs(x) <= 7.0]
    best = None
    for k in range(int(ymin / 0.25), int(ymax / 0.25)):
        y0 = k * 0.25
        xs = sorted(x for x, y in P if y0 - 0.3 <= y <= y0 + 0.3)
        if len(xs) < 4 or xs[-1] - xs[0] < xspan: continue
        # the opening: biggest x gap between line points that beams pass through
        cands = []
        for a, b in zip(xs, xs[1:]):
            if b - a < 2.2 or b - a > 4.0: continue        # ramp opening 2.7-3.8 m; wider = open space / stairs
            cx = (a + b) / 2
            far = [y for x, y in P if a + 0.2 < x < b - 0.2 and y > y0 + 0.6]
            near = [y for x, y in P if a + 0.2 < x < b - 0.2 and y < y0 - 0.6 and y > 0.3]
            if near: continue                              # something in front of the opening (person / post)
            # v10.3: the RAMP opening has the KARL FACADE running on from its left end (an L corner): >= 3 points
            # within 0.5 m of x = a, 0.4-8 m beyond the line. Bench rows / open lawn do not have it.
            fac = sum(1 for x, y in P if abs(x - a) <= 0.5 and y0 + 0.4 <= y <= y0 + 8.0)
            rail = sum(1 for x, y in P if abs(x - b) <= 0.5 and y0 + 0.4 <= y <= y0 + 5.0)   # or the RAILING on from b
            if fac < 3 and rail < 3: continue
            cands.append((b - a, cx, a, b))
        cands = [c for c in cands if abs(c[1]) <= 4.0]
        if cands:                                          # v10.3: the RAMP is the LEFTMOST opening (beside the KARL
            w, cx, a, b = min(cands, key=lambda c: c[2])   # facade); the stairs right of the railing also look open
            best = (y0, cx, w, a, b)
        break                                              # the nearest line only
    return best


if __name__ == '__main__':
    for name, f in RUNS.items():
        R = D.decode('../../Log_run/' + f); l4 = [r for r in R if r['leg'] == 4 and r['leg_dist_m'] >= 12]
        out = []; nxt = 0
        for r in l4:
            if r['leg_dist_m'] < nxt: continue
            nxt = r['leg_dist_m'] + 0.5
            g = find_gap([r['lidar_%03d' % (i * 4)] for i in range(90)])
            if g: out.append(f"{r['leg_dist_m']:.1f}m: line {g[0]:.1f} ahead, opening x {g[3]:+.1f}..{g[4]:+.1f} (centre {g[1]:+.2f}, w {g[2]:.1f})")
        print(f"== {name}  leg4 length {l4[-1]['leg_dist_m']:.1f} m"); print('\n'.join(out[:40]) or '   no opening found')
