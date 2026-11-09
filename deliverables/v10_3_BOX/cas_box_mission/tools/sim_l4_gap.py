#!/usr/bin/env python3
"""sim_l4_gap.py - closed-loop test of the v10.3 leg-4 RAMP OPENING steering (RouteMission::_l4Gap mirror).
World (walkway frame, x right, y forward, line at y = Y), measured from the leg-4 maps of c102_2/3/4 and campus_9:
  KARL facade (left) x = -1.45 from the line on; cross line (step edge / low wall / lights) left of it from x = -7;
  ramp opening 2.9 m (-1.45 .. +1.45); railing x = +1.45 from the line on + its cement block (1.45..2.4, -0.3..0.6);
  STAIRS right of the railing = invisible to the LiDAR (below it); only sparse step lights (variant) mark the line there;
  benches 3.5 m left on the walkway; the far building behind the opening.
Rover 0.30 m/s, heading follows the target (2 deg per 250 ms scan, DriveStraight ~12 deg/s, pivot if > 18 deg off),
people rule: something within 0.6 m +-16 deg -> HOLD (wait, never swerve).
Result: PASS = crossed the line inside the opening with >= 0.2 m clearance each side; STOP = stopped before the line
(safe, asks the user); HOLD = waiting for a person (safe); HIT = crossed outside the opening (= run 4)."""
import sys, math, random; sys.path.insert(0, '.')
from replay_l4_gap import find_gap, pts
from sim_open_sea import ray

HALF = 0.13          # rover half width
Y = 14.0             # the cross line is 14 m ahead of the start (left wall ended ~12-16 m before it in the logs)


def box(x0, y0, x1, y1): return [((x0, y0), (x1, y0)), ((x1, y0), (x1, y1)), ((x1, y1), (x0, y1)), ((x0, y1), (x0, y0))]


def world(variant):
    S = [((-7, Y), (-1.45, Y)), ((-1.45, Y), (-1.45, Y + 12)), ((1.45, Y), (1.45, Y + 5)), ((-1.45, Y + 12), (8, Y + 12))]
    S += box(1.45, Y - 0.3, 2.4, Y + 0.6)
    for by in (2.0, 4.5, 7.0): S += box(-3.9, by, -3.3, by + 1.6)                    # benches
    if variant in ('lights', 'person_in', 'person_front'): 
        for lx in (3.6, 4.8, 6.0): S += box(lx, Y - 0.08, lx + 0.15, Y + 0.08)        # step lights right of the railing
    if variant == 'person_in': S += box(-0.2, Y + 0.8, 0.2, Y + 1.2)                  # someone standing in the ramp
    if variant == 'person_front': S += box(-0.2, Y - 1.5, 0.2, Y - 1.1)               # someone in front of the opening
    return S


def scan(x, y, th, S, rng):
    L = []
    for i in range(90):
        b = th + math.radians(i * 4 + 2)                                               # clockwise from +y
        d = ray(x, y, math.pi / 2 - b, S)
        L.append(0 if d is None or rng.random() < 0.12 else max(35, int(round((d + rng.gauss(0, 0.02)) * 100))))
    return L


def line_ahead(L, ymin=0.3, ymax=10.0, rot=0.0):
    """nearest cross line (>= 6 points spread over >= 2 m, any y bin) - also when no opening is found"""
    P = [(x, y) for x, y in pts(L, rot) if abs(x) <= 5.0]
    for k in range(int(ymin / 0.25), int(ymax / 0.25)):
        y0 = k * 0.25; xs = sorted(x for x, y in P if y0 - 0.3 <= y <= y0 + 0.3)
        if len(xs) >= 6 and xs[-1] - xs[0] >= 2.0: return y0
    return None


class GapCtl:
    """mirror of RouteMission::_l4Gap. Opening confirmed = 3 of the last 5 scans agree within 0.5 m. Then aim at its
    centre 1 m beyond the line (max 25 deg). Line <= 1.6 m ahead and the rover not inside the confirmed opening (0.35 m
    margin) -> STOP. A cross line seen on 3 scans but no confirmed opening when it is <= 2.0 m ahead -> STOP."""
    def __init__(s): s.buf = []; s.conf = None; s.age = 0; s.lineN = 0; s.commit = False

    def step(s, L, hdg=0.0):
        """hdg = rover heading rel. to the walkway (deg, + = right). Returns (want heading rel. walkway or None, stop)"""
        if s.commit: return 0.0, False                       # inside the opening at <= 1.6 m: straight through
        gap = find_gap(L, ymin=0.8 if s.conf else 2.5, rot=hdg); yl = line_ahead(L, 1.0, rot=hdg)   # confirmed: track it down to 0.8 m
        s.lineN = s.lineN + 1 if yl is not None and yl <= 6.0 else 0
        if gap: s.buf = (s.buf + [gap])[-5:]; s.age = 0
        else: s.age += 1
        if s.age > 8: s.buf = []; s.conf = None
        if len(s.buf) >= 3:
            cs = sorted(g[1] for g in s.buf); m = cs[len(cs) // 2]
            good = [g for g in s.buf if abs(g[1] - m) <= 0.5]
            if len(good) >= 3: s.conf = good[-1]
        if s.conf is None:
            if s.lineN >= 3 and yl <= 2.0: return None, True
            return None, False
        y0, c, w, a, b = s.conf
        if y0 <= 1.6:
            if not (a + 0.35 <= -HALF and HALF <= b - 0.35): return None, True
            s.commit = True; return 0.0, False
        if abs(c) <= 0.3: return 0.0, False                  # lined up: walkway direction (perpendicular to the line)
        # aim at a point 1 m before the opening centre (rover frame); >= 18 deg away = stop + pivot (full power)
        left = math.degrees(math.atan2(-c, max(y0 - 1.0, 0.8)))   # walkway frame: bearing of the aim point
        return max(-50.0, min(50.0, -left)), False


def run(seed, dx, dth, variant, use, leave=None):
    """leave = seconds after which the person walks away (STOP becomes a WAIT, like the people hold)"""
    rng = random.Random(seed); S = world(variant); waited = 0.0
    x, y, th = dx, 0.0, math.radians(dth); tgt = th; ctl = GapCtl(); t = 0
    while t < 200:
        if leave is not None and t >= leave and S is not None and variant.startswith('person'): S = world('plain'); variant = 'plain'
        L = scan(x, y, th, S, rng)
        if any(c and c < 60 and (i <= 4 or i >= 86) for i, c in enumerate(L)):
            if leave is None: return 'HOLD', x                                          # people rule: wait
            t += 0.25; waited += 0.25; continue
        want = 0.0
        if use:
            w, stop = ctl.step(L, math.degrees(th))
            if stop:
                if leave is None or t > leave + 20: return 'STOP', x                   # 20 s more: ask the user
                t += 0.25; waited += 0.25; ctl.lineN = 0; continue                       # wait in place (ramped stop)
            if w is not None: want = math.radians(w)
        e = (want - tgt + math.pi) % (2 * math.pi) - math.pi
        if abs(e) > math.radians(18): tgt = want; th = want; t += 2.0      # stop (ramp) + pivot at full power: time only
        else: tgt += max(-math.radians(2), min(math.radians(2), e))
        for _ in range(5):
            e = (tgt - th + math.pi) % (2 * math.pi) - math.pi
            th += max(-math.radians(0.6), min(math.radians(0.6), e))
            x += 0.30 * 0.05 * math.sin(th); y += 0.30 * 0.05 * math.cos(th)
        t += 0.25
        if y >= Y:
            return ('PASS' if -1.45 + 0.2 <= x - HALF and x + HALF <= 1.45 - 0.2 else 'HIT'), x
    return 'TIMEOUT', x


if __name__ == '__main__':
    OFF = (-2.0, -1.5, -1.0, -0.5, 0.0, 0.5, 1.0, 1.5, 2.0, 2.5)
    for variant in ('plain', 'lights', 'person_in', 'person_front'):
        for use in (False, True):
            res = [(dx, dth) + run(100 + k, dx, dth, variant, use) for k, (dx, dth) in
                   enumerate((dx, dth) for dx in OFF for dth in (-5, 0, 5))]
            c = {}
            for r in res: c[r[2]] = c.get(r[2], 0) + 1
            print(f"{variant:12s} {'v10.3 opening' if use else 'v10.2 heading'}: {len(res)} starts {c}")
            if use: print('     HIT at', [(dx, dth, round(x, 2)) for dx, dth, r, x in res if r == 'HIT'])
    for variant in ('person_in', 'person_front'):
        res = [run(300 + k, dx, dth, variant, True, leave=30.0) for k, (dx, dth) in
               enumerate((dx, dth) for dx in OFF for dth in (-5, 0, 5))]
        c = {}
        for r, x in res: c[r] = c.get(r, 0) + 1
        print(f"{variant:12s} v10.3, person leaves after 30 s: {len(res)} starts {c}")
