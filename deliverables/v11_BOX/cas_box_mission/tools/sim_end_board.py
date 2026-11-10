#!/usr/bin/env python3
"""sim_end_board.py - Open Sea with the END BOARD (low wall / sign board on the right, seen only in the last ~5 m before the end wall). Mirror of the v10.3 rule in
RouteMission.cpp (L2_GO end board block): after 85 % of the crossing a right-side line 0.4-2.2 m, parallel within 12 deg, 4 consistent fits -> steer to ebTarget 1.2 m
(10 deg/m, max 8 deg, +-1 deg per scan), fades out when the line is lost. Everything else is the unchanged open-sea mirror (fwmirror.OpenSea V7).
World = sim_open_sea.world + the board at 1.2 m right of the strip from 34.5 to 39.0 m. Metric: lateral error to the strip when the rover reaches 38.5 m (the end wall is at 40 m).
  python3 sim_end_board.py [--quick]"""
import sys, math, random; sys.path.insert(0, '.')
import sim_open_sea as S, fwmirror as F
w180 = F.w180
EB_TARGET = 1.2
LIM, WIN_HI = 12.0, 3.0         # steering limit (deg) and farthest board distance accepted (m)

def run_eb(seed, eb, bias_dpm=0.0, turn_err=4.0, x0=0.0, slide=0.0, blind=0.3, board_x=-1.2, **wk):
    rng = random.Random(seed); segs = S.world(rng, **wk)
    segs.append(((board_x, -34.5), (board_x, -39.0)))                       # the board: right of the strip (right = -x), seen in the last 5 m
    th = math.radians(-90 + turn_err); x, y = x0, -1.0; drift = 0.0
    yaw = lambda: w180(math.degrees(th) + drift)
    C = F.OpenSea(-90.0, **dict(F.OpenSea.V7, leg2=38.0))
    v = 0.22; dt = 0.05; t = 0; last = -1; d2 = 0; tgt = -90.0; on = False; n = 0; edir = 0.0; eoff = 0.0; seen = 0.0; first = None
    while t < 500:
        if t - last >= 0.25:
            last = t; L = S.scan(x, y, th, segs, rng)
            if blind and rng.random() < blind: L = [0 if 50 <= i <= 85 else c for i, c in enumerate(L)]
            tgt = C.step(L, yaw(), d2, 0.25)
            if eb and d2 >= 0.85 * 38.0:
                w = F.wallfit(L, False)                                          # right side line: (dist, angle, rms, n, length)
                ok = w and w[2] <= 0.05 and 0.4 <= w[0] <= WIN_HI and abs(w[1]) <= 12.0
                if ok:
                    if not on:
                        n = n + 1 if (n and abs(w[1] - edir) < 5.0) else 1; edir = w[1]
                        if n >= 4: on = True; first = (round(d2, 1), round(w[0], 2))
                    else:
                        seen = t; eoff += max(-1.0, min(1.0, max(-LIM, min(LIM, 10.0 * (w[0] - EB_TARGET))) - eoff))
                elif on and t - seen > 1.0: eoff += max(-0.5, min(0.5, 0.0 - eoff))
                elif not on: n = 0
            if on: tgt = w180(tgt - eoff)
        e = w180(tgt - yaw()); th += math.radians(max(-3, min(3, 0.8 * e)) * dt)
        drift += bias_dpm / 60 * dt
        x += v * math.cos(th) * dt + slide * v * math.cos(th - math.pi / 2) * dt
        y += v * math.sin(th) * dt + slide * v * math.sin(th - math.pi / 2) * dt
        d2 += v * dt; t += dt
        if -y >= 38.5: break
    return x, first

def q(v, p): v = sorted(v); return v[min(len(v) - 1, int(p * len(v)))]

def main():
    quick = '--quick' in sys.argv; n = 4 if quick else 10
    DIST = {'clean': dict(), 'sparse row + gyro +3/min': dict(missing=0.6, blind=0.5, bias_dpm=3), 'sparse + gyro -3/min': dict(missing=0.6, blind=0.5, bias_dpm=-3),
            'sparse + slide +3 %': dict(missing=0.6, blind=0.5, slide=0.03), 'sparse + slide -3 %': dict(missing=0.6, blind=0.5, slide=-0.03)}
    print('lateral error to the strip at 38.5 m (m, + = LEFT of the strip). board at 1.2 m right; the end wall is the next thing')
    for dn, kw in DIST.items():
        print('--', dn)
        for x0 in ((-1.0, 0.0, 1.0) if not quick else (-1.0, 1.0)):
            a = [run_eb(300 + k, False, x0=x0, **kw)[0] for k in range(n)]; r = [run_eb(300 + k, True, x0=x0, **kw) for k in range(n)]; b = [z[0] for z in r]
            on = sum(1 for z in r if z[1])
            print(f"   start {x0:+.1f} m: eb OFF median {q(a,.5):+.2f} (|.| 90% {q([abs(z) for z in a],.9):.2f}) | eb ON median {q(b,.5):+.2f} (|.| 90% {q([abs(z) for z in b],.9):.2f}) | board used in {on}/{n}", flush=True)

if __name__ == '__main__': main()
