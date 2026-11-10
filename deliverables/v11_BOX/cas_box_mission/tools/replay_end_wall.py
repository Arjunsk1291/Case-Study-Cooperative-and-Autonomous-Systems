#!/usr/bin/env python3
"""replay_end_wall.py - every real Open Sea log: where would the v9 end-wall rule and the v10 rule stop?
v9: gate 0.70*36 m, _frontWallWide with MEDIAN f0, condition continuous 0.8 s.
v10: gate 0.85*36 m, NEAREST f0, leaky counter +2/-1 per 250 ms, trigger 6.  Both: front <= 1.5 m, level (pitch vs 3 s LP, 2 deg, 1 s)."""
import sys, math, glob; sys.path.insert(0, '.')
import decode_mission_log as D, fwmirror as F

def wide(L, max_m, nearest):
    if not F.front_wall(L, max_m): return False
    fc = sorted(L[k % 90] / 100 * math.cos(math.radians(k * 4 + 2)) for k in range(-4, 4) if L[k % 90])
    if not fc: return False
    f0 = fc[0] if nearest else fc[len(fc) // 2]
    def side(ks):
        n = 0
        for k in ks:
            c = L[k % 90]
            if not c: continue
            a = math.radians(k * 4 + 2); d = c / 100
            if abs(d * math.cos(a) - f0) <= 0.5 and abs(d * math.sin(a)) >= 1.1: n += 1
        return n
    return side(range(5, 16)) >= 2 and side(range(-16, -5)) >= 2

for f in sorted([g for g in glob.glob('../../Log_run/**/*.bin', recursive=True) if 'img_' not in g]):
    try: R = D.decode(f)
    except BaseException: continue
    l2 = [r for r in R if r['leg'] == 2]
    go = [r for r in l2 if r['state'] == 'l2_go']
    if len(go) < 50: continue
    real_end = go[-1]['leg_dist_m']; nxt = l2[l2.index(go[-1]) + 1]['state'] if l2.index(go[-1]) + 1 < len(l2) else 'log end'
    lp = None; lvl_ms = 0; last = None; on9 = None; t9 = None; cnt = 0; t10 = None; tick = -1
    for r in go:
        t = r['t_s']
        if last is not None and 0 < t - last < 0.2:
            dt = t - last; lp = r['pitch'] if lp is None else lp + (r['pitch'] - lp) * dt / (3 + dt)
            if abs(r['pitch'] - lp) > 2.0: lvl_ms = t
        elif lp is None: lp = r['pitch']
        last = t; level = t - lvl_ms >= 1.0
        L = [r['lidar_%03d' % (i * 4)] for i in range(90)]; d = r['leg_dist_m']; fm = r['front_m']
        c9 = d >= 0.70 * 36 and fm <= 1.5 and level and wide(L, 1.8, False)
        c10 = d >= 0.85 * 36 and fm <= 1.5 and level and wide(L, 1.8, True)
        if t9 is None:
            if c9: on9 = on9 or t
            else: on9 = None
            if on9 is not None and t - on9 >= 0.8: t9 = (round(d, 1), round(t, 1))
        if t10 is None and t - tick >= 0.25:
            tick = t
            cnt = min(8, cnt + 2) if c10 else max(0, cnt - 1)
            if cnt >= 6: t10 = (round(d, 1), round(t, 1))
    name = f.split('Log_run/')[1]
    print(f'{name[:40]:40s} real end of l2_go at {real_end:5.1f} m -> {nxt:10s} | v9 rule {t9} | v10 rule {t10}')
