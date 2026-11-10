#!/usr/bin/env python3
"""replay_leg4_v12.py - the leg-4 walkway rules on the REAL scans of 6 logs, v11 versus v12: where the railing corridor (parapet right 0.35-1.5 m + facade left,
width 2.4-3.3 m) is accepted (v11: 4 scans in a row, v12: 3 of the last 5), and which ramp opening gets CONFIRMED (3 of 5 within 0.5 m) with the steering it would
command (v11: up to +-50 deg -> pivot; v12: -12..+25 deg, 2 deg per scan, opening centre -2.5..+1.5 m, not beyond the parapet). Walkway frame = median wall direction."""
import sys, math, statistics as st; sys.path.insert(0, '.')
import decode_mission_log as D, fwmirror as F
from replay_l4_gap import find_gap
RUNS = {'campus_11 (v11, turned 50 deg)': 'campus_11/mis_001 (6).bin', 'campus_10.3': 'campus_10.3/mis_001 (5).bin', 'c102_4 (hit the steps)': 'campus_10.2/mis_001 (4).bin',
        'c102_3 (reached I)': 'campus_10.2/mis_001 (3).bin', 'c102_2 (reached I)': 'campus_10.2/mis_001 (2).bin', 'campus_9': 'campus_9/mis_001.bin'}
def w180(a): return (a + 180) % 360 - 180
for name, f in RUNS.items():
    R = D.decode('../../Log_run/' + f); l4 = [r for r in R if r['leg'] == 4]
    wl = [r['leg_dist_m'] for r in l4 if r['event'] == 'wall_lost']
    if not wl: print(f'{name:30s} no wall end'); continue
    seg = [w180(r['yaw'] + r['wall_ang_deg']) for r in l4 if 2 < r['leg_dist_m'] < wl[0] - 0.05 and r['wall_d_m'] > 0]
    l4dir = st.median(seg[-48:]) if seg else l4[0]['yaw']
    res = {}
    for ver in ('v11', 'v12'):
        bits = []; corr = None; buf = []; conf = None; last = -9
        for r in l4:
            if r['leg_dist_m'] < wl[0] or r['t_s'] - last < 0.25: continue
            last = r['t_s']; L = [r['lidar_%03d' % (i * 4)] for i in range(90)]; rot = w180(l4dir - r['yaw'])
            x = F.row_fit(L, 90, 40, 0.3, 1.8, 0.08, 4, 0.8, 0.4); rail = x is not None and x[1] > 0 and abs(w180(r['yaw'] - x[0] - l4dir)) <= 15
            roff = x[1] if rail else 99.0
            left = None
            for i in range(62, 73):
                cm = L[i]
                if not cm: continue
                a = math.radians(i * 4 + 2); sd = -cm / 100 * math.sin(a); fw = cm / 100 * math.cos(a)
                if abs(fw) <= 0.6 and 1.4 <= sd <= 2.6: left = sd
            ok = rail and 0.35 <= roff <= (1.5 if ver == 'v12' else 1.3) and left is not None and 2.4 <= left + roff <= 3.3
            bits = (bits + [ok])[-5 if ver == 'v12' else -4:]
            if corr is None and ((ver == 'v12' and sum(bits) >= 3) or (ver == 'v11' and len(bits) == 4 and all(bits))): corr = round(r['leg_dist_m'], 1)
            if conf is None:
                g = find_gap(L, rot=rot, rail_off=(roff if ver == 'v12' else 99.0), v12=(ver == 'v12'))
                if g: buf = (buf + [g])[-5:]
                if len(buf) >= 3:
                    m = sorted(q[1] for q in buf)[len(buf) // 2]; good = [q for q in buf if abs(q[1] - m) <= 0.5]
                    if len(good) >= 3:
                        c = good[-1]; lft = math.degrees(math.atan2(-c[1], max(c[0] - 1.0, 0.8)))
                        steer = max(-50, min(50, lft)) if ver == 'v11' else max(-12, min(25, lft))
                        conf = (round(r['leg_dist_m'], 1), round(c[1], 2), round(steer, 0))
        res[ver] = (corr, conf)
    print(f"{name:30s} wall end {wl[0]:5.1f} | v11: corridor {res['v11'][0]}, opening (at m, centre m, steer deg + = left) {res['v11'][1]} | v12: corridor {res['v12'][0]}, opening {res['v12'][1]}")
