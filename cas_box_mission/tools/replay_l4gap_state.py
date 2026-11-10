#!/usr/bin/env python3
"""replay_l4gap_state.py - faithful replay of RouteMission::_l4Gap's STATE on real scans (buffer of 5, 3 within 0.5 m confirm, ymin 2.5 -> 0.8 once confirmed, age 8).
Walkway frame from the logged target (the firmware's _l4Dir on the walkway = the target before the gap logic). v11 vs v12 (centre window, parapet rule, tracking
gate |c - confirmed| <= 0.6 m, steering -12..+25 deg). Prints every change of the confirmed opening and the steering it commands."""
import sys, math; sys.path.insert(0, '.')
import decode_mission_log as D, fwmirror as F
from replay_l4_gap import find_gap
def w180(a): return (a + 180) % 360 - 180
f = sys.argv[1] if len(sys.argv) > 1 else '../../Log_run/campus_11/mis_001 (6).bin'
R = D.decode(f); l4 = [r for r in R if r['leg'] == 4]; ph = [r['leg_dist_m'] for r in l4 if r['event'] == 'leg4_walkway']
l4dir = [r['target_deg'] for r in l4 if r['leg_dist_m'] >= ph[0] + 1][0]
for ver in ('v11', 'v12'):
    buf = []; have = False; conf = None; age = 0; last = -9; out = []
    for r in l4:
        if r['leg_dist_m'] < ph[0] or r['t_s'] - last < 0.25 or r['state'] != 'l4_follow': continue
        last = r['t_s']; L = [r['lidar_%03d' % (i * 4)] for i in range(90)]; rot = w180(l4dir - r['yaw'])
        x = F.row_fit(L, 90, 40, 0.3, 1.8, 0.08, 4, 0.8, 0.4); roff = x[1] if (x and x[1] > 0 and abs(w180(r['yaw'] - x[0] - l4dir)) <= 15) else 99.0
        g = find_gap(L, ymin=0.8 if have else 2.5, rot=rot, rail_off=(roff if ver == 'v12' else 99.0), v12=(ver == 'v12'))
        if g and ver == 'v12' and have and abs(g[1] - conf[1]) > 0.6: g = None          # v12 tracking gate
        if g: buf = (buf + [g])[-5:]; age = 0
        else: age += 1
        if age > 8: buf = []; have = False
        if len(buf) >= 3:
            m = sorted(q[1] for q in buf)[len(buf) // 2]; good = [q for q in buf if abs(q[1] - m) <= 0.5]
            if len(good) >= 3:
                new = good[-1]
                if conf is None or abs(new[1] - conf[1]) > 0.3 or not have: 
                    lft = math.degrees(math.atan2(-new[1], max(new[0] - 1.0, 0.8))); st = max(-50, min(50, lft)) if ver == 'v11' else max(-12, min(25, lft))
                    out.append((round(r['t_s'], 1), round(r['leg_dist_m'], 1), round(new[0], 1), round(new[1], 2), round(st)))
                have = True; conf = new
    print(f"{ver}: confirmed opening changes (t s, leg m, line ahead m, centre m, steering deg + = left):", out[:8])
