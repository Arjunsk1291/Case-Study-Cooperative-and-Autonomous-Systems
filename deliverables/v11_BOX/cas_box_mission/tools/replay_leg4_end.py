#!/usr/bin/env python3
"""replay_leg4_end.py - v10 leg-4 end on real logs: railing corridor (RouteMission::_l4Walk, phase 1->2) and KARL entrance.
Approximation: line direction checked against the rover heading (+-15 deg) instead of _l4Dir; scans every 250 ms."""
import sys, math; sys.path.insert(0, '.')
import decode_mission_log as D, fwmirror as F
RUNS = {'campus_9 (v9, autonomous)': 'campus_9/mis_001.bin', 'log8 (manual climb drive)': 'log8/mis_001 17.bin'}
for name, f in RUNS.items():
    R = D.decode('../../Log_run/' + f); l4 = [r for r in R if r['leg'] == 4]
    railN = 0; corr = None; entN = 0; ent = None; last = -9; first_rail = None
    for r in l4:
        if r['t_s'] - last < 0.25 or r['leg_dist_m'] < 10: continue
        last = r['t_s']; d = r['leg_dist_m']; L = [r['lidar_%03d' % (i * 4)] for i in range(90)]
        if corr is None:
            x = F.row_fit(L, 90, 40, 0.3, 1.8, 0.08, 4, 0.8, 0.4)
            rail = x is not None and x[1] > 0 and abs((x[0] + 90) % 180 - 90) <= 15
            roff = x[1] if rail else 0
            leftD = None
            for i in range(62, 73):
                cm = L[i]
                if not cm: continue
                a = math.radians(i * 4 + 2); dd = cm / 100; sd = -dd * math.sin(a); fw = dd * math.cos(a)
                if abs(fw) <= 0.6 and 1.4 <= sd <= 2.6: leftD = sd
            ok = rail and 0.35 <= roff <= 1.3 and leftD is not None and 2.4 <= leftD + roff <= 3.3
            railN = railN + 1 if ok else 0
            if ok and first_rail is None: first_rail = round(d, 1)
            if railN >= 4: corr = round(d, 1)
        elif ent is None and d - corr > 3.0:
            hit = miss = 0
            for i in range(60, 75):
                cm = L[i]
                if not cm: miss += 1; continue
                if -cm / 100 * math.sin(math.radians(i * 4 + 2)) > 0.5: hit += 1
            entN = entN + 1 if (miss >= 3 and hit >= 4) else 0
            if entN >= 3: ent = round(d, 1)
    print(f'{name:28s} leg-4 length {l4[-1]["leg_dist_m"]:.1f} m | corridor at {corr} m | KARL entrance at {ent} m | stop at {round(ent + 1.5, 1) if ent else None} m')
