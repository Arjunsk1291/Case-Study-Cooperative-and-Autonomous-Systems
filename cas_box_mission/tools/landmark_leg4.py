#!/usr/bin/env python3
"""landmark_leg4.py - LANDMARK TABLE of the leg-4 walkway to the KARL glass, from the real logs (writes landmark_leg4_db.json).
Along-track distance = metres since the left wall ENDS (the T3 building corner). Per 2 m bin and log: LEFT line (benches, planter edge, building base with the black
handrails, facade) and RIGHT line (parapet with railing of the ramp) offsets, plus: corridor accepted (v12 rule), ramp opening confirmed, KARL entrance (glass).
Video (campus_11 test_run.MOV, video = log - 23 s): bench + grass bed left (log 523 s), building base with black handrails ahead-left (543-569 s), drain grate across
the path (flat, LiDAR-invisible, 572-581 s), the ramp corridor between facade (left) and concrete parapet with railing (right). Route video navigation.path.mp4 116-147 s:
tactile strip, stepped curb right, red benches + planter left, Gebaeude H (gray cube) with its sign at the end."""
import sys, json, math, statistics as st; sys.path.insert(0, '.')
import decode_mission_log as D, fwmirror as F
RUNS = {'campus_11': 'campus_11/mis_001 (6).bin', 'campus_10.3': 'campus_10.3/mis_001 (5).bin', 'c102_4': 'campus_10.2/mis_001 (4).bin', 'c102_3': 'campus_10.2/mis_001 (3).bin',
        'c102_2': 'campus_10.2/mis_001 (2).bin', 'campus_9': 'campus_9/mis_001.bin'}
db = {'note': __doc__.strip().split('\n')[0], 'bins_m': 2.0, 'runs': {}}
for n, f in RUNS.items():
    R = D.decode('../../Log_run/' + f); l4 = [r for r in R if r['leg'] == 4]; wl = [r['leg_dist_m'] for r in l4 if r['event'] == 'wall_lost']
    if not wl: continue
    w0 = wl[0]; bins = {}
    for r in l4:
        d = r['leg_dist_m'] - w0
        if d < 0 or d > 40: continue
        L = [r['lidar_%03d' % (i * 4)] for i in range(90)]; b = int(d // 2) * 2
        x = F.row_fit(L, 90, 45, 0.3, 4.5, 0.12, 4, 0.8, 0.4); y = F.row_fit(L, 270, 45, 0.4, 4.5, 0.12, 4, 0.8, 0.4)
        e = bins.setdefault(b, {'n': 0, 'R': [], 'L': []}); e['n'] += 1
        if x and x[1] > 0: e['R'].append(x[1])
        if y and y[1] < 0: e['L'].append(-y[1])
    ev = {k: round(r['leg_dist_m'] - w0, 1) for r in l4 for k in ('railing', 'at_block_I', 'ramp_opening', 'ramp') if r['event'] == k}
    db['runs'][n] = {'events_since_wall_end': ev, 'bins': {str(b): {'scans': e['n'], 'right_m': round(st.median(e['R']), 2) if len(e['R']) >= 0.3 * e['n'] else None,
                                                                    'left_m': round(st.median(e['L']), 2) if len(e['L']) >= 0.3 * e['n'] else None} for b, e in sorted(bins.items())}}
json.dump(db, open('landmark_leg4_db.json', 'w'), indent=1)
print('since the wall end (m): right line (parapet) / left line, only bins where the line is seen in >= 30 % of the scans')
for n, v in db['runs'].items():
    cells = [f"{b}:{('R%.1f' % c['right_m']) if c['right_m'] else '  -  '}/{('L%.1f' % c['left_m']) if c['left_m'] else '  -  '}" for b, c in v['bins'].items() if int(b) <= 26]
    print(f"{n:12s} events {v['events_since_wall_end']}\n   " + ' '.join(cells))
