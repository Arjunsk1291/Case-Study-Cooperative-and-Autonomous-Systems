#!/usr/bin/env python3
"""replay_sea_gate.py - real Open Sea scans of the good runs through fwmirror.OpenSea: old gate (4.0-9.5 m) vs v10.1 gate (+-1.0 m)."""
import sys, glob; sys.path.insert(0, '.')
import decode_mission_log as D, fwmirror as F
RUNS = {'campus_2': 'campus_2/mis_001 3.bin', 'campus_3': 'campus_3/mis_001 4.bin', 'campus_5': 'campus5/mis_001 8.bin', 'campus_6': 'campus_6/mis_001 14 (1).bin',
        'campus_9': 'campus_9/mis_001.bin', 'log8': 'log8/mis_001 17.bin'}
old = dict(F.OpenSea.V7, row_min=4.0, row_max=9.5); new = F.OpenSea.V7  # gate 5.15-8.15
for name, f in RUNS.items():
    R = D.decode('../../Log_run/' + f); l2 = [r for r in R if r['leg'] == 2 and r['state'] == 'l2_go']
    if len(l2) < 50: print(name, 'no leg-2 data'); continue
    h0 = sorted(r['yaw'] for r in l2[:30])[15]
    out = {}
    for tag, P in (('old', old), ('new', new)):
        C = F.OpenSea(h0, **dict(P, leg2=38.0)); tg = []; typ = []; last = -9
        for r in l2:
            if r['t_s'] - last < 0.25: continue
            dt = r['t_s'] - last if last > 0 else 0.25; last = r['t_s']
            L = [r['lidar_%03d' % (i * 4)] for i in range(90)]
            tg.append(C.step(L, r['yaw'], r['leg_dist_m'], dt)); typ.append(C.type)
        out[tag] = (tg, typ)
    n = len(out['old'][0]); rows = [k for k in range(n) if out['old'][1][k] == 4]; rows2 = [k for k in range(n) if out['new'][1][k] == 4]
    dmax = max(abs(F.w180(a - b)) for a, b in zip(out['old'][0], out['new'][0]))
    diff = sum(1 for a, b in zip(out['old'][0], out['new'][0]) if abs(F.w180(a - b)) > 0.05)
    print(f'{name:9s} scans {n:4d} | row used old {len(rows):3d} new {len(rows2):3d} | target differs (>0.05 deg) in {diff} scans, max {dmax:.2f} deg | final target old {out["old"][0][-1]:.1f} new {out["new"][0][-1]:.1f}')
