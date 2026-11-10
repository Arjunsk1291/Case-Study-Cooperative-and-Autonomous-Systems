#!/usr/bin/env python3
"""replay_end_board.py - the v10.3 end-board rule on the REAL Open Sea scans of every log: when would it engage, which board distance, and what steering (deg) would it
command (10 deg/m to ebTarget 1.2 m, limit LIM, ramp 1 deg per scan)? Window = board distance accepted. Also lists lines found in the gate region that are NOT the board."""
import sys; sys.path.insert(0, '.')
import decode_mission_log as D, fwmirror as F
RUNS = {'campus_10.3 (rover 7)': 'campus_10.3/mis_001 (5).bin', 'campus_6': 'campus_6/mis_001 14 (1).bin', 'campus_9': 'campus_9/mis_001.bin', 'log8': 'log8/mis_001 17.bin',
        'campus_3': 'campus_3/mis_001 4.bin', 'c102_3': 'campus_10.2/mis_001 (3).bin', 'c102_4': 'campus_10.2/mis_001 (4).bin', 'c102_2': 'campus_10.2/mis_001 (2).bin', 'campus_5': 'campus5/mis_001 8.bin'}
LIM, HI, TARGET = (float(sys.argv[1]), float(sys.argv[2]), 1.2) if len(sys.argv) > 2 else (12.0, 3.0, 1.2)
print(f'limit {LIM} deg, window 0.4-{HI} m, target {TARGET} m')
for n, f in RUNS.items():
    R = D.decode('../../Log_run/' + f); l2 = [r for r in R if r['leg'] == 2 and r['state'] == 'l2_go']
    if not l2: continue
    end = max(r['leg_dist_m'] for r in l2); gate = 0.85 * 36.0
    on = False; cnt = 0; edir = 0; first = None; off = 0.0; offs = []; fits = []
    for r in l2:
        if r['leg_dist_m'] < gate: continue
        w = F.wallfit([r['lidar_%03d' % (i * 4)] for i in range(90)], False)
        ok = w and w[2] <= 0.05 and 0.4 <= w[0] <= HI and abs(w[1]) <= 12.0
        if ok:
            fits.append(round(w[0], 2))
            if not on:
                cnt = cnt + 1 if (cnt and abs(w[1] - edir) < 5.0) else 1; edir = w[1]
                if cnt >= 4: on = True; first = (round(r['leg_dist_m'], 1), round(w[0], 2))
            else:
                off += max(-1.0, min(1.0, max(-LIM, min(LIM, 10.0 * (w[0] - TARGET))) - off)); offs.append(round(off, 1))
        elif not on: cnt = 0
    print(f"{n:22s} end {end:5.1f} m | engaged at {first} | distances seen {sorted(set(fits))[:6]}{'...' if len(set(fits))>6 else ''} | steering (deg, + = right): last {offs[-1] if offs else None}, max {max(offs, key=abs) if offs else None}")
