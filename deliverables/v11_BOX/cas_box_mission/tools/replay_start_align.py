#!/usr/bin/env python3
"""replay_start_align.py - the v10.3 start-alignment decision (sim_start_align.StartAlign, = RouteMission::_alignStart) on the REAL
scans right after the first turn of every log: what would it have done? 'ok' = untouched (good turn), 'move' = it would line up."""
import sys; sys.path.insert(0, '.')
import decode_mission_log as D, sim_start_align as A
B = '../../Log_run/'
RUNS = {'campus_3': 'campus_3/mis_001 4.bin', 'campus_5': 'campus5/mis_001 8.bin', 'campus_6 (v6 ok)': 'campus_6/mis_001 14 (1).bin',
        'test1 (failed sea)': 'campus_6/New_log/test1_failed in open sea.bin', 'test2': 'campus_6/New_log/test2_successfull till the turn.bin',
        'log8': 'log8/mis_001 17.bin', 'campus_9': 'campus_9/mis_001.bin', 'campus_10 (turned early)': 'campus_10/mis_001 (1).bin',
        'c102_2 (late)': 'campus_10.2/mis_001 (2).bin', 'c102_3 (1 m off row)': 'campus_10.2/mis_001 (3).bin', 'c102_4': 'campus_10.2/mis_001 (4).bin'}
for name, f in RUNS.items():
    R = D.decode(B + f)
    t0 = [r['t_s'] for r in R if r['leg'] == 1 and r['state'] == 't1_settle'] or [r['t_s'] for r in R if r['leg'] == 2 and r['state'] in ('l2_go', 'l2_board')]
    if not t0: print(f'{name:26s} no data'); continue
    a = A.StartAlign(); res = None; last = -9
    for r in R:
        if not t0[0] <= r['t_s'] <= t0[0] + 8 or r['t_s'] - last < 0.25: continue
        last = r['t_s']; m = a.measure([r['lidar_%03d' % (i * 4)] for i in range(90)])
        if m is not None: res = (m, a.decide(m[0])); break
    print(f"{name:26s} " + (f"sign {res[0][0]:.2f} m right (abeam y {res[0][1]:+.2f}) -> {res[1][0]}" + (f", move {res[1][1]:+.2f} m" if res[1][0] == 'move' else '') if res else 'no consistent sign in 8 s -> fallback (lateness) / going on'))
