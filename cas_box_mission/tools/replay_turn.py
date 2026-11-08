#!/usr/bin/env python3
"""replay_turn.py - replay the v9 leg-1 turn decision (fwmirror.TurnV9) on recorded runs.
Prints, per run: where the rover really turned, the structure lock and where v9 would turn.
  python3 replay_turn.py
"""
import sys, os
sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
import decode_mission_log as D, fwmirror as F
B = os.path.join(os.path.dirname(os.path.abspath(__file__)), '../../Log_run/')
RUNS = {'campus2': 'campus_2/mis_002.bin', 'campus3': 'campus_3/mis_001 4.bin', 'campus5': 'campus5/mis_001 8.bin',
        'campus6': 'campus_6/mis_001 14 (1).bin', 'test1': 'campus_6/New_log/test1_failed in open sea.bin',
        'test2': 'campus_6/New_log/test2_successfull till the turn.bin', 'log8': 'log8/mis_001 17.bin', 'campus9': 'campus_9/mis_001.bin'}


def leg1(name):
    R = D.decode(B + RUNS[name]); l1 = [r for r in R if r['leg'] == 1 and r['state'] == 'leg1_follow']
    T = [r for r in R if r['leg'] == 1 and r['event'] in ('turn_pillar', 't1_ask', 'op_turn')][0]['leg_dist_m']
    h0 = sorted(r['yaw'] for r in l1[:40])[20]
    # gap anchor: left wall found again after >= 3 m without it (as the firmware sees it: wall_lost / wall_found)
    anchor = None; lost = None
    for r in R:
        if r['leg'] != 1 or r['state'] != 'leg1_follow': continue
        if r['event'] == 'wall_lost': lost = r['leg_dist_m']
        if r['event'] == 'wall_found' and lost is not None and r['leg_dist_m'] - lost >= 3.0 and anchor is None: anchor = r['leg_dist_m']; lost = None
    return [([r['lidar_%03d' % (i * 4)] for i in range(90)], r['leg_dist_m'], r['yaw'] - h0, anchor if anchor is not None and r['leg_dist_m'] >= anchor else None) for r in l1], T


def run(scans, **kw):
    tv = F.TurnV9(**kw); res = None
    for L, d, rel, anc in scans:
        if anc is not None and tv.anchor is None: tv.set_anchor(anc)
        res = tv.step(L, d, rel)
        if res: break
    pred = res[1] if res else (round(tv.lock + tv.sp, 2) if tv.lock is not None else None)
    return tv, res, pred


if __name__ == '__main__':
    for n in RUNS:
        sc, T = leg1(n); tv, res, pred = run(sc)
        print(f"{n:8s} gap end {tv.anchor} real turn {T:5.2f} | lock {tv.lock} | v9 turn {res if res else ('(log ends) would be ' + str(pred))} | "
              f"sightings {[round(a, 2) for a, _ in tv.sight]}")
