#!/usr/bin/env python3
"""sweep_start.py - the v10 turn must not depend on where along B the rover was started.
Every log's leg 1 is shifted by -8..+8 m (odometry offset, gap anchor shifted with it); the turn position relative to
the same sign pillar must stay the same (<= 0.3 m). python3 sweep_start.py"""
import sys, os
sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
import replay_turn as RT, fwmirror as F
rows = []
for n in RT.RUNS:
    sc, T = RT.leg1(n)
    base = None; out = []
    for off in (-8, -4, -2, 0, 2, 4, 8):
        tv = F.TurnV9(); res = None
        for L, d, rel, anc in sc:
            dd = d + off
            if anc is not None and tv.anchor is None: tv.set_anchor(anc + off)
            res = tv.step(L, dd, rel)
            if res: break
        pred = res[1] - off if res else (tv.lock - off + tv.sp if tv.lock is not None else None)
        out.append(pred)
        if off == 0: base = pred
    if base is None: print(f"{n:8s} no structure inside the log (turn would come later) - skipped"); continue
    sp = max(abs(o - base) for o in out if o is not None) if all(o is not None for o in out) else 99
    print(f"{n:8s} turn (log frame) for shifts -8..+8: {[None if o is None else round(o,2) for o in out]}  max deviation {sp:.2f} m  {'OK' if sp <= 0.3 else 'CHECK'}")
