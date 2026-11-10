#!/usr/bin/env python3
"""replay_leg4_dir.py - leg-4 walkway heading after the wall ends. v10.3 used the LAST smoothed wall direction (_wallDir at the wall end, the wall corner and the
edge push distort it: campus_10.3 -24.9 deg while the wall ran at -36.6 deg = 11.7 deg toward the mud). New: median of the last N (48 = 12 s) wall-direction samples.
Prints per log: the old value (target after the wall end), the new value, and the difference to the median over the whole wall stretch (the best estimate)."""
import sys, statistics as st; sys.path.insert(0, '.')
import decode_mission_log as D
RUNS = {'campus_10.3 (rover 7)': 'campus_10.3/mis_001 (5).bin', 'c102_3': 'campus_10.2/mis_001 (3).bin', 'c102_4': 'campus_10.2/mis_001 (4).bin', 'c102_2': 'campus_10.2/mis_001 (2).bin', 'campus_9': 'campus_9/mis_001.bin'}
def w180(a): return (a + 180) % 360 - 180
for n, f in RUNS.items():
    R = D.decode('../../Log_run/' + f); l4 = [r for r in R if r['leg'] == 4 and r['state'] == 'l4_follow']
    wl = [r['leg_dist_m'] for r in l4 if r['event'] == 'wall_lost']
    if not wl: print(n, 'no wall end'); continue
    end = wl[0]; seg = [r for r in l4 if 2 < r['leg_dist_m'] < end - 0.05 and r['wall_d_m'] > 0]
    dirs = [w180(r['yaw'] + r['wall_ang_deg']) for r in seg]      # = the smoothed _wallDir at that scan
    if len(dirs) < 12: print(n, 'few samples'); continue
    whole = st.median(dirs[:max(8, len(dirs) - 8)]); last = dirs[-1]; med48 = st.median(dirs[-48:]); med24 = st.median(dirs[-24:])
    after = [r['target_deg'] for r in l4 if end + 1 < r['leg_dist_m'] < end + 6]; old = st.median(after) if after else float('nan')
    print(f"{n:22s} wall ends {end:5.1f} m | old (v10.3) {old:7.1f} | last smoothed {last:7.1f} | median 24 {med24:7.1f} | median 48 {med48:7.1f} | whole wall {whole:7.1f} | old error {w180(old - whole):+5.1f} deg, new error {w180(med48 - whole):+5.1f} deg")
