#!/usr/bin/env python3
"""sim_loop_timing.py - mirror of the FIRMWARE LOOP TIMING of the open-sea end (RouteMission::update, case L2_GO): the control loop runs every ~20 ms,
the reference refresh that rebuilds _tgt runs every 250 ms (_refMs), the end-board block updates _ebOff every 250 ms (_aimMs). The python sims step once per
250 ms scan, so they could not see a per-tick mistake. campus_11 (v11): `if (_ebOn) _tgt = wrap180(_tgt - _ebOff)` sat outside both timers -> the 12 deg offset
was subtracted ~12 times per refresh -> target -98 -> +102 deg in one record, the rover turned 18 deg right.
  python3 sim_loop_timing.py          # v11 (per tick) versus v12 (once, inside the refresh)"""
import math

def w180(a): return (a + 180) % 360 - 180

def run(version, tick_ms=20, secs=12.0, board0=1.74, target=1.2, lim=12.0):
    yaw = -94.0; base = -94.0; tgt = base; eb_off = 0.0; eb_on = False; n_ok = 0; ref_ms = aim_ms = -999; t_ms = 0; x_lat = 0.0
    max_jump = 0.0; prev_logged = tgt; worst_err = 0.0; log = []
    while t_ms < secs * 1000:
        # --- reference refresh (every 250 ms): rebuilds the target from the path direction
        if t_ms - ref_ms >= 250:
            ref_ms = t_ms
            tgt = w180(base - (eb_off if version == 'v12' else 0.0))
        # --- end-board block (every 250 ms): board distance depends on the sideways position (+ = toward the board)
        if t_ms - aim_ms >= 250:
            aim_ms = t_ms; dist = board0 - x_lat
            if not eb_on:
                n_ok += 1
                if n_ok >= 4: eb_on = True
            else:
                want = max(-lim, min(lim, 10.0 * (dist - target))); eb_off += max(-1.0, min(1.0, want - eb_off))
        if version == 'v11' and eb_on: tgt = w180(tgt - eb_off)            # THE BUG: every tick
        # --- heading hold (DriveStraight-like) and motion at 0.22 m/s
        e = w180(tgt - yaw); yaw += max(-0.6, min(0.6, 0.8 * e * tick_ms / 1000.0 * 5)); worst_err = max(worst_err, abs(e))
        x_lat += 0.22 * math.sin(math.radians(-(yaw - base))) * tick_ms / 1000.0
        if t_ms % 250 == 0:                                                   # what the log records (250 ms)
            max_jump = max(max_jump, abs(w180(tgt - prev_logged))); prev_logged = tgt; log.append((t_ms / 1000, tgt, yaw, x_lat))
        t_ms += tick_ms
    return dict(max_jump=max_jump, worst_err=worst_err, yaw_end=yaw, x_end=x_lat, log=log)

if __name__ == '__main__':
    print('end board seen at 1.74 m (campus_11), target 1.2 m, 12 s, control tick 20 ms')
    for v in ('v11', 'v12'):
        r = run(v)
        print(f"  {v}: biggest target jump between two log records {r['max_jump']:6.1f} deg | worst heading error {r['worst_err']:6.1f} deg | heading at the end {r['yaw_end']:7.1f} (start -94) | moved toward the board {r['x_end']:.2f} m")
    for tick in (10, 20, 40):
        r = run('v12', tick_ms=tick); print(f"  v12 with a {tick} ms tick: jump {r['max_jump']:.1f} deg, worst error {r['worst_err']:.1f} deg, moved {r['x_end']:.2f} m")
