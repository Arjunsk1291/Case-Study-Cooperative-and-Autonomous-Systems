#!/usr/bin/env python3
"""sim_turn_curve.py - STUDY ONLY (nothing is implemented in the firmware): smooth curved 90 deg turns versus today's full-power pivot, for rover 7.
ROVER MODEL (fitted to the logs; the yaw rates were measured in 6 logs, 250 ms records):
  command ~ m/s at cruise (odometry matches landmarks within +-5 % over 40 m); wheel speed follows the (acceleration-limited) command with a motor lag 0.12 s;
  yaw rate = K * (v_right - v_left)   (rad/s); K = 0.75 rad/s per unit = 43 deg/s for a full pivot (campus_6 / campus_10.3 median 35-43 deg/s on tiles)
  ground: the WHEELS LOCK, not the motors (code comment): a pivot (wheels in opposite directions) stalls with p_pivot, a swing (one wheel only) with p_swing, an arc / curve
  (both wheels forward) with p_arc: tile 0.15 / 0.2 / 0.02, gravel 0.4 / 0.4 / 0.05, cobble 0.72 / 0.55 / 0.06 (logs: cobble pivot rate < 3 deg/s in 66-76 % of the records, swing 29-81 %,
  arc 5 %). A lock = yaw rate 0 until the motion type changes (the tactic cycle: pivot, swing, shift); drawn once per attempt.
  footprint 0.30 x 0.26 m (+ 0.15 m margin), track factor inside K. Heading from the gyro: noise 0.05 deg, heading hold PD (kP 0.045, kD 0.0015 like DriveStraight).
GEARBOX (conservative, no measured limit): peak wheel-command change <= 1.0 per s (today's pivot: 5.0/s), no direct reversal (through 0), stall at |cmd| >= 0.4 for <= 2 s.
  python3 sim_turn_curve.py --calib | --sites | --accel"""
import sys, math, random
import numpy as np
K_RAD = 0.75           # rad/s per unit of (vr - vl)
LAG = 0.12
DEAD = 0.08            # motor deadband: a wheel command below this does not turn the wheel (ramp model: flat minimum 0.08)
P = {'tile': (0.15, 0.2, 0.02), 'gravel': (0.4, 0.4, 0.05), 'cobble': (0.65, 0.55, 0.06)}      # stall probability (per 1 s of attempt): pivot, swing, arc
HALF_L, HALF_W = 0.15, 0.13
LIM_ACC, LIM_STALL = 1.0, 2.0

class Rover:
    def __init__(s, x=0.0, y=0.0, phi=math.pi / 2, ground='tile', seed=1, kscale=1.0, acc=5.0):
        s.x, s.y, s.phi = x, y, phi; s.vl = s.vr = 0.0; s.cl = s.cr = 0.0; s.rng = random.Random(seed); s.g = ground; s.k = K_RAD * kscale; s.acc = acc
        s.stall_t = 0.0; s.locked = False; s.lockscale = 1.0; s.kind_prev = 'none'; s.peak = 0.0; s.rev = 0; s.stall_hi = 0.0; s.t = 0.0; s.energy = 0.0; s.hist = []; s.yawrate = 0.0; s.lastc = (0.0, 0.0)
    def step(s, tl, tr, dt=0.02):
        for name, tgt in (('cl', tl), ('cr', tr)):                         # per-wheel acceleration limiter (GentleDrive turn mode)
            c = getattr(s, name); d = max(-s.acc * dt, min(s.acc * dt, tgt - c)); n = c + d
            s.peak = max(s.peak, abs(d) / dt)
            if c * n < -1e-9 and abs(d) > 0.2 * dt * s.acc * 0.999 and abs(c) > 0.05 and abs(n) > 0.05: s.rev += 1       # a sign flip without resting at 0
            setattr(s, name, n)
        a = dt / (LAG + dt); el = s.cl if abs(s.cl) >= DEAD else 0.0; er = s.cr if abs(s.cr) >= DEAD else 0.0; s.vl += a * (el - s.vl); s.vr += a * (er - s.vr)
        kind = 'pivot' if (s.cl * s.cr < 0 and min(abs(s.cl), abs(s.cr)) >= DEAD) else ('swing' if (abs(s.cl) < DEAD) != (abs(s.cr) < DEAD) else ('arc' if abs(s.cl - s.cr) > 0.03 and (s.cl > 0 and s.cr > 0) else 'none'))
        if kind != s.kind_prev:                                                  # a new motion attempt: the wheels may lock on the ground (until the motion type changes)
            s.locked = kind != 'none' and s.rng.random() < min(0.95, P[s.g][('pivot', 'swing', 'arc').index(kind)] * s.lockscale); s.kind_prev = kind
        stalled = s.locked and kind != 'none'
        if stalled and max(abs(s.cl), abs(s.cr)) >= 0.4: s.stall_hi += dt
        w = 0.0 if stalled else s.k * (s.vr - s.vl); v = 0.5 * (s.vl + s.vr)
        if stalled and kind == 'pivot': v = 0.0
        s.yawrate = w; s.phi += w * dt; s.x += v * math.cos(s.phi) * dt; s.y += v * math.sin(s.phi) * dt; s.t += dt; s.energy += (abs(s.cl) + abs(s.cr)) * dt
        s.hist.append((s.t, s.x, s.y, s.phi, s.cl, s.cr, v, w))
    def corners(s):
        c, sn = math.cos(s.phi), math.sin(s.phi); f = np.array([c, sn]); r = np.array([sn, -c]); p = np.array([s.x, s.y])
        return [p + sx * HALF_L * f + sy * HALF_W * r for sx in (1, -1) for sy in (1, -1)]
def clearance(pts, x, y, phi, margin=0.15):
    """True if no obstacle point is inside the footprint expanded by margin"""
    if pts is None or len(pts) == 0: return True
    c, sn = math.cos(phi), math.sin(phi); d = pts - np.array([x, y]); u = d[:, 0] * c + d[:, 1] * sn; v = d[:, 0] * sn - d[:, 1] * c
    return not np.any((np.abs(u) <= HALF_L + margin) & (np.abs(v) <= HALF_W + margin))
def w180(a): return (a + math.pi) % (2 * math.pi) - math.pi

def _pivot_loop(r, tgt, direction, pts, rng, tmax, soft=False):
    """today's pivot with the tactic cycle: pivot -> swing -> shift (0.3 m) -> pivot ...; stall = < 3 deg in 2 s (pivot) / 1.5 s (swing). release 3-7 deg early."""
    ok = True; tac = 0; tac_t = r.t; ph0 = r.phi; last_prog = r.t; amp = 0.35 if soft else 0.5
    while r.t < tmax:
        err = w180(tgt - (r.phi + rng.gauss(0, math.radians(0.05)))) * direction
        if err <= math.radians(min(7, max(3, abs(r.yawrate) * 0.15 * 57.3))): break
        if tac == 0: tl, tr = (-amp, amp) if direction > 0 else (amp, -amp)
        elif tac == 1: tl, tr = (0.0, amp) if direction > 0 else (amp, 0.0)
        else: tl, tr = 0.25, 0.25
        r.step(tl, tr)
        if abs(w180(r.phi - ph0)) >= math.radians(3): ph0 = r.phi; last_prog = r.t
        if tac == 2 and r.t - tac_t > 1.2: tac = 0; tac_t = r.t; ph0 = r.phi; last_prog = r.t; [r.step(0, 0) for _ in range(25)]
        elif tac != 2 and r.t - last_prog > (2.0 if tac == 0 else 1.5):
            tac = 1 if tac == 0 else 2; tac_t = r.t; ph0 = r.phi; last_prog = r.t; [r.step(0, 0) for _ in range(25)]
        if not clearance(pts, r.x, r.y, r.phi): ok = False
    return ok

def turn(kind, v, R, ground, seed, pts, direction=-1, acc_lim=1.0, angle=math.pi / 2, kscale=1.0, tmax=30.0, ramp_up=2.0):
    """kind: 'pivot' (today: wheel step 5.0/s, amplitude 0.5) | 'pivot_soft' (amplitude 0.35, wheel change <= acc_lim per s; locks more often: teaching logs 13 % worked with a slow ramp vs 36 %
    with a quick step) | 'arc' (constant radius R) | 'scurve' (yaw-rate profile ramped up/down). Curves: no progress for 2 s -> release (0.6 s), retry; after 2 retries the pivot cycle
    finishes the turn. direction -1 = right (clockwise), +1 = left. Returns a dict (hist = the whole trajectory for the videos)."""
    soft = kind == 'pivot_soft'
    acc = 5.0 if kind == 'pivot' else acc_lim
    r = Rover(ground=ground, seed=seed, kscale=kscale, acc=acc); tgt = r.phi + direction * angle; ok = True; rng = random.Random(seed + 7); fell = False
    if soft or kind == 'pivot':
        r.lockscale = 1.0 + 0.4 * (1.0 - min(acc, 5.0) / 5.0)
        ok = _pivot_loop(r, tgt, direction, pts, rng, tmax, soft=soft)
    else:
        r.lockscale = 1.0; w_ref = v / R; rests = 0; t_prog = 0.0; ph0 = r.phi
        while r.t < tmax:
            err = w180(tgt - (r.phi + rng.gauss(0, math.radians(0.05)))) * direction
            if err <= math.radians(4): break
            wp = w_ref * (min(1.0, r.t / ramp_up) if kind == 'scurve' else 1.0)
            if kind == 'scurve': wp = min(wp, math.sqrt(max(0.0, 2 * err * w_ref / ramp_up)))
            dcmd = direction * (wp / r.k)
            vcmd = min(v, (r.t / 1.5) * v + 0.05)
            tl, tr = vcmd - dcmd / 2, vcmd + dcmd / 2
            if min(tl, tr) < 0: tl, tr = max(tl, 0.0), max(tr, 0.0)
            r.step(tl, tr)
            if abs(w180(r.phi - ph0)) >= math.radians(1.0): ph0 = r.phi; t_prog = r.t
            if r.t - t_prog > 2.0 and r.t > 4.0:                                   # (no check during the start-up ramp) locked: release and retry, then fall back to the pivot cycle
                [r.step(0, 0) for _ in range(30)]; rests += 1; t_prog = r.t; ph0 = r.phi
                if rests > 2: fell = True; break
            if not clearance(pts, r.x, r.y, r.phi): ok = False
        if fell: r.acc = 5.0; ok = _pivot_loop(r, tgt, direction, pts, rng, tmax) and ok
    for _ in range(60): r.step(0.0, 0.0)
    herr = math.degrees(w180(tgt - r.phi)); dist = math.hypot(r.x, r.y)
    return dict(ok=ok and abs(herr) <= 8.0 and r.t < tmax - 0.5, herr=herr, t=r.t, travel=dist, peak=r.peak, rev=r.rev, stall_hi=r.stall_hi, energy=r.energy, hist=r.hist, fell=fell)

def calibrate():
    print('CALIBRATION: simulated yaw rate while turning (deg/s) vs the logs (median of the 250 ms records; fraction < 3 deg/s)')
    def rates(kind, ground):
        out = []
        for sd in range(40):
            r = Rover(ground=ground, seed=sd, acc=5.0); cmd = {'pivot': (0.5, -0.5), 'swing': (0.5, 0.0), 'arc': (0.5, 0.15)}[kind]
            for _ in range(int(4.0 / 0.02)): r.step(*cmd)
            h = r.hist
            for i in range(0, len(h) - 12, 12): out.append(abs(math.degrees(w180(h[i + 12][3] - h[i][3])) / (0.24)))
        out.sort(); return out[len(out) // 2], sum(1 for x in out if x < 3) / len(out)
    LOG = {('pivot', 'tile'): '35-43 deg/s, 4-30 % stalled (campus_6, campus_10.3)', ('pivot', 'cobble'): '0 deg/s median, 66-76 % stalled (c102_2/3/4, campus_9)',
           ('swing', 'cobble'): '0-7 deg/s, 29-81 % stalled', ('arc', 'cobble'): '5.8-16 deg/s (n small), 5 % stalled'}
    for (kind, g), txt in LOG.items():
        m, f = rates(kind, g); print(f"  {kind:5s} {g:7s}: sim median {m:5.1f} deg/s, stalled {100 * f:3.0f} %   | logs: {txt}")

if __name__ == '__main__':
    if '--calib' in sys.argv: calibrate()


# ------------------------------------------------------------------------------------------------ real free space at the 90 deg sites
import os
sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
import decode_mission_log as D
B = os.path.join(os.path.dirname(os.path.abspath(__file__)), '../../Log_run/')
SITES = {   # name: (direction -1 right / +1 left, event, leg of the record, [(label, log)])
    'T1 first right turn (sign pillar)': (-1, 'turn_pillar', 1, [('campus_6', 'campus_6/mis_001 14 (1).bin'), ('campus_9', 'campus_9/mis_001.bin'), ('c102_3', 'campus_10.2/mis_001 (3).bin')]),
    'end-wall right turn (open sea -> leg 3)': (-1, 'end_wall', 2, [('campus_6', 'campus_6/mis_001 14 (1).bin'), ('campus_10.3', 'campus_10.3/mis_001 (5).bin'), ('c102_3', 'campus_10.2/mis_001 (3).bin')]),
    'T3 left turn (arcade corner -> leg 4)': (+1, 't3_turn', 3, [('campus_10.3', 'campus_10.3/mis_001 (5).bin'), ('c102_3', 'campus_10.2/mis_001 (3).bin'), ('c102_4', 'campus_10.2/mis_001 (4).bin')]),
}
def site_points(path, event, leg, back=8):
    """obstacle points (rover frame at the start of the turn: X right, Y forward) from the scan at the turn start + the 8 scans before it (moved by the driven distance)"""
    R = D.decode(B + path); idx = [i for i, r in enumerate(R) if r['event'] == event and r['leg'] == leg]
    if not idx: return None
    i0 = idx[0]; P = []
    for k in range(max(0, i0 - back), i0 + 1):
        r = R[k]; dd = R[i0]['leg_dist_m'] - r['leg_dist_m'] if leg != 2 else 0.0
        for j in range(90):
            cm = r['lidar_%03d' % (j * 4)]
            if not cm or cm < 35 or cm > 500: continue
            a = math.radians(j * 4 + 2); px, py = cm / 100 * math.sin(a), cm / 100 * math.cos(a)
            P.append((px, py - max(0.0, min(1.5, dd))))
    return np.array(P)
def world_pts(P):          # initial frame: rover at (0,0) heading +Y (phi = pi/2); X right, Y forward -> same numbers
    return P

CANDS = [('pivot (today)', 'pivot', 0.5, None), ('pivot soft 1.0/s, 0.35', 'pivot_soft', 0.5, None)] + [(f'{k} R={R} v={v}', k, v, R) for k in ('arc', 'scurve') for R in (0.7, 0.9, 1.2, 1.5) for v in (0.10, 0.15, 0.22, 0.30)]

def run_sites(n=8, quick=False):
    GR = ('tile', 'gravel', 'cobble')
    for site, (direction, ev, leg, logs) in SITES.items():
        maps = [(lab, site_points(p, ev, leg)) for lab, p in logs]; maps = [(l, m) for l, m in maps if m is not None and len(m)]
        print(f'\n=== {site}: {len(maps)} real maps ({", ".join(l for l, _ in maps)}), {n} seeds per cell')
        print(f"{'candidate':26s} | clear (all maps) | success tile / gravel / cobble | time s (tile) | peak wheel acc /s | reversals | stall>=0.4 s (cobble) | travel m")
        for name, kind, v, R in CANDS:
            if quick and kind not in ('pivot', 'pivot_soft') and (R not in (0.7, 1.2) or v not in (0.15, 0.22)): continue
            res = {g: [] for g in GR}; clr = 0; tot = 0; peaks = []; revs = []; stl = []; tt = []; trav = []
            for lab, pts in maps:
                for g in GR:
                    for sd in range(n):
                        o = turn(kind, v, R if R else 0.0, g, 100 + sd, pts, direction, acc_lim=LIM_ACC)
                        res[g].append(o['ok']); tot += 1; clr += 1 if o['ok'] or o['herr'] is not None and abs(o['herr']) > 8 else 1
                        peaks.append(o['peak']); revs.append(o['rev']); trav.append(o['travel'])
                        if g == 'cobble': stl.append(o['stall_hi'])
                        if g == 'tile': tt.append(o['t'])
            sr = [100 * sum(res[g]) / len(res[g]) for g in GR]
            print(f"{name:26s} |                  | {sr[0]:5.0f} % {sr[1]:5.0f} % {sr[2]:5.0f} %    | {sum(tt)/len(tt):6.1f}        | {max(peaks):5.1f}            | {sum(revs)/len(revs):5.1f}     | {sum(stl)/len(stl):5.1f}                 | {sum(trav)/len(trav):4.2f}", flush=True)
if __name__ == '__main__' and '--sites' in sys.argv: run_sites(n=6 if '--quick' in sys.argv else 8, quick='--quick' in sys.argv)
