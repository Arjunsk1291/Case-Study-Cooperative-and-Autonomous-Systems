#!/usr/bin/env python3
"""sim_ramp_climb.py - STUDY ONLY (nothing implemented): the leg-4 RAMP (Block I) and the edges before it: today's fixed speed + push rule versus a GRADUAL speed increase "only if needed".
MODEL (fitted to the logs): steady speed on a slope v = min(command, 3 * (command_eff - c_req)); c_req = 0.08 + 1.84 * sin(pitch) + surface (wet +0.05, cobble edge +0.10);
(log8: 7.8 deg needed pushes of 0.35-0.40, command 0.22 stalled; campus_9: 13.8 deg edge at 0.34 climbed only as a short edge; c102_3: 3.2 deg fine at 0.45);
command_eff = command * battery (1.0 / 0.9 / 0.8). The rover follows with a motor lag of 0.5 s (mass). No wheel encoders: speed ESTIMATE = LiDAR range-rate (70 % of the time, noise 0.03 m/s).
Slow/stuck detection: pitch above the flat value + 2.5 deg and the estimated speed < 40 % of the command for 1.5 s (or the estimate says < 0.02 m/s).
CONTROLLERS: TODAY = 0.30 on the walkway, 0.35 once the ramp is seen, edge assist: after 3 s stuck push 0.40 for 1.5 s (ramped 0.3 s), settle 0.7 s, 3 pushes, then 0.45, then wait 30 s.
GRADUAL = start 0.30; when slow/stuck raise the command by 0.02 per 1 s up to 0.45; once it moves at >= 70 % of the command hold; on the level fall back to 0.30 (0.02 per s); no push, no jump.
Ramp 4.0 m long at the pitch; metrics: climbs, time, peak acceleration (m/s^2), peak wheel-command rate, max command, stuck seconds.
  python3 sim_ramp_climb.py"""
import math, random, sys

def sim(ctrl, pitch, surf, batt, seed, L=4.0, v_cruise=0.30, dt=0.05, tmax=120.0, trace=False):
    rng = random.Random(seed); x = -3.0; v = 0.0; cmd = 0.0; t = 0.0; hist = []; stuck_s = 0.0; peak_a = 0.0; peak_rate = 0.0; max_cmd = 0.0
    creq_extra = {'dry': 0.0, 'wet': 0.05, 'cobble edge': 0.10}[surf]; target = v_cruise; slow_t = 0.0; hold = False; push_t = -1.0; pushes = 0; wait = 0.0; vest = 0.0; stuck_t = 0.0
    while t < tmax and x < L + 1.0:
        on = 0.0 <= x <= L; pit = pitch if on else 0.0
        creq = 0.08 + 1.84 * math.sin(math.radians(pit)) + (creq_extra if on else 0.0)
        # ---- controller (what the box would command)
        flat = 0.0; sensed = rng.random() < 0.7
        if sensed: vest = max(0.0, v + rng.gauss(0, 0.03))
        if ctrl == 'today':
            want = v_cruise if x < 0.0 else 0.35
            if on and vest < 0.02 and (hist and hist[-1][1] < 0.03): stuck_t += dt
            else: stuck_t = 0.0 if vest > 0.05 else stuck_t
            if stuck_t > 3.0 and push_t < 0 and wait <= 0: push_t = t; pushes += 1; stuck_t = 0.0
            if push_t >= 0:
                el = t - push_t
                want = (0.40 if pushes <= 3 else 0.45) * min(1.0, el / 0.3) if el < 1.5 else 0.0
                if el > 2.2: push_t = -1.0
                if pushes > 3 and pushes <= 4 and el > 1.5: pass
            if pushes > 4: wait = 30.0; pushes = 0
            if wait > 0: wait -= dt; want = 0.0
        else:
            if on and pit >= 2.5 and v_cruise > 0 and vest < 0.4 * cmd and cmd > 0.05: slow_t += dt
            else: slow_t = max(0.0, slow_t - dt)
            want = target
            if slow_t > 1.5:
                target = min(0.45, target + 0.02 * dt); want = target                      # +0.02 per second
            if on and vest >= 0.7 * cmd and slow_t < 0.5: hold = True
            if not on and x > L: target = max(v_cruise, target - 0.02 * dt)               # back to cruise on the level
            if x < 0.0: target = v_cruise
        # gentle command ramp (GentleDrive: from standstill linear in 2.5 s; changes <= 0.4/s)
        d = max(-0.4 * dt, min(0.4 * dt, want - cmd)); peak_rate = max(peak_rate, abs(d) / dt); cmd += d; max_cmd = max(max_cmd, cmd)
        ceff = cmd * batt; vss = min(ceff, max(0.0, 3.0 * (ceff - creq))) if ceff > 0 else 0.0
        a = (vss - v) / 0.5; v += a * dt; peak_a = max(peak_a, abs(a)); x += v * dt; t += dt
        if on and v < 0.02 and cmd > 0.1: stuck_s += dt
        hist.append((t, v, a, cmd, x, pit))
    ok = x >= L + 0.5
    return dict(ok=ok, t=t, peak_a=peak_a, peak_rate=peak_rate, max_cmd=max_cmd, stuck=stuck_s, hist=hist)

if __name__ == '__main__':
    n = 20
    print('RAMP 4 m, speed estimate from LiDAR range-rate only. n = %d seeds per cell. climbed %% / time on the ramp s / peak accel m/s2 / max command / stuck s' % n)
    print(f"{'pitch':>5s} {'surface':12s} {'battery':>7s} | {'TODAY (0.35 + pushes)':46s} | GRADUAL (+0.02/s only if slow)")
    tot = {'today': [0, 0], 'gradual': [0, 0]}
    for pitch in (3, 5, 7, 9, 11, 13):
        for surf in ('dry', 'wet', 'cobble edge'):
            for batt in (1.0, 0.85):
                row = {}
                for ctrl in ('today', 'gradual'):
                    R = [sim(ctrl, pitch, surf, batt, 10 + k) for k in range(n)]; okc = sum(1 for r in R if r['ok'])
                    tot[ctrl][0] += okc; tot[ctrl][1] += n
                    ts = [r['t'] for r in R if r['ok']]
                    row[ctrl] = f"{100*okc/n:3.0f} % / {sum(ts)/len(ts) if ts else float('nan'):5.1f} / {max(r['peak_a'] for r in R):.2f} / {max(r['max_cmd'] for r in R):.2f} / {sum(r['stuck'] for r in R)/n:4.1f}"
                print(f"{pitch:4d}d {surf:12s} {batt:7.2f} | {row['today']:46s} | {row['gradual']}", flush=True)
    print('TOTAL climbed:', {k: f"{100*v[0]/v[1]:.0f} % of {v[1]}" for k, v in tot.items()})
