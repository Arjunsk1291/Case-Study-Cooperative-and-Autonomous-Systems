#!/usr/bin/env python3
"""make_fig_v103.py - figures for the v10.3 rover-7 update: start poses, end-board correction, leg-4 walkway (old vs new), learned trim.
  python3 make_fig_v103.py OUTDIR"""
import sys, os, math, random; sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
import matplotlib; matplotlib.use('Agg'); import matplotlib.pyplot as plt
import decode_mission_log as D, fwmirror as F, sim_start_leg1 as SL, sim_l4_walk as W, sim_open_sea as S
OUT = sys.argv[1] if len(sys.argv) > 1 else '.'; os.makedirs(OUT, exist_ok=True)
B = os.path.join(os.path.dirname(os.path.abspath(__file__)), '../../Log_run/'); BLUE, ORG, GRY, RED = '#1f6fb2', '#e07b1a', '#8a939c', '#c0392b'

def fig_start():
    fig, ax = plt.subplots(1, 3, figsize=(16, 5.6))
    R = D.decode(B + 'campus_10.3/mis_001 (5).bin'); r = [q for q in R if q['leg'] == 1 and q['state'] == 'leg1_follow'][3]
    for i in range(90):
        cm = r['lidar_%03d' % (i * 4)]
        if cm and 35 <= cm <= 600: a = math.radians(i * 4 + 2); ax[0].plot(cm / 100 * math.sin(a), cm / 100 * math.cos(a), '.', color=BLUE, ms=4)
    ax[0].plot(0, 0, 's', color='k', ms=10); ax[0].arrow(0, 0, 0, 0.9, head_width=0.2, color='k'); ax[0].axvline(-0.98, color=ORG, ls='--'); ax[0].text(-0.9, 3.2, 'pink facade\n(wall on the LEFT,\n0.98 m)', color=ORG)
    ax[0].set_xlim(-5, 5); ax[0].set_ylim(-4, 8); ax[0].set_aspect('equal'); ax[0].grid(alpha=.3); ax[0].set_title('Real start of leg 1 (rover 7, campus_10.3): the target pose', fontsize=10)
    for k, a in enumerate((ax[1], ax[2])):
        for sd in range(2000, 2000 + (16 if k == 0 else 16)):
            rr = SL.trial(sd, person=False)
            if k == 0:
                a.arrow(rr['x0'], rr['y0'], 0.7 * math.cos(rr['th0']), 0.7 * math.sin(rr['th0']), head_width=0.15, color=GRY, alpha=.8)
            else:
                a.arrow(rr['x0'], rr['y0'], 0.7 * math.cos(rr['th1']), 0.7 * math.sin(rr['th1']), head_width=0.15, color=BLUE if abs(rr['herr']) <= 10 else RED)
        for (x0, y0), (x1, y1) in SL.world(random.Random(1), False)[:6]: a.plot([x0, x1], [y0, y1], color='k', lw=2)
        a.axvspan(-0.1, 0.0, color=ORG, alpha=.3); a.set_xlim(-0.5, 4); a.set_ylim(-3, 6); a.set_aspect('equal'); a.grid(alpha=.3)
        a.set_title(('BEFORE: 16 random placements, any heading (arrows)' if k == 0 else 'AFTER the orient: wall on the left, facing along it (blue = ok)'), fontsize=10)
    fig.suptitle('Mission start at the B facade: found the wall, turned, then starts (sim_start_leg1.py: 295/300 clean, 280/300 stress; 5/300 = "no wall found" = refuses to start)', fontsize=11)
    plt.tight_layout(); plt.savefig(OUT + '/fig1_start_pose.png', dpi=90); plt.close()

def fig_board():
    fig, ax = plt.subplots(1, 2, figsize=(14, 4.8))
    RUNS = {'campus_3': 'campus_3/mis_001 4.bin', 'c102_3': 'campus_10.2/mis_001 (3).bin', 'c102_4': 'campus_10.2/mis_001 (4).bin', 'campus_6': 'campus_6/mis_001 14 (1).bin', 'log8': 'log8/mis_001 17.bin', 'campus_5': 'campus5/mis_001 8.bin', 'rover 7': 'campus_10.3/mis_001 (5).bin', 'campus_9': 'campus_9/mis_001.bin', 'c102_2': 'campus_10.2/mis_001 (2).bin'}
    names, dist, steer = [], [], []
    for n, f in RUNS.items():
        Rr = D.decode(B + f); l2 = [r for r in Rr if r['leg'] == 2 and r['state'] == 'l2_go']; vals = []; off = 0.0
        for r in l2:
            if r['leg_dist_m'] < 30.6: continue
            w = F.wallfit([r['lidar_%03d' % (i * 4)] for i in range(90)], False)
            if w and w[2] <= .05 and 0.4 <= w[0] <= 3.0 and abs(w[1]) <= 12: vals.append(w[0]); off += max(-1, min(1, max(-12, min(12, 10 * (w[0] - 1.2))) - off))
        if vals: names.append(n); dist.append(sum(vals[-5:]) / len(vals[-5:])); steer.append(off)
    ax[0].bar(names, dist, color=[ORG if n == 'rover 7' else BLUE for n in names]); ax[0].axhline(1.2, color='k', ls='--'); ax[0].text(0, 1.25, 'target 1.2 m', fontsize=9)
    ax[0].set_ylabel('board distance seen in the last 3 m (m)'); ax[0].set_title('END BOARD in 9 real logs: distance = how wide the rover ended', fontsize=10); ax[0].tick_params(axis='x', rotation=30)
    ax[1].bar(names, steer, color=[ORG if n == 'rover 7' else BLUE for n in names]); ax[1].axhline(0, color='k'); ax[1].set_ylabel('steering the rule would command at the end (deg, + = right)')
    ax[1].set_title('What the v10.3 end-board rule would do on these real scans (limit 12 deg)', fontsize=10); ax[1].tick_params(axis='x', rotation=30)
    plt.tight_layout(); plt.savefig(OUT + '/fig2_end_board_real.png', dpi=90); plt.close()

def fig_leg4():
    fig, ax = plt.subplots(1, 3, figsize=(16, 6.2), sharey=True)
    Lb = 1.2; rng = random.Random(7); Wd = W.world(Lb, rng, True, False)
    for a, (title, new, bias) in zip(ax, (('OLD v10.3 (rover 7 run): heading bias +12 deg', 0, 12.0), ('NEW: median wall direction + clearance', 1, 12.0), ('NEW, same bias but heading kept (clearance alone)', 2, 12.0))):
        for (x0, y0), (x1, y1) in Wd: a.plot([x0, x1], [y0, y1], color='k', lw=2)
        for s in range(3):
            tr = []; W.run(Lb, bias, (-0.3, 0.0, 0.3)[s], new, 100 + s, trace=tr); a.plot([p[0] for p in tr], [p[1] for p in tr], color=RED if new == 0 else BLUE, lw=1.8)
        a.axvline(-1.5, color=ORG, ls='--'); a.text(-1.45, 1, 'mud line\n(1.5 m left)', color=ORG, fontsize=8); a.set_xlim(-3, 4); a.set_ylim(0, 22); a.set_aspect('equal'); a.grid(alpha=.3); a.set_title(title, fontsize=10)
    fig.suptitle('Leg 4 walkway after the wall ends (benches + hydrant 1.2 m left): 90 runs per bias, old rule: mud 31/90 at +12 deg, new rule: mud 0/90, closest approach >= 0.52 m', fontsize=11)
    plt.tight_layout(); plt.savefig(OUT + '/fig3_leg4_walkway.png', dpi=90); plt.close()

def fig_trim():
    R = D.decode(B + 'campus_10.3/mis_001 (5).bin'); t = [r['t_s'] for r in R]; tr = [r['trim_learned'] for r in R]; d = [r['drive_s'] for r in R]
    fig, ax = plt.subplots(1, 2, figsize=(14, 4.4))
    ax[0].plot(t, tr, color=BLUE); ax[0].axhline(0.40, color=RED, ls='--'); ax[0].text(5, 0.41, 'old limit 0.40 (v10.3 first flash)', color=RED); ax[0].set_xlabel('s'); ax[0].set_ylabel('learned trim'); ax[0].set_title('Rover 7: learned wheel trim in campus_10.3 (it sat at the limit)', fontsize=10); ax[0].grid(alpha=.3)
    ax[1].plot(t, [r['heading_err_deg'] for r in R], color=ORG, lw=.8); ax[1].set_ylim(-8, 8); ax[1].set_xlabel('s'); ax[1].set_ylabel('heading error (deg)'); ax[1].set_title('heading error stays small: the hold works (new default trim +0.25, limit 0.60)', fontsize=10); ax[1].grid(alpha=.3)
    plt.tight_layout(); plt.savefig(OUT + '/fig4_trim_rover7.png', dpi=90); plt.close()

if __name__ == '__main__': fig_start(); fig_board(); fig_leg4(); fig_trim(); print(sorted(os.listdir(OUT)))
