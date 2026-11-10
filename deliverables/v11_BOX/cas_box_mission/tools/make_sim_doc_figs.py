#!/usr/bin/env python3
"""make_sim_doc_figs.py - figures for deliverables/report/SIMULATOR.md (pipeline, world + scanner, rover model, real vs simulated scan)."""
import sys, math, random, os; sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
import matplotlib; matplotlib.use('Agg')
import matplotlib.pyplot as plt
from matplotlib.patches import FancyBboxPatch, FancyArrowPatch
import sim_open_sea as S, decode_mission_log as D
OUT = os.path.join(os.path.dirname(os.path.abspath(__file__)), '../../deliverables/report/sim_img/')
BLUE, ORG, GRY, INK = '#1f6fb2', '#e07b1a', '#8a939c', '#1d2733'


def pipeline():
    fig, ax = plt.subplots(figsize=(11, 3.1)); ax.axis('off'); ax.set_xlim(0, 11); ax.set_ylim(0, 3.1)
    boxes = [('World', 'walls, store front,\npoles, trees, people\n(segments, metres)'), ('Scanner', '90 beams x 4 deg,\n2 cm noise, 12 % drops,\nghosts, sun, snow'),
             ('Firmware mirror', 'fwmirror.py = the box\nrules line by line,\n1 step / 0.25 s'), ('Rover model', '0.22 m/s, turn rate,\nslip, gyro bias,\nstops = ramps'), ('Metrics', 'side error, false end\nwall, holds, waiting\ntime, reached?')]
    for i, (t, s) in enumerate(boxes):
        x = 0.15 + i * 2.17
        ax.add_patch(FancyBboxPatch((x, 0.55), 1.8, 1.85, boxstyle='round,pad=0.03,rounding_size=0.12', fc='white', ec=BLUE if i != 2 else ORG, lw=1.8))
        ax.text(x + 0.9, 2.1, t, ha='center', va='center', fontsize=12, weight='bold', color=INK); ax.text(x + 0.9, 1.3, s, ha='center', va='center', fontsize=8.5, color=INK)
        if i < 4: ax.add_patch(FancyArrowPatch((x + 1.83, 1.5), (x + 2.14, 1.5), arrowstyle='-|>', mutation_scale=14, color=GRY))
    ax.annotate('', xy=(0.9, 0.5), xytext=(9.4, 0.5), arrowprops=dict(arrowstyle='-|>', color=GRY, connectionstyle='arc3,rad=0.0'))
    ax.text(5.1, 0.22, 'new rover position and heading -> next scan (closed loop)', ha='center', fontsize=9, color=GRY)
    plt.tight_layout(); plt.savefig(OUT + 'sim_pipeline.png', dpi=110); plt.close()


def world_scan():
    rng = random.Random(5); segs = S.world(rng, rowoff=6.65, people=3)
    x, y, th = 0.0, -14.0, math.radians(-90)
    fig, ax = plt.subplots(figsize=(11, 4.4))
    for (a, b) in segs: ax.plot([a[1], b[1]], [a[0], b[0]], color=INK, lw=1.6)
    for i in range(90):
        a = math.radians(i * 4 + 2); d = S.ray(x, y, th - a, segs)
        if d is not None and rng.random() > 0.12:
            ex, ey = x + d * math.cos(th - a), y + d * math.sin(th - a); ax.plot([y, ey], [x, ex], color=BLUE, lw=0.25, alpha=0.5); ax.plot(ey, ex, '.', color=ORG, ms=3)
    ax.plot(y, x, 's', color='k', ms=7); ax.annotate('rover (heading: to the left)', (y, x), (y + 1.2, x - 2.3), fontsize=9, arrowprops=dict(arrowstyle='->'))
    ax.set_xlim(-41, 2); ax.set_ylim(-6, 11); ax.set_aspect('equal'); ax.invert_xaxis(); ax.set_xlabel('metres along the open sea (end wall at the left)'); ax.set_ylabel('metres left of the strip')
    ax.set_title('One simulated LiDAR scan (orange = returned points, blue = beams; 12 % of the beams randomly lost)', fontsize=10)
    ax.text(-8, 7.2, 'store front + poles (6.65 m)', fontsize=8, color=GRY); ax.text(-31, 3.4, 'building front (3 m)', fontsize=8, color=GRY); ax.text(-39.5, 8.2, 'end wall', fontsize=8, color=GRY)
    plt.tight_layout(); plt.savefig(OUT + 'sim_world_scan.png', dpi=110); plt.close()


def real_vs_sim():
    R = D.decode(os.path.join(os.path.dirname(os.path.abspath(__file__)), '../../Log_run/campus_10.2/mis_001 (3).bin'))
    r = [q for q in R if q['leg'] == 2 and 16 < q['leg_dist_m'] < 17][0]
    rng = random.Random(3); segs = S.world(rng, rowoff=6.65, people=2)
    L = S.scan(0.0, -16.5, math.radians(-90), segs, rng)
    fig, axs = plt.subplots(1, 2, figsize=(10, 4.2), subplot_kw=dict(projection='polar'))
    for ax, data, t, c in ((axs[0], [r['lidar_%03d' % (i * 4)] for i in range(90)], 'REAL scan (campus_10.2 run 3, 16.5 m)', BLUE), (axs[1], L, 'SIMULATED scan (same place)', ORG)):
        th = [math.radians(i * 4 + 2) for i in range(90) if data[i] and data[i] < 1200]; rr = [data[i] / 100 for i in range(90) if data[i] and data[i] < 1200]
        ax.scatter(th, rr, s=9, color=c); ax.set_theta_zero_location('N'); ax.set_theta_direction(-1); ax.set_rmax(12); ax.set_title(t, fontsize=10)
    fig.suptitle('Same 90-beam format: the landmark row (left, about 6.5 m) appears at the same bearing and range (up = front, clockwise)', fontsize=10)
    plt.tight_layout(); plt.savefig(OUT + 'sim_real_vs_sim.png', dpi=110); plt.close()


def rover_model():
    fig, ax = plt.subplots(1, 3, figsize=(11, 3.2))
    t = [i * 0.05 for i in range(0, 100)]; v = [max(0, 0.25 - 0.30 * max(0, x - 1.0)) for x in t]
    ax[0].plot(t, v, color=BLUE, lw=2); ax[0].set_title('stop = ramp (0.30 per second)', fontsize=10); ax[0].set_xlabel('s'); ax[0].set_ylabel('wheel command')
    d = [i * 0.5 for i in range(0, 90)]; ax[1].plot(d, [0.0 + 6 / 60 * x / 0.22 * 0.5 * 0.0 + (6.0 / 60.0) * (x / 0.22) for x in d], color=ORG, lw=2); ax[1].set_title('gyro drift +6 deg/min', fontsize=10); ax[1].set_xlabel('driven metres'); ax[1].set_ylabel('heading error, deg')
    ax[2].arrow(0, 0, 4, 0, head_width=0.2, color=BLUE); ax[2].arrow(0, 0, 4, -0.32, head_width=0.2, color=ORG); ax[2].set_xlim(-0.5, 5); ax[2].set_ylim(-1, 1); ax[2].set_title('slip: 8 % sideways per metre', fontsize=10); ax[2].axis('off')
    ax[2].text(4.1, 0.05, 'commanded', color=BLUE, fontsize=8); ax[2].text(4.1, -0.45, 'actual', color=ORG, fontsize=8)
    plt.tight_layout(); plt.savefig(OUT + 'sim_rover_model.png', dpi=110); plt.close()


if __name__ == '__main__':
    pipeline(); world_scan(); real_vs_sim(); rover_model(); print(sorted(os.listdir(OUT)))
