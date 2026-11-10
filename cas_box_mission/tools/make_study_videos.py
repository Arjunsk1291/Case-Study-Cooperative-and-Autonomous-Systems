#!/usr/bin/env python3
"""make_study_videos.py - videos + UI mock-up of the turn / ramp study (nothing is implemented in the firmware). Output: OUTDIR/*.mp4 (cv2, 10 fps, 1280x720 per video).
Turn videos: real LiDAR points of the site (from the logs), rover footprint + trail, both wheel commands, velocity and acceleration of each option, side by side.
  python3 make_study_videos.py OUTDIR"""
import sys, os, math; sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
import numpy as np, cv2
import matplotlib; matplotlib.use('Agg'); import matplotlib.pyplot as plt
from matplotlib.patches import Polygon
import sim_turn_curve as T, sim_ramp_climb as RC
OUT = sys.argv[1] if len(sys.argv) > 1 else '.'; os.makedirs(OUT, exist_ok=True)
BLUE, ORG, GRY, RED, GRN = '#1f6fb2', '#e07b1a', '#8a939c', '#c0392b', '#2e8b57'

def accel(h, i, w=6):
    j = max(0, i - w); dt = h[i][0] - h[j][0]; return (h[i][6] - h[j][6]) / dt if dt > 0 else 0.0

def writer(path, w=1280, h=720, fps=10): return cv2.VideoWriter(path, cv2.VideoWriter_fourcc(*'mp4v'), fps, (w, h))

def render_turn(path, panels, title, xlim, ylim, fps=10):
    """panels: list of dict(label, hist, pts, ok, peak, t, color)"""
    n = len(panels); fig = plt.figure(figsize=(12.8, 7.2), dpi=100); gs = fig.add_gridspec(3, n, height_ratios=[3.2, 1, 1.2], hspace=0.38, wspace=0.18, left=0.05, right=0.99, top=0.9, bottom=0.07)
    fig.suptitle(title, fontsize=13)
    A = []
    tmax = max(p['hist'][-1][0] for p in panels); frames = int(tmax * fps) + 1 + fps
    for k, p in enumerate(panels):
        am = fig.add_subplot(gs[0, k]); ac = fig.add_subplot(gs[1, k]); av = fig.add_subplot(gs[2, k])
        pts = p['pts']
        if pts is not None and len(pts): am.plot(pts[:, 0], pts[:, 1], '.', color='k', ms=2.5, alpha=.45)
        am.set_xlim(*xlim); am.set_ylim(*ylim); am.set_aspect('equal'); am.grid(alpha=.3); am.set_title(f"{p['label']}   |   peak wheel change {p['peak']:.1f}/s", fontsize=10, color=p['color'])
        H = p['hist']; tt = [h[0] for h in H]
        trail, = am.plot([], [], '-', color=p['color'], lw=1.5); body = Polygon(np.zeros((4, 2)), closed=True, fc=p['color'], ec='k', alpha=.8); am.add_patch(body)
        ac.plot(tt, [h[4] for h in H], color=BLUE, lw=1, label='left'); ac.plot(tt, [h[5] for h in H], color=ORG, lw=1, label='right'); ac.set_ylim(-0.6, 0.6); ac.set_ylabel('wheel cmd', fontsize=8); ac.grid(alpha=.3); ac.legend(fontsize=7, loc='upper right', ncol=2)
        av.plot(tt, [h[6] for h in H], color=GRN, lw=1, label='velocity m/s'); av.plot(tt, [accel(H, i) for i in range(len(H))], color=RED, lw=1, label='acceleration m/s2'); av.set_ylim(-0.6, 0.6); av.grid(alpha=.3); av.legend(fontsize=7, loc='upper right', ncol=2); av.set_xlabel('s', fontsize=8)
        c1 = ac.axvline(0, color='k', lw=1); c2 = av.axvline(0, color='k', lw=1); txt = am.text(0.02, 0.97, '', transform=am.transAxes, fontsize=9, va='top', bbox=dict(fc='w', alpha=.8))
        A.append((p, trail, body, c1, c2, txt, tt))
        for a in (ac, av): a.set_xlim(0, tmax + 1)
    vw = writer(path, fps=fps)
    for f in range(frames):
        t = f / fps
        for p, trail, body, c1, c2, txt, tt in A:
            H = p['hist']; i = min(len(H) - 1, int(np.searchsorted(tt, t)))
            trail.set_data([h[1] for h in H[:i + 1]], [h[2] for h in H[:i + 1]])
            x, y, phi = H[i][1], H[i][2], H[i][3]; c, s = math.cos(phi), math.sin(phi)
            body.set_xy([(x + sx * T.HALF_L * c + sy * T.HALF_W * s, y + sx * T.HALF_L * s - sy * T.HALF_W * c) for sx, sy in ((1, 1), (1, -1), (-1, -1), (-1, 1))])
            c1.set_xdata([tt[i]]); c2.set_xdata([tt[i]])
            txt.set_text(f"t {tt[i]:4.1f} s   v {H[i][6]:+.2f} m/s   a {accel(H, i):+.2f} m/s2\n{'DONE' if i == len(H) - 1 else ''} {'' if p['ok'] or i < len(H) - 1 else '(failed / touched)'}")
        fig.canvas.draw(); im = np.asarray(fig.canvas.buffer_rgba())[:, :, :3]; vw.write(cv2.cvtColor(im, cv2.COLOR_RGB2BGR))
    vw.release(); plt.close(fig); print('video', path, frames, 'frames')

def pick(site_key, kinds, ground='cobble'):
    direction, ev, leg, logs = T.SITES[site_key]
    pts = None
    for lab, pth in logs:
        pts = T.site_points(pth, ev, leg)
        if pts is not None and len(pts): break
    out = []
    for kind, v, R, label, col in kinds:
        best = None
        for sd in range(100, 130):
            o = T.turn(kind, v, R or 0.0, ground, sd, pts, direction, acc_lim=1.0)
            score = (o['ok'], o['t'] if kind in ('pivot', 'pivot_soft') else -o['t'])      # for the pivot show a seed that stalls (longest), for curves the typical
            if kind in ('pivot', 'pivot_soft'):
                if best is None or (o['t'] > best[1]['t']): best = (sd, o)
            else:
                if o['ok'] and (best is None or o['t'] < best[1]['t'] + 3): best = (sd, o); break
        sd, o = best; out.append(dict(label=label, hist=o['hist'], pts=pts, ok=o['ok'], peak=o['peak'], color=col, t=o['t']))
    return out, pts

def render_ramp(path, runs, title, fps=10):
    fig = plt.figure(figsize=(12.8, 7.2), dpi=100); n = len(runs); gs = fig.add_gridspec(4, n, height_ratios=[1.6, 1, 1, 1], hspace=0.4, wspace=0.18, left=0.06, right=0.99, top=0.9, bottom=0.07); fig.suptitle(title, fontsize=13)
    tmax = max(r['hist'][-1][0] for r in runs); frames = int(min(tmax, 70) * fps) + fps; A = []
    for k, r in enumerate(runs):
        H = r['hist']; tt = [h[0] for h in H]; pit = max(h[5] for h in H)
        a0 = fig.add_subplot(gs[0, k]); a1 = fig.add_subplot(gs[1, k]); a2 = fig.add_subplot(gs[2, k]); a3 = fig.add_subplot(gs[3, k])
        a0.fill([-3, 0, 4, 5], [0, 0, 4 * math.tan(math.radians(pit)), 4 * math.tan(math.radians(pit))], color='#ddd'); a0.set_xlim(-3, 5); a0.set_ylim(-0.1, 0.8); a0.set_title(f"{r['label']}  ({'CLIMBED' if r['ok'] else 'STUCK'}, max command {r['max_cmd']:.2f})", fontsize=10, color=GRN if r['ok'] else RED); a0.set_aspect('equal')
        dot, = a0.plot([], [], 'o', color=BLUE, ms=12)
        a1.plot(tt, [h[3] for h in H], color=ORG); a1.set_ylabel('command', fontsize=8); a1.set_ylim(0, 0.55); a1.grid(alpha=.3)
        a2.plot(tt, [h[1] for h in H], color=GRN, label='velocity m/s'); a2.plot(tt, [h[2] for h in H], color=RED, lw=.8, label='acceleration m/s2'); a2.set_ylim(-0.8, 0.8); a2.grid(alpha=.3); a2.legend(fontsize=7, loc='upper right', ncol=2)
        a3.plot(tt, [h[5] for h in H], color=BLUE); a3.set_ylabel('pitch deg', fontsize=8); a3.set_xlabel('s', fontsize=8); a3.grid(alpha=.3)
        cur = [a.axvline(0, color='k', lw=1) for a in (a1, a2, a3)]; A.append((r, dot, cur, tt, pit))
        for a in (a1, a2, a3): a.set_xlim(0, min(tmax, 70))
    vw = writer(path, fps=fps)
    for f in range(frames):
        t = f / fps
        for r, dot, cur, tt, pit in A:
            H = r['hist']; i = min(len(H) - 1, int(np.searchsorted(tt, t))); x = H[i][4]; y = max(0.0, min(x, 4.0)) * math.tan(math.radians(pit)) + 0.05
            dot.set_data([x], [y]); [c.set_xdata([tt[i]]) for c in cur]
        fig.canvas.draw(); im = np.asarray(fig.canvas.buffer_rgba())[:, :, :3]; vw.write(cv2.cvtColor(im, cv2.COLOR_RGB2BGR))
    vw.release(); plt.close(fig); print('video', path, frames, 'frames')

def ui_mockup(path):
    r = RC.sim('gradual', 7, 'wet', 0.85, 11); H = r['hist']; t = [h[0] for h in H]
    fig = plt.figure(figsize=(9, 5.2), dpi=110, facecolor='#1d2733'); ax = fig.add_axes([0, 0, 1, 1]); ax.axis('off'); ax.set_xlim(0, 9); ax.set_ylim(0, 5.2)
    ax.text(0.3, 4.85, 'Speed  (planned card for the mission page)', color='w', fontsize=13, weight='bold')
    i = int(len(H) * 0.55); ax.text(0.3, 3.9, f"{H[i][1]:.2f} m/s", color='#6fd08c', fontsize=30, weight='bold'); ax.text(3.7, 3.9, f"{H[i][2]:+.2f} m/s²", color='#ff8a7a', fontsize=30, weight='bold')
    ax.text(0.3, 3.55, 'velocity (command + LiDAR range-rate)', color='#9aa7b4', fontsize=8); ax.text(3.7, 3.55, 'acceleration (BNO055 accel, 20 Hz)', color='#9aa7b4', fontsize=8)
    for k, (key, col, lab, y0, lim) in enumerate(((1, '#6fd08c', 'velocity m/s', 1.95, 0.5), (2, '#ff8a7a', 'acceleration m/s²', 0.4, 0.8))):
        a = fig.add_axes([0.07, y0 / 5.2, 0.9, 1.25 / 5.2], facecolor='#27333f'); a.plot(t, [h[key] for h in H], color=col, lw=1.4); a.axvline(t[i], color='w', lw=.8); a.set_ylim(-lim, lim if key == 2 else 0.5); a.tick_params(colors='#9aa7b4', labelsize=7); a.set_ylabel(lab, color='#9aa7b4', fontsize=8); a.grid(alpha=.2)
        for s in a.spines.values(): s.set_color('#46525e')
    ax.text(7.0, 4.2, f"command {H[i][3]:.2f}\npitch {H[i][5]:.1f}°\nlearned trim +0.27", color='w', fontsize=10, va='top')
    fig.savefig(path, facecolor='#1d2733'); plt.close(fig); print('mock-up', path)

if __name__ == '__main__':
    K = T.SITES
    k1 = 'T1 first right turn (sign pillar)'; k2 = 'end-wall right turn (open sea -> leg 3)'; k3 = 'T3 left turn (arcade corner -> leg 4)'
    ui_mockup(os.path.join(OUT, 'ui_speed_acceleration_mockup.png'))
    for key, name, kinds, xl, yl in ((k1, '1_T1_sign_pillar_turn', [('pivot', 0.5, None, 'TODAY: full-power pivot (cobble)', RED), ('scurve', 0.30, 1.0, 'smooth curve R 1.0 m, v 0.30 (inner wheel 0.10)', BLUE)], (-2.5, 3.5), (-2.5, 3.5)),
                                      (k2, '2_end_wall_turn', [('pivot', 0.5, None, 'TODAY: full-power pivot (cobble)', RED), ('pivot_soft', 0.5, None, 'GENTLER pivot: wheel change 1.0/s, 0.35 (no curve fits: wall 1.3 m)', BLUE)], (-3.0, 3.0), (-2.0, 3.0)),
                                      (k3, '3_T3_left_turn', [('pivot', 0.5, None, 'TODAY: full-power pivot (cobble)', RED), ('arc', 0.30, 1.0, 'constant-radius curve R 1.0 m, v 0.30 (inner wheel 0.10)', BLUE)], (-3.0, 3.0), (-2.0, 3.5))):
        panels, pts = pick(key, kinds); render_turn(os.path.join(OUT, name + '.mp4'), panels, key + '   (real LiDAR map of the site, cobble ground model)', xl, yl)
    # speed comparison at the T1 site: same smooth curve at four speeds
    kinds = [('scurve', v, R, f'smooth curve R {R} m, v {v:.2f} (inner wheel {v*(1-1/(1.5*R)):.2f})', c) for v, R, c in ((0.15, 2.0, '#6a3d9a'), (0.22, 1.5, BLUE), (0.22, 1.2, GRN), (0.30, 1.0, ORG))]
    panels, pts = pick(k1, kinds, ground='tile'); render_turn(os.path.join(OUT, '4_speed_comparison_T1.mp4'), panels, 'smooth curve at four speeds, each with the inner wheel kept rolling (T1 site, tile)', (-2.0, 3.5), (-2.0, 3.5))
    runs = []
    for ctrl, lab in (('today', 'TODAY: 0.35 + pushes'), ('gradual', 'GRADUAL: +0.02 per s only if slow')):
        r = RC.sim(ctrl, 7, 'wet', 0.85, 11); r['label'] = lab + ', 7 deg wet, battery 85 %'; runs.append(r)
    render_ramp(os.path.join(OUT, '5_ramp_climb_today_vs_gradual.mp4'), runs, 'Ramp climb: slow detection and gradual speed increase (model)')
