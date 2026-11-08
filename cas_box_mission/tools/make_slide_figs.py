#!/usr/bin/env python3
"""make_slide_figs.py - figures for the weekly-progress slides (landmark corridor map, Block I map, people-sim bars).
Built from the real mission logs; colours = the team deck (blue 1F6FB2, orange E07B1A)."""
import sys, os, math, re
sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
import matplotlib; matplotlib.use('Agg')
import matplotlib.pyplot as plt
import decode_mission_log as D
BL, OR, GR, INK, LG = '#1F6FB2', '#E07B1A', '#6A737D', '#1D2733', '#D5DBE1'
OUT = os.path.join(os.path.dirname(os.path.abspath(__file__)), '../../PPT/new_slides_assets/')
plt.rcParams.update({'font.family': ['Liberation Sans', 'DejaVu Sans'], 'font.size': 9, 'axes.edgecolor': LG, 'text.color': INK, 'axes.labelcolor': GR, 'xtick.color': GR, 'ytick.color': GR})
LOG = lambda p: '../../Log_run/' + p


BOX = dict(boxstyle='round,pad=0.15', fc='white', ec='none', alpha=0.85)


def project(R, h0, x0=0.0, y0=0.0, along=None):
    """points of the scans in a leg frame (x along, y LEFT) by integrating commanded distance with the heading"""
    pts = []; x, y = x0, y0; prev = None; path = []
    for r in R:
        rel = math.radians(r['yaw'] - h0)
        if prev is not None:
            dd = r['leg_dist_m'] - prev['leg_dist_m']
            x += dd * math.cos(rel); y += dd * math.sin(rel)
        prev = r; path.append((x, y, r))
        for i in range(90):
            cm = r['lidar_%03d' % (i * 4)]
            if not cm or cm > 1000: continue
            a = math.radians(i * 4 + 2) - rel; d = cm / 100            # clockwise from the leg direction
            pts.append((x + d * math.cos(a), y - d * math.sin(a)))
    return pts, path


def fig_sea():
    R = D.decode(LOG('campus_9/mis_001.bin')); l2 = [r for r in R if r['leg'] == 2 and r['state'] == 'l2_go']
    h0 = sorted(r['yaw'] for r in l2[:30])[15]
    # open sea frame: x = metres along the strip, y = LEFT of the strip (the strip is the tactile line the rover drives on)
    pts, path = project(l2, h0)
    fig, ax = plt.subplots(figsize=(7.8, 2.9), dpi=220)
    xs = [p[0] for p in pts if -6 < p[0] < 50 and -4 < p[1] < 11]; ys = [p[1] for p in pts if -6 < p[0] < 50 and -4 < p[1] < 11]
    ax.scatter(xs, ys, s=0.25, c='#8A939C', linewidths=0, rasterized=True)
    ax.plot([p[0] for p in path], [p[1] for p in path], c=OR, lw=1.6, label='rover track')
    end = max(p[0] for p in path)
    # landmarks = circles around the real LiDAR clusters (no rule lines): tree + structure, two store poles, pole group
    from matplotlib.patches import Ellipse
    def ring(cx, cy, w, h, label, lx, ly, ha='center'):
        ax.add_patch(Ellipse((cx, cy), w, h, fill=False, ec=BL, lw=2.0))
        ax.text(lx, ly, label, color=BL, ha=ha, va='center', fontsize=9, fontweight='bold', bbox=BOX)
    ring(10.6, 4.1, 3.6, 3.2, 'tree + structure 1', 9.4, 5.9)
    ring(14.8, 2.2, 1.5, 1.4, 'store pole 1', 15.4, 3.9)
    ring(22.6, 2.1, 1.6, 1.4, 'store pole 2', 22.6, 3.9)
    ring(28.6, 3.2, 4.6, 3.2, 'tree + structure 2', 28.6, 5.9)
    ax.annotate('', xy=(12.6, 7.3), xytext=(24.6, 7.3), arrowprops=dict(arrowstyle='<->', color=OR, lw=1.4))
    ax.text(20.5, 8.5, 'store front (U-shape) 6.65 m = PID reference', color=OR, ha='center', fontsize=9, fontweight='bold', bbox=BOX)
    ax.axvline(0, c=BL, lw=2.2, ymin=0.0, ymax=1.0, alpha=0.9); ax.text(0.6, 9.4, 'B wall behind\n(straight, always there)', color=BL, fontsize=9, fontweight='bold', bbox=BOX, va='top')
    ax.axvline(end + 1.2, c=BL, lw=2.2); ax.text(end + 0.9, 9.4, 'end wall ahead\n(straight)', color=BL, fontsize=9, fontweight='bold', bbox=BOX, ha='right', va='top')
    ax.set_xlim(-3, end + 3); ax.set_ylim(-3.2, 10.2); ax.set_aspect('equal')
    ax.set_xlabel('metres along the Open Sea (commanded odometry)'); ax.set_ylabel('left of the strip (m)')
    for s in ('top', 'right'): ax.spines[s].set_visible(False)
    ax.legend(loc='lower center', frameon=False, fontsize=8.5)
    fig.subplots_adjust(left=0.08, right=0.99, top=0.98, bottom=0.17); fig.savefig(OUT + 'fig_sea.png'); plt.close(fig)
    print('fig_sea', len(pts), end)


def fig_blockI():
    R = D.decode(LOG('campus_9/mis_001.bin')); l4 = [r for r in R if r['leg'] == 4 and r['state'] in ('l4_follow', 'hold', 'end_ask', 'paused')]
    h0 = -92.0
    pts, path = project(l4, h0)
    at = {}
    for x, y, r in path:
        for k, lo in (('walkway', 14.8), ('corr', 35.63), ('ent', 46.96), ('hold', 56.4)):
            if k not in at and r['leg_dist_m'] >= lo: at[k] = (x, y)
    fig, ax = plt.subplots(figsize=(7.8, 2.9), dpi=220)
    xs = [p[0] for p in pts if -3 < p[0] < 70 and -6 < p[1] < 7]; ys = [p[1] for p in pts if -3 < p[0] < 70 and -6 < p[1] < 7]
    ax.scatter(xs, ys, s=0.25, c='#8A939C', linewidths=0, rasterized=True)
    ax.plot([p[0] for p in path], [p[1] for p in path], c=OR, lw=1.6)
    x1, x2, x3, x4 = at['walkway'][0], at['corr'][0], at['ent'][0], at['hold'][0]
    ax.axvspan(x2, x3 + 1.5, ymin=0, ymax=1, color=BL, alpha=0.10, lw=0)
    ax.text((x2 + x3) / 2, -4.3, 'railing corridor\nright 0.9 m / left 1.9 m', color=BL, ha='center', fontsize=8.5, fontweight='bold', bbox=BOX)
    ax.text((x1 + x2) / 2 - 1, -4.3, 'interlock walkway', color=GR, ha='center', fontsize=8.5, fontweight='bold', bbox=BOX)
    ax.text(x1 / 2, -4.3, 'wall follow\n1.0 m', color=GR, ha='center', fontsize=8.5, fontweight='bold', bbox=BOX)
    ax.axvline(x3, c=OR, lw=1.4, ls='--'); ax.text(x3 + 0.5, -4.6, 'KARL glass\nentrance', color=OR, fontsize=8.5, fontweight='bold', bbox=BOX)
    ax.axvline(x3 + 1.5, c=INK, lw=1.8); ax.text(x3 + 2.0, 0.2, 'v10 stop\n(entrance + 1.5 m)', color=INK, fontsize=8.5, fontweight='bold', bbox=BOX, va='center')
    ax.plot([x4 + 0.5], [at['hold'][1]], marker='s', c='#B00020', ms=6); ax.text(x4 + 0.4, at['hold'][1] + 1.0, 'bollard\n(v9 hit it)', color='#B00020', fontsize=8.5, fontweight='bold', bbox=BOX)
    from matplotlib.patches import Ellipse
    def ring(cx, cy, w, h, label, lx, ly, ang=0):
        ax.add_patch(Ellipse((cx, cy), w, h, angle=ang, fill=False, ec=BL, lw=2.0))
        ax.text(lx, ly, label, color=BL, ha='center', va='center', fontsize=8.5, fontweight='bold', bbox=BOX)
    ring(17.8, 4.7, 4.0, 4.6, 'benches + hydrant\non the grass', 16.0, 7.8)
    ring(28.6, 5.2, 10.5, 3.6, 'low railing + drain at\nthe building base', 30.5, 7.8, ang=12)
    ax.set_xlim(-2, x4 + 6); ax.set_ylim(-5.4, 8.6); ax.set_aspect('equal')
    ax.set_xlabel('metres along leg 4 (commanded odometry)'); ax.set_ylabel('left of the track (m)')
    for s in ('top', 'right'): ax.spines[s].set_visible(False)
    fig.subplots_adjust(left=0.08, right=0.99, top=0.98, bottom=0.17); fig.savefig(OUT + 'fig_blockI.png'); plt.close(fig)
    print('fig_blockI', len(pts), at)


def parse_people(path):
    rows = {}
    for ln in open(path):
        m = re.match(r'^(.*?)\s+(\d+) \|\s+([\d.]+)%\s+(\d+)\s+(\d+) \|\s+([\d.]+)%\s+(\d+)\s+(\d+)\s*$', ln.rstrip())
        if m: rows[m.group(1).strip()] = tuple(float(x) for x in m.groups()[1:])
    return rows


def fig_people(path=OUT + 'people_sim_result.txt'):
    rows = parse_people(path)
    def grp(keys):
        n = sum(rows[k][0] for k in keys)
        ok9 = sum(rows[k][0] * rows[k][1] for k in keys) / n
        early9 = 100 * sum(rows[k][2] for k in keys) / n; early8 = 100 * sum(rows[k][5] for k in keys) / n
        return early8, early9, ok9
    G = [('1 person standing at the sign pillar', ['1 standing person near the turn pillar']),
         ('group of 2-5 at the sign pillar', [k for k in rows if k.startswith('group of') and 'near the turn pillar' in k]),
         ('1 person at the pillar before', ['1 standing person near the pillar before']),
         ('group of 2-5 at the pillar before', [k for k in rows if k.startswith('group of') and 'pillar before' in k]),
         ('group beyond the sign (open area)', ['group BEYOND the turn pillar (open area)']),
         ('person walking with the rover', ['person walking along WITH the rover']),
         ('follower behind-right (log8 case)', ['FOLLOWER behind-right (log8 case)']),
         ('person crossing the walkway', ['person crossing (walks across)']),
         ('crowd of 8, half walking', ['crowd of 8 (half walking)'])]
    fig, ax = plt.subplots(figsize=(7.8, 3.55), dpi=220)
    y = list(range(len(G)))[::-1]
    for yy, (name, keys) in zip(y, G):
        e8, e9, ok9 = grp(keys)
        ax.barh(yy + 0.19, e8, height=0.34, color='#C9D1D9'); ax.barh(yy - 0.19, e9, height=0.34, color=BL)
        ax.text(e8 + 0.4, yy + 0.19, f'{e8:.0f} %', va='center', fontsize=9, color=GR)
        ax.text(e9 + 0.4, yy - 0.19, f'{e9:.0f} %', va='center', fontsize=10, color=BL, fontweight='bold')
        ax.text(61.5, yy, f'on target {ok9:.0f} %', va='center', ha='right', fontsize=10, color=INK)
    ax.set_yticks(y); ax.set_yticklabels([g[0] for g in G], fontsize=10)
    ax.set_xlim(0, 62); ax.set_xlabel('turn at a WRONG pillar (> 2 m too early), % of cases  -  lower is better', fontsize=9.5)
    from matplotlib.patches import Patch
    ax.legend(handles=[Patch(color='#C9D1D9', label='v8 rule (pillar landmark only)'), Patch(color=BL, label='v10 (sign structure + B-gap anchor)')],
              loc='lower center', bbox_to_anchor=(0.45, 1.0), ncol=2, frameon=False, fontsize=9.5)
    for s_ in ('top', 'right'): ax.spines[s_].set_visible(False)
    fig.subplots_adjust(left=0.33, right=0.985, top=0.92, bottom=0.14); fig.savefig(OUT + 'fig_people.png'); plt.close(fig)
    print('fig_people', {g[0]: tuple(round(v, 1) for v in grp(g[1])) for g in G})


def fig_turn():
    import replay_turn as RT, fwmirror as F
    runs = [('campus_6 (teach, night)', 'campus6'), ('test2 (day)', 'test2'), ('log8: turned one pillar early', 'log8'), ('campus_9: started 7.7 m earlier', 'campus9')]
    fig, axs = plt.subplots(1, 4, figsize=(9.2, 5.6), dpi=200, sharey=False)
    for ax, (title, key) in zip(axs, runs):
        sc, T = RT.leg1(key)
        pts = []
        for L, d, rel, anc in sc:
            for i in range(90):
                cm = L[i]
                if not cm or cm > 600: continue
                a = math.radians(i * 4 + 2 - rel); dd = cm / 100
                pts.append((dd * math.sin(a), d + dd * math.cos(a)))
        ax.scatter([p[0] for p in pts if -2.5 < p[0] < 6], [p[1] for p in pts if -2.5 < p[0] < 6], s=0.25, c='#8A939C', linewidths=0, rasterized=True)
        tv = F.TurnV9(); res = None; anchor = None
        for L, d, rel, anc in sc:
            if anc is not None and tv.anchor is None: tv.set_anchor(anc)
            res = tv.step(L, d, rel)
            if res: break
        end = max(d for _, d, _, _ in sc)
        if key == 'campus9': tv.anchor = 22.5                      # the wall is back in the last scans (logged event missing: it turned at that moment)
        ax.axvline(0, c='#C9D1D9', lw=1)
        ax.axhline(T, c='#B00020', lw=1.4, ls='--'); ax.text(-2.4, T + 0.3, f'turned {T:.1f} m', color='#B00020', fontsize=7.5, ha='left')
        if tv.anchor is not None: ax.axhline(tv.anchor, c=OR, lw=1.2); ax.text(5.9, tv.anchor - 0.95, f'B gap ends {tv.anchor:.1f} m', color=OR, fontsize=7.5, ha='right')
        if tv.lock is not None:
            ax.axhline(tv.lock, c=BL, lw=2); ax.text(5.9, tv.lock + 0.3, f'sign structure {tv.lock:.1f} m', color=BL, fontsize=7.5, ha='right', fontweight='bold')
            ax.axhline(tv.lock + tv.sp, c=BL, lw=1, ls=':')
        elif tv.anchor is not None:
            ax.axhspan(tv.anchor + 4.15, tv.anchor + 6.55, color=BL, alpha=0.12, lw=0); ax.text(5.9, tv.anchor + 6.8, 'v10 waits here\n(sign ~ gap end + 5.35 m)', color=BL, fontsize=7.5, ha='right', fontweight='bold')
        ax.set_title(title, fontsize=8.5, color=INK, pad=8); ax.set_xlim(-2.5, 6); ax.set_ylim(-1, max(end, T, (tv.lock or 0), (tv.anchor + 7.5 if (key == 'campus9') else 0)) + 3)
        ax.set_xlabel('right of the track (m)', fontsize=8)
        for s_ in ('top', 'right'): ax.spines[s_].set_visible(False)
        ax.set_aspect('equal')
    axs[0].set_ylabel('metres along the B facade (commanded odometry)', fontsize=8)
    fig.tight_layout(pad=0.6); fig.savefig(OUT + 'fig_turn.png'); plt.close(fig)


def fig_tools():
    """replay vs real turn position per log (v8 rule vs v10 structure) + start-shift sweep, numbers from replay_turn.py / sweep_start.py"""
    runs = ['campus_2', 'campus_3', 'campus_5', 'campus_6', 'test1', 'test2', 'log8']
    v10 = [19.03, 17.63, 19.80, 18.70, 21.16, 20.60, 21.80]
    v8 = [None, None, 19.82, 18.51, 21.49, 20.66, 18.20]
    fig, ax = plt.subplots(figsize=(7.2, 3.3), dpi=220)
    y = list(range(len(runs)))[::-1]
    for yy, a, b in zip(y, v8, v10):
        if a is not None: ax.plot([a, b], [yy, yy], c=LG, lw=3, zorder=1)
        ax.scatter([b], [yy], s=60, c=BL, zorder=3)
        if a is not None: ax.scatter([a], [yy], s=45, c='#C9D1D9', ec=GR, zorder=2)
        ax.text(max(a or 0, b) + 0.3, yy, f'{b:.1f} m', va='center', fontsize=9, color=BL, fontweight='bold')
    ax.set_yticks(y); ax.set_yticklabels(runs, fontsize=10); ax.set_xlim(16.5, 23.5)
    ax.set_xlabel('where the replay turns (m along B)   -   log8 was the run that turned one pillar early', fontsize=9)
    from matplotlib.lines import Line2D
    ax.legend(handles=[Line2D([], [], marker='o', ls='', c=BL, label='v10 replay'), Line2D([], [], marker='o', ls='', mfc='#C9D1D9', mec=GR, label='v8 replay (where run)')],
              loc='upper right', frameon=False, fontsize=9)
    for s_ in ('top', 'right'): ax.spines[s_].set_visible(False)
    fig.subplots_adjust(left=0.13, right=0.98, top=0.97, bottom=0.17); fig.savefig(OUT + 'fig_tools.png'); plt.close(fig)


def fig_ghost():
    """daylight LiDAR ghosts in the logs recorded before the 350 mm filter (campus_2/3/5, campus_4)"""
    import glob
    runs = [('campus_2', 'campus_2/mis_001 3.bin'), ('campus_2 (2nd)', 'campus_2/mis_002.bin'), ('campus_3', 'campus_3/mis_001 4.bin'), ('campus_5', 'campus5/mis_001 8.bin'), ('campus_4', 'campus4/mis_001 5.bin')]
    share, allr = [], []
    for name, f in runs:
        R = D.decode(LOG(f)); sc = 0
        for r in R:
            g = [r['lidar_%03d' % (i * 4)] for i in range(90) if 0 < r['lidar_%03d' % (i * 4)] < 35]
            if g: sc += 1; allr += g
        share.append(100 * sc / len(R))
    fig, (a1, a2) = plt.subplots(1, 2, figsize=(7.2, 3.2), dpi=220, gridspec_kw={'width_ratios': [1.1, 1]})
    a1.hist(allr, bins=range(15, 36), color=BL); a1.axvline(35, c=OR, lw=2); a1.text(34.6, a1.get_ylim()[1] * 0.92, 'min range\n35 cm', color=OR, ha='right', fontsize=9, fontweight='bold')
    a1.set_xlabel('range of the false returns (cm)'); a1.set_ylabel('beams')
    a2.barh(range(len(runs))[::-1], share, color=BL); a2.set_yticks(range(len(runs))[::-1]); a2.set_yticklabels([r[0] for r in runs], fontsize=9)
    for y, v in zip(range(len(runs))[::-1], share): a2.text(v + 0.2, y, f'{v:.1f} %', va='center', fontsize=9, color=BL, fontweight='bold')
    a2.set_xlabel('scans with a false return < 35 cm (%)'); a2.set_xlim(0, 10)
    for a in (a1, a2):
        for s_ in ('top', 'right'): a.spines[s_].set_visible(False)
    fig.subplots_adjust(left=0.09, right=0.98, top=0.97, bottom=0.18, wspace=0.55); fig.savefig(OUT + 'fig_ghost.png'); plt.close(fig)


def fig_faults():
    """Open Sea fault simulation: side error at the end wall (median of 20 runs), V6 / V7 (old gate) / v10.1 (gate +-1.0 m)"""
    import re
    rd = lambda f: {m.group(1) + ' ' + m.group(2).strip(): float(m.group(3)) for m in (re.match(r'^(V\d) (.*?)\s+24 m: .*end: med ([\d.]+)', l) for l in open(OUT + f)) if m}
    b, n = rd('open_sea_faults_before.txt'), rd('open_sea_faults_v10_1.txt')
    names = ['row missing completely (trees/poles hidden)', 'row gap 14-20 m (cars / a stall in front)', 'false line 4.5 m left, 12-22 m (bike rack)',
             'false line 8.5 m left, 10-24 m (hoarding)', 'B wall behind hidden (people at the sign)', 'heading glitch +10 deg at 10 s',
             'worst: bias -6, glitch, slide, wobble, row gap', 'gyro +6/min and row gone']
    lab = ['row missing completely', 'row gap 14-20 m (stall, cars)', 'false line 4.5 m (bike rack)', 'false line 8.5 m (hoarding)', 'B wall behind hidden',
           'heading glitch +10 deg', 'worst mix of 5 faults', 'gyro +6 deg/min, row gone']
    fig, ax = plt.subplots(figsize=(7.6, 3.5), dpi=220); y = list(range(len(names)))[::-1]
    for yy, k in zip(y, names):
        v6, v7, v10 = b['V6 ' + k], b['V7 ' + k], n['V7 ' + k]
        ax.barh(yy + 0.27, v6, height=0.24, color='#C9D1D9'); ax.barh(yy, v7, height=0.24, color='#E9B27A'); ax.barh(yy - 0.27, v10, height=0.24, color=BL)
        ax.text(max(v10, 0) + 0.04, yy - 0.27, f'{v10:.2f}', va='center', fontsize=8, color=BL, fontweight='bold')
        if v7 > 0.5: ax.text(v7 + 0.04, yy, f'{v7:.2f}', va='center', fontsize=8, color=ORANGE_T if False else '#C0620C')
    ax.set_yticks(y); ax.set_yticklabels(lab, fontsize=9); ax.set_xlim(0, 2.4); ax.set_xlabel('side error at the end wall (m), median of 20 runs  -  lower is better', fontsize=9)
    from matplotlib.patches import Patch
    ax.legend(handles=[Patch(color='#C9D1D9', label='V6 (row only as heading)'), Patch(color='#E9B27A', label='V7 (PID, gate 4.0-9.5 m)'), Patch(color=BL, label='v10.1 (PID, gate +-1.0 m)')],
              loc='lower center', bbox_to_anchor=(0.45, 1.0), ncol=3, frameon=False, fontsize=8.5)
    for s_ in ('top', 'right'): ax.spines[s_].set_visible(False)
    fig.subplots_adjust(left=0.27, right=0.985, top=0.9, bottom=0.14); fig.savefig(OUT + 'fig_faults.png'); plt.close(fig)


if __name__ == '__main__':
    fig_sea(); fig_blockI(); fig_people(); fig_turn(); fig_tools(); fig_ghost(); fig_faults()
