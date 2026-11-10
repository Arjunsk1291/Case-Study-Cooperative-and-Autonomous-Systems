#!/usr/bin/env python3
"""fig_v12.py - figures for deliverables/v12_BOX: (1) the v11 end-board bug in the real campus_11 log and in the 20 ms loop sim, v11 vs v12;
(2) the leg-4 landmark table (landmark_leg4_db.json); (3) campus_11 run-video frames synced to the log (video = log - 23 s; rotated, gamma 0.6)."""
import sys, json, math, numpy as np, cv2, matplotlib; matplotlib.use('Agg'); import matplotlib.pyplot as plt; sys.path.insert(0, '.')
import decode_mission_log as D, sim_loop_timing as S
OUT = '../../deliverables/v12_BOX/figures/'
R = D.decode('../../Log_run/campus_11/mis_001 (6).bin'); seg = [r for r in R if 355 <= r['t_s'] <= 392]
fig, ax = plt.subplots(1, 2, figsize=(13, 4.2))
ax[0].plot([r['t_s'] for r in seg], [r['target_deg'] for r in seg], '.', ms=4, c='#c0392b', label='target (log)')
ax[0].plot([r['t_s'] for r in seg], [r['yaw'] for r in seg], c='#2c3e50', label='yaw (log)')
ax[0].set(title='campus_11 (v11), end of the Open Sea: target jumps up to 160 deg', xlabel='log time (s)', ylabel='deg'); ax[0].legend(); ax[0].grid(alpha=.3)
for v, c in (('v11', '#c0392b'), ('v12', '#27ae60')):
    L = S.run(v)['log']; ax[1].plot([a[0] for a in L], [a[2] for a in L], c=c, label=f'{v} heading'); ax[1].plot([a[0] for a in L], [a[1] for a in L], ':', c=c, label=f'{v} target')
ax[1].set(title='20 ms loop sim, board at 1.74 m: v11 offset applied every tick, v12 once', xlabel='s after the board is seen', ylabel='deg'); ax[1].legend(fontsize=8); ax[1].grid(alpha=.3)
plt.tight_layout(); plt.savefig(OUT + 'v12_end_board_bug.png', dpi=110); plt.close()
db = json.load(open('landmark_leg4_db.json')); fig, ax = plt.subplots(figsize=(12, 4.6))
for i, (n, v) in enumerate(db['runs'].items()):
    b = [int(k) + 1 for k in v['bins']]; c = plt.cm.tab10(i)
    ax.plot(b, [v['bins'][k]['left_m'] if v['bins'][k]['left_m'] else np.nan for k in v['bins']], 'o-', c=c, label=n)
    ax.plot(b, [-v['bins'][k]['right_m'] if v['bins'][k]['right_m'] else np.nan for k in v['bins']], 's--', c=c)
ax.axhline(2.1, c='k', lw=1, ls=':'); ax.text(0.3, 2.2, 'v12 walkway keep 2.1 m (left line)', fontsize=8)
ax.axhspan(-1.5, -0.35, color='#27ae60', alpha=.08); ax.text(0.3, -1.4, 'v12 corridor: parapet 0.35-1.5 m right (3 of 5 scans)', fontsize=8)
ax.set(xlim=(0, 28), xlabel='metres since the left wall ends (T3 corner)', ylabel='left line (+) / right parapet line (-), m',
       title='Leg 4 landmarks in 6 logs: benches/planter/facade on the left, the ramp parapet on the right from ~16 m'); ax.legend(fontsize=8, ncol=3); ax.grid(alpha=.3)
plt.tight_layout(); plt.savefig(OUT + 'v12_leg4_landmarks.png', dpi=110); plt.close()
cap = cv2.VideoCapture('../../Log_run/campus_11/test_run.MOV'); lut = np.array([((i / 255) ** 0.6) * 255 for i in range(256)], np.uint8); tiles = []
for vt, txt in ((345, 'log 368 s: Open Sea end, on cobbles left of the strip'), (530, 'log 553 s: leg 4, rover on cobbles, building base ahead'),
                (549, 'log 572 s: v11 50 deg pivot to the RIGHT (bad opening)'), (557, 'log 580 s: the real corridor: facade | parapet + railing')):
    cap.set(cv2.CAP_PROP_POS_MSEC, vt * 1000); ok, f = cap.read()
    if not ok: continue
    f = cv2.LUT(cv2.rotate(f, cv2.ROTATE_180), lut); f = cv2.resize(f, (640, int(640 * f.shape[0] / f.shape[1])))
    cv2.rectangle(f, (0, 0), (640, 30), (0, 0, 0), -1); cv2.putText(f, txt, (8, 21), cv2.FONT_HERSHEY_SIMPLEX, 0.55, (255, 255, 255), 1, cv2.LINE_AA); tiles.append(f)
h = min(t.shape[0] for t in tiles); tiles = [t[:h] for t in tiles]
cv2.imwrite(OUT + 'v12_video_frames.jpg', np.vstack([np.hstack(tiles[0:2]), np.hstack(tiles[2:4])]), [cv2.IMWRITE_JPEG_QUALITY, 80]); print('ok')
