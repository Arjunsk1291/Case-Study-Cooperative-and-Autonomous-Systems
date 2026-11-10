#!/usr/bin/env python3
"""plot_bev.py - bird's-eye LiDAR map of one leg per log (dead-reckoned: leg distance + gyro), events annotated.
  python3 plot_bev.py out.png LEG Y0 Y1 XMIN XMAX log1.bin [log2.bin ...]   (campus_10.2 analysis, v10.3)"""
import sys, math; sys.path.insert(0,'.')
import decode_mission_log as D, matplotlib; matplotlib.use('Agg'); import matplotlib.pyplot as plt
# usage: bev.py out.png leg file1 file2 ...
out, leg, Y0, Y1, X0, X1 = sys.argv[1], int(sys.argv[2]), float(sys.argv[3]), float(sys.argv[4]), float(sys.argv[5]), float(sys.argv[6]); files = sys.argv[7:]
fig, axs = plt.subplots(1, len(files), figsize=(4.5*len(files), 8))
if len(files)==1: axs=[axs]
for ax, f in zip(axs, files):
    R=D.decode(f); rows=[r for r in R if r['leg']==leg]
    h0=rows[0]['yaw']; x=y=0; pd=rows[0]['leg_dist_m']; last=-9; P=[]; T=[]
    for r in rows:
        d=r['leg_dist_m']; dd=d-pd; pd=d; th=math.radians(r['yaw']-h0)
        x+=dd*math.sin(th); y+=dd*math.cos(th); T.append((x,y,r['event'],d))
        if r['t_s']-last<0.5: continue
        last=r['t_s']
        for i in range(90):
            cm=r['lidar_%03d'%(i*4)]
            if not cm or cm<35 or cm>900: continue
            a=math.radians(i*4+2)+th; P.append((x+cm/100*math.sin(a), y+cm/100*math.cos(a)))
    ax.scatter([p[0] for p in P],[p[1] for p in P],s=1,c='k',alpha=.35)
    ax.plot([t[0] for t in T],[t[1] for t in T],'r-')
    for t in T:
        if t[3]>Y0 and t[2] and t[2] not in ('lidar_ghost_ignored','',): ax.annotate(f"{t[2]} {t[3]:.1f}",(t[0],t[1]),fontsize=7,color='b')
    for k in range(0,int(T[-1][3])+1,5):
        t=min(T,key=lambda t:abs(t[3]-k)); ax.plot(t[0],t[1],'go',ms=3); ax.annotate(str(k),(t[0]+.2,t[1]),fontsize=7,color='g')
    ax.set_aspect('equal'); ax.set_xlim(X0,X1); ax.set_ylim(Y0,Y1); ax.set_title(f.split('/')[-2]+'/'+f.split('/')[-1]+f' leg{leg}'); ax.grid(alpha=.3)
plt.tight_layout(); plt.savefig(out,dpi=80)
