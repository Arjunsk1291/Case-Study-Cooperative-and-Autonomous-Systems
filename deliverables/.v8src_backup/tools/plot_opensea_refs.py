#!/usr/bin/env python3
"""plot_opensea_refs.py - top view of the open sea from a mission log: LiDAR points along the driven track,
the landmark row the v6 firmware follows (fitted with the same robust line fit) and the intended strip line
(rowOffsetM to the right of the row). For checking on the laptop that the strip line is where the real strip is.

  python3 plot_opensea_refs.py mis_002.bin [--rowoff 6.5] [--out opensea.png]
Needs numpy + matplotlib. Track = odometry (leg_dist_m increments) + gyro yaw (good enough for a picture).
"""
import argparse, math, os, sys
sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
import decode_mission_log as D


def w180(a):
    return (a + 180) % 360 - 180


def fitline(P):
    n = len(P); mx = sum(p[0] for p in P) / n; my = sum(p[1] for p in P) / n
    sxx = sum((p[0] - mx) ** 2 for p in P); syy = sum((p[1] - my) ** 2 for p in P)
    sxy = sum((p[0] - mx) * (p[1] - my) for p in P)
    phi = 0.5 * math.atan2(2 * sxy, sxx - syy); ux, uy = math.cos(phi), math.sin(phi)
    if ux < 0: ux, uy = -ux, -uy
    return mx, my, ux, uy


def row_fit(L, center=270, half=70, rmin=1.5, rmax=9.0, tol=0.25, min_n=5, min_len=4.0, min_pair=1.0):
    """same as LineRef::rowFit in the firmware"""
    P = []
    for i in range(90):
        cm = L[i]
        if not cm or cm < rmin * 100 or cm > rmax * 100: continue
        a = i * 4 + 2; o = abs((a - center + 540) % 360 - 180)
        if o > half: continue
        d = cm / 100; P.append((d * math.cos(math.radians(a)), d * math.sin(math.radians(a))))
    if len(P) < min_n: return None
    best, bl = 0, None
    for i in range(len(P)):
        for j in range(i + 1, len(P)):
            dx = P[j][0] - P[i][0]; dy = P[j][1] - P[i][1]; n = math.hypot(dx, dy)
            if n < min_pair: continue
            nx, ny = -dy / n, dx / n; c = nx * P[i][0] + ny * P[i][1]
            k = sum(1 for p in P if abs(nx * p[0] + ny * p[1] - c) < tol)
            if k > best: best, bl = k, (nx, ny, c)
    if best < min_n: return None
    nx, ny, c = bl; Q = [p for p in P if abs(nx * p[0] + ny * p[1] - c) < tol]
    mx, my, ux, uy = fitline(Q); al = [x * ux + y * uy for x, y in Q]
    rms = math.sqrt(sum((-uy * (x - mx) + ux * (y - my)) ** 2 for x, y in Q) / len(Q))
    if max(al) - min(al) < min_len or rms > 0.5 * tol: return None
    return math.degrees(math.atan2(uy, ux)), -uy * mx + ux * my, Q


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("log"); ap.add_argument("--rowoff", type=float, default=6.5); ap.add_argument("--out", default=None)
    a = ap.parse_args()
    import matplotlib; matplotlib.use("Agg"); import matplotlib.pyplot as plt
    R = [r for r in D.decode(a.log) if r["state"] == "l2_go"]
    if not R: raise SystemExit("no open-sea records (state l2_go) in this log")
    yaw0 = R[0]["yaw"]
    # path frame: x = along the first open-sea heading, y = to the right
    x = y = 0.0; prev = R[0]["leg_dist_m"]; pts = []; track = []; rows = []
    for r in R:
        ds = r["leg_dist_m"] - prev; prev = r["leg_dist_m"]
        if ds < 0 or ds > 0.5: ds = 0
        th = math.radians(-(r["yaw"] - yaw0))           # + = turned right in the path frame
        x += ds * math.cos(th); y += ds * math.sin(th); track.append((x, y))
        L = [r["lidar_%03d" % (i * 4)] for i in range(90)]
        for i in range(0, 90, 2):
            cm = L[i]
            if 30 <= cm <= 1000:
                ang = math.radians(i * 4 + 2) + th
                pts.append((x + cm / 100 * math.cos(ang), y + cm / 100 * math.sin(ang)))
        f = row_fit(L)
        # same gates as the firmware: left side, 4-9.5 m, parallel to the current heading within 8 deg
        if f and f[1] < 0 and 4.5 <= -f[1] <= 8.5 and abs(w180(f[0])) <= 8.0 and 10.0 <= r["leg_dist_m"] <= 26.0:
            for qx, qy in f[2]:
                rows.append((x + qx * math.cos(th) - qy * math.sin(th), y + qx * math.sin(th) + qy * math.cos(th)))
    fig, ax = plt.subplots(figsize=(7, 11))
    ax.scatter([p[1] for p in pts], [p[0] for p in pts], s=1, c="#999999", label="LiDAR points")
    ax.plot([p[1] for p in track], [p[0] for p in track], "b-", lw=2, label="driven track (odometry + gyro)")
    if rows:
        ax.scatter([p[1] for p in rows], [p[0] for p in rows], s=10, c="g", label="landmark row points (firmware fit)")
        mx, my, ux, uy = fitline([(p[0], p[1]) for p in rows])
        xs = [min(p[0] for p in rows) - 3, max(p[0] for p in rows) + 3]
        ry = [my + (xx - mx) * uy / ux for xx in xs]
        ax.plot(ry, xs, "g--", lw=1.5, label="landmark row")
        ax.plot([v + a.rowoff for v in ry], xs, "r-", lw=2.5, label=f"intended strip ({a.rowoff} m right of the row)")
    ax.set_xlabel("right (m)"); ax.set_ylabel("along the open sea (m)"); ax.set_aspect("equal"); ax.grid(alpha=.3)
    ax.legend(loc="lower left", fontsize=8); ax.set_title(os.path.basename(a.log) + " - open sea, v6 references")
    out = a.out or a.log.rsplit(".", 1)[0] + "_opensea.png"
    fig.savefig(out, dpi=110, bbox_inches="tight"); print("wrote", out)


if __name__ == "__main__":
    main()
