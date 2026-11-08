#!/usr/bin/env python3
"""landmark_benchmark.py - measure the route landmarks in a mission log, keep them in a database, recommend settings.

  python3 landmark_benchmark.py LOG.bin --name campus_6 [--teach]   # measure + store (teach = it drove ON the path)
  python3 landmark_benchmark.py --report                            # all runs side by side + recommended settings

Every run adds evidence: the landmark distances/directions the firmware relies on are measured with the SAME code
the firmware uses (fwmirror.py). Teach runs (driven correctly on the strip) set the recommended values; the other
runs show the spread. Database: landmark_db.json next to this file.
"""
import argparse, json, math, os, statistics as st, sys
sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
import decode_mission_log as D
import fwmirror as F
from fwmirror import w180

DB = os.path.join(os.path.dirname(os.path.abspath(__file__)), "landmark_db.json")
FIRMWARE = {"rowOffsetM": 6.65, "pillarPastM": 0.40, "endWallM": 1.5, "leg4WallM": 1.0, "leg3WallM": 0.6}     # v8 defaults


def med(v): return round(st.median(v), 3) if v else None
def iqr(v):
    if len(v) < 4: return None
    q = st.quantiles(v, n=4); return round(q[2] - q[0], 3)


def measure(path):
    R = D.decode(path)
    L = lambda r: [r["lidar_%03d" % (i * 4)] for i in range(90)]
    m = {"records": len(R), "seconds": round(R[-1]["t_s"], 1)}
    # ---- leg 1
    l1 = [r for r in R if r["state"] == "leg1_follow"]
    m["leg1_wall_m"] = med([r["wall_d_m"] for r in l1 if r["wall_d_m"] > 0])
    m["leg1_gap_m"] = [round(r["leg_dist_m"], 1) for r in R if r["state"] == "leg1_follow" and r["event"] in ("wall_lost", "wall_found")]
    tp = [r for r in R if r["event"] in ("turn_pillar", "t1_ask", "op_turn") and r["leg"] == 1]
    m["turn_at_m"] = round(tp[0]["leg_dist_m"], 2) if tp else None
    t1 = [r for r in R if r["state"] == "t1_turn" and r["leg"] == 1]
    bperp = t1[0]["target_deg"] if t1 else None
    m["b_perp_deg"] = bperp
    # ---- open sea
    l2 = [r for r in R if r["state"] == "l2_go"]
    m["open_sea_m"] = round(max([r["leg_dist_m"] for r in l2], default=0), 1)
    offs, dirs, rear, front, n_row, n_win = [], [], [], [], 0, 0
    for r in l2:
        d2 = r["leg_dist_m"]; sc = L(r)
        if 10 <= d2 <= 26:
            n_win += 1
            x = F.row_fit(sc, 270, 70, 1.5, 9.0, 0.25, 5, 4.0, 1.0)
            if x and x[1] < 0 and 4.5 <= -x[1] <= 8.5:
                n_row += 1; offs.append(-x[1]); dirs.append(w180(r["yaw"] - x[0]))
        if 2 <= d2 < 14:
            x = F.sector_fit(sc, 180, 50, 1.0, 12.0, 1.5)
            if x and 2.5 <= x[1] <= d2 + 6 and bperp is not None: rear.append(F.perp(r["yaw"], x[0], bperp))
        if d2 > 18:
            x = F.sector_fit(sc, 0, 40, 2.5, 12.0, 1.5)
            if x and bperp is not None:
                h = F.perp(r["yaw"], x[0], bperp)
                if abs(w180(h - bperp)) < 15: front.append(h)
    m["row_lock_pct"] = round(100 * n_row / n_win) if n_win else None
    m["row_offset_m"] = med(offs); m["row_offset_iqr"] = iqr(offs)
    if dirs and bperp is not None:
        rd = [w180(d - bperp) for d in dirs]
        rd = [d + 180 if d < -90 else (d - 180 if d > 90 else d) for d in rd]
        m["row_dir_vs_bperp_deg"] = med(rd); m["row_dir_spread_deg"] = iqr(rd)
    m["rear_perp_vs_bperp_deg"] = med([w180(h - bperp) for h in rear]) if rear and bperp is not None else None
    m["end_perp_vs_bperp_deg"] = med([w180(h - bperp) for h in front]) if front and bperp is not None else None
    # ---- leg 3 / T3 / leg 4
    l3 = [r for r in R if r["state"] == "l3_follow"]
    m["arcade_wall_m"] = med([r["wall_d_m"] for r in l3 if r["wall_d_m"] > 0])
    ev = [r for r in R if r["event"] == "t3_turn"]
    m["arcade_len_m"] = round(ev[0]["leg_dist_m"], 1) if ev else None
    arc = [w180(r["yaw"] + x[1]) for r in l3 for x in [F.wallfit(L(r), True)] if x and x[2] < 0.04]
    if arc and ev:
        exp = w180(st.median(arc) + 90)
        l4w = []
        for r in R:
            if r["leg"] == 4 and r["state"] in ("l4_follow", "paused", "t1_settle", "manual", "end_ask"):
                x = F.row_fit(L(r), 300, 45, 0.5, 5.0, 0.12, 5, 1.0, 0.5)
                if x and x[1] < 0:
                    d = w180(r["yaw"] - x[0]); d = w180(d + 180) if abs(w180(d - exp)) > 90 else d
                    if abs(w180(d - exp)) <= 15: l4w.append((w180(d - exp), -x[1]))
        m["leg4_wall_dir_vs_expected_deg"] = med([a for a, b in l4w]); m["leg4_wall_dist_m"] = med([b for a, b in l4w])
    # ---- LiDAR daylight noise (v8): share of scans with a point closer than 35 cm, lowest near intensity (v8 logs)
    m["near_ghost_pct"] = round(100 * sum(1 for r in R if any(0 < L(r)[i] < 35 for i in range(90))) / len(R), 1)
    # v8 logs carry the near intensity in the old GPS hdop slot (255 = no near point); older logs have GPS hdop there
    v8 = any(r.get("near_inten") == 255 for r in R)
    ni = [r["near_inten"] for r in R if v8 and r.get("near_inten") not in (None, 255)]
    m["near_inten_min"] = min(ni) if ni else None
    # ---- problems
    m["failsafes"] = sum(1 for r in R if r["event"] == "failsafe")
    m["takeovers"] = sum(1 for r in R if r["event"] == "manual_on")
    m["pauses"] = sum(1 for r in R if r["event"] == "pause")
    m["end_wall_reached"] = any(r["event"] == "end_wall" for r in R)
    return m


def load():
    return json.load(open(DB)) if os.path.exists(DB) else {}


def report(db):
    keys = ["teach", "turn_at_m", "leg1_wall_m", "open_sea_m", "row_lock_pct", "row_offset_m", "row_offset_iqr",
            "row_dir_vs_bperp_deg", "row_dir_spread_deg", "rear_perp_vs_bperp_deg", "end_perp_vs_bperp_deg",
            "arcade_wall_m", "arcade_len_m", "leg4_wall_dir_vs_expected_deg", "leg4_wall_dist_m", "near_ghost_pct", "near_inten_min",
            "failsafes", "takeovers", "pauses"]
    names = sorted(db)
    print(f"{'measure':32s}" + "".join(f"{n:>13s}" for n in names))
    for k in keys:
        print(f"{k:32s}" + "".join(f"{str(db[n].get(k, '')):>13s}" for n in names))
    teach = [db[n] for n in names if db[n].get("teach")]
    src = teach or [db[n] for n in names]
    rec = {}
    v = [x["row_offset_m"] for x in src if x.get("row_offset_m")]
    if v: rec["rowOffsetM"] = round(st.median(v), 2)
    print("\nrecommended from " + ("TEACH runs " + ", ".join(n for n in names if db[n].get("teach")) if teach else "all runs (no teach run yet)"))
    for k, val in rec.items():
        cur = FIRMWARE.get(k)
        print(f"  {k}: {val}  (firmware now {cur}{'' if cur is None or abs(cur - val) < 0.05 else '  -> CHANGE'})")
    ends = [x["end_perp_vs_bperp_deg"] for x in src if x.get("end_perp_vs_bperp_deg") is not None]
    if ends: print(f"  end wall vs B-perpendicular: {', '.join(f'{e:+.1f}' for e in ends)} deg (gyro drift over the crossing + wall angle)")


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("log", nargs="?"); ap.add_argument("--name"); ap.add_argument("--teach", action="store_true")
    ap.add_argument("--report", action="store_true")
    a = ap.parse_args()
    db = load()
    if a.log:
        name = a.name or os.path.basename(a.log)
        m = measure(a.log); m["teach"] = a.teach; m["file"] = os.path.basename(a.log)
        db[name] = m; json.dump(db, open(DB, "w"), indent=1, sort_keys=True)
        print(f"stored {name}: " + ", ".join(f"{k}={v}" for k, v in m.items() if k not in ('file',)))
    if a.report or not a.log: report(db)


if __name__ == "__main__":
    main()
