#!/usr/bin/env python3
"""Plot a 90-bin R07 LiDAR signature. This is not SLAM or a metric occupancy map."""
import argparse, csv, json, math, statistics
from pathlib import Path
import matplotlib.pyplot as plt

HERE=Path(__file__).resolve().parent
DEFAULT=(HERE/"../../cas_box"/"route_waypoint_signatures.json").resolve()

def find_signature(root,label):
    # Route signature file used by this repository.
    if isinstance(root,dict) and "waypoints" in root:
        for wp in root["waypoints"]:
            if wp.get("label")==label:
                return wp["lidar_median_cm"]
        raise ValueError(f"label {label!r} not found in route waypoint signatures")
    # Raw investigation data can have events[] with sensors.lidar.d.
    events=root.get("events",[]) if isinstance(root,dict) else []
    matching=[]
    for e in events:
        if label and e.get("label")!=label: continue
        d=e.get("sensors",{}).get("lidar",{}).get("d",[])
        if isinstance(d,list) and len(d)==90: matching.append(d)
    if matching:
        result=[]
        for i in range(90):
            vals=[row[i] for row in matching if isinstance(row[i],(int,float)) and 10<=row[i]<=350]
            result.append(round(statistics.median(vals)) if vals else 0)
        print(f"Computed median from {len(matching)} matching event scans.")
        return result
    # Single diagnostic/export structure with sensors at root.dashboard.sensors
    sensors=root.get("dashboard",{}).get("sensors",root.get("sensors",{})) if isinstance(root,dict) else {}
    d=sensors.get("lidar",{}).get("d",[])
    if len(d)==90: return d
    raise ValueError("Could not find a 90-bin scan. Use the exported investigation JSON or route_waypoint_signatures.json.")

def main():
    p=argparse.ArgumentParser(description=__doc__)
    p.add_argument("json_file",nargs="?",default=str(DEFAULT),help="input route signatures or investigation/diagnostic JSON")
    p.add_argument("--label",default="END_SCAN_entrance",help="waypoint/event label; default uses final scan signature")
    p.add_argument("--out",default=".",help="output directory")
    args=p.parse_args()
    source=Path(args.json_file).expanduser().resolve()
    root=json.loads(source.read_text(encoding="utf-8"))
    ranges=find_signature(root,args.label)
    if len(ranges)!=90: raise ValueError("Expected exactly 90 LiDAR bins")
    out=Path(args.out).expanduser().resolve(); out.mkdir(parents=True,exist_ok=True)
    safe="".join(ch if ch.isalnum() or ch in "-_" else "_" for ch in args.label)
    csv_path=out/f"r07_lidar_{safe}.csv"
    with csv_path.open("w",newline="",encoding="utf-8") as f:
        w=csv.writer(f); w.writerow(["bin","angle_clockwise_deg","range_cm","valid"])
        for i,d in enumerate(ranges):
            ok=isinstance(d,(int,float)) and 10<=d<=350
            w.writerow([i,i*4,int(d) if ok else "",int(ok)])
    fig=plt.figure(figsize=(8,8))
    ax=fig.add_subplot(111,projection="polar")
    theta=[math.radians(-i*4) for i in range(90)]
    radii=[(d/100.0 if isinstance(d,(int,float)) and 10<=d<=350 else float("nan")) for d in ranges]
    ax.set_theta_zero_location("E"); ax.set_theta_direction(-1)
    ax.scatter(theta,radii,s=14)
    ax.set_ylim(0,3.5); ax.set_title(f"R07 LiDAR signature: {args.label}\n90 bins × 4°; valid ranges only")
    png_path=out/f"r07_lidar_{safe}.png"
    fig.savefig(png_path,dpi=150,bbox_inches="tight")
    print(f"CSV: {csv_path}\nPlot: {png_path}")
    print("This is a single-position polar signature. It does not infer x/y poses or build a campus occupancy map.")

if __name__=="__main__":
    main()
