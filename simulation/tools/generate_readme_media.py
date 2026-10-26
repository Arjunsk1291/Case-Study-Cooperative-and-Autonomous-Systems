#!/usr/bin/env python3
"""Build README screenshots and a short animated walkthrough from the detailed R07 campus terrain JSON."""
from __future__ import annotations
import argparse, json, math
from pathlib import Path
import matplotlib
matplotlib.use("Agg")
import matplotlib.pyplot as plt
from matplotlib.animation import FuncAnimation, PillowWriter, FFMpegWriter
from matplotlib.patches import Rectangle, Circle, FancyArrowPatch
from mpl_toolkits.mplot3d.art3d import Poly3DCollection

C={
 "granite_slabs":"#b8b8b2","cobblestone":"#827e72","smooth_concrete":"#d1d0c9",
 "timber_deck":"#b8875d","grass":"#73936c","warm_pink":"#bf887f",
 "glass_grey":"#8c9da0","dark_forum":"#626c74","route":"#00a98f"
}
def rect(ax,x,y,w,h,fc,ec="#596165",lw=.6,z=2,alpha=1,hatch=None):
    p=Rectangle((x-w/2,y-h/2),w,h,facecolor=fc,edgecolor=ec,linewidth=lw,zorder=z,alpha=alpha,hatch=hatch)
    ax.add_patch(p); return p
def in_view(x,y,w,h,extent):
    if extent is None: return True
    xmin,xmax,ymin,ymax=extent
    return x+w/2>=xmin and x-w/2<=xmax and y+h/2>=ymin and y-h/2<=ymax
def tree_points(t):
    out=[]
    if "x_start" in t and "x_end" in t and "y" in t:
        n=max(1,int((t["x_end"]-t["x_start"])/t.get("spacing",6)))
        out=[(t["x_start"]+(t["x_end"]-t["x_start"])*i/n,t["y"]) for i in range(n+1)]
    elif "x" in t and "y_start" in t and "y_end" in t:
        lo,hi=sorted([t["y_start"],t["y_end"]]); n=max(1,int((hi-lo)/t.get("spacing",6)))
        out=[(t["x"],lo+(hi-lo)*i/n) for i in range(n+1)]
    elif all(k in t for k in ["x_start","x_end","y_start","y_end"]):
        xs=[]; x=t["x_start"]
        while x<=t["x_end"]+1e-6: xs.append(x); x+=t.get("spacing_x",5)
        ys=[]; y=t["y_start"]
        while y<=t["y_end"]+1e-6: ys.append(y); y+=t.get("spacing_y",4)
        out=[(x,y) for x in xs for y in ys]
    return out

def draw_plan(ax,cfg,extent=None,labels=True,progress=None,rover_xy=None,rover_heading=0):
    bnd=cfg["bounds"]; ax.set_facecolor("#e8ebe7")
    # Paved/green surface layers
    for s in cfg.get("surface_zones",[]):
        if not in_view(s["x"],s["y"],s["w"],s["d"],extent): continue
        rect(ax,s["x"],s["y"],s["w"],s["d"],C.get(s["type"],"#bbb"),ec="#6c7473",lw=.45,z=0,alpha=.91)
        if s["type"]=="granite_slabs" and s["w"]>3 and s["d"]>3:
            x=s["x"]-s["w"]/2+.8
            while x<s["x"]+s["w"]/2:
                ax.plot([x,x],[s["y"]-s["d"]/2,s["y"]+s["d"]/2],color="#7e8580",lw=.22,alpha=.42,zorder=1); x+=.8
            y=s["y"]-s["d"]/2+.6
            while y<s["y"]+s["d"]/2:
                ax.plot([s["x"]-s["w"]/2,s["x"]+s["w"]/2],[y,y],color="#7e8580",lw=.22,alpha=.42,zorder=1); y+=.6
        if s["type"]=="cobblestone" and s["w"]>3 and s["d"]>1.5:
            y=s["y"]-s["d"]/2+.2
            while y<s["y"]+s["d"]/2:
                ax.plot([s["x"]-s["w"]/2,s["x"]+s["w"]/2],[y,y],color="#514f49",lw=.45,alpha=.6,zorder=1); y+=.25
    for b in cfg.get("buildings",[]):
        if not in_view(b["x"],b["y"],b["w"],b["d"],extent): continue
        rect(ax,b["x"],b["y"],b["w"],b["d"],C.get(b.get("facade"),"#bf887f"),ec="#514849",lw=1,z=3)
        if labels: ax.text(b["x"],b["y"],b["name"],ha="center",va="center",fontsize=8,fontweight="bold",color="#17202a",zorder=4)
        ax.plot([b["x"]-b["w"]*.4,b["x"]+b["w"]*.4],[b["y"]-b["d"]*.36]*2,color="#334b55",lw=1,zorder=4)
    for f in cfg.get("furniture",[]):
        w,h=f.get("w",.5),f.get("d",.5)
        if not in_view(f["x"],f["y"],w,h,extent): continue
        is_p=f.get("type")=="pavilion"
        rect(ax,f["x"],f["y"],w,h,"#6d4c3d" if is_p else "#4c555b",ec="#202a30",lw=.8,z=6)
        if is_p: ax.text(f["x"],f["y"],"P\\nCAMPUS STORE",ha="center",va="center",fontsize=7,color="white",fontweight="bold",zorder=7)
    for w in cfg.get("water_features",[]):
        if not in_view(w["x"],w["y"],w["w"],w["d"],extent): continue
        rect(ax,w["x"],w["y"],w["w"],w["d"],"#68acbc",ec="#3b6a78",lw=1.6,z=5)
        rect(ax,w["x"],w["y"],w["w"]-.4,w["d"]-.4,"#8bc9d5",ec="#d8eeee",lw=.5,z=6)
        if labels: ax.text(w["x"],w["y"],"WATER\\nNO-GO",ha="center",va="center",fontsize=5,color="#183843",zorder=7)
    for ch in cfg.get("drainage_channels",[]):
        ax.plot([ch["x1"],ch["x2"]],[ch["y1"],ch["y2"]],color="#263e49",lw=max(1.2,ch["width"]*6),zorder=5)
        if in_view((ch["x1"]+ch["x2"])/2,(ch["y1"]+ch["y2"])/2,abs(ch["x2"]-ch["x1"])+1,abs(ch["y2"]-ch["y1"])+1,extent):
            ax.plot([ch["x1"],ch["x2"]],[ch["y1"],ch["y2"]],color="#72a7b7",lw=max(.6,ch["width"]*2.5),zorder=6)
    for br in cfg.get("bridges",[]):
        if in_view(br["x"],br["y"],br["w"],br["d"],extent):
            rect(ax,br["x"],br["y"],br["w"],br["d"],"#d6d3c8",ec="#59646a",lw=.8,z=8)
    for r in cfg.get("ramps",[]):
        x,y,w,h=r["x"],r["y_start"]+r["length"]/2,r["width"],r["length"]
        if not in_view(x,y,w,h,extent): continue
        rect(ax,x,y,w,h,"#e7c66b",ec="#786c45",lw=.9,z=7,alpha=.92)
        ax.annotate("",xy=(x,y+h*.18),xytext=(x,y-h*.18),arrowprops=dict(arrowstyle="->",color="#6d623f",lw=1),zorder=9)
    for s in cfg.get("stairs",[]):
        run=s["steps"]*s["tread"]
        if not in_view(s["x"],s["y"],s["width"],run,extent): continue
        rect(ax,s["x"],s["y"],s["width"],run,"#c7bcb0",ec="#9b483f",lw=1,z=7,hatch="////")
        for i in range(s["steps"]):
            y=s["y"]-run/2+(i+.5)*s["tread"]
            ax.plot([s["x"]-s["width"]/2,s["x"]+s["width"]/2],[y,y],color="#8f433b",lw=.6,zorder=8)
        ax.text(s["x"],s["y"],"STAIRS\\nNO-GO",ha="center",va="center",fontsize=5.5,color="#782e29",fontweight="bold",zorder=9)
    pd=cfg.get("photo_derived_features",{})
    for a in pd.get("covered_arcades",[]):
        if extent and not (extent[0]-2<a["column_x"]<extent[1]+2): continue
        y0,y1=a["y_start"],a["y_end"]; n=max(1,int(abs(y1-y0)/a.get("column_spacing_m",3.6)))
        ax.plot([a["wall_x"]]*2,[min(y0,y1),max(y0,y1)],color="#bd887f",lw=4,zorder=6)
        for i in range(n+1):
            y=y0+(y1-y0)*i/n; ax.add_patch(Circle((a["column_x"],y),a.get("column_radius_m",.22),facecolor="#7d878b",edgecolor="#41484b",lw=.6,zorder=9))
    for t in pd.get("tactile_guides",[]):
        if extent and not (extent[0]-1<t["x_center"]<extent[1]+1): continue
        for k in range(t.get("strips",4)):
            x=t["x_center"]+(k-(t.get("strips",4)-1)/2)*t.get("strip_spacing_m",.052)
            ax.plot([x,x],[t["y_start"],t["y_end"]],color="#3f5057",lw=1.1,zorder=8)
    for pad in pd.get("tactile_warning_pads",[]):
        if extent and not in_view(pad["x"],pad["y"],pad["width_m"],pad["depth_m"],extent): continue
        dx=pad["width_m"]/max(1,pad["columns"]-1); dy=pad["depth_m"]/max(1,pad["rows"]-1)
        for i in range(pad["columns"]):
            for j in range(pad["rows"]):
                ax.add_patch(Circle((pad["x"]-pad["width_m"]/2+i*dx,pad["y"]-pad["depth_m"]/2+j*dy),pad.get("stud_radius_m",.024),facecolor="#566167",edgecolor="#faf9f2",lw=.15,zorder=10))
    for d in pd.get("linear_drains",[]):
        if d.get("axis")=="x":
            ax.plot([d["x"]-d["length_m"]/2,d["x"]+d["length_m"]/2],[d["y"]]*2,color="#26363e",lw=2,zorder=8)
        else: ax.plot([d["x"]]*2,[d["y"]-d["length_m"]/2,d["y"]+d["length_m"]/2],color="#26363e",lw=2,zorder=8)
    for k in pd.get("thresholds",[]):
        if k.get("axis")!="x": continue
        gc=k.get("gap_center_x",(k["x_min"]+k["x_max"])/2); gw=k.get("gap_width_m",1.1); y=k["y"]
        ax.plot([k["x_min"],gc-gw/2],[y,y],color="#aaa79d",lw=5,zorder=9,solid_capstyle="butt")
        ax.plot([gc+gw/2,k["x_max"]],[y,y],color="#aaa79d",lw=5,zorder=9,solid_capstyle="butt")
        ax.plot([gc-gw/2,gc+gw/2],[y,y],color="#e2c26d",lw=3,zorder=9,solid_capstyle="butt")
    for wp in pd.get("wet_patches",[]):
        if in_view(wp["x"],wp["y"],wp["w"],wp["d"],extent): rect(ax,wp["x"],wp["y"],wp["w"],wp["d"],"#438fa3",ec="none",lw=0,z=6,alpha=.28)
    for light in pd.get("ground_lights_and_covers",[]):
        if extent and not in_view(light["x"],light["y"],.5,.5,extent): continue
        ax.add_patch(Circle((light["x"],light["y"]),light.get("outer_radius_m",.19),facecolor="#aebbb7",edgecolor="#46545a",lw=.5,zorder=10))
    for sign in pd.get("signboards",[]):
        ax.add_patch(Circle((sign["x"],sign["y"]),.25,facecolor="#26343d",edgecolor="white",lw=.4,zorder=11))
        ax.text(sign["x"],sign["y"],sign["text"].split()[-1],color="white",fontsize=5,ha="center",va="center",zorder=12)
    for f in pd.get("facade_runs",[]):
        rect(ax,f["x"],(f["y_start"]+f["y_end"])/2,f.get("thickness_m",.25),abs(f["y_end"]-f["y_start"]),C.get(f.get("finish"),"#bc887e"),ec="#644b46",lw=.5,z=7)
    for t in cfg.get("trees",[]):
        for x,y in tree_points(t):
            if in_view(x,y,1.2,1.2,extent): ax.add_patch(Circle((x,y),.55,facecolor="#4f7b4f",edgecolor="#345a39",lw=.4,alpha=.86,zorder=5))
    for row in cfg.get("bollard_rows",[]):
        if "x_start" in row:
            n=max(1,int((row["x_end"]-row["x_start"])/row["spacing"])); pts=[(row["x_start"]+(row["x_end"]-row["x_start"])*i/n,row["y"]) for i in range(n+1)]
        else:
            n=max(1,int(abs(row["y_end"]-row["y_start"])/row["spacing"])); pts=[(row["x"],row["y_start"]+(row["y_end"]-row["y_start"])*i/n) for i in range(n+1)]
        for x,y in pts:
            if not extent or in_view(x,y,.2,.2,extent): ax.add_patch(Circle((x,y),row.get("radius",.07),facecolor="#32383a",edgecolor="#d9d8cf",lw=.2,zorder=10))
    pts=[(p["x"],p["y"]) for p in cfg["route"]]
    ax.plot([p[0] for p in pts],[p[1] for p in pts],color=C["route"],lw=2.5,zorder=15,solid_capstyle="round")
    important={0,3,5,7,8,11,13,15}
    for i,p in enumerate(cfg["route"]):
        if extent and not (extent[0]<=p["x"]<=extent[1] and extent[2]<=p["y"]<=extent[3]): continue
        col="#2478bd" if p.get("type")=="start" else "#cc5448" if p.get("type")=="goal" else "#079a86"
        ax.scatter(p["x"],p["y"],s=28,c=col,edgecolors="white",linewidths=.5,zorder=17)
        if labels and (extent is not None or i in important):
            label=p["name"].replace("_"," ")
            ax.annotate(label,(p["x"],p["y"]),xytext=(3,4),textcoords="offset points",fontsize=5.7,color="#16353a",zorder=20,bbox=dict(boxstyle="round,pad=.12",fc="white",ec="none",alpha=.7))
    if rover_xy:
        x,y=rover_xy; ax.add_patch(Circle((x,y),.48,facecolor="#efa64f",edgecolor="#25343c",lw=1.4,zorder=30))
        ax.add_patch(FancyArrowPatch((x,y),(x+.85*math.cos(rover_heading),y+.85*math.sin(rover_heading)),arrowstyle="-|>",mutation_scale=13,color="#25343c",lw=1.6,zorder=31))
    if progress: ax.text(.02,.02,progress,transform=ax.transAxes,fontsize=9,ha="left",va="bottom",color="#162b32",bbox=dict(boxstyle="round,pad=.4",fc="white",ec="#9aa6a6",alpha=.94),zorder=32)
    if extent: ax.set_xlim(extent[0],extent[1]); ax.set_ylim(extent[2],extent[3])
    else: ax.set_xlim(bnd["xmin"],bnd["xmax"]); ax.set_ylim(bnd["ymin"],bnd["ymax"])
    ax.set_aspect("equal",adjustable="box"); ax.grid(color="#aeb6b2",lw=.35,alpha=.45)
    ax.set_xlabel("Local X (m; schematic)",fontsize=8); ax.set_ylabel("Local Y (m; schematic)",fontsize=8)

def make_3d(cfg,out):
    fig=plt.figure(figsize=(12,8),facecolor="#edf1f3"); ax=fig.add_subplot(111,projection="3d",facecolor="#edf1f3")
    bnd=cfg["bounds"]; z=-.15
    ax.add_collection3d(Poly3DCollection([[(bnd["xmin"],bnd["ymin"],z),(bnd["xmax"],bnd["ymin"],z),(bnd["xmax"],bnd["ymax"],z),(bnd["xmin"],bnd["ymax"],z)]],facecolors="#c4c9c2",edgecolors="#818a86",linewidths=.4))
    for s in cfg.get("surface_zones",[]):
        x1,x2=s["x"]-s["w"]/2,s["x"]+s["w"]/2; y1,y2=s["y"]-s["d"]/2,s["y"]+s["d"]/2; z=s.get("z",0)
        ax.add_collection3d(Poly3DCollection([[(x1,y1,z),(x2,y1,z),(x2,y2,z),(x1,y2,z)]],facecolors=C.get(s["type"],"#bbb"),edgecolors="none",alpha=.8))
    for b in cfg.get("buildings",[]):
        x,y,w,d,h,base=b["x"],b["y"],b["w"],b["d"],b["height"],b.get("base_z",0); x1,x2=x-w/2,x+w/2; y1,y2=y-d/2,y+d/2; z1,z2=base,base+h
        faces=[[(x1,y1,z1),(x2,y1,z1),(x2,y1,z2),(x1,y1,z2)],[(x2,y1,z1),(x2,y2,z1),(x2,y2,z2),(x2,y1,z2)],[(x2,y2,z1),(x1,y2,z1),(x1,y2,z2),(x2,y2,z2)],[(x1,y2,z1),(x1,y1,z1),(x1,y1,z2),(x1,y2,z2)],[(x1,y1,z2),(x2,y1,z2),(x2,y2,z2),(x1,y2,z2)]]
        ax.add_collection3d(Poly3DCollection(faces,facecolors=C.get(b.get("facade"),"#bf887f"),edgecolors="#4d5255",linewidths=.35,alpha=.96))
        ax.text(x,y,z2+.8,b["name"],ha="center",fontsize=7,color="#25333c")
    for w in cfg.get("water_features",[]):
        x1,x2=w["x"]-w["w"]/2,w["x"]+w["w"]/2; y1,y2=w["y"]-w["d"]/2,w["y"]+w["d"]/2
        ax.add_collection3d(Poly3DCollection([[(x1,y1,.02),(x2,y1,.02),(x2,y2,.02),(x1,y2,.02)]],facecolors="#4eafca",edgecolors="#365e6e",alpha=.9))
    pts=cfg["route"]; ax.plot([p["x"] for p in pts],[p["y"] for p in pts],[p.get("z",0)+.35 for p in pts],color="#00a98f",lw=2.5)
    ax.scatter([pts[0]["x"],pts[-1]["x"]],[pts[0]["y"],pts[-1]["y"]],[pts[0]["z"]+.4,pts[-1]["z"]+.4],c=["#2776bd","#c64e46"],s=42)
    ax.set_title("TH Deggendorf · R07 Block B → Block H terrain (schematic 3D)",fontsize=14,pad=16)
    ax.set_xlabel("Local X (m)"); ax.set_ylabel("Local Y (m)"); ax.set_zlabel("Height (m; estimated)")
    ax.view_init(elev=32,azim=-59); ax.set_xlim(bnd["xmin"],bnd["xmax"]); ax.set_ylim(bnd["ymin"],bnd["ymax"]); ax.set_zlim(0,16); ax.set_box_aspect((1.3,1,.45))
    fig.text(.5,.02,"Building footprints, terrain levels and route coordinates are approximate—not a surveyed map.",ha="center",fontsize=9,color="#7c413a")
    fig.tight_layout(rect=(0,.04,1,1)); fig.savefig(out,dpi=110,bbox_inches="tight"); plt.close(fig)

def make_corridor_detail(cfg,out):
    fig,ax=plt.subplots(figsize=(12,8),facecolor="#f3f5f3")
    draw_plan(ax,cfg,extent=(-9,10,23,41),labels=False)
    ax.text(-8.6,40.3,"COBBLE / SLAB TRANSITION",fontsize=9,fontweight="bold",color="#25343c")
    ax.text(-8.6,23.3,"*Threshold centre crossing and all dimensions are hypotheses",fontsize=8,color="#7c413a")
    ax.set_title("Photo-enriched corridor: cobbles, Campus Store, threshold, columns and drains",fontsize=12,fontweight="bold")
    fig.tight_layout(); fig.savefig(out,dpi=125); plt.close(fig)

def route_samples(route,n=44):
    seg=[]; total=0
    for a,b in zip(route,route[1:]):
        d=math.hypot(b["x"]-a["x"],b["y"]-a["y"]); seg.append(d); total+=d
    samples=[]
    for i in range(n):
        distance=total*i/max(1,n-1); acc=0
        for j,d in enumerate(seg):
            if distance<=acc+d or j==len(seg)-1:
                q=0 if d==0 else (distance-acc)/d; a,b=route[j],route[j+1]
                x=a["x"]+q*(b["x"]-a["x"]); y=a["y"]+q*(b["y"]-a["y"])
                samples.append((x,y,j,distance,total)); break
            acc+=d
    return samples

def make_video(cfg,out):
    route=cfg["route"]; samples=route_samples(route,44)
    fig,ax=plt.subplots(figsize=(10,7),facecolor="#f3f5f3")
    draw_plan(ax,cfg,labels=False)
    ax.set_title("WAVE ROVER R07 — Block B → Block H | Terrain walkthrough",fontsize=13,fontweight="bold",pad=10)
    x,y=samples[0][0],samples[0][1]
    rover=Circle((x,y),.5,facecolor="#efa64f",edgecolor="#24303a",lw=1.4,zorder=30); ax.add_patch(rover)
    arrow=FancyArrowPatch((x,y),(x+.9,y),arrowstyle="-|>",mutation_scale=13,color="#25343c",lw=1.6,zorder=31); ax.add_patch(arrow)
    hud=ax.text(.02,.02,"",transform=ax.transAxes,fontsize=8.5,ha="left",va="bottom",color="#162b32",bbox=dict(boxstyle="round,pad=.4",fc="white",ec="#9aa6a6",alpha=.94),zorder=32)
    def update(i):
        x,y,seg,distance,total=samples[i]; rover.center=(x,y)
        if i<len(samples)-1: dx,dy=samples[i+1][0]-x,samples[i+1][1]-y
        else: dx,dy=1,0
        norm=max(1e-6,math.hypot(dx,dy)); dx,dy=dx/norm*.9,dy/norm*.9
        arrow.set_positions((x,y),(x+dx,y+dy))
        cp=min(seg+1,len(route)-1); label=route[cp]["name"].replace("_"," ")
        hud.set_text(f"SIMULATION ONLY  |  Checkpoint {cp+1}/{len(route)}: {label}\\nRoute progress {100*i/(len(samples)-1):.0f}%  |  relative path {distance:.1f}/{total:.1f} m")
        return rover,arrow,hud
    anim=FuncAnimation(fig,update,frames=len(samples),interval=160,blit=True,repeat=True)
    anim.save(out/"route_walkthrough.gif",writer=PillowWriter(fps=6),dpi=52)
    anim.save(out/"route_walkthrough.mp4",writer=FFMpegWriter(fps=10,bitrate=650,metadata={"title":"TH Deggendorf R07 Block B to H terrain simulation"}),dpi=62)
    plt.close(fig)

def main():
    parser=argparse.ArgumentParser(); parser.add_argument("--terrain",default=str(Path(__file__).resolve().parents[1]/"terrain"/"dit_deggendorf_b_to_h_detailed.json")); parser.add_argument("--outdir",default=str(Path(__file__).resolve().parents[1]/"media")); args=parser.parse_args()
    cfg=json.loads(Path(args.terrain).read_text()); out=Path(args.outdir); out.mkdir(parents=True,exist_ok=True)
    fig,ax=plt.subplots(figsize=(13,9),facecolor="#f3f5f3"); draw_plan(ax,cfg)
    ax.set_title("TH Deggendorf campus terrain · Block B → H candidate route",fontsize=15,fontweight="bold",pad=12)
    fig.text(.5,.015,"Schematic terrain. Building footprints, grades and route coordinates are estimated.",ha="center",fontsize=9,color="#7c413a")
    fig.tight_layout(rect=(0,.04,1,1)); fig.savefig(out/"campus_overview.png",dpi=125,bbox_inches="tight"); plt.close(fig)
    make_corridor_detail(cfg,out/"corridor_detail.png")
    make_3d(cfg,out/"campus_3d_overview.png")
    make_video(cfg,out)
    print("README media generated:",", ".join(p.name for p in sorted(out.iterdir()) if p.is_file()))
if __name__=="__main__": main()
