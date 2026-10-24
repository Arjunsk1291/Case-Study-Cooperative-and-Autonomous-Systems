#!/usr/bin/env python3
"""R07 2D campus digital-twin prototype.

Uses the R07 known 0.30 x 0.20 m chassis footprint, 90-bin/4-degree LiDAR,
five simulated ultrasonic rays and differential-drive command interface.
All map dimensions other than the chassis footprint are schematic estimates.
"""
from __future__ import annotations
import heapq, json, math, os, sys, time
from pathlib import Path

try:
    import pygame
except ImportError:
    print("pygame is required. Run: pip install -r requirements.txt")
    raise

HERE = Path(__file__).resolve().parent
MAP_PATH = (HERE / "../terrain/dit_deggendorf_b_to_h.json").resolve()
TAU = 2 * math.pi

def clamp(v, lo, hi):
    return max(lo, min(hi, v))

def angle_delta(target, current):
    return (target - current + math.pi) % TAU - math.pi

def point_rect_distance(x, y, r):
    dx = max(r["x"] - r["w"]/2 - x, 0.0, x - (r["x"] + r["w"]/2))
    dy = max(r["y"] - r["h"]/2 - y, 0.0, y - (r["y"] + r["h"]/2))
    return math.hypot(dx, dy)

def point_blocked(x, y, buildings, inflate=0.0):
    for b in buildings:
        if (b["x"] - b["w"]/2 - inflate <= x <= b["x"] + b["w"]/2 + inflate and
            b["y"] - b["h"]/2 - inflate <= y <= b["y"] + b["h"]/2 + inflate):
            return True
    return False

def segment_clear(a, b, buildings, clearance, bounds, step=0.12):
    d = math.hypot(b[0]-a[0], b[1]-a[1])
    n = max(2, int(d / step))
    for k in range(n+1):
        f = k/n
        x, y = a[0] + f*(b[0]-a[0]), a[1] + f*(b[1]-a[1])
        if x < bounds["xmin"]+clearance or x > bounds["xmax"]-clearance or y < bounds["ymin"]+clearance or y > bounds["ymax"]-clearance:
            return False
        if point_blocked(x, y, buildings, clearance):
            return False
    return True

def astar(start, goal, buildings, bounds, clearance, grid=0.30):
    """8-connected A* with footprint-inflated buildings; output is in metres."""
    def snap(p):
        return (round((p[0]-bounds["xmin"])/grid), round((p[1]-bounds["ymin"])/grid))
    def unsnap(n):
        return (bounds["xmin"] + n[0]*grid, bounds["ymin"] + n[1]*grid)
    s, g = snap(start), snap(goal)
    def valid(n):
        x,y=unsnap(n)
        return (bounds["xmin"]+clearance <= x <= bounds["xmax"]-clearance and
                bounds["ymin"]+clearance <= y <= bounds["ymax"]-clearance and
                not point_blocked(x,y,buildings,clearance))
    # Use nearest valid cells near exact endpoints, because endpoint may be near a wall.
    def nearest_valid(node, maxr=14):
        if valid(node): return node
        for r in range(1,maxr+1):
            candidates=[]
            for dx in range(-r,r+1):
                for dy in (-r,r):
                    n=(node[0]+dx,node[1]+dy)
                    if valid(n): candidates.append(n)
            for dy in range(-r+1,r):
                for dx in (-r,r):
                    n=(node[0]+dx,node[1]+dy)
                    if valid(n): candidates.append(n)
            if candidates:
                return min(candidates,key=lambda n: math.hypot(n[0]-node[0],n[1]-node[1]))
        return None
    s, g = nearest_valid(s), nearest_valid(g)
    if s is None or g is None: return []
    dirs=[(1,0),(0,1),(-1,0),(0,-1),(1,1),(1,-1),(-1,1),(-1,-1)]
    def h(n): return math.hypot(g[0]-n[0], g[1]-n[1])
    openq=[(h(s),0,s)]
    came={}; cost={s:0.0}; closed=set()
    while openq:
        _, gc, cur=heapq.heappop(openq)
        if cur in closed: continue
        if cur==g:
            path=[cur]
            while cur in came:
                cur=came[cur]; path.append(cur)
            path.reverse()
            raw=[start]+[unsnap(n) for n in path]+[goal]
            # Greedy line-of-sight smoothing using footprint clearance.
            smooth=[raw[0]]; i=0
            while i < len(raw)-1:
                j=len(raw)-1
                while j>i+1 and not segment_clear(raw[i],raw[j],buildings,clearance,bounds):
                    j-=1
                smooth.append(raw[j]); i=j
            return smooth
        closed.add(cur)
        for dx,dy in dirs:
            nxt=(cur[0]+dx,cur[1]+dy)
            if nxt in closed or not valid(nxt): continue
            # Diagonal corner-cutting is prohibited.
            if dx and dy and (not valid((cur[0]+dx,cur[1])) or not valid((cur[0],cur[1]+dy))):
                continue
            step=math.sqrt(2) if dx and dy else 1
            ng=gc+step
            if ng < cost.get(nxt,1e99):
                cost[nxt]=ng; came[nxt]=cur
                heapq.heappush(openq,(ng+h(nxt),ng,nxt))
    return []

class R07Sim:
    def __init__(self, cfg):
        self.cfg=cfg
        self.buildings=cfg["buildings"]
        self.bounds=cfg["bounds"]
        self.rover=cfg["model"]
        self.start=cfg["start"]
        self.goal=cfg["goal"]
        self.x=float(self.start["x"]); self.y=float(self.start["y"])
        self.theta=math.radians(self.start.get("heading_deg",-90))
        self.cmd_l=0.; self.cmd_r=0.
        self.auto=False; self.finished=False; self.path=[]; self.path_i=1
        self.status="MANUAL STOP"; self.corrections=0; self.last_route_plan=0
        self.lidar=[0.0]*90; self.us=[3.0]*5
        self.actual_scan_heading=None
        self.prev_t=time.perf_counter()
        self.max_speed=0.42   # estimated max linear m/s at normalized command 0.5
        self.track=float(self.rover.get("wheel_track_m",0.17))
        self.footprint_radius=math.hypot(self.rover["rover_length_m"]/2,self.rover["rover_total_width_m"]/2)
        self.clearance=self.footprint_radius+0.10
        self.sensor_origin=float(self.rover.get("lidar_height_m",0.17))
        self.route=[]

    def hit_ray(self, x, y, ang, max_range):
        ux,uy=math.cos(ang),math.sin(ang)
        best=max_range
        # Building rectangles are hard obstacles for idealized line-of-sight rays.
        for b in self.buildings:
            xmin,xmax=b["x"]-b["w"]/2,b["x"]+b["w"]/2
            ymin,ymax=b["y"]-b["h"]/2,b["y"]+b["h"]/2
            tmin,tmax=-1e9,1e9
            if abs(ux)<1e-8:
                if x<xmin or x>xmax: continue
            else:
                t1,t2=(xmin-x)/ux,(xmax-x)/ux
                tmin=max(tmin,min(t1,t2)); tmax=min(tmax,max(t1,t2))
            if abs(uy)<1e-8:
                if y<ymin or y>ymax: continue
            else:
                t1,t2=(ymin-y)/uy,(ymax-y)/uy
                tmin=max(tmin,min(t1,t2)); tmax=min(tmax,max(t1,t2))
            t=tmin if tmin>=0 else tmax
            if t>=0 and t<best and tmin<=tmax: best=t
        # Campus extent acts as a virtual boundary to keep the robot in the mapped area.
        for val,origin,unit in [(self.bounds["xmin"],x,ux),(self.bounds["xmax"],x,ux),
                                (self.bounds["ymin"],y,uy),(self.bounds["ymax"],y,uy)]:
            if abs(unit)<1e-8: continue
            t=(val-origin)/unit
            if t>=0:
                px,py=x+t*ux,y+t*uy
                if self.bounds["xmin"]-1e-5<=px<=self.bounds["xmax"]+1e-5 and self.bounds["ymin"]-1e-5<=py<=self.bounds["ymax"]+1e-5:
                    best=min(best,t)
        return best

    def sense(self):
        # One full 360-deg scan, 90 bins at 4 deg per bin; bin 0 is robot-forward.
        self.lidar=[]
        maxr=self.rover.get("lidar_max_range_m",3.5)
        for i in range(90):
            self.lidar.append(self.hit_ray(self.x,self.y,self.theta-math.radians(4*i),maxr))
        # Ultrasound order is firmware order: front, left, right, back, front angled.
        angs=[0,math.pi/2,-math.pi/2,math.pi,math.radians(-35)]
        self.us=[self.hit_ray(self.x,self.y,self.theta+a,2.5) for a in angs]

    def plan(self):
        target=(float(self.goal["x"]),float(self.goal["y"]))
        start=(self.x,self.y)
        self.path=astar(start,target,self.buildings,self.bounds,self.clearance)
        self.path_i=1; self.last_route_plan=time.perf_counter()
        if len(self.path)>=2:
            self.status=f"ROUTE PLANNED · B → H · {len(self.path)} points"
            return True
        self.status="WAITING: no collision-free B → H path in current terrain"
        self.cmd_l=self.cmd_r=0
        return False

    def safety_blocked(self, left, right):
        # Simulated close-range stop based on LiDAR sector and fresh-style ultrasound.
        front=self.lidar[:4]
        if right>=left: pass
        return min(front+[self.us[0],self.us[4]]) < 0.30

    def drive_auto(self, dt):
        if self.finished:
            self.cmd_l=self.cmd_r=0; self.status="FINISHED · motors latched to zero"; return
        if not self.path and not self.plan():
            self.cmd_l=self.cmd_r=0; return
        # If terrain edited while running or path has gone stale, replan occasionally.
        now=time.perf_counter()
        if now-self.last_route_plan>2.0 and self.path_i < len(self.path):
            if not segment_clear((self.x,self.y),self.path[-1],self.buildings,self.clearance,self.bounds):
                self.corrections+=1
                if not self.plan(): return
        tx,ty=self.path[min(self.path_i,len(self.path)-1)]
        dist=math.hypot(tx-self.x,ty-self.y)
        if dist<0.22 and self.path_i<len(self.path)-1:
            self.path_i+=1; tx,ty=self.path[self.path_i]
            dist=math.hypot(tx-self.x,ty-self.y)
        goal_dist=math.hypot(self.goal["x"]-self.x,self.goal["y"]-self.y)
        if goal_dist<0.32:
            self.finished=True; self.auto=False; self.cmd_l=self.cmd_r=0
            self.status="FINISHED · arrived at Block H approach; motors latched to zero"; return
        desired=math.atan2(ty-self.y,tx-self.x)
        err=angle_delta(desired,self.theta)
        # Rotate in place if not generally facing the segment; otherwise advance slowly.
        v=clamp(dist*0.65,0,0.30)
        if abs(err)>0.65: v=0.0
        w=clamp(err*1.9,-1.1,1.1)
        vl=v-w*self.track/2
        vr=v+w*self.track/2
        # Convert wheel m/s to CAS Box normalized command [-0.5,0.5].
        self.cmd_l=clamp(vl/self.max_speed*0.5,-0.5,0.5)
        self.cmd_r=clamp(vr/self.max_speed*0.5,-0.5,0.5)
        if self.safety_blocked(self.cmd_l,self.cmd_r):
            self.cmd_l=self.cmd_r=0; self.corrections+=1
            self.status="RECOVERING · obstacle/clearance stop; replan"
            self.path=[]
            return
        self.status=f"NAVIGATING · Block B → Block H · waypoint {self.path_i}/{len(self.path)-1}"

    def step(self, dt):
        self.sense()
        if self.auto: self.drive_auto(dt)
        else:
            self.status="FINISHED · motors latched to zero" if self.finished else "MANUAL · stopped" if self.cmd_l==self.cmd_r==0 else "MANUAL DRIVE"
        # Differential-drive kinematics; no encoder noise model unless added later.
        vl=self.cmd_l/0.5*self.max_speed
        vr=self.cmd_r/0.5*self.max_speed
        v=(vl+vr)/2
        w=(vr-vl)/max(self.track,0.05)
        nx=self.x+v*math.cos(self.theta)*dt
        ny=self.y+v*math.sin(self.theta)*dt
        nt=(self.theta+w*dt+math.pi)%TAU-math.pi
        if not point_blocked(nx,ny,self.buildings,self.footprint_radius) and (
            self.bounds["xmin"]+self.footprint_radius < nx < self.bounds["xmax"]-self.footprint_radius and
            self.bounds["ymin"]+self.footprint_radius < ny < self.bounds["ymax"]-self.footprint_radius):
            self.x,self.y,self.theta=nx,ny,nt
        elif abs(v)>1e-5:
            self.cmd_l=self.cmd_r=0
            if self.auto:
                self.status="RECOVERING · collision boundary; stopping and replanning"
                self.path=[]; self.corrections+=1
        self.sense()

def main():
    if not MAP_PATH.exists():
        raise FileNotFoundError(f"Terrain file not found: {MAP_PATH}")
    cfg=json.loads(MAP_PATH.read_text(encoding="utf-8"))
    pygame.init()
    pygame.display.set_caption("CAS Box B3 / R07 — TH Deggendorf Block B → H Simulation")
    screen=pygame.display.set_mode((1200,820),pygame.RESIZABLE)
    clock=pygame.time.Clock()
    sim=R07Sim(cfg)
    zoom=20.0; panx=650.; pany=420.
    font=pygame.font.SysFont("dejavusans",16)
    small=pygame.font.SysFont("dejavusans",13)
    running=True
    def screen_xy(x,y):
        return int(panx+x*zoom),int(pany-y*zoom)
    def world_xy(sx,sy):
        return (sx-panx)/zoom,(pany-sy)/zoom
    def draw_text(txt,xy,color=(230,237,245),f=None):
        screen.blit((f or font).render(txt,True,color),xy)
    def draw_rect_world(b,color):
        sx,sy=screen_xy(b["x"]-b["w"]/2,b["y"]+b["h"]/2)
        pygame.draw.rect(screen,color,(sx,sy,int(b["w"]*zoom),int(b["h"]*zoom)),border_radius=5)
        pygame.draw.rect(screen,(100,113,132),(sx,sy,int(b["w"]*zoom),int(b["h"]*zoom)),2,border_radius=5)
        label=f'{b["name"]}'
        label_y=sy+int(b["h"]*zoom/2)-8
        draw_text(label,(sx+int(b["w"]*zoom/2)-font.size(label)[0]//2,label_y),(240,244,248))
    while running:
        dt=min(clock.tick(60)/1000.0,0.05)
        for ev in pygame.event.get():
            if ev.type==pygame.QUIT: running=False
            elif ev.type==pygame.VIDEORESIZE: screen=pygame.display.set_mode(ev.size,pygame.RESIZABLE)
            elif ev.type==pygame.MOUSEWHEEL: zoom=clamp(zoom*(1.1 if ev.y>0 else 1/1.1),8,60)
            elif ev.type==pygame.MOUSEBUTTONDOWN and ev.button==1:
                mx,my=ev.pos; wx,wy=world_xy(mx,my)
                sim.cmd_l=sim.cmd_r=0
                sim.auto=False
                sim.status="MANUAL · stopped"
            elif ev.type==pygame.KEYDOWN:
                if ev.key==pygame.K_ESCAPE: running=False
                elif ev.key==pygame.K_SPACE:
                    sim.cmd_l=sim.cmd_r=0; sim.auto=False
                    if not sim.finished: sim.status="STOPPED BY OPERATOR"
                elif ev.key==pygame.K_a and not sim.finished:
                    sim.cmd_l=sim.cmd_r=0; sim.auto=True; sim.path=[]; sim.plan()
                elif ev.key==pygame.K_p:
                    sim.auto=False; sim.cmd_l=sim.cmd_r=0; sim.status="PAUSED · zero command"
                elif ev.key==pygame.K_r:
                    sim.__init__(cfg); sim.status="RESET · ready at Block B"
                elif ev.key in (pygame.K_PLUS,pygame.K_EQUALS,pygame.K_KP_PLUS): zoom=min(zoom*1.12,60)
                elif ev.key in (pygame.K_MINUS,pygame.K_KP_MINUS): zoom=max(zoom/1.12,8)
        keys=pygame.key.get_pressed()
        if not sim.auto and not sim.finished:
            fwd=keys[pygame.K_w] or keys[pygame.K_UP]
            rev=keys[pygame.K_s] or keys[pygame.K_DOWN]
            left=keys[pygame.K_a] or keys[pygame.K_LEFT]
            right=keys[pygame.K_d] or keys[pygame.K_RIGHT]
            if fwd and not rev: sim.cmd_l=sim.cmd_r=0.22
            elif rev and not fwd: sim.cmd_l=sim.cmd_r=-0.16
            elif left and not right: sim.cmd_l=-0.14; sim.cmd_r=0.14
            elif right and not left: sim.cmd_l=0.14; sim.cmd_r=-0.14
            else: sim.cmd_l=sim.cmd_r=0
        sim.step(dt)
        screen.fill((15,20,29))
        # Terrain bounds.
        sx,sy=screen_xy(cfg["bounds"]["xmin"],cfg["bounds"]["ymax"])
        bw=int((cfg["bounds"]["xmax"]-cfg["bounds"]["xmin"])*zoom)
        bh=int((cfg["bounds"]["ymax"]-cfg["bounds"]["ymin"])*zoom)
        pygame.draw.rect(screen,(27,37,46),(sx,sy,bw,bh))
        # Grid as a visual scale reference only.
        for x in range(math.ceil(cfg["bounds"]["xmin"]),math.floor(cfg["bounds"]["xmax"])+1,2):
            ax,ay=screen_xy(x,cfg["bounds"]["ymin"]); bx,by=screen_xy(x,cfg["bounds"]["ymax"])
            pygame.draw.line(screen,(39,50,61),(ax,ay),(bx,by),1)
        for y in range(math.ceil(cfg["bounds"]["ymin"]),math.floor(cfg["bounds"]["ymax"])+1,2):
            ax,ay=screen_xy(cfg["bounds"]["xmin"],y); bx,by=screen_xy(cfg["bounds"]["xmax"],y)
            pygame.draw.line(screen,(39,50,61),(ax,ay),(bx,by),1)
        # Walkable terrain and building footprints.
        for b in sim.buildings:
            draw_rect_world(b,(66,79,96))
        # Planned route.
        if len(sim.path)>1:
            pts=[screen_xy(x,y) for x,y in sim.path]
            pygame.draw.lines(screen,(52,198,170),False,pts,3)
            for p in pts: pygame.draw.circle(screen,(52,198,170),p,4)
        # Start / target markers.
        for p,color,label in [(sim.start,(60,200,120),"START B"),(sim.goal,(244,96,91),"GOAL H")]:
            px,py=screen_xy(p["x"],p["y"])
            pygame.draw.circle(screen,color,(px,py),7)
            draw_text(label,(px+10,py-21),color,small)
        # LiDAR spokes (only 90 sparse simulated rays, attenuated for legibility).
        for i,d in enumerate(sim.lidar):
            if d<=0: continue
            a=sim.theta-math.radians(4*i)
            ex,ey=screen_xy(sim.x+d*math.cos(a),sim.y+d*math.sin(a))
            bx,by=screen_xy(sim.x,sim.y)
            pygame.draw.line(screen,(46,95,112),(bx,by),(ex,ey),1)
            if i%3==0: pygame.draw.circle(screen,(63,202,225),(ex,ey),2)
        # Robot footprint rotated in map coordinates.
        pts=[]
        L=sim.rover["rover_length_m"]/2; W=sim.rover["rover_total_width_m"]/2
        for fx,fy in [(L,W),(L,-W),(-L,-W),(-L,W)]:
            wx=sim.x+fx*math.cos(sim.theta)-fy*math.sin(sim.theta)
            wy=sim.y+fx*math.sin(sim.theta)+fy*math.cos(sim.theta)
            pts.append(screen_xy(wx,wy))
        pygame.draw.polygon(screen,(231,169,74),pts)
        pygame.draw.polygon(screen,(255,224,167),pts,2)
        # CAS Box enclosure mounted on top of the known rover footprint.
        # Box dimensions are placeholders until measured; render it in the same local frame.
        bL=sim.rover.get("box_length_m",0.12)/2
        bW=sim.rover.get("box_width_m",0.10)/2
        bpts=[]
        for fx,fy in [(bL,bW),(bL,-bW),(-bL,-bW),(-bL,bW)]:
            wx=sim.x+fx*math.cos(sim.theta)-fy*math.sin(sim.theta)
            wy=sim.y+fx*math.sin(sim.theta)+fy*math.cos(sim.theta)
            bpts.append(screen_xy(wx,wy))
        pygame.draw.polygon(screen,(80,157,235),bpts)
        pygame.draw.polygon(screen,(218,238,255),bpts,1)
        rx,ry=screen_xy(sim.x,sim.y)
        hx,hy=screen_xy(sim.x+0.25*math.cos(sim.theta),sim.y+0.25*math.sin(sim.theta))
        pygame.draw.line(screen,(255,245,220),(rx,ry),(hx,hy),3)
        pygame.draw.circle(screen,(255,245,220),(rx,ry),3)
        # HUD.
        pygame.draw.rect(screen,(9,13,21),(0,0,screen.get_width(),122))
        draw_text("CAS BOX B3 / R07  ·  TH DEGGENDORF BLOCK B → H",(18,12),(120,194,255))
        draw_text("SCHEMATIC MAP — building footprints / walkway scale are approximate, edit terrain JSON",(18,39),(255,198,120),small)
        draw_text(f"State: {sim.status}",(18,61),(222,231,240),small)
        draw_text(f"Pose: x={sim.x:.2f} m  y={sim.y:.2f} m  yaw={math.degrees(sim.theta):.1f}°   cmd L/R={sim.cmd_l:.2f}/{sim.cmd_r:.2f}   replans={sim.corrections}",(18,82),(183,201,218),small)
        draw_text("A run B→H   P pause   SPACE stop   R reset   arrows/WASD manual   mouse wheel zoom   ESC quit",(18,101),(183,201,218),small)
        # sensor summary panel
        panel_x=screen.get_width()-245
        pygame.draw.rect(screen,(24,32,44),(panel_x,139,230,166),border_radius=8)
        draw_text("SIMULATED SENSORS",(panel_x+12,151),(120,194,255),small)
        draw_text(f"LiDAR: 90 bins × 4° / max {cfg['model'].get('lidar_max_range_m',3.5):.1f} m",(panel_x+12,175),f=small)
        draw_text(f"Front {sim.us[0]:.2f} m",(panel_x+12,199),f=small)
        draw_text(f"Left {sim.us[1]:.2f} m   Right {sim.us[2]:.2f} m",(panel_x+12,221),f=small)
        draw_text(f"Back {sim.us[3]:.2f} m   Angled {sim.us[4]:.2f} m",(panel_x+12,243),f=small)
        draw_text(f"Footprint {cfg['model']['rover_length_m']:.2f} × {cfg['model']['rover_total_width_m']:.2f} m",(panel_x+12,265),f=small)
        draw_text(f"Clearance model: {sim.clearance:.2f} m",(panel_x+12,287),f=small)
        pygame.display.flip()
    pygame.quit()

if __name__=="__main__":
    main()
