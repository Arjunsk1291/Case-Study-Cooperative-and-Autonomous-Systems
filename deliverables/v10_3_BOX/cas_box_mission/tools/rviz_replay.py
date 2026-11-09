#!/usr/bin/env python3
"""rviz_replay.py - play a recorded mission log (mis_*.bin) in RViz 2. READ-ONLY: it only reads the log and publishes ROS topics;
nothing is ever sent to the box or the rover.
  source /opt/ros/jazzy/setup.bash
  python3 rviz_replay.py "../../Log_run/campus_10.2/mis_001 (4).bin" --leg 4 --from-m 18 --speed 5
  rviz2 -d rviz_replay.rviz                      (in another terminal)
Topics: /scan (LaserScan, 90 beams, REP-103: x forward, y left, counter-clockwise), /path (dead-reckoned track: leg distance + gyro),
/markers (events, the v10.3 ramp opening in green, state text), tf map -> base_link."""
import sys, os, math, time, argparse
sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import LaserScan
from nav_msgs.msg import Path
from geometry_msgs.msg import PoseStamped, TransformStamped, Point
from visualization_msgs.msg import Marker, MarkerArray
from tf2_ros import TransformBroadcaster
import decode_mission_log as D
from replay_l4_gap import find_gap


class Replay(Node):
    def __init__(self, a):
        super().__init__('mission_log_replay')
        R = D.decode(a.log); rows = [r for r in R if (a.leg == 0 or r['leg'] == a.leg) and r['leg_dist_m'] >= a.from_m]
        self.rows, self.speed, self.a = rows, a.speed, a
        self.scan_pub = self.create_publisher(LaserScan, '/scan', 10); self.path_pub = self.create_publisher(Path, '/path', 10)
        self.mk_pub = self.create_publisher(MarkerArray, '/markers', 10); self.tf = TransformBroadcaster(self)
        self.i = 0; self.x = self.y = 0.0; self.pd = rows[0]['leg_dist_m']; self.h0 = rows[0]['yaw']; self.path = Path(); self.path.header.frame_id = 'map'
        self.evid = 0; self.t0 = rows[0]['t_s']; self.wall0 = time.time()
        self.timer = self.create_timer(0.05, self.tick)
        self.get_logger().info(f'{len(rows)} scans, {rows[-1]["t_s"] - rows[0]["t_s"]:.0f} s of log, speed x{a.speed}')

    def pose(self, r):
        th = math.radians(r['yaw'] - self.h0)                      # clockwise from the first heading (plot frame: x right, y forward)
        d = r['leg_dist_m'] - self.pd; self.pd = r['leg_dist_m']
        X = d * math.sin(th); Y = d * math.cos(th)                  # plot frame step -> ROS: x_ros = Y_plot, y_ros = -X_plot
        self.x += Y; self.y += -X
        return self.x, self.y, -th                                  # psi (counter-clockwise) = -th

    def tick(self):
        now = self.get_clock().now().to_msg(); elapsed = (time.time() - self.wall0) * self.speed
        while self.i < len(self.rows) and self.rows[self.i]['t_s'] - self.t0 <= elapsed:
            self.publish(self.rows[self.i], now); self.i += 1
        if self.i >= len(self.rows): self.get_logger().info('end of log', once=True)

    def publish(self, r, now):
        x, y, psi = self.pose(r)
        t = TransformStamped(); t.header.stamp = now; t.header.frame_id = 'map'; t.child_frame_id = 'base_link'
        t.transform.translation.x, t.transform.translation.y = x, y; t.transform.rotation.z, t.transform.rotation.w = math.sin(psi / 2), math.cos(psi / 2)
        self.tf.sendTransform(t)
        L = [r['lidar_%03d' % (i * 4)] for i in range(90)]
        s = LaserScan(); s.header.stamp = now; s.header.frame_id = 'base_link'
        s.angle_min = -math.radians(358); s.angle_max = -math.radians(2); s.angle_increment = math.radians(4); s.range_min = 0.35; s.range_max = 12.0
        s.ranges = [(L[89 - k] / 100.0 if 35 <= L[89 - k] <= 1200 else float('inf')) for k in range(90)]
        self.scan_pub.publish(s)
        p = PoseStamped(); p.header.frame_id = 'map'; p.header.stamp = now; p.pose.position.x, p.pose.position.y = x, y
        self.path.poses.append(p); self.path.header.stamp = now; self.path_pub.publish(self.path)
        M = MarkerArray(); c, sn = math.cos(psi), math.sin(psi)
        to_map = lambda fx, fy: (x + fx * c - fy * sn, y + fx * sn + fy * c)       # rover frame (fx forward, fy left) -> map
        txt = Marker(); txt.header.frame_id = 'map'; txt.ns = 'state'; txt.id = 0; txt.type = Marker.TEXT_VIEW_FACING; txt.pose.position.x, txt.pose.position.y, txt.pose.position.z = x - 2.0, y + 3.2, 1.0
        txt.scale.z = 0.7; txt.color.r = txt.color.g = txt.color.b = 1.0; txt.color.a = 1.0
        txt.text = f"t {r['t_s'] - self.t0:5.1f} s   {r['state']}   {r['leg_dist_m']:.1f} m   front {r['front_m']:.2f} m"; M.markers.append(txt)
        ev = r['event']
        if ev and ev not in ('lidar_ghost_ignored',):
            m = Marker(); m.header.frame_id = 'map'; m.ns = 'event'; self.evid += 1; m.id = self.evid; m.type = Marker.TEXT_VIEW_FACING
            m.pose.position.x, m.pose.position.y, m.pose.position.z = x, y - 2.0 - 0.8 * (self.evid % 3), 0.5; m.scale.z = 0.6
            bad = ev in ('pause_tilt', 'hold', 'wall_end', 'pause')
            m.color.r, m.color.g, m.color.b, m.color.a = (1.0, 0.3, 0.3, 1.0) if bad else (1.0, 0.8, 0.3, 1.0); m.text = ev; M.markers.append(m)
        if self.a.gap:                                              # the v10.3 ramp-opening finder, run on the same scan; the line stays on screen
            g = find_gap(L, ymin=2.5)
            if g:
                gm = Marker(); gm.header.frame_id = 'map'; gm.ns = 'gap'; gm.id = 0; gm.type = Marker.LINE_STRIP; gm.scale.x = 0.22; gm.color.g = 1.0; gm.color.a = 1.0
                for fx in (g[3], g[4]):                              # rover frame: x right -> fy left = -x
                    px, py = to_map(g[0], -fx); gm.points.append(Point(x=px, y=py, z=0.1))
                M.markers.append(gm)
                tm = Marker(); tm.header.frame_id = 'map'; tm.ns = 'gap'; tm.id = 1; tm.type = Marker.TEXT_VIEW_FACING; tm.scale.z = 0.8; tm.color.g = 1.0; tm.color.a = 1.0
                cx, cy = to_map(g[0], -g[1]); tm.pose.position.x, tm.pose.position.y, tm.pose.position.z = cx + 1.0, cy, 0.5; tm.text = 'v10.3: ramp opening %.1f m wide, %.1f m ahead' % (g[4] - g[3], g[0]); M.markers.append(tm)
        self.mk_pub.publish(M)


if __name__ == '__main__':
    ap = argparse.ArgumentParser(); ap.add_argument('log'); ap.add_argument('--leg', type=int, default=0); ap.add_argument('--from-m', type=float, default=0.0)
    ap.add_argument('--speed', type=float, default=5.0); ap.add_argument('--gap', action='store_true', help='also draw the v10.3 ramp opening (leg 4)')
    a = ap.parse_args(); rclpy.init(); n = Replay(a)
    try: rclpy.spin(n)
    except KeyboardInterrupt: pass
