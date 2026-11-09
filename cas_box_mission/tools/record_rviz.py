#!/usr/bin/env python3
"""record_rviz.py - screen-records the RViz window (X11) while rviz_replay.py plays a log. Needs RViz open (title contains 'RViz').
  python3 record_rviz.py out.mp4 SECONDS            (the video is NOT pushed to GitHub)"""
import sys, subprocess, time, re, cv2, numpy as np
from PIL import ImageGrab
out, secs = sys.argv[1], float(sys.argv[2])
info = subprocess.run(['xwininfo', '-root', '-tree'], capture_output=True, text=True, env={'DISPLAY': ':1'}).stdout
m = [l for l in info.splitlines() if 'RViz' in l and 'mutter' in l][0]; g = re.search(r'(\d+)x(\d+)\+(-?\d+)\+(-?\d+)', m); w, h, x, y = map(int, g.groups())
frames = []; t0 = time.time()
while time.time() - t0 < secs: frames.append(np.array(ImageGrab.grab(xdisplay=':1', bbox=(x, y, x + w, y + h)).convert('RGB'))); time.sleep(0.02)
fps = len(frames) / (time.time() - t0); vw = cv2.VideoWriter(out, cv2.VideoWriter_fourcc(*'mp4v'), fps, (w, h))
for f in frames: vw.write(cv2.cvtColor(f, cv2.COLOR_RGB2BGR))
vw.release(); print(len(frames), 'frames', round(fps, 1), 'fps ->', out)
