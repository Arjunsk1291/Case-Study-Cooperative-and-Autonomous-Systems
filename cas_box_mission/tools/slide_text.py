# Text of the weekly slides + speaker notes (layman style like PPT/sampple speaker notes). Numbers: logs / simulations in tools/.
N = dict(
    row_lo='6.5', row_hi='6.8',
    sim1='simulation, worst mix: 0.41 m off at 24 m (before 1.02 m)',
    open_lines=['1 person in the path: false end wall 18 → 0 of 20',
                'group of 3: 20 → 0; walking crowd of 20: 13 → 0',
                'snow, low sun: no false holds'],
    bar_note='v8 rule: 421 of 5,670 cases turned at a wrong pillar, v10: 66. On target = within ±0.5 m of the turn on clean scans.',
)

# each: speaker, title, minutes, script (paragraphs), understand (paragraphs / bullets starting with '- '), questions [(q, a)]
S = []
S.append(dict(speaker='Arjun', title='Landmark proximity corridor for the Open Sea', secs=100,
 script=[
 "After the right turn the rover crosses about 40 metres of open ground with no wall beside it, so LiDAR wall matching has nothing to match, GPS was 6 to 15 metres off here, and the gyro drifts. We therefore steer on fixed landmarks that the 90-beam LiDAR scan can see.",
 "Two straight walls give the direction: the B wall behind us, used for the first 14 metres, and the end wall ahead. In between, the landmark is the U-shaped store front on the left, with the two campus-store poles and two trees with their structures in front of it. The box fits a straight line through the store-front points, a RANSAC fit with 0.25 metre tolerance, at least 5 points over 4 metres, and measures its distance. A PID keeps that distance at 6.65 metres, the value from the teaching run. After 24.5 metres the building front at 3.0 metres takes over.",
 "What the simulation showed: a false straight line, like a bike rack or a hoarding, would pull the rover 1.1 to 2 metres off, because the box accepts any line 4 to 9.5 metres away. A tighter gate fixes it in simulation, but on the real logs it also rejects drifted but real rows, so we did not change the working code. It is a known risk, not a fix.",
 "Results: two real autonomous crossings, log 8 and campus 9, with no hold or failsafe and under 6 degrees heading change over more than 40 metres. Limit: with a gyro bias of 6 degrees per minute and no landmark at all, the end error is about 0.75 metres."],
 understand=[
 "Why landmarks: a fixed object gives the side position directly, from every scan, without GPS and without trusting how long we have driven. The walls give the heading, the store front gives the side position.",
 "- Code: RouteMission.cpp, case L2_GO, reference order: left row (10 to 24.5 m), left building (26 to 40 m), rear B wall (0 to 14 m), front end wall (after half the crossing). The first reference that fits wins. The same logic is mirrored in tools/fwmirror.py (class OpenSea) so it can be simulated.",
 "- PID: e = measured distance − 6.65 m. Output = kp·e + ki·∫e + kd·ė with 10, 1.5 and 20 degrees per metre, limited to ±8 degrees, and moved by at most 0.5 degrees per scan so the heading failsafe never sees a jump. ė comes from the heading relative to the line: −0.22 m/s · sin(angle). The integral is kept after the row ends, so a constant sideways pull is not lost.",
 "- Line fit: LineRef::rowFit on the 270 degree sector ±70, range 1.5 to 11 m. The line direction must be within 8 degrees of the path direction. The line gives only the side position, because its direction wobbled by ±6 degrees in campus_6.",
 "- Measured with tools/landmark_benchmark.py on nine logs: line distance 6.5 to 6.8 m in five runs (teach run 6.67 m), seen on 39 to 56 percent of the scans, B wall behind within ±1.3 degrees in six runs.",
 "- The circles on the slide are our reading of the LiDAR dots together with the walk-through video; the box itself only sees points and a fitted line."],
 questions=[
 ("Why not use GPS?", "In log 10 the GPS was 6 to 15 metres off next to the buildings, even with good satellite numbers. Walls and store fronts do not move."),
 ("What if the line is wrong or missing?", "Missing: it holds the heading with the gyro (simulated: 0.01 m error). Wrong, like a bike rack: the box would follow it (about 1 to 2 metres off in simulation). A tighter gate fixes that but also rejects real drifted rows, so it is not applied."),
 ("Is it tested on the real rover?", "Two real autonomous crossings (log 8, campus 9) and the closed-loop simulation on the next slide. Version 10 has not been driven yet.")]))

S.append(dict(speaker='Kamal', title='Open Sea simulation: crowds, sun, snow and faults', secs=100,
 script=[
 "We cannot bring crowds and snow to the campus on demand, so we built a closed-loop simulation of the Open Sea. A map of the square in metres, a simulated LiDAR with 90 beams that sees that map, a simple model of the rover, and the box's own rules copied to Python. One run is one complete crossing, from the B wall to the end wall.",
 "Chart A is about crowds. The danger is that the rover takes people for the end wall and turns too early. Orange is the old version 9 rule, blue is version 10. A person standing 0.8 metres in front of the end wall fooled version 9 in 9 of 20 runs; version 10 in none. One person in the path: 3 to 0. Twenty people at the store poles: 2 to 0. In every crowd case version 10 made no wrong stop; the rover only stopped, waited for the people and drove on.",
 "Chart B is about sun and snow, 12 runs each. Sun glare puts fake dots close to the LiDAR, and low sun blinds a 40 degree slice. Falling snow adds random close dots and loses a quarter of the beams. Snow on the ground makes the wheels slip sideways while the gyro drifts. Clear, sun glare, low sun and snowy ground: 12 of 12 crossings reached the end wall. Falling snow: 10 of 12. Snow, slip, drift and sun all at once: 7 of 12. In no run did it cause a wrong stop.",
 "We also tested faults: a hidden landmark, a gap in it, the B wall hidden: still on track within 2 centimetres. One weakness: a fake straight line, like a bike rack, can pull the rover 1 to 2 metres off. A fix exists in simulation, but it changed the real runs, so it is not in the code yet."],
 understand=[
 "Files: tools/sim_open_sea.py is the simulator (world(), scan(), run(), crowd(), env_noise()); tools/fwmirror.py class OpenSea is the mirror of the firmware logic in RouteMission.cpp. Run it with python3 sim_open_sea.py --crowd, --env or --faults. Results are saved in PPT/new_slides_assets/open_sea_faults_*.txt.",
 "- Metrics: side error at 24 m and at the end wall, whether the end wall was reached, false end walls, false holds, waiting time. 20 random starts per case, 12 for the weather cases.",
 "- People: 0.4 m squares standing, walking or bouncing; the rover rule is: something within 0.6 m and ±16 degrees for 0.5 s means stop and wait, go on after 2 s clear. It never swerves.",
 "- End wall: it must be wide (at least 1.1 m on both sides), only after 85 percent of the crossing, and a leaky counter (+2 for a good scan, −1 for a bad one, trigger 6).",
 "- The nearest-surface rule, in plain words: the 8 beams straight ahead each give a forward distance, for example a person at 1.0 m on 3 beams and the wall at 2.0 m on 5 beams past the person. Version 9 took the MIDDLE value of the sorted list (here 2.0 m, the wall behind the person), found the wall also to the sides at 2.0 m and said 'end wall' with a person in front. Version 10 takes the SMALLEST value (1.0 m, the person); nothing 1.1 m to the sides is at 1.0 m, so it is not a wall, and the people rule stops and waits instead. With no one in front all 8 beams see the wall, smallest and middle are the same, so real end walls are found exactly as before (real logs: same result on all but 0 to 2 scans per log).",
 "- Crowd test of this rule (tools/sim_open_sea.py --nearest, 20 starts each): false end walls middle → smallest: people walking near the store poles 1 → 0, 20 people at the poles 2 → 0, one person in the path 3 → 0, a person 0.8 m in front of the end wall 9 → 0, group of 5, 8 shoulder to shoulder, 8 standing at the poles, 20 walking: 0 → 0. A pole that never moves in the path: the rover stops and waits, never a false end wall.",
 "- How snow was simulated (sim_open_sea.py, env_noise()): every simulated LiDAR beam has an 8 percent chance to return a random close distance of 0.35 to 3 m (a snowflake) and a 25 percent chance to return nothing (beam lost in the snow); 10 percent of the scans also get a sun-type fake dot.",
 "- How slip and drift were simulated (run()): slip = the rover moves sideways by 8 percent of its forward speed on every step, like sliding on a snowy slant; drift = the gyro heading is wrong by 6 degrees more every minute (a slow, steady error), so the box believes it is straight while it is not. The landmarks and walls must correct both.",
 "- Why falling snow fails in 2 of 12: too many lost and random beams, so the end wall is never confirmed. There is no wrong stop. On the real box the close-range safety stops (LiDAR 0.45 m, ultrasonic 35 cm) would still stop it at the wall and it would wait for the operator; the simulator does not model the ultrasonic at walls, so this part is the firmware rule, not a simulated result.",
 "- Sun model: daylight fake dots (20 percent of scans, 1 to 2 beams at 20 to 34 cm, as measured in the logs) and a 40 degree blind slice for low sun (30 percent of scans).",
 "- Fault list: row missing, row gap 14 to 20 m, row 0.7 m off the taught distance, false line 4.5 m and 8.5 m, B wall hidden, heading glitch +10 and −15 degrees, gyro bias with no row, and a mix of five faults.",
 "- Limits: it is a simulation with our own world model, not the real campus. The row 0.7 m off the taught distance gives 0.3 to 1.1 m error because the box cannot tell it from a start offset; in the logs the row was within ±0.15 m."],
 questions=[
 ("How do you know the simulation is realistic?", "The ghost, dropout and noise numbers come from the logs, the row distance from nine logs, and the crowd rule is the firmware rule. It still is a model; the real test is the field run."),
 ("What did the simulation change in the code?", "The end wall check (wide, nearest surface, 85 percent, leaky counter) and the landmark gate of plus or minus 1.0 m."),
 ("What is still wrong?", "A 15 degree heading glitch late in the crossing (8 m error), gyro bias of 6 degrees per minute with no landmark (0.75 m), and eight people shoulder to shoulder at the end wall.")]))

S.append(dict(speaker='[name to be added]', title='Last leg to Block I: corridor, entrance, stop', secs=90,
 script=[
 "The last leg starts after the left turn at the end of the arcade and ends at the glass entrance of Block I, the KARL entrance, where the rover has to stop. First it follows the wall, then it crosses the interlock walkway, then it drives between two railings.",
 "The walkway has no wall. On its left there is grass with benches and a hydrant, and later the low railing and drain at the foot of the building. The rover must not drift into the grass or mud, so it centres itself on that left edge: the LiDAR fits a line through those objects, the rover remembers the distance it first saw, and steers gently to keep it. The honest limit: in the campus 9 replay this line was found in only about 10 percent of the scans; the rest of the time it holds its heading.",
 "In campus 9 the rover did not stop at the entrance and hit a bollard nine metres later. The railing rule was too strict, and the bollard came closer than 35 centimetres, below the LiDAR minimum range.",
 "Version 10 recognises the corridor by its width and the entrance where the facade steps back and the glass loses LiDAR points, then stops 1.5 metres later. In replay: corridor at 35.8 metres, entrance at 47.0, stop at 48.5, before the bollard at 56. Replay only; it still has to be driven."],
 understand=[
 "The goal: stop gently at the right place, not just before an obstacle.",
 "- Phases: wall follow at 1.0 m (power 0.30), walkway (centering on the left edge), railing corridor, KARL entrance, stop. All stops are slow ramps because of the gearbox.",
 "- Walkway centering, in detail: the box fits a straight line through LiDAR points on the left, 1 to 3 m away, at least 5 points over 0.8 m, within ±12 degrees of the heading. The first distance it sees (limited to 1.3 to 2.4 m) is kept. Too far from the edge: steer left, 10 degrees per metre, at most ±6 degrees, changed slowly. The LiDAR sees shapes, not names: the circles on the slide (benches and hydrant on the grass, low railing and drain) are our reading of the dots with the walk-through video, not labels the box knows.",
 "- Small lines in the LiDAR map: the short slanted lines left of the track are the foot of the building with its low railing and drain, seen at an angle; they are too short and slanted to count as an edge line.",
 "- What we measured (replay of the same line detector): the left edge line is found in about 10 percent of the walkway scans in campus 9 and 35 percent in log 8. In between the rover holds the wall direction. A stronger centering, using the nearest object on the left, is the next improvement and is not in version 10.",
 "- Corridor: right railing line 0.35 to 1.3 m, left facade 1.4 to 2.6 m, total width 2.4 to 3.3 m, for 1 second. Found at 35.8 m in campus 9 and 40.0 m in log 8, never earlier.",
 "- Entrance: the facade steps back by 0.2 m or more and the glass drops LiDAR beams. The stop is 1.5 m later.",
 "- Edge assist: in log 8 the rover stood 60 s at the cobble-to-interlock edge. Now it pushes with more power, backs up 0.25 m and pushes again, but only when nothing is closer than 1.2 m ahead, so it never pushes against an obstacle.",
 "- Code: RouteMission.cpp, _l4Walk() for walkway, corridor and entrance, _edgeAssist() for the edge, _leg4Wall() for the wall part. The railing line is a RANSAC fit (LineRef::rowFit, 90 degrees ±40, 0.3 to 1.8 m, tolerance 0.08 m, at least 4 points). The entrance needs at least 3 missing and 4 valid beams in bins 60 to 74 on three scans in a row, and only after 3 m inside the corridor.",
 "- Hold rule: an obstacle that vanishes closer than 0.45 m without being seen moving away is still there. It is released only when several beams see more than 1 m free, or the ultrasonic reads more than 1 m."],
 questions=[
 ("Does it keep out of the grass?", "It tries to: it keeps the distance to the left edge when it sees a line there, otherwise it holds the heading of the wall. In campus 9 the line was found in only about 10 percent of scans, so this is a weak point; the video of the run is the check."),
 ("Why did it not stop in campus 9?", "Two reasons: the corridor rule missed (right railing at 0.9 m, rule wanted 0.85 or less) and the bollard was closer than the LiDAR minimum range."),
 ("Will it stop at the right door?", "Replay says 48.5 m, the entrance is at 47.0 m. It must be checked outdoors."),
 ("What if a person stands at the entrance?", "It holds and finishes the route there. It never drives into or around people.")]))

