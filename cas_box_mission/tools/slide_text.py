# Text of the weekly slides + speaker notes (layman style like PPT/sampple speaker notes). Numbers: logs / simulations in tools/.
N = dict(
    row_lo='6.5', row_hi='6.8',
    sim1='simulation, worst mix: 0.41 m off at 24 m (before 1.02 m)',
    open_lines=['1 person in the path: false end wall 18 → 0 of 20',
                'group of 3: 20 → 0; walking crowd of 20: 13 → 0',
                'rain, snow, low sun: no false holds'],
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

S.append(dict(speaker='Kamal', title='Open Sea simulation: crowds, weather and faults', secs=100,
 script=[
 "We cannot test every crowd, every weather and every fault outdoors, so we built a closed-loop simulation of the Open Sea. The world is a set of line segments: the B wall, the store-front row with its stakes, the building front, the end wall and random people. A ray-casting scanner makes 90 beams of 4 degrees with 2 centimetre noise, 12 percent dropouts and the real 35 centimetre minimum range. A rover model drives at 0.22 metres per second with limited turn rate, slide and gyro bias. The box logic itself is a Python mirror of the firmware, the same rules, stepped every 0.25 seconds.",
 "Crowds first. With the old flat end-wall rule of version 8, one person standing in the path caused a false end wall in 18 of 20 runs, version 9's wide rule still in 4, version 10 in 0. A group of three: 20 to 0. A walking crowd of 20: 13 to 0. The rover still stops and waits for people and never steers around them.",
 "Weather: daylight ghosts, low sun and light rain are fine, 12 of 12. Heavy rain or snow 10 of 12, with no false stop. Snow plus slip plus drift plus sun together: 7 of 12.",
 "Faults, the chart. Row missing, row gap, B wall hidden, heading glitch: fine. But a false straight line, like a bike rack or a hoarding, pulls the rover 1.1 to 2 metres off. A tighter gate of plus or minus 1.5 metres fixes it in simulation, but we have not put it in the code, because the working crossing from version 5 to 9 must not change. Still open: a 15 degree heading glitch late in the crossing, and gyro bias with no landmark."],
 understand=[
 "Files: tools/sim_open_sea.py is the simulator (world(), scan(), run(), crowd(), env_noise()); tools/fwmirror.py class OpenSea is the mirror of the firmware logic in RouteMission.cpp. Run it with python3 sim_open_sea.py --crowd, --env or --faults. Results are saved in PPT/new_slides_assets/open_sea_faults_*.txt.",
 "- Metrics: side error at 24 m and at the end wall, whether the end wall was reached, false end walls, false holds, waiting time. 20 random starts per case, 12 for the weather cases.",
 "- People: 0.4 m squares standing, walking or bouncing; the rover rule is: something within 0.6 m and ±16 degrees for 0.5 s means stop and wait, go on after 2 s clear. It never swerves.",
 "- End wall: it must be wide (at least 1.1 m on both sides of the nearest surface, so one person is not a wall), only after 85 percent of the crossing, and a leaky counter (+2 for a good scan, −1 for a bad one, trigger 6). A bug we found in the simulation: the median front distance could land on the far wall behind a person.",
 "- Weather model: daylight ghosts (20 percent of scans, beams at 20 to 34 cm), a 40 degree blind wedge for low sun, random near returns for rain and snow (3 to 8 percent of beams, 10 to 25 percent dropouts), slide up to 8 percent sideways, gyro bias ±6 degrees per minute.",
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

S.append(dict(speaker='[name to be added]', title='Daylight ghost filter: LiDAR and ultrasonic cross-check', secs=100,
 script=[
 "In daylight the LiDAR sometimes reports objects at 20 to 34 centimetres that are not there. In the older logs, 3 to 8 percent of the scans had such a beam. If the rover believed them it would stop for nothing.",
 "The fix has three layers. The LiDAR ignores everything below 35 centimetres. A nearer obstacle only counts if at least four beams agree, or if the front ultrasonic sensor agrees within 15 centimetres. And the downward-angled ultrasonic sensor sees the floor at 50 to 90 centimetres, so that echo is ignored.",
 "The numbers come from the datasheets: the LD19 family is rated for 30 kilolux of ambient light, so direct sun is at its limit, and the HC-SR04 measures 2 to 400 centimetres in a cone of about 15 degrees, one ping every 60 milliseconds. We use each sensor where it is strong.",
 "The price: a real obstacle closer than 35 centimetres is invisible to the LiDAR, as the bollard in campus 9 was. So the ultrasonic stop distance is 35 centimetres, and an obstacle that vanishes from closer than 45 centimetres counts as still there."],
 understand=[
 "Where the filter lives: LidarScan.h (minimum range 350 mm) and RouteMission.cpp, _front() and _usMed(): the verification uses the median of the front ultrasonic readings and the count of valid beams.",
 "- The chart: left, the range of beams below 35 cm in the daytime logs of campus_2, campus_3 and campus_5 (mostly 20 to 34 cm, nothing in the real world was that close); right, the share of scans with such a beam in each log (1 to 8 percent). Logs from campus_6 on contain none because the filter was already in the firmware.",
 "- Rule: a near obstacle is accepted if at least 4 valid beams agree, or if the front ultrasonic median is below 50 cm and not more than 15 cm above the LiDAR. In replay 27 of 28 ghost events were dropped.",
 "- Ultrasonic floor echo: one front sensor is angled 45 degrees down, so the floor at 50 to 90 cm gives false near readings; they are masked.",
 "- Datasheet facts, checked online: LD19 0.02 to 12 m, 4,500 samples per second, 30 klux; the LD20 datasheet was not found, so the LD19 values are used. HC-SR04 2 to 400 cm, about 15 degrees, more than 60 ms per measurement.",
 "- Tilt and the ramp are handled separately; this slide is only about false near obstacles."],
 questions=[
 ("Why not just lower the LiDAR sensitivity?", "The ghosts are real returns from sunlight at the sensor, not noise we can threshold. Cross-checking with a different physical principle (ultrasound) is more robust."),
 ("What does the filter cost?", "Anything closer than 35 cm is invisible to the LiDAR. The ultrasonic stop at 35 cm and the hold rule for vanished obstacles cover that."),
 ("Is it tested?", "Replayed on the logs (27 of 28 ghosts dropped) and in the Open Sea simulation with ghosts in 20 percent of scans: 12 of 12 runs reach the end wall.")]))
