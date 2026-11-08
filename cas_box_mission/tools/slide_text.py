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
S.append(dict(speaker='Arjun', title='Landmark proximity corridor for the Open Sea', secs=90,
 script=[
 "The Open Sea is the big empty square after the right turn, about 40 metres long. There is no wall next to the rover, so our wall-following has nothing to hold on to. GPS was 6 to 15 metres off here, and the gyro slowly drifts, so a rover that only trusts those two would wander.",
 "So we steer on things that do not move. Two of them are plain walls: the B wall behind us and the end wall ahead. In the middle are the landmarks: a tree with its structure, the two poles of the campus store, and the store front. In the picture, the grey dots are what the LiDAR saw, and the blue circles are those landmarks.",
 "The rover measures how far the poles are to its left and keeps that distance, using a small controller we call the proximity PID. The distance to keep, 6.65 metres, comes from the teaching run. Later, the store front at 3.0 metres takes over. The B wall behind and the end wall ahead keep the heading straight.",
 "It works: in two full runs, log 8 and campus 9, the rover crossed from wall to wall with no hold, no pause and no failsafe, and its heading changed by under 6 degrees over more than 40 metres. In simulation the worst case is 0.41 metres off, before it was 1.02. Still open: a strong gyro drift gives about 0.4 metres of error at the end."],
 understand=[
 "Why landmarks: in the Open Sea the LiDAR map matching has no walls to match. A landmark is a fixed object we can measure our side distance to, so every scan corrects the position by itself. It does not depend on GPS or on how long we have been driving.",
 "- The blue circles are the LiDAR points of the tree and its structure, the two campus-store poles and the store front. The LiDAR sees shapes, not names, so the box fits a straight line through the points it sees there and measures the distance to that line.",
 "- The proximity PID: error = measured distance minus the target (6.65 m). The correction is kp·error + ki·sum of errors + kd·change of error, with 10, 1.5 and 20, limited to ±8 degrees and changed slowly (0.5 degrees per scan), so the heading failsafe never sees a jump.",
 "- The poles give only the side position. Their direction wobbles by about ±6 degrees, so the heading comes from the straight walls (B wall behind within ±1.3 degrees in six runs).",
 "- Nine logs were measured with the same functions as the firmware: the pole-line distance was 6.5 to 6.8 m in five runs (teach run 6.67 m) and was seen in 39 to 56 percent of the scans."],
 questions=[
 ("Why not use GPS?", "In log 10 the GPS was 6 to 15 metres off next to the buildings, even with good satellite numbers. Walls and poles do not move."),
 ("What if a person stands in front of a pole?", "The box only uses the line when enough points fit it. If not, it keeps the last good heading for a while. The people simulation (next slide) covers crowds."),
 ("Is it tested on the real rover?", "Two real autonomous crossings (log 8, campus 9) and a closed-loop simulation. Version 10 has not been driven yet.")]))

S.append(dict(speaker='Kamal', title='People simulation: wrong turns 7.4 % → 1.2 %', secs=90,
 script=[
 "In log 8 the rover turned one pillar too early. A person walking behind the rover looked, to the LiDAR, like the pillar we search for. We cannot tell people to stay away, so we built a simulation that adds people to the real recordings.",
 "The method is on top of the slide. We take real LiDAR scans, add people as small circles, hide what is behind them, replay the same rules as the box, and compare the turn with the turn on the clean scan. We did this for 5,670 cases: people standing, walking along, towards and across, a follower, groups of two to five, and a crowd of eight.",
 "The result is in the chart. With the old rule, 421 cases, 7.4 percent, turned at the wrong pillar. With version 10, which looks for the black sign structure beside the turn pillar, only 66, 1.2 percent. In 93.5 percent of cases the turn is within half a metre of the clean turn.",
 "In the Open Sea the same idea: one person standing in the path made a false end wall in 18 of 20 runs; now in 0. A group of three, 20 to 0. A walking crowd, 13 to 0. Rain, snow and low sun cause no false stops. Still open: groups of three to five at the pillar before the sign, about 6 percent wrong turns, and this is simulation only."],
 understand=[
 "The idea: we cannot walk a crowd through every pillar, so we make the crowd in the computer and test our rules on it. This is a test of the rule, not a prediction of real crowds.",
 "- The old rule looked for a pillar-like object. Every person standing near a pillar can look like one. The new rule needs the structure of the sign: it reaches from the pillar line to more than 2.3 m to the right, and it has to be seen several times from different positions.",
 "- The turn also uses the place where the wall behind the pillars comes back after the gap (the 'B gap') and adds 5.35 m. This does not depend on where the run started.",
 "- If nothing is found, the rover stops and asks instead of turning blindly.",
 "- 'On target' means within ±0.5 m of where the clean scan turns. 'Wrong pillar' means more than 2 m too early. We show wrong turns in the chart because they are the dangerous ones; the old rule mostly turned late or not at all, which looks bad on 'on target' but is not as risky.",
 "- Open Sea: 20 random starts per case, 12 for weather cases. The end wall must now be wide (≥ 1.1 m on both sides of the nearest surface), only after 85 percent of the crossing."],
 questions=[
 ("Are the people realistic?", "Circles of 0.15 to 0.30 m, speeds 0.8 to 1.6 m/s, placed 0.5 to 4 m from the track. Real people are less regular, so we also tested groups and a crowd."),
 ("Does the rover swerve around people?", "No. People in front: it stops and waits. It never steers around them."),
 ("What is still wrong?", "Groups of 3 to 5 at the pillar before the sign (about 6 percent wrong turns) and eight people shoulder to shoulder at the end wall.")]))

S.append(dict(speaker='[name to be added]', title='Last leg to Block I: corridor, entrance, stop', secs=90,
 script=[
 "The last leg starts after the left turn at the end of the arcade and ends at the glass entrance of Block I, the KARL entrance, where the rover has to stop. First it follows the wall, then it crosses the interlock walkway, then it drives between two railings.",
 "The walkway has no wall. On its left there is grass with benches and a hydrant, and later the low railing and drain at the foot of the building. The rover must not drift into the grass or mud, so it centres itself on that left edge: the LiDAR fits a line through those objects, the rover remembers the distance it first saw, and steers gently to keep it. The honest limit: in the campus 9 replay this line was found in only about 10 percent of the scans; the rest of the time it holds its heading.",
 "In campus 9 the rover did not stop at the entrance and hit a bollard nine metres later. The railing rule was too strict, and the bollard came closer than 35 centimetres, below the LiDAR minimum range.",
 "Version 10 recognises the corridor by its width and the entrance where the facade steps back and the glass loses LiDAR points, then stops 1.5 metres later. In replay: corridor at 35.6 metres, entrance at 47.0, stop at 48.5, before the bollard at 56. Replay only; it still has to be driven."],
 understand=[
 "The goal: stop gently at the right place, not just before an obstacle.",
 "- Phases: wall follow at 1.0 m (power 0.30), walkway (centering on the left edge), railing corridor, KARL entrance, stop. All stops are slow ramps because of the gearbox.",
 "- Walkway centering, in detail: the box fits a straight line through LiDAR points on the left, 1 to 3 m away, at least 5 points over 0.8 m, within ±12 degrees of the heading. The first distance it sees (limited to 1.3 to 2.4 m) is kept. Too far from the edge: steer left, 10 degrees per metre, at most ±6 degrees, changed slowly. The LiDAR sees shapes, not names: the circles on the slide (benches and hydrant on the grass, low railing and drain) are our reading of the dots with the walk-through video, not labels the box knows.",
 "- Small lines in the LiDAR map: the short slanted lines left of the track are the foot of the building with its low railing and drain, seen at an angle; they are too short and slanted to count as an edge line.",
 "- What we measured (replay of the same line detector): the left edge line is found in about 10 percent of the walkway scans in campus 9 and 35 percent in log 8. In between the rover holds the wall direction. A stronger centering, using the nearest object on the left, is the next improvement and is not in version 10.",
 "- Corridor: right railing line 0.35 to 1.3 m, left facade 1.4 to 2.6 m, total width 2.4 to 3.3 m, for 1 second. Found at 35.6 m in campus 9 and 40.0 m in log 8, never earlier.",
 "- Entrance: the facade steps back by 0.2 m or more and the glass drops LiDAR beams. The stop is 1.5 m later.",
 "- Edge assist: in log 8 the rover stood 60 s at the cobble-to-interlock edge. Now it pushes with more power, backs up 0.25 m and pushes again, but only when nothing is closer than 1.2 m ahead, so it never pushes against an obstacle.",
 "- Hold rule: an obstacle that vanishes closer than 0.45 m without being seen moving away is still there. It is released only when several beams see more than 1 m free, or the ultrasonic reads more than 1 m."],
 questions=[
 ("Does it keep out of the grass?", "It tries to: it keeps the distance to the left edge when it sees a line there, otherwise it holds the heading of the wall. In campus 9 the line was found in only about 10 percent of scans, so this is a weak point; the video of the run is the check."),
 ("Why did it not stop in campus 9?", "Two reasons: the corridor rule missed (right railing at 0.9 m, rule wanted 0.85 or less) and the bollard was closer than the LiDAR minimum range."),
 ("Will it stop at the right door?", "Replay says 48.5 m, the entrance is at 47.0 m. It must be checked outdoors."),
 ("What if a person stands at the entrance?", "It holds and finishes the route there. It never drives into or around people.")]))

S.append(dict(speaker='[name to be added]', title='Log replay and benchmark toolchain', secs=90,
 script=[
 "This slide is how we test changes without driving again. Every run writes a log. A decoder turns it into a table, a narrator prints what the rover was thinking at each moment, and a Python copy of the firmware rules replays the log. So a change can be tried on old runs before we flash it.",
 "Two more tools sit on top. The landmark benchmark measures the same landmarks in nine logs and keeps them in one file, so we know which numbers are normal. The simulators add people, weather and slip to real scans.",
 "Here is what it did this week. The chart shows where the replay turns on seven runs. The old rule turned too early in log 8, at 18.2 metres; the new one turns at 21.8. On the others it stays where the good runs turned. If we shift the start of a run by up to 8 metres, the turn moves by at most 0.13 metres.",
 "It also found the cause of the three failures: the follower in log 8, the start-dependent window in campus 9, and the hold that was released too early. The limit: it is a copy of the firmware, so it has to be kept in sync, and distances are commanded, not measured, because we have no wheel encoders."],
 understand=[
 "The pipeline: log file (.bin from the box) → decoder (table) → narrate_run (plain-language story of the run) → landmark_benchmark (numbers per log) → fwmirror (Python copy of the rules) → replay_turn / sweep_start / simulators → results → firmware → flash.",
 "- Why a copy: the real firmware runs on the ESP32. The copy runs the same rules on the laptop in seconds, on every old log, so we see the effect of a change before it can break the rover.",
 "- Start-shift sweep: we cut or pad the start of each log by −8 to +8 m. A good rule turns at the same place; ours moved by 0.00 to 0.13 m.",
 "- Benchmark: nine logs, same functions as the firmware. It showed, for example, the pole-line distance 6.5 to 6.8 m in five runs, and a B wall behind within ±1.3 degrees in six.",
 "- We do not tune to bad runs: a drifted run is used to find a cause, not to set a number."],
 questions=[
 ("Does the replay match the real rover?", "It uses the same rules and logged sensor data, but it is a Python copy. A change in the C++ code has to be copied over; we check the numbers against the logs."),
 ("Why not just drive again?", "Every outdoor run costs time and risk to the rover. Replay tells us in seconds which change is worth driving."),
 ("What is not covered?", "Real crowds, real weather and wheel slip are simulated, not measured. Distances are commanded, not from encoders.")]))
