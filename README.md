# Case-Study-Cooperative-and-Autonomous-Systems

This branch adds a simulation workspace for CAS Box B3 / WAVE ROVER R07 alongside the existing ESP32 sensor-box firmware.

## Start here
- [Simulation documentation](simulation/README.md)
- [TH Deggendorf Block B → Block H terrain (schematic JSON)](simulation/terrain/dit_deggendorf_b_to_h.json)
- [Interactive 2D simulator](simulation/tools/r07_campus_sim.py)
- [ROS 2 / Gazebo Classic R07 robot model](simulation/ros2_ws/src/r07_description/urdf/r07_rover.urdf.xacro)
- [Campus world for Gazebo](simulation/ros2_ws/src/r07_simulation/worlds/dit_b_to_h.world)

## Important accuracy notes
- The known rover footprint represented in the simulation is 0.30 m long × 0.20 m wide.
- CAS Box dimensions, wheel track/radius, sensor bearings/mounting points and metric campus geometry are editable estimates where not measured.
- The campus diagram is not surveyed. The B → H route is a candidate path-planner test; confirm entrances, walkways, terrain and obstacles before using a real rover.
- The main ESP32 firmware and rover firmware are not changed by the simulation additions.


## Manual survey and investigation firmware

The isolated branch [survey/r07-investigation-2026-10-30](https://github.com/Arjunsk1291/Case-Study-Cooperative-and-Autonomous-Systems/tree/survey/r07-investigation-2026-10-30) adds a laptop-owned survey pipeline for B3 / R07.

- [Survey runbook and architecture](laptop_survey/README.md)
- [Laptop FastAPI service](laptop_survey/app.py)
- [Survey data/session manager](laptop_survey/manager.py)
- [Operator survey HMI](laptop_survey/static/index.html)
- [ESP32-S3 camera transfer template and hardware blocker](camera_s3/README.md)
- [Dataset schema and backend tests](laptop_survey/tests/)

The operator starts an investigation while the rover is stopped, explicitly enables manual driving, captures annotations/segments while logging continues, and uses **Stop Investigation & Finalize Dataset** as the only action that ends logging and generates a verified ZIP on the laptop. The camera path is directly from ESP32-S3 to laptop; the main ESP32 is not used as a JPEG relay.

**Hardware-dependent limitations:** no camera board model or pin map exists in the repository, so the ESP32-S3 sketch is intentionally blocked until that is provided. No wheel encoders or physical E-stop input are present in the inspected code. Wall-following motion remains disabled pending calibration; wall/centreline and tactile-strip results are data-collection suggestions only.

This survey work is in a separate branch and does not change the default branch or rover firmware.

