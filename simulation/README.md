# R07 digital twin and indoor map

This branch is for simulation only; it does not modify the firmware on `main`.

## What is known from the box firmware
- WAVE ROVER is commanded over Wi-Fi using normalized left/right drive values in [-0.5, 0.5].
- CAS Box 7 is a sensor-box controller, not the motor controller.
- LiDAR: 90 bins at 4 degrees per bin in logged data. `LIDAR_OFFSET_DEG=90` is applied in the box code; verify the simulated coordinate convention against this offset.
- Five ultrasound ranges ordered front, left, right, back, front angled.
- BNO055 yaw; convention in the firmware README says positive yaw is left.
- No wheel encoders. Simulated wheel odometry must not be represented as measured hardware odometry.
- Rover and box networks remain separate. Simulation uses a ROS 2/Gazebo model and can be bridged to the same high-level controller later.

## Map status
The investigation JSON is a set of stationary 90-bin scan signatures sampled at browser polling rate, not a 2-D occupancy map or SLAM trajectory. It constrains relative landmarks (pillar, turn after pillar, door approach, near entrance, final scan inside the washroom per the operator) but not unique metric x/y coordinates. The initial world included here is an editable **approximate test terrain**: do not call it a surveyed reconstruction.

## Workspace
- `ros2_ws/src/r07_description`: robot dimensions as explicit parameters; URDF with differential-drive base, dummy CAS Box, LiDAR and five ultrasound sensor placements.
- `ros2_ws/src/r07_simulation`: Gazebo world and launch file.
- `terrain/approximate_floorplan.yaml`: editable walls, door, pillar, start and waypoints.
- `tools/scan_to_polar_map.py`: converts one 90-bin scan to a polar diagnostic CSV/plot; does not infer wall coordinates.

## Run (Ubuntu with ROS 2 + Gazebo Sim)
Install a matching ROS 2 distribution and Gazebo Sim integration for it; versions are not guessed by this package.
1. Copy or clone this repository branch.
2. Source ROS 2 and create a workspace:
   ```bash
   cd simulation/ros2_ws
   rosdep install --from-paths src --ignore-src -r -y
   colcon build --symlink-install
   source install/setup.bash
   ros2 launch r07_simulation r07_world.launch.py
   ```
3. Tune room, corridor and door dimensions in `terrain/approximate_floorplan.yaml` and the world SDF. Dimensions marked as estimates are placeholders until measured.
4. Test stop/finish behavior in simulation. Do not transfer a simulated success directly to physical hardware without supervised validation.

## Critical modelling limitation
The recorded scans have no reliable robot x/y pose, no encoders and no laser scan-matching trajectory. We must manually parameterize an approximate terrain first and later calibrate it from measurements or use proper SLAM data. The final logged scan is treated as the intended inside-washroom finish **reference by operator confirmation**, even though the original label is `entrance`.
