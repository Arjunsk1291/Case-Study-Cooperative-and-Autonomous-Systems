# ROS 2 / Gazebo Classic workspace

This package targets ROS 2 with Gazebo Classic (`gazebo_ros_pkgs`). Gazebo Classic availability depends on the ROS 2 distribution and OS. If the relevant packages are unavailable on your Ubuntu install, use the standalone Pygame simulator in `../../tools` while we port the robot model to modern Gazebo Sim.

```bash
cd simulation/ros2_ws
rosdep install --from-paths src --ignore-src -r -y
colcon build --symlink-install
source install/setup.bash
ros2 launch r07_simulation r07_world.launch.py
```

The robot publishes simulation-only topics under `/r07`: `/r07/scan`, `/r07/us_*/range`, `/r07/imu`, `/r07/cmd_vel`, and `/r07/odom_sim_only`. The world geometry and building heights are schematic approximations; this is not a surveyed geospatial digital twin. The campus world uses 8 m tall building prisms as placeholders because the public campus diagram does not provide height data.

## Differential-drive controls
Use `geometry_msgs/msg/Twist` on `/r07/cmd_vel`, for example:
```bash
ros2 topic pub --once /r07/cmd_vel geometry_msgs/msg/Twist \
  "{linear: {x: 0.08}, angular: {z: 0.0}}"
```
The simulator's `/r07/odom_sim_only` comes from Gazebo's differential-drive plugin; it is not a sensor present on physical R07. The physical rover has no wheel encoders and must not depend on this topic.
