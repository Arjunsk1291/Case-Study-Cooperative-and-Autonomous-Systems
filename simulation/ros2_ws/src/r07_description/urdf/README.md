# R07 URDF model

The URDF uses the known chassis planform 0.30 m × 0.20 m. Wheel track/radius, caster layout, CAS Box enclosure size, sensor mounting offsets and sensor bearings are editable estimates because there is no hardware inspection available in this task. The base model uses two driven wheels plus two passive caster approximations.

The model is for Gazebo Classic + ROS 2 `gazebo_ros` plugins. The laser has 90 samples across 360 degrees (4° nominal spacing), 0.10–3.50 m range. Five simulated ultrasonic Range topics use the box firmware sensor ordering. IMU output is simulated, not a calibrated BNO055 model.

The differential-drive plugin publishes `/r07/odom_sim_only`. This is simulator output only. The physical rover has no wheel encoders, so do not treat this topic as available from the real rover or use it to claim hardware odometry.
