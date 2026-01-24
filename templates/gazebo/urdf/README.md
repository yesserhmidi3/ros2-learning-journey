# Gazebo URDF Template

This is a **template URDF** for creating a simple robot in Gazebo with:

- Basic links with visual, collision, and inertial properties
- An IMU sensor
- A differential drive plugin for movement

It can be used as a starting point for simulating robots in Gazebo with ROS2.

---

## Overview

### 1. Link Definition
Each link in the URDF includes:

- **Visual**: How the link looks in Gazebo
- **Collision**: Defines the collision shape
- **Inertial**: Mass and inertia for physics simulation

Example:

```xml
<link name="part_name">
  <visual>
    <geometry><box size="0.1 0.1 0.1"/></geometry>
  </visual>
  <collision>
    <geometry><box size="0.1 0.1 0.1"/></geometry>
  </collision>
  <inertial>
    <mass value="1.0"/>
    <origin xyz="0 0 0" rpy="0 0 0"/> 
    <inertia ixx="0.01" ixy="0" ixz="0" iyy="0.01" iyz="0" izz="0.01"/>
  </inertial>
</link>
```

### 2. Sensors (IMU)
The IMU sensor can be added to a link for orientation and acceleration data.

```xml
<gazebo reference="link_name">
  <sensor name="my_sensor" type="imu"> 
    <always_on>1</always_on>
    <update_rate>100</update_rate>
    <topic>sensor_data</topic>
    <plugin filename="gz-sim-imu-system" name="gz::sim::systems::Imu"/>
  </sensor>
</gazebo>

```

Notes:
- `reference="link_name"` attaches the sensor to a specific link.
- `update_rate` controls how frequently data is published.
- `topic` is the ROS2 topic for sensor messages.
  
### 3. Differential Drive Plugin
Adds wheel control to the robot for moving using velocity commands.
```xml
<gazebo>
  <plugin filename="gz-sim-diff-drive-system" name="gz::sim::systems::DiffDrive">
    <left_joint>left_wheel_joint</left_joint>
    <right_joint>right_wheel_joint</right_joint>
    <wheel_separation>0.2</wheel_separation>
    <wheel_diameter>0.1</wheel_diameter>
    <topic>cmd_vel</topic>
    <max_wheel_torque>20</max_wheel_torque>               <!-- Optional: limits torque -->
    <max_wheel_acceleration>10.0</max_wheel_acceleration> <!-- Optional: limits acceleration -->
  </plugin>
</gazebo>
```

Notes:
-`left_joint` and `right_joint` must match your URDF wheel joints.
-`wheel_separation` is the distance between wheels.
-`wheel_diameter` is the size of the wheels.
-`topic` is the ROS2 topic where velocity commands are received.
- `max_wheel_torque` and `max_wheel_acceleration` can be **added optionally** to limit torque and acceleration of the wheels for realistic physics.

---

## How to Use

1. Save this URDF in your package `urdf/` folder (e.g., `robot_template.urdf`).
2. Modify link names, geometry, and inertial properties to match your robot.
3. Attach sensors and plugins as needed.
4. Use with a Gazebo launch file to spawn the robot and bridge ROS2 topics:
```bash
ros2 launch my_robot_package launch_robot.py
```

---

## Tips

- Use simple boxes or cylinders to start before modeling complex shapes.
- Ensure joint names in plugins match URDF joints.
- Tune `update_rate` for sensors according to your control loop requirements.
- Combine with the Gazebo launch template for seamless simulation.




