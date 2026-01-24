# Self-Balancing Robot Simulation (Gazebo + ESP32 Bridge)

This project simulates a **self-balancing robot** in Gazebo and demonstrates real-time **ESP32-to-Gazebo bridging** using ROS2.  
It combines URDF modeling, launch setup, and a PID controller in Python.

---

## Structure

- **URDF** (`urdf/balancing_bot.urdf`)
  - Defines robot links, wheels, and IMU sensor
  - Includes **diff-drive plugin** for wheel control
  - Configurable limits:  
    ```xml
    <max_wheel_torque>20</max_wheel_torque>
    <max_wheel_acceleration>10.0</max_wheel_acceleration>
    ```
  - Pay attention to **inertial properties** for stable simulation

- **Launch File** (`launch/launch_bot.py`)
  - Starts **Gazebo empty world**
  - Runs **robot_state_publisher** (publishes URDF)
  - Spawns robot in Gazebo
  - Starts **ros_gz_bridge** to map topics:
    - `/cmd_vel` ↔ Motor commands
    - `/imu` ↔ IMU sensor
    - `/clock` ↔ Simulation time

- **Bridge Node** (`main.py`)
  - Subscribes to `/imu` (from Gazebo)
  - Applies a **PID controller** to compute wheel velocity
  - Publishes velocity commands to `/cmd_vel`
  - Features:
    - Deadzone compensation to overcome friction
    - Safety check: stops robot if tilt > 45°
    - Limits output to realistic velocities

---

## How to Run

1. **Start Gazebo + ROS2 Bridge**
   ```bash
   ros2 launch sel_bal launch_bot.py
    ```
2. Observe the robot in Gazebo.  
   - Robot should stay upright if PID parameters (`Kp`, `Ki`, `Kd`) are tuned
   - Logs show pitch angle and applied command

---

## Notes / Attention

- **PID Tuning:** The robot’s stability depends heavily on `Kp`, `Ki`, `Kd`. Start with small values and tune gradually.
- **URDF Inertia:** Always define `<inertial>` blocks for each link; otherwise Gazebo may behave unpredictably.
- **Diff-Drive Limits:** `max_wheel_torque` and `max_wheel_acceleration` control motor constraints; adjust for heavier or faster robots.
- **Bridge Node:** Logs help debug pitch vs applied velocity; deadzone ensures friction is overcome.
- **Launch File:** All topic names must match URDF and bridge node to function correctly.

> For more information on URDF structure and control templates, check the [templates/gazebo](https://github.com/yesserhmidi3/ros2-learning-journey/tree/main/templates/gazebo) folder.
