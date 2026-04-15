# Gazebo URDF Template Guide

Building a robot for simulation boils down to three main parts: defining the physics, adding the plugins for hardware behavior, and bridging it to the ROS 2 control system.

---

## 1. Link Definition (The Physics)
Every part of your robot (link) must have these three blocks to behave correctly in Gazebo:

* **Visual**: How the robot looks. You can use simple shapes (box, cylinder) or complex `.stl/.dae` meshes.
* **Collision**: The "invisible hit-box." Keep this as a simple shape even if the visual is complex to save CPU power.
* **Inertial**: Defines the mass and how weight is distributed.
    * *Tip:* If your robot is shaking or flying away, your inertial values are likely too small or zero.

---

## 2. Plugins (Motors & Sensors)
Plugins tell Gazebo how a link should behave. You put these inside `<gazebo>` tags.

### Common Plugins:
* **Differential Drive**: Used for wheeled robots. It listens to velocity commands and moves the wheels.
* **IMU Sensor**: Provides orientation and acceleration data.
* **Joint State Publisher**: Required so ROS 2 knows the real-time position of your joints for RViz.

---

## 3. The Bridge (Control)
To use the advanced `ros2_control` framework (for servos or spider legs), you add a specific **Hardware Bridge**. This connects your URDF joints to your external `.yaml` configuration file.

This bridge allows you to send precise position or velocity commands to every joint simultaneously through the Controller Manager.

---

## How to Use
1. Copy the `urdf_temp.urdf` file.
2. Define your robot links using the **Link Template**.
3. Choose the **Plugin** that fits your robot (Diff Drive for wheels, ros2_control for servos/legs).
4. Launch your simulation.
