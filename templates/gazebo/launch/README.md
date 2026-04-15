# Gazebo Launch Template Guide

The launch file is the orchestrator. It ensures that every node starts in the correct order so that the physics engine, the robot model, and the control software can communicate.

A complete Gazebo launch consists of three main phases:

---

## 1. Environment & World Setup
This part tells ROS 2 where to find your files and starts the Gazebo simulator.
* **Path Resolution:** Uses `get_package_share_directory` to find your URDF and Config folders.
* **Gazebo Process:** Launches the `gz sim` executable with your chosen world (e.g., `empty.sdf`).

---

## 2. Spawning the Robot
To see the robot in the 3D world, we need two nodes:
* **Robot State Publisher:** Reads your URDF and broadcasts the "Transforms" (TF). This tells ROS 2 how the links are connected.
* **Gazebo Spawner:** Physically "drops" the robot into the Gazebo world at the coordinates you specify.

---

## 3. The Bridges & Spawners (Communication)
This is the most critical part for control:
* **The Parameter Bridge:** Maps standard Gazebo topics (like `/clock` or `/scan`) to ROS 2 topics.
* **Controller Spawners:** These are special nodes that "activate" the controllers defined in your `.yaml` file. If you don't spawn these, your robot will just sit limp in the simulation.

---

## How to Use
1. Copy `launch_temp.py` to your `launch/` folder.
2. Change `'my_robot_package'` to your actual package name.
3. Add any extra topic bridges needed for your specific sensors (Lidar, Camera, etc.).
4. Run: `ros2 launch your_package_name launch_temp.py`
