# Gazebo Simulation Templates Guide

This directory contains the modular boilerplate files required to build, simulate, and control a robot in **Gazebo Harmonic** using **ROS 2 Jazzy**. 

---

## The "Full Stack" Simulation Architecture
In ROS 2, a simulation is a distributed system. Each folder in this directory represents a layer of the robot's existence:

| Component | Folder | Responsibility |
| :--- | :--- | :--- |
| **The Body** | `/urdf` | Physical geometry, mass, inertia, and collision properties. |
| **The Settings** | `/config` | YAML file defining controller types and joint mappings. |
| **The Power** | `/launch` | The Python script that starts the nodes, Gazebo, and the Spawners. |
| **The Logic** | `/control` | Your custom Python scripts (PID, RL, AI) that send commands. |



---

## The "Hand-Off" Workflow
To get a robot moving, the data must flow through the files in this specific order:

### 1. The URDF Connection
Your URDF must include the `<ros2_control>` tags. This tells the physics engine: *"I have joints, but I am giving control of them over to the ROS 2 Controller Manager."*
* **Template location:** `urdf/urdf_temp.urdf`

### 2. The YAML Mapping
The `controllers.yaml` file acts as the "middle-man." It looks at the joints you named in the URDF and assigns them to a specific controller (like a `JointTrajectoryController`).
* **Template location:** `config/controllers_temp.yaml`

### 3. The Launch Orchestration
The launch file is the most critical piece. It doesn't just "open Gazebo"; it:
1.  Calculates **Robot State** (Transforms).
2.  Sets **Environment Paths** (so Gazebo finds your 3D meshes).
3.  Loads the **Controller Manager** and tells it to read your YAML.
4.  Calls the **Spawners** to physically activate the motors.
* **Template location:** `launch/launch_temp.py`

---

## Essential Debugging Checklist
If your simulation isn't working, check these "Big Three" common errors:

1.  **Invisible Robot:** Did you set `GZ_SIM_RESOURCE_PATH` in your launch file? If not, Gazebo cannot find your mesh files.
2.  **Controller Failure:** Did you add your `config/` folder to `setup.py`? If you don't register it in `data_files`, the code will run on your machine but fail once installed/built.
3.  **The "Exploding" Robot:** Are your `inertial` values (mass/inertia) too small? Gazebo’s ODE physics engine becomes unstable if mass is set to $0$ or near-zero on moving parts.

---

## Sub-Template Directories
For specific code breakdowns and line-by-line explanations, visit each folder:

* [**URDF Templates**](./urdf/README.md) – Physical modeling and Gazebo plugins.
* [**Config Templates**](./config/README.md) – Controller parameters and YAML structure.
* [**Launch Templates**](./launch/README.md) – Python Launch API and node orchestration.
* [**Control Templates**](./control/README.md) – Custom Python control nodes and PID logic.

---

## Resources I Used to Understand This
These are the specific guides that helped me understand how .yaml and launch files work, their structure, and how and what to add to the URDF as plugins:

* **[Robotics Backend: ROS 2 Gazebo Tutorials](https://roboticsbackend.com/category/ros2/)** – This was my main source for learning how the `robot_state_publisher` and control loops connect.
* **[gz_ros2_control Plugin Guide](https://control.ros.org/jazzy/doc/gz_ros2_control/doc/index.html)** – This is what I used to understand exactly how to implement the control plugins inside the URDF to bridge Gazebo with ROS 2.

---

> **Important** > This modularity is the strength of ROS 2. You can use the **exact same** YAML and Control scripts for a real physical robot just by changing one line in your URDF (the hardware plugin). This is known as **Hardware Abstraction**.
