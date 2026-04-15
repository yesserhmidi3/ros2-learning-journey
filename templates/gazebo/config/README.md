# Controller Config Template (.yaml)

The `.yaml` file is where you define the "Nervous System" logic. While the URDF defines the physical joints, the config file defines how those joints should actually behave (e.g., following a path, holding a position, or reporting their state).

---

## 1. The Controller Manager
Every config file starts with the `controller_manager`. This is the "Boss" node that manages all other controllers.
* **Update Rate:** How fast the control loop runs (usually 50Hz or 100Hz).
* **Use Sim Time:** Must be `true` for Gazebo simulations to keep the clock synced.

---

## 2. Controller Types
You must "hire" specific workers (plugins) for your robot. Common ones include:
* **Joint State Broadcaster:** Required for all robots. It "broadcasts" where the joints are so you can see the robot move in RViz.
* **Joint Trajectory Controller:** The standard for multi-joint robots (Spider legs, arms). It allows for smooth, synchronized movement.
* **Position/Velocity Controllers:** For simpler, individual joint control.

---

## 3. Joint Mapping
In this section, you list the **exact names** of the joints you defined in your URDF.
* **Command Interfaces:** What the software sends to the motor (usually `position`).
* **State Interfaces:** What the software reads back from the motor (usually `position` and `velocity`).

---

## How to Use
1. Copy `controllers_temp.yaml` to your package's `config/` folder.
2. Replace `joint_name_1`, etc., with the actual names from your URDF.
3. Ensure this folder is registered in your `setup.py` so the launch file can find it.