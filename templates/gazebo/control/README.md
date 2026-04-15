# Main Control Node Template

This is the "Brain" of your robot. While the URDF and YAML files define the body and the motors, this Python node is where you write the logic to make the robot perform tasks, stay balanced, or navigate autonomously.

---

## 1. Input: Subscription (The Senses)
A control loop starts by listening to the robot's environment.
* **Sensor Data:** Subscribing to IMU data (for balance), Lidars (for obstacles), or Joint States (to know current leg positions).
* **The Callback:** Every time a new sensor reading arrives, the `callback` function is triggered to process the new information.

---

## 2. Logic: The PID Controller (The Brain)
This is where the math happens.
* **Setpoint (Target):** Where you want the robot to be (e.g., Angle = 0).
* **Error Calculation:** The difference between where the robot is and where it should be.
* **Control Output:** Applying gains ($K_p$, $K_i$, $K_d$) to determine how much power to send to the motors to fix the error.

---

## 3. Output: Publication (The Action)
Once the math is done, the node sends a command back to Gazebo.
* **Teleop commands:** Sending a `Twist` message to move wheels.
* **Joint commands:** Sending a `JointTrajectory` message to move specific limbs.

---

## How to Use
1. Copy `main_control_temp.py` to your package scripts folder.
2. Register the node in `setup.py` under `console_scripts`.
3. In the `callback`, replace the example logic with your specific control algorithm (PID, State-Space, or RL).
4. Build, source, and run:
```bash
ros2 run your_package_name your_node_name
```
