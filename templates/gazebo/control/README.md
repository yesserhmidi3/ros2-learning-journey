# Gazebo Control Template

This is a **ROS2 Python template** for controlling a robot in Gazebo.  
It provides a basic structure for subscribing to sensor data, applying control logic, and publishing commands to the robot.

---

## Overview

The template includes:

1. **Publisher/Subscriber setup**  
   - Publishes `Twist` messages to `/cmd_vel` to control robot motion.
   - Subscribes to sensor data (example: IMU) on `/sensor_data`.

2. **Control variables**  
   - `target` – desired setpoint (e.g., target angle or position).  
   - `kp` – proportional gain for a simple proportional controller (tune as needed).

3. **Callback function**  
   - Receives sensor data.
   - Calculates error between current value and target.
   - Computes control output.
   - Publishes commands to actuators.

---

## Example Code

```python
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
from sensor_msgs.msg import Imu # Change this based on your sensor

class RobotController(Node):
    def __init__(self):
        super().__init__('robot_controller')
        
        # 1. Pub/Sub Setup
        self.pub = self.create_publisher(Twist, '/cmd_vel', 10)
        self.sub = self.create_subscription(Imu, '/sensor_data', self.callback, 10)
        
        # 2. Control Variables
        self.target = 0.0
        self.kp = 1.0 # Tune me!

    def callback(self, msg):
        # 3. Read Sensor (Processing Logic)
        current_value = msg.orientation.x # Example
        
        # 4. Control Logic (The Brain)
        error = self.target - current_value
        output = self.kp * error
        
        # 5. Send Action
        cmd = Twist()
        cmd.linear.x = float(output)
        self.pub.publish(cmd)

def main():
    rclpy.init()
    rclpy.spin(RobotController())
    rclpy.shutdown()
```

---

## How to Use

1.Place this file in your ROS2 Python package inside a suitable folder (e.g., `control/`).

2.Add it to your `setup.py` `entry_points` under `console_scripts`:
```python
entry_points={
    'console_scripts': [
        'robot_control = your_package_name.control_template:main',
    ],
},
```

3.Build your workspace and source it:
```bash
colcon build
source install/setup.bash
```

4.Run the control node:
```bash
ros2 run your_package_name robot_control
```

---

## Notes

- Modify the sensor topic and message type (`Imu`) to match your setup.
- Tune the `kp` (proportional gain) for your robot to achieve smooth control.
- This template can be extended to PID control by adding `ki` and `kd` terms.
