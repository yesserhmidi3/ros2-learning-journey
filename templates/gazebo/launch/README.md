# Gazebo Launch Template

This is a **ROS2 Python launch file template** for spawning a robot in Gazebo and bridging topics between ROS2 and Gazebo.  
It provides a basic structure for starting Gazebo, publishing the robot URDF, and setting up ROS2 ↔ Gazebo communication.

---

## Overview

The template does the following:

1. **Start Gazebo**  
   Launches an empty Gazebo world using `gz sim`.

2. **Robot State Publisher**  
   Publishes the URDF of your robot so ROS2 knows its structure and TF frames.

3. **Robot Spawner**  
   Spawns the robot into the Gazebo world at a specified height.

4. **ROS2 ↔ Gazebo Bridge**  
   Bridges topics between ROS2 and Gazebo:  
   - `/cmd_vel` → `/cmd_vel` (Twist messages)  
   - `/sensor_data` → `/sensor_data` (IMU messages)  
   - `/clock` → `/clock`  

---

## Example Code

```python
from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import ExecuteProcess
import os
from ament_index_python.packages import get_package_share_directory

def generate_launch_description():
    pkg_path = get_package_share_directory('my_robot_package')
    urdf_path = os.path.join(pkg_path, 'urdf', 'robot.urdf')

    return LaunchDescription([
        # 1. Start Gazebo
        ExecuteProcess(cmd=['gz', 'sim', 'empty.sdf'], output='screen'),

        # 2. Robot State Publisher (Publishes the URDF)
        Node(package='robot_state_publisher', executable='robot_state_publisher',
             parameters=[{'robot_description': open(urdf_path).read()}]),

        # 3. Spawner (Puts the robot in the world)
        Node(package='ros_gz_sim', executable='create',
             arguments=['-name', 'my_robot', '-topic', 'robot_description', '-z', '0.2']),

        # 4. THE BRIDGE (The most important part!)
        Node(package='ros_gz_bridge', executable='parameter_bridge',
             arguments=[
                 '/cmd_vel@geometry_msgs/msg/Twist@gz.msgs.Twist',
                 '/sensor_data@sensor_msgs/msg/Imu@gz.msgs.IMU',
                 '/clock@rosgraph_msgs/msg/Clock[gz.msgs.Clock'
             ])
    ])
```

---

## How to Use

1. Save this launch file in your package `launch/` folder, e.g., `launch_robot.py`.
2. Update `my_robot_package` to the name of your ROS2 package.
3. Make sure your URDF is in `urdf/robot.urdf`.
4. Run the launch file:
```bash
ros2 launch my_robot_package launch_robot.py
```
5. Gazebo will start, your robot will spawn, and ROS2 ↔ Gazebo topics will be bridged.

---

## Notes

- Modify the bridge topics to match your robot sensors and actuators.
- You can replace `empty.sdf` with a custom world.
- Ensure `ros_gz_sim` and `ros_gz_bridge` are installed for ROS2 Gazebo integration.
