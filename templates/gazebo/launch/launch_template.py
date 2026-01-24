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
