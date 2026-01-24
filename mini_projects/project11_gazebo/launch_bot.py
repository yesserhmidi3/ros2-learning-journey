from launch import LaunchDescription
from launch.actions import ExecuteProcess
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory
import os

def generate_launch_description():
    # Make sure to change 'spider_sim' to 'sel_bal' to match your folder
    package_name = 'sel_bal' 
    pkg_path = get_package_share_directory(package_name)
    urdf_file = os.path.join(pkg_path, 'urdf', 'balancing_bot.urdf')

    return LaunchDescription([
        # 1. Start Gazebo Sim (Empty World)
        ExecuteProcess(
            cmd=['gz', 'sim'
                 #, '-r'
                 , 'empty.sdf'], #-'r' for 'run headless' (no GUI) 
            output='screen'
        ),

        # 2. Robot State Publisher
        Node(
            package='robot_state_publisher',
            executable='robot_state_publisher',
            name='robot_state_publisher',
            output='screen',
            parameters=[{'robot_description': open(urdf_file).read()}]
        ),

        # 3. Spawn Robot
        Node(
            package='ros_gz_sim',
            executable='create',
            arguments=['-name', 'balancing_bot', '-topic', 'robot_description', '-z', '0.2'],
            output='screen'
        ),

        # 4. ROS-GZ Bridge (CRITICAL FOR JAZZY)
        # This maps Gazebo topics to ROS 2 topics
        Node(
            package='ros_gz_bridge',
            executable='parameter_bridge',
            arguments=[
                '/cmd_vel@geometry_msgs/msg/Twist@gz.msgs.Twist',
                '/imu@sensor_msgs/msg/Imu@gz.msgs.IMU',
                '/clock@rosgraph_msgs/msg/Clock[gz.msgs.Clock'
            ],
            output='screen'
        )
    ])