import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import ExecuteProcess, IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node

def generate_launch_description():
    # 1. SETUP PATHS
    package_name = 'my_robot_package' # <--- CHANGE THIS
    pkg_path = get_package_share_directory(package_name)
    urdf_path = os.path.join(pkg_path, 'urdf', 'robot.urdf')
    # If using xacro: urdf_content = xacro.process_file(xacro_path).toxml()
    
    # ==========================================
    # 2. START GAZEBO & SPAWN ROBOT
    # ==========================================
    
    # Launch Gazebo Sim
    start_gazebo = ExecuteProcess(
        cmd=['gz', 'sim', 'empty.sdf'], 
        output='screen'
    )

    # Robot State Publisher
    node_robot_state_publisher = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        output='screen',
        parameters=[{'robot_description': open(urdf_path).read(), 'use_sim_time': True}]
    )

    # Spawn the robot in Gazebo
    spawn_robot = Node(
        package='ros_gz_sim',
        executable='create',
        arguments=['-name', 'my_robot', '-topic', 'robot_description', '-z', '0.5'],
        output='screen'
    )

    # ==========================================
    # 3. BRIDGES & CONTROLLER SPAWNERS
    # ==========================================

    # GZ to ROS Bridge (Clock is essential!)
    bridge = Node(
        package='ros_gz_bridge',
        executable='parameter_bridge',
        arguments=[
            '/clock@rosgraph_msgs/msg/Clock[gz.msgs.Clock',
            '/cmd_vel@geometry_msgs/msg/Twist@gz.msgs.Twist',
            '/imu@sensor_msgs/msg/Imu@gz.msgs.IMU'
        ],
        output='screen'
    )

    # Controller Spawner: Joint State Broadcaster
    # This allows ROS2 to see joint positions
    spawn_jsb = Node(
        package="controller_manager",
        executable="spawner",
        arguments=["joint_state_broadcaster"],
    )

    # Controller Spawner: Joint Trajectory Controller
    # This activates the motors defined in your .yaml
    spawn_jtc = Node(
        package="controller_manager",
        executable="spawner",
        arguments=["joint_trajectory_controller"],
    )

    return LaunchDescription([
        start_gazebo,
        node_robot_state_publisher,
        spawn_robot,
        bridge,
        spawn_jsb,
        spawn_jtc
    ])
