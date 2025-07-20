import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node

def generate_launch_description():
    # Package name - CHANGED from articubot_one to rosbot
    package_name = 'rosbot'

    # Include the robot_state_publisher launch file, provided by our own package
    # Force sim time to be enabled for Gazebo synchronization
    rsp = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([os.path.join(
            get_package_share_directory(package_name), 'launch', 'rsp.launch.py'
        )]), launch_arguments={'use_sim_time': 'true'}.items()
    )

    # Include the Gazebo Harmonic launch file (ros_gz_sim instead of gazebo_ros)
    gazebo = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([os.path.join(
            get_package_share_directory('ros_gz_sim'), 'launch', 'gz_sim.launch.py')]),
        launch_arguments={'gz_args': '-r -v 4 empty.sdf'}.items()
    )

    # Spawn entity using ros_gz_sim (instead of gazebo_ros spawn_entity.py)
    spawn_entity = Node(
        package='ros_gz_sim',
        executable='create',
        arguments=['-topic', 'robot_description', '-name', 'rosbot'],
        output='screen'
    )

    # Launch them all!
    return LaunchDescription([
        rsp,
        gazebo,
        spawn_entity,
    ])