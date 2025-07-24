from launch import LaunchDescription
from launch.actions import ExecuteProcess, IncludeLaunchDescription, TimerAction
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory
from launch.substitutions import Command
import os
import xacro

def generate_launch_description():
    pkg_path = get_package_share_directory('rosbot')
    xacro_file = os.path.join(pkg_path, 'description', 'rosbot.urdf.xacro')

    # Evaluate xacro using Command so it’s evaluated at runtime
    robot_description = {
        'robot_description': Command(['xacro ', xacro_file, ' sim_mode:=true'])
    }

    world_file = os.path.join(pkg_path, 'worlds', 'empty.world')
    controller_yaml = os.path.join(pkg_path, 'config', 'diff_drive_controllers.yaml')
    control_launch = os.path.join(pkg_path, 'launch', 'rosbot_control.launch.py')

    return LaunchDescription([
        # 1. Start robot_state_publisher first
        Node(
            package='robot_state_publisher',
            executable='robot_state_publisher',
            output='screen',
            parameters=[robot_description]
        ),

        # 2. Start joint_state_publisher_gui
        Node(
            package='joint_state_publisher_gui',
            executable='joint_state_publisher_gui',
            output='screen'
        ),

        # 3. Start Gazebo world
        ExecuteProcess(
            cmd=['gz', 'sim', '-v', '4', world_file],
            output='screen'
        ),

        # 4. Spawn robot after Gazebo starts
        TimerAction(
            period=2.0,
            actions=[
                Node(
                    package='ros_gz_sim',
                    executable='create',
                    arguments=[
                        '-name', 'rosbot',
                        '-x', '0', '-y', '0', '-z', '0',
                        '-topic', 'robot_description'
                    ],
                    output='screen'
                )
            ]
        ),

        # 5. Launch controller manager after robot is spawned
        TimerAction(
            period=5.0,
            actions=[
                Node(
                    package='controller_manager',
                    executable='ros2_control_node',
                    parameters=[robot_description, controller_yaml],
                    output='screen'
                )
            ]
        ),

        # 6. Delay spawner until controller manager is ready
        TimerAction(
            period=8.0,
            actions=[
                IncludeLaunchDescription(
                    PythonLaunchDescriptionSource(control_launch)
                )
            ]
        )
    ])
