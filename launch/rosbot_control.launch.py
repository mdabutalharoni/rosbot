from launch import LaunchDescription
from launch.actions import TimerAction
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription([
        TimerAction(
            period=2.0,
            actions=[
                Node(
                    package="controller_manager",
                    executable="spawner",
                    arguments=["joint_state_broadcaster"]
                ),
                Node(
                    package="controller_manager",
                    executable="spawner",
                    arguments=["diff_cont"]
                )
            ]
        )
    ])
