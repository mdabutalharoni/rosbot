from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription([
        # Bridge for /cmd_vel from ROS 2 → Gazebo Harmonic
        Node(
            package='ros_gz_bridge',
            executable='parameter_bridge',
            arguments=['/cmd_vel@geometry_msgs/msg/Twist@gz.msgs.Twist'],
            output='screen'
        ),

        # Bridge for TF (optional but useful)
        Node(
            package='ros_gz_bridge',
            executable='parameter_bridge',
            arguments=['/tf@tf2_msgs/msg/TFMessage@gz.msgs.Pose_V'],
            output='screen'
        )
    ])
