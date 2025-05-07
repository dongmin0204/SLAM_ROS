from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription([
        Node(
            package='slam_ros',
            executable='movement_recorder',
            name='movement_recorder',
            output='screen'
        )
    ]) 