from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription([
        Node(
            package='slam_ros',
            executable='movement_controller',
            name='movement_controller',
            output='screen'
        )
    ]) 