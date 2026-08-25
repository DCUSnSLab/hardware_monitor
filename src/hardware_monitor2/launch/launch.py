# hardware_monitor2/launch/launch.py
from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription([
        Node(
            package='hardware_monitor2',
            executable='send_hunter',
            name='send_hunter',
            output='screen',
        ),

        Node(
            package='hardware_monitor2',
            executable='add_two_ints',
            name='add_two_ints',
            output='screen',
            # respawn=True,
        ),

        Node(
            package='rosapi',
            executable='rosapi_node',
            name='rospapi',
            output='screen',
        ),
    ])
