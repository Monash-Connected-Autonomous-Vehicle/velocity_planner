from launch import LaunchDescription

from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription([
        Node(
            package='velocity_planner',
            executable='fake_waypoint_publisher',
            name='fake_waypoint_publisher',
        ),
        Node(
            package='velocity_planner',
            executable='fake_object_publisher',
            name='fake_object_publisher',
        ),
    ])

