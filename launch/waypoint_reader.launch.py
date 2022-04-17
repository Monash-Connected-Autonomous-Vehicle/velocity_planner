from launch import LaunchDescription

from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription([
        Node(
            package='velocity_planner',
            executable='waypoint_reader',
            name='waypoint_reader',
            parameters=[{
                'waypoint_filename': waypoint_filename,
            }]
        ),
    ])

