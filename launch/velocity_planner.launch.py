from launch import LaunchDescription

from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription([
        Node(
            package='velocity_planner',
            executable='velocity_planner',
            name='velocity_planner',
            parameters=[{
                'max_acceleration': 0.5,
                'local_plan_max_length': 25,
            }]
        ),
        Node(
            package='velocity_planner',
            executable='waypoint_visualiser',
            name='waypoint_visualiser',
        ),
        Node(
            package='velocity_planner',
            executable='object_visualiser',
            name='object_visualiser',
        ),
    ])

