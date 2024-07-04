import os
from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    get_cube_pose_node = Node(
        package='get_cube_pose',
        name='get_cube_pose_node',
        executable='get_cube_pose_node',
        output='screen',
        parameters=[
            {'use_sim_time': True}
        ]
    )

    return LaunchDescription(
        [get_cube_pose_node]
    )