import os
from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    # MoveItCpp demo executable
    moveit_cpp_node = Node(
        name="pick_and_place",
        package="moveit2_scripts",
        executable="pick_and_place",
        output="screen",
        parameters=[
            {'use_sim_time': True},
        ],
    )

    return LaunchDescription(
        [moveit_cpp_node]
    )