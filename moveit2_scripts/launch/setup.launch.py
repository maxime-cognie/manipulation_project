import os
from launch import LaunchDescription
from launch_ros.actions import Node

from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource

from launch_ros.substitutions import FindPackageShare
from launch.substitutions import PathJoinSubstitution


def generate_launch_description():
    simple_grasping_node = Node(
        package='simple_grasping',
        name='basic_grasping_perception_node',
        executable='basic_grasping_perception_node',
        output='screen',
        parameters=[
            {'use_sim_time': True,
            'debug_topics': True}
        ]
    )

    move_group = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([
            PathJoinSubstitution([
                FindPackageShare('my_moveit_config'),
                'launch',
                'move_group.launch.py'
            ])
        ]),
    )

    rviz = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([
            PathJoinSubstitution([
                FindPackageShare('my_moveit_config'),
                'launch',
                'moveit_rviz.launch.py'
            ])
        ]),
        launch_arguments={
                'rviz_config': PathJoinSubstitution([
                    FindPackageShare('moveit2_scripts'),
                    'rviz',
                    'config.rviz'
                    ]),
            }.items()
    )

    return LaunchDescription(
        [simple_grasping_node,
        move_group,
        rviz]
    )